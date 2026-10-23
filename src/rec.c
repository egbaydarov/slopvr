/* Session recorder: everything the tracker consumes and decides, in one append-only file, so a bad moment
 * can be replayed offline exactly (psvr2linuxtest --replay FILE.slop) without the headset or the user.
 *
 *   ~/.cache/slopvr_rec/YYYYmmdd-HHMMSS.slop   (setting "record": on by default in the test tool, opt-in in the driver; "recordMaxMb" cap, 12 newest / 3 GB kept)
 *
 * Layout: magic "SLOPREC\1", then records { u8 type; u8 sub; u16 len; u64 host_us; payload[len] } (little endian):
 *   REC_CALIB   factory camera calibration (GenData item 10, raw)
 *   REC_TUNE    "name=value name=value ..." of all tunables at start
 *   REC_LD      an LED-detector frame: u32 ts, u32 frame, f32 gain, f32 exposure, then per camera u8 n + n x
 *               { u16 xmin,xmax,ymin,ymax; u8 peak; u32 m00,m10,m01 } (only valid records)
 *   REC_HEAD    head pose used for that frame: u32 ts, f32 q[4] (w x y z), f32 p[3], u8 valid, u64 exposure host time
 *   REC_IMU     sub = hand: u32 dev_ticks, i16 gyro[3], i16 accel[3], f32 q[4] (fused world <- IMU), f32 qc[4], u8 qc_ok,
 *               u8 battery, u8 btn[3], u32 echo_host, u32 rtt
 *   REC_SENSECAL sub = hand: the controller's IMU calibration (f32 gs[3] as[3] ab[3], i16 gb[3])
 *   REC_LEDCMD  sub = hand: u8 phase, seq, period, par_fix, freerun; u32 pos, cycle; i32 lat   (on every (re)anchor)
 *   REC_CLK     sub = 0/1 controller, 2 headset: f64 a, b; i64 y0; u64 t0; i32 n; u32 rtt  (once a second)
 *   REC_FIT     sub = hand: u32 frame; u8 ok, acq, n; f32 res; f32 R[9], t[3] (cam0 <- model); f32 pw[3] (world)
 *   REC_PUB     sub = hand: f32 p[3], q[4], v[3]; u8 mode  (published pose, ~100 Hz per hand)
 *   REC_TEXT    a log line
 */
#include <dirent.h>
#include <sys/stat.h>
#include <fcntl.h>

enum { REC_CALIB = 1, REC_TUNE, REC_LD, REC_HEAD, REC_IMU, REC_SENSECAL, REC_LEDCMD, REC_CLK, REC_FIT, REC_PUB, REC_TEXT };

static struct {
	FILE *f; pthread_mutex_t lock; bool on, full; uint64_t bytes, cap, flush_us; char path[320];
} REC = { .lock = PTHREAD_MUTEX_INITIALIZER };

static void rec_prune(const char *root, int keep)
{
	DIR *d = opendir(root); if (!d) return;
	char names[64][40]; int n = 0; struct dirent *e;
	while ((e = readdir(d)) && n < 64) { size_t l = strlen(e->d_name); if (l > 5 && !strcmp(e->d_name + l - 5, ".slop")) snprintf(names[n++], 40, "%.39s", e->d_name); }
	closedir(d);
	for (int i = 1; i < n; i++) { char t[40]; memcpy(t, names[i], 40); int j = i; while (j > 0 && strcmp(names[j - 1], t) > 0) { memcpy(names[j], names[j - 1], 40); j--; } memcpy(names[j], t, 40); }
	uint64_t total = 0; char p[400];
	for (int i = n - 1; i >= 0; i--) { /* newest first: keep `keep` files or 3 GB, whichever comes first */
		snprintf(p, sizeof p, "%s/%s", root, names[i]);
		int fd = open(p, O_RDONLY); if (fd < 0) continue; /* size via lseek: stat() is a glibc 2.33 symbol, the Steam runtime has 2.31 */
		off_t size = lseek(fd, 0, SEEK_END); close(fd);
		if (size > 0) total += (uint64_t)size;
		if (n - i > keep || total > 3000000000ull) unlink(p);
	}
}

static void rec_write(uint8_t type, uint8_t sub, uint64_t host_us, const void *payload, size_t len)
{
	if (!REC.on || len > 65535) return;
	pthread_mutex_lock(&REC.lock);
	if (REC.f && !REC.full) {
		uint8_t h[12]; uint16_t l = (uint16_t)len;
		h[0] = type; h[1] = sub; memcpy(h + 2, &l, 2); memcpy(h + 4, &host_us, 8);
		fwrite(h, 1, sizeof h, REC.f); if (len) fwrite(payload, 1, len, REC.f);
		REC.bytes += sizeof h + len;
		if (REC.bytes > REC.cap) { REC.full = true; fflush(REC.f); }
		else if (host_us - REC.flush_us > 1000000) { fflush(REC.f); REC.flush_us = host_us; }
	}
	pthread_mutex_unlock(&REC.lock);
}

static void rec_text(const char *line) { if (REC.on) rec_write(REC_TEXT, 0, now_us(), line, strlen(line)); }

static void rec_open(bool enable, int max_mb)
{
	if (!enable) return;
	const char *home = getenv("HOME") ? getenv("HOME") : "/tmp";
	char dir[300]; snprintf(dir, sizeof dir, "%s/.cache/slopvr_rec", home); mkdir(dir, 0755);
	rec_prune(dir, 11); /* this session makes the twelfth */
	time_t t = time(NULL); struct tm tm; localtime_r(&t, &tm); char stamp[32]; strftime(stamp, sizeof stamp, "%Y%m%d-%H%M%S", &tm);
	snprintf(REC.path, sizeof REC.path, "%s/%s.slop", dir, stamp);
	REC.f = fopen(REC.path, "wb");
	if (!REC.f) { slog("rec: cannot write %s", REC.path); return; }
	setvbuf(REC.f, NULL, _IOFBF, 1 << 16);
	fwrite("SLOPREC\1", 1, 8, REC.f);
	REC.cap = (uint64_t)(max_mb > 10 ? max_mb : 10) * 1000000ull; REC.bytes = 8; REC.full = false; REC.on = true;
	slog("rec: recording the session to %s (cap %d MB; replay: psvr2linuxtest --replay %s)", REC.path, max_mb, REC.path);
}

static void rec_close(void)
{
	pthread_mutex_lock(&REC.lock);
	if (REC.f) { fclose(REC.f); REC.f = NULL; }
	REC.on = false;
	pthread_mutex_unlock(&REC.lock);
}

/* ---- typed writers (payload packing) ---- */
#define REC_PUT(p, v) do { memcpy(p, &(v), sizeof(v)); p += sizeof(v); } while (0)

static void rec_calib(const uint8_t *b, int len) { rec_write(REC_CALIB, 0, now_us(), b, (size_t)len); }

static void rec_ld(const uint8_t *b, int len, uint64_t now) /* compact copy of the LD packet (valid blobs only) */
{
	if (!REC.on || len < 0x40) return;
	uint8_t buf[16 + 4 * (1 + 128 * 23)], *p = buf;
	uint32_t ts, frame; float gain, expo; memcpy(&ts, b + 8, 4); memcpy(&frame, b + 20, 4); memcpy(&gain, b + 32, 4); memcpy(&expo, b + 36, 4);
	REC_PUT(p, ts); REC_PUT(p, frame); REC_PUT(p, gain); REC_PUT(p, expo);
	for (int cam = 0; cam < 4; cam++) {
		const uint8_t *blk = b + 0x40 + cam * 0x2404; uint32_t n = 0; if (blk + 4 <= b + len) memcpy(&n, blk, 4); if (n > 128) n = 128;
		uint8_t *pn = p++; uint8_t cnt = 0;
		for (uint32_t i = 0; i < n; i++) {
			const uint8_t *r = blk + 4 + i * 36; if (r + 36 > b + len) break;
			uint16_t valid; uint32_t m00; memcpy(&valid, r, 2); memcpy(&m00, r + 12, 4);
			if (!valid || !m00) continue;
			memcpy(p, r + 4, 8); p += 8; *p++ = r[2]; memcpy(p, r + 12, 12); p += 12; cnt++;
		}
		*pn = cnt;
	}
	rec_write(REC_LD, 0, now, buf, (size_t)(p - buf));
}

static void rec_head(uint32_t ts, quat q, const double pos[3], bool valid, uint64_t now, uint64_t t_exp)
{
	if (!REC.on) return;
	uint8_t buf[64], *p = buf; float f;
	REC_PUT(p, ts);
	f = (float)q.w; REC_PUT(p, f); f = (float)q.x; REC_PUT(p, f); f = (float)q.y; REC_PUT(p, f); f = (float)q.z; REC_PUT(p, f);
	for (int i = 0; i < 3; i++) { f = (float)pos[i]; REC_PUT(p, f); }
	*p++ = valid;
	REC_PUT(p, t_exp); /* host time of the exposure the tracker used (replay timing) */
	rec_write(REC_HEAD, 0, now, buf, (size_t)(p - buf));
}

static void rec_imu(int hand, uint32_t dev_ticks, const int16_t gyro[3], const int16_t accel[3], quat q, quat qc, bool qc_ok, uint8_t battery, const uint8_t btn[3], uint32_t echo_host, uint32_t rtt, uint64_t now)
{
	if (!REC.on) return;
	uint8_t buf[96], *p = buf; float f;
	REC_PUT(p, dev_ticks); memcpy(p, gyro, 6); p += 6; memcpy(p, accel, 6); p += 6;
	f = (float)q.w; REC_PUT(p, f); f = (float)q.x; REC_PUT(p, f); f = (float)q.y; REC_PUT(p, f); f = (float)q.z; REC_PUT(p, f);
	f = (float)qc.w; REC_PUT(p, f); f = (float)qc.x; REC_PUT(p, f); f = (float)qc.y; REC_PUT(p, f); f = (float)qc.z; REC_PUT(p, f);
	*p++ = qc_ok; *p++ = battery; memcpy(p, btn, 3); p += 3; REC_PUT(p, echo_host); REC_PUT(p, rtt);
	rec_write(REC_IMU, (uint8_t)hand, now, buf, (size_t)(p - buf));
}

static void rec_sensecal(int hand, const float gs[3], const float as[3], const float ab[3], const int16_t gb[3])
{
	if (!REC.on) return;
	uint8_t buf[48], *p = buf;
	memcpy(p, gs, 12); p += 12; memcpy(p, as, 12); p += 12; memcpy(p, ab, 12); p += 12; memcpy(p, gb, 6); p += 6;
	rec_write(REC_SENSECAL, (uint8_t)hand, now_us(), buf, (size_t)(p - buf));
}

static void rec_ledcmd(int hand, uint8_t phase, uint8_t seq, uint8_t period, int par_fix, bool freerun, uint32_t pos, uint32_t cycle, int32_t lat, uint64_t now)
{
	if (!REC.on) return;
	uint8_t buf[24], *p = buf;
	*p++ = phase; *p++ = seq; *p++ = period; *p++ = (uint8_t)par_fix; *p++ = freerun; REC_PUT(p, pos); REC_PUT(p, cycle); REC_PUT(p, lat);
	rec_write(REC_LEDCMD, (uint8_t)hand, now, buf, (size_t)(p - buf));
}

static void rec_clk(int which, double a, double b, int64_t y0, uint64_t t0, int n, uint32_t rtt, uint64_t now)
{
	if (!REC.on) return;
	uint8_t buf[48], *p = buf; int32_t n32 = n;
	REC_PUT(p, a); REC_PUT(p, b); REC_PUT(p, y0); REC_PUT(p, t0); REC_PUT(p, n32); REC_PUT(p, rtt);
	rec_write(REC_CLK, (uint8_t)which, now, buf, (size_t)(p - buf));
}

static void rec_fit(int hand, uint32_t frame, bool ok, bool acq, int n, double res, const double R[9], const double t[3], const double pw[3], uint64_t now)
{
	if (!REC.on) return;
	uint8_t buf[80], *p = buf; float f;
	REC_PUT(p, frame); *p++ = ok; *p++ = acq; *p++ = (uint8_t)(n < 0 ? 0 : n > 255 ? 255 : n); f = (float)res; REC_PUT(p, f);
	for (int i = 0; i < 9; i++) { f = (float)(ok ? R[i] : 0); REC_PUT(p, f); }
	for (int i = 0; i < 3; i++) { f = (float)(ok ? t[i] : 0); REC_PUT(p, f); }
	for (int i = 0; i < 3; i++) { f = (float)(ok ? pw[i] : 0); REC_PUT(p, f); }
	rec_write(REC_FIT, (uint8_t)hand, now, buf, (size_t)(p - buf));
}

static void rec_pub(int hand, const double pos[3], quat q, const double v[3], int mode, uint64_t now)
{
	if (!REC.on) return;
	uint8_t buf[48], *p = buf; float f;
	for (int i = 0; i < 3; i++) { f = (float)pos[i]; REC_PUT(p, f); }
	f = (float)q.w; REC_PUT(p, f); f = (float)q.x; REC_PUT(p, f); f = (float)q.y; REC_PUT(p, f); f = (float)q.z; REC_PUT(p, f);
	for (int i = 0; i < 3; i++) { f = (float)v[i]; REC_PUT(p, f); }
	*p++ = (uint8_t)mode;
	rec_write(REC_PUB, (uint8_t)hand, now, buf, (size_t)(p - buf));
}
#undef REC_PUT
