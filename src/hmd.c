/* slopvr — PSVR2 headset: usbfs I/O thread, 6DoF (on-device SLAM + 2 kHz
 * gyro dead-reckoning), proximity/function button/IPD, brightness, gaze
 * (eye tracking), and the IVRDisplayComponent (direct mode, lens distortion). */
#include "usb.h"
#include "lut.h"

#define PSVR2_VID 0x054C
#define PSVR2_PID 0x0CDE
#define IF_SLAM 3
#define IF_GAZE 5
#define IF_CAM 6
#define IF_STATUS 7
#define IF_LEDDET 8   /* on-device LED blob detection stream */
#define IF_GENDATA 10 /* factory calibration blobs ("VD") */
#define EP_SLAM 0x83
#define EP_GAZE 0x85
#define EP_CAM 0x87
#define EP_STATUS 0x88
#define EP_LEDDET 0x89
#define EP_GENDATA 0x8B
#define EP_GAZE_OUT 0x05
#define RPT_CAMERA_MODE 0x0B
#define RPT_GAZE_STREAM 0x0C
#define RPT_LEDDET 0x14
#define LD_PKT_SIZE 0x9050
#define SLAM_RING 32
#define RPT_BRIGHTNESS 0x12
#define RPT_FIRMWARE 0x81
#define RPT_DISTORTION 0x8F

#define PANEL_W 4000
#define PANEL_H 2040
#define GYRO_SCALE (2000.0 / 32767.0 * M_PI / 180.0) /* rad/s per LSB */
#define GYRO_RING 2048 /* ~1 s at 2 kHz */
#define N_URB 10
/* half-tangents of the asymmetric FOV: must match the distortion fx/fy/cx */
#define TAN_OUT 1.84177f /* temporal (left eye: left edge) */
#define TAN_IN 0.94719f  /* nasal */
#define TAN_V 1.32898f

enum { URB_STATUS0, URB_STATUS1, URB_SLAM0, URB_SLAM1, URB_GAZE, URB_CAM0, URB_CAM1, URB_LD0, URB_LD1, URB_GD };

struct __attribute__((packed)) sie_ctrl_pkt { uint16_t report_id, subcmd; uint32_t len; uint8_t data[504]; };
struct __attribute__((packed)) imu_rec { uint32_t vts_us; int16_t accel[3], gyro[3]; uint16_t dp_frame, dp_line, imu_ts, status; };
struct __attribute__((packed)) status_hdr { uint8_t dprx, prox, button, pad[2], ipd_mm, rest[26]; };
struct __attribute__((packed)) slam_rec { char hdr[3]; uint8_t c1; uint32_t size, vts_us, unknown1; float pos[3], orient[4]; uint8_t rest[468]; };
struct __attribute__((packed)) fw_info { uint32_t version, res0; char hash[40]; uint32_t unk0, unk1; char pcb_id[16]; uint32_t unk2, res1, recovery; };
struct __attribute__((packed)) eye_gaze {
	uint32_t point_valid; float point_mm[3]; uint32_t dir_valid; float dir[3];
	uint32_t pupil_valid; float pupil_mm; uint32_t b2; float f2[2]; uint32_t b3; float f4[2]; uint32_t blink_valid, blink;
};
struct __attribute__((packed)) gaze_combined {
	uint32_t point_valid; float point_mm[3]; uint32_t dir_valid; float dir[3]; uint32_t is_valid, timestamp;
	uint32_t b7; float f8; uint32_t b9; float f12[3], f15[3], f18[3];
};
struct __attribute__((packed)) gaze_pkt {
	char hdr[2]; uint16_t version;
	uint32_t size, unk1, unk2, unk3, ts1, ts2, ts3;
	uint32_t b1; float f1; uint32_t b2, b3; float f2; uint32_t b4, b5, b6, b7, b8; float f3; uint32_t b9; float f4; uint32_t b10; float f5;
	struct eye_gaze left, right;
	struct gaze_combined combined;
};
static_assert(sizeof(struct imu_rec) == 24 && sizeof(struct status_hdr) == 32 && sizeof(struct slam_rec) == 512, "psvr2 structs");
static_assert(sizeof(struct gaze_pkt) == 324 && sizeof(struct fw_info) == 84, "psvr2 structs");

struct gyro_sample { uint32_t t; float g[3]; };

static struct hmd {
	const ovr_fn *vt;                     /* ITrackedDeviceServerDriver_005 */
	struct { const ovr_fn *vt; } display; /* IVRDisplayComponent_003 (sub-object) */
	uint32_t id;
	uint64_t container;
	uint64_t h_prox, h_system, h_gaze;
	atomic_bool run, connected, active, ready;
	bool added, fake;

	int fd;
	pthread_t thread;
	struct usbdevfs_urb urb[N_URB];
	uint8_t *buf[N_URB];
	int camera_mode;

	pthread_mutex_t lock;
	struct gyro_sample ring[GYRO_RING];
	unsigned ring_n;
	uint32_t imu_t; bool imu_ok; quat imu_q;
	uint64_t imu_host_us, slam_host_us;
	quat slam_q, slam_q_raw; double slam_p[3], slam_v[3]; uint32_t slam_t; bool slam_ok, slam_init;
	struct { uint32_t t; quat q; double p[3]; } sring[SLAM_RING]; unsigned sring_n;
	uint8_t prox, button, ipd_mm, dprx;
	uint64_t gaze_keepalive_us, ld_keepalive_us, gd_retry_us;
	bool optical; int gd_tries, ld_gain, ld_exposure;

	float calib[8];
	float display_hz;
	uint32_t render_w, render_h;
	float analog_gain;
	char serial[32];
	uint32_t fw_version;
} H;

static const quat SLAM_FIX = { 0.70710678118654752, 0, 0, 0.70710678118654752 }; /* +90° about Z */

/* ---- vendor control transfers (report id in wValue, sie_ctrl_pkt payload) ---- */
static bool hmd_ctrl_set(uint16_t report, uint16_t subcmd, const void *data, uint32_t len)
{
	struct sie_ctrl_pkt p = { report, subcmd, len, { 0 } };
	if (len) memcpy(p.data, data, len);
	return H.fd >= 0 && usb_ctrl(H.fd, 0x42, 0x09, report, 0, &p, (uint16_t)(len + 8), 100) >= 0;
}
static bool hmd_ctrl_get(uint16_t report, uint16_t subcmd, void *out, uint32_t len)
{
	struct sie_ctrl_pkt p = { report, subcmd, len, { 0 } };
	if (H.fd < 0 || usb_ctrl(H.fd, 0xC2, 0x01, report, 0, &p, (uint16_t)(len + 8), 100) < 0) return false;
	memcpy(out, p.data, len);
	return true;
}
static void hmd_set_brightness(float gain01)
{
	uint8_t b = (uint8_t)CLAMP(ceilf(powf(CLAMP(gain01, 0.f, 1.f), 1.f / 2.2f) * 31.f), 0.f, 31.f);
	hmd_ctrl_set(RPT_BRIGHTNESS, 1, &b, 1);
}

/* ---- pose: SLAM sample + gyro integration up to the newest IMU sample ---- */
static void hmd_submit_pose_locked(void)
{
	if (!atomic_load(&H.active)) return;
	DriverPose_t p = pose_default();
	uint64_t now = now_us();
	p.deviceIsConnected = atomic_load(&H.connected);
	p.vecDriverFromHeadTranslation[0] = 0.000247; /* SLAM/IMU origin -> head centre */
	p.vecDriverFromHeadTranslation[1] = -0.000273;
	p.vecDriverFromHeadTranslation[2] = 0.104826;
	p.poseTimeOffset = -0.001;

	if (!H.imu_ok || now - H.imu_host_us > 500000) {
		p.result = p.deviceIsConnected ? TrackingResult_Calibrating_InProgress : TrackingResult_Uninitialized;
		pose_submit(H.id, &p);
		return;
	}

	quat q = H.slam_ok ? H.slam_q : H.imu_q;
	double pos[3] = { H.slam_p[0], H.slam_p[1], H.slam_p[2] };
	const float *g_last = H.ring[(H.ring_n - 1) % GYRO_RING].g;
	if (H.slam_ok) {
		unsigned n = H.ring_n < GYRO_RING ? H.ring_n : GYRO_RING;
		uint32_t prev_t = H.slam_t;
		for (unsigned i = H.ring_n - n; i < H.ring_n; i++) {
			const struct gyro_sample *s = &H.ring[i % GYRO_RING];
			if ((int32_t)(s->t - H.slam_t) <= 0) continue;
			double dt = CLAMP((int32_t)(s->t - prev_t) * 1e-6, 0.0, 0.002);
			q = qmul(q, qrotvec(s->g[0] * dt, s->g[1] * dt, s->g[2] * dt));
			prev_t = s->t;
		}
		q = qnorm(q);
		double dt_slam = CLAMP((int32_t)(H.imu_t - H.slam_t) * 1e-6, 0.0, 0.1);
		for (int i = 0; i < 3; i++) pos[i] += H.slam_v[i] * dt_slam;
	}

	double wb[3] = { g_last[0], g_last[1], g_last[2] }, ww[3];
	qrot(q, wb, ww);
	p.qRotation = q;
	memcpy(p.vecPosition, pos, sizeof pos);
	memcpy(p.vecVelocity, H.slam_v, sizeof p.vecVelocity);
	memcpy(p.vecAngularVelocity, ww, sizeof ww);
	p.poseIsValid = true;
	p.result = H.slam_ok && now - H.slam_host_us < 300000 ? TrackingResult_Running_OK : TrackingResult_Running_OutOfRange;
	pose_submit(H.id, &p);

	pthread_mutex_lock(&g_head_lock);
	g_head_q = q;
	g_head_p[0] = pos[0] + 0.000247; g_head_p[1] = pos[1] - 0.000273; g_head_p[2] = pos[2] + 0.104826;
	g_head_valid = true;
	pthread_mutex_unlock(&g_head_lock);
}

static void hmd_on_status(const uint8_t *b, int len)
{
	if (len < (int)sizeof(struct status_hdr)) return;
	const struct status_hdr *h = (const void *)b;
	int n = (len - (int)sizeof *h) / (int)sizeof(struct imu_rec);
	const struct imu_rec *r = (const void *)(b + sizeof *h);

	pthread_mutex_lock(&H.lock);
	for (int i = 0; i < n; i++, r++) {
		struct gyro_sample *s = &H.ring[H.ring_n % GYRO_RING];
		s->t = r->vts_us;
		s->g[0] = (float)(-r->gyro[1] * GYRO_SCALE); /* IMU -> head axes */
		s->g[1] = (float)(r->gyro[2] * GYRO_SCALE);
		s->g[2] = (float)(-r->gyro[0] * GYRO_SCALE);
		double dt = H.imu_ok ? CLAMP((int32_t)(s->t - H.imu_t) * 1e-6, 0.0, 0.002) : 0.0;
		H.imu_q = qnorm(qmul(H.imu_q, qrotvec(s->g[0] * dt, s->g[1] * dt, s->g[2] * dt)));
		H.imu_t = s->t;
		H.imu_ok = true;
		H.ring_n++;
	}
	if (n > 0) { H.imu_host_us = now_us(); optical_hmd_clock(H.imu_t, H.imu_host_us); }
	bool prox_changed = h->prox != H.prox, btn_changed = h->button != H.button, ipd_changed = h->ipd_mm != H.ipd_mm;
	H.prox = h->prox; H.button = h->button; H.ipd_mm = h->ipd_mm; H.dprx = h->dprx;
	hmd_submit_pose_locked();
	pthread_mutex_unlock(&H.lock);

	if (!atomic_load(&H.active)) return;
	if (prox_changed) ovr_input_UpdateBooleanComponent(g_input, H.h_prox, h->prox != 0, 0);
	if (btn_changed) ovr_input_UpdateBooleanComponent(g_input, H.h_system, h->button != 0, 0);
	if (ipd_changed && h->ipd_mm >= 50 && h->ipd_mm <= 80) prop_f32(H.container, Prop_UserIpdMeters_Float, h->ipd_mm / 1000.f);
}

static void hmd_on_slam(const uint8_t *b, int len)
{
	if (len < (int)sizeof(struct slam_rec)) return; /* zero-length packets at stream start */
	const struct slam_rec *s = (const void *)b;
	/* header is "SLA" or "SLP" depending on firmware; Monado doesn't check it at all */
	if (s->hdr[0] != 'S' || s->hdr[1] != 'L' || s->size != sizeof *s) return;
	/* unknown1 low byte: 0/1 while the tracker initialises (~1 s after the panel lights up), then 3 with
	 * single-record flickers to 0/1/2; bits 8 and 24 toggle per record. Like Monado, use every record
	 * once the tracker is up — gating on the flicker would drop samples. */
	if (!H.slam_init) {
		if ((s->unknown1 & 0xff) != 3) return;
		H.slam_init = true;
		slog("hmd: SLAM \"%.3s\" tracking", s->hdr);
	}
	quat raw = { s->orient[0], -s->orient[2], -s->orient[1], s->orient[3] }; /* wire -> OpenVR axes */
	double p[3] = { s->pos[2], s->pos[1], -s->pos[0] };

	pthread_mutex_lock(&H.lock);
	if (H.slam_ok) {
		quat o = H.slam_q_raw; /* keep quaternion sign continuous */
		if (o.w * raw.w + o.x * raw.x + o.y * raw.y + o.z * raw.z < 0) raw = (quat){ -raw.w, -raw.x, -raw.y, -raw.z };
		double dt = (int32_t)(s->vts_us - H.slam_t) * 1e-6;
		if (dt > 1e-4 && dt < 0.5)
			for (int i = 0; i < 3; i++) H.slam_v[i] = 0.7 * H.slam_v[i] + 0.3 * (p[i] - H.slam_p[i]) / dt;
	}
	H.slam_q_raw = raw;
	H.slam_q = qnorm(qmul(SLAM_FIX, raw));
	memcpy(H.slam_p, p, sizeof p);
	H.slam_t = s->vts_us;
	H.slam_ok = true;
	H.slam_host_us = now_us();
	H.sring[H.sring_n % SLAM_RING].t = s->vts_us;
	H.sring[H.sring_n % SLAM_RING].q = H.slam_q;
	memcpy(H.sring[H.sring_n % SLAM_RING].p, p, sizeof p);
	H.sring_n++;
	hmd_submit_pose_locked();
	pthread_mutex_unlock(&H.lock);
}

/* SLAM pose (world <- IMU origin) nearest to device time t; latest if t is off the ring */
static bool hmd_head_at(uint32_t t, quat *q, double p[3])
{
	bool ok = false;
	pthread_mutex_lock(&H.lock);
	if (H.slam_ok && H.sring_n) {
		unsigned n = H.sring_n < SLAM_RING ? H.sring_n : SLAM_RING, best = H.sring_n - 1;
		int32_t bd = INT32_MAX;
		for (unsigned i = H.sring_n - n; i < H.sring_n; i++) {
			int32_t d = (int32_t)(H.sring[i % SLAM_RING].t - t);
			if (abs(d) < bd) { bd = abs(d); best = i; }
		}
		if (bd > 50000) best = H.sring_n - 1;
		*q = H.sring[best % SLAM_RING].q;
		memcpy(p, H.sring[best % SLAM_RING].p, 3 * sizeof *p);
		ok = true;
	}
	pthread_mutex_unlock(&H.lock);
	return ok;
}

/* GenData "VD" block: items {u16 id, u16, u32 size, u32 off} at 0x20; item 10 = camera calibration */
static void hmd_on_gendata(const uint8_t *b, int len)
{
	if (len < 0x20 || b[0] != 'V' || b[1] != 'D') return;
	uint16_t n; memcpy(&n, b + 8, 2);
	for (uint16_t i = 0; i < n && 0x20 + (i + 1) * 12 <= len; i++) {
		uint16_t id; uint32_t size, off;
		memcpy(&id, b + 0x20 + i * 12, 2); memcpy(&size, b + 0x20 + i * 12 + 4, 4); memcpy(&off, b + 0x20 + i * 12 + 8, 4);
		if (id == 10 && off < (uint32_t)len && size <= (uint32_t)len - off) {
			bool had = optical_calib_ok();
			opt_set_calib(b + off, (int)size);
			if (!had) slog("hmd: factory camera calibration %s (GenData item 10, %u bytes, after %d requests)", optical_calib_ok() ? "loaded" : "REJECTED", size, H.gd_tries);
		}
	}
}

static bool hmd_start_leddet(void)
{
	/* subcmd 1 payload: analog gain, exposure (us), flags (as the official driver) */
	uint32_t cfg[4] = { (uint32_t)H.ld_gain, (uint32_t)H.ld_exposure, 0x24800001, 0 };
	H.ld_keepalive_us = now_us();
	hmd_ctrl_set(RPT_LEDDET, 2, NULL, 0); /* a stream left running by a previous instance ignores a plain start */
	return hmd_ctrl_set(RPT_LEDDET, 1, cfg, sizeof cfg);
}

static void hmd_on_gaze(const uint8_t *b, int len)
{
	if (len < (int)sizeof(struct gaze_pkt) || b[0] != 'G' || b[1] != 'S' || !H.h_gaze || !atomic_load(&H.active)) return;
	const struct gaze_combined *c = &((const struct gaze_pkt *)(const void *)b)->combined;
	static bool logged;
	if (!logged && c->is_valid && c->dir_valid) { logged = true; slog("hmd: gaze tracking"); }
	VREyeTrackingData_t d = { 0 };
	d.bActive = true;
	d.bValid = c->dir_valid != 0;
	d.bTracked = c->is_valid != 0 && c->dir_valid != 0;
	d.vGazeTarget.v[0] = -c->dir[0]; /* device -> head frame */
	d.vGazeTarget.v[1] = c->dir[1];
	d.vGazeTarget.v[2] = -c->dir[2];
	ovr_input_UpdateEyeTrackingComponent(g_input, H.h_gaze, &d, 0);
}

/* ---- USB lifecycle ---- */
static bool hmd_submit(int i)
{
	static const struct { uint8_t type, ep; int len; } cfg[N_URB] = {
		{ USBDEVFS_URB_TYPE_INTERRUPT, EP_STATUS, 1024 }, { USBDEVFS_URB_TYPE_INTERRUPT, EP_STATUS, 1024 },
		{ USBDEVFS_URB_TYPE_BULK, EP_SLAM, 1024 }, { USBDEVFS_URB_TYPE_BULK, EP_SLAM, 1024 },
		{ USBDEVFS_URB_TYPE_BULK, EP_GAZE, 16384 },
		/* one camera frame per URB (mode 0x10: 1040640 B, mode 1: 819456 B; both end on a short packet) */
		{ USBDEVFS_URB_TYPE_BULK, EP_CAM, 1040640 }, { USBDEVFS_URB_TYPE_BULK, EP_CAM, 1040640 },
		{ USBDEVFS_URB_TYPE_BULK, EP_LEDDET, LD_PKT_SIZE }, { USBDEVFS_URB_TYPE_BULK, EP_LEDDET, LD_PKT_SIZE },
		{ USBDEVFS_URB_TYPE_BULK, EP_GENDATA, 0x8000 },
	};
	if (!H.buf[i]) H.buf[i] = mem_alloc(cfg[i].len); /* arena: kept across reopen (the sizes per slot never change) */
	return usb_submit(H.fd, &H.urb[i], cfg[i].type, cfg[i].ep, H.buf[i], cfg[i].len) == 0;
}

static bool hmd_open(void)
{
	int fd = usb_open_vidpid(PSVR2_VID, PSVR2_PID);
	if (fd < 0) return false;
	H.fd = fd;
	if (usb_claim(fd, IF_STATUS, 1) < 0 || usb_claim(fd, IF_SLAM, 0) < 0 || usb_claim(fd, IF_GAZE, 0) < 0) {
		slog("hmd: cannot claim interfaces (%s) — check udev rules", strerror(errno));
		close(fd); H.fd = -1;
		return false;
	}
	struct fw_info fw;
	if (hmd_ctrl_get(RPT_FIRMWARE, 1, &fw, sizeof fw)) {
		snprintf(H.serial, sizeof H.serial, "%.16s", fw.pcb_id);
		H.fw_version = fw.version;
		slog("hmd: firmware %08x serial %s", fw.version, H.serial);
	}
	/* lens calibration (report 0x8f) -> per-eye distortion constants */
	struct { uint8_t version, unk[7]; float p[32]; } cb = { 0 };
	uint8_t raw[256];
	if (hmd_ctrl_get(RPT_DISTORTION, 1, raw, sizeof raw)) memcpy(&cb, raw, sizeof cb);
	if (cb.version >= 4) {
		const float *p = cb.p;
		H.calib[0] = (float)((((-p[0] - p[6]) * 29.9 + 14.95) / 1000.0 - 3.22) / 32.46199);
		H.calib[1] = (float)((((-p[1] * 29.9) + 14.95) / 1000.0) / 32.46199);
		H.calib[2] = (float)((((p[6] - p[2]) * 29.9 + 14.95) / 1000.0 + 3.22) / 32.46199);
		H.calib[3] = (float)((((-p[3] * 29.9) + 14.95) / 1000.0) / 32.46199);
		double l = -p[4] * M_PI / 180.0, r = -p[5] * M_PI / 180.0;
		H.calib[4] = (float)cos(l); H.calib[5] = (float)sin(l);
		H.calib[6] = (float)cos(r); H.calib[7] = (float)sin(r);
	}
	hmd_set_brightness(H.analog_gain);
	if (H.camera_mode) { /* optional: keeps the tracking cameras streaming; frames are drained */
		uint32_t cmd[2] = { 1, (uint32_t)H.camera_mode };
		if (usb_claim(fd, IF_CAM, 0) == 0 && hmd_ctrl_set(RPT_CAMERA_MODE, 1, cmd, sizeof cmd)) { hmd_submit(URB_CAM0); hmd_submit(URB_CAM1); }
	}
	if (H.optical) { /* LED detection stream + factory calibration (GenData answers the camera-mode report) */
		if (usb_claim(fd, IF_LEDDET, 0) < 0 || usb_claim(fd, IF_GENDATA, 0) < 0) slog("hmd: cannot claim LED detection interfaces (%s)", strerror(errno));
		else {
			uint32_t cmd[2] = { 1, (uint32_t)H.camera_mode };
			H.gd_tries = 1; H.gd_retry_us = now_us();
			hmd_ctrl_set(RPT_CAMERA_MODE, 1, cmd, sizeof cmd);
			if (!hmd_start_leddet()) slog("hmd: LED detection start failed (%s)", strerror(errno));
			for (int i = URB_LD0; i <= URB_GD; i++) if (!hmd_submit(i)) slog("hmd: URB %d submit failed: %s", i, strerror(errno));
		}
	}
	/* optional user gaze calibration blob (captured from the official PC app) */
	char path[512];
	snprintf(path, sizeof path, "%s/.config/slopvr/eye_calibration.bin", getenv("HOME") ? getenv("HOME") : "");
	FILE *f = fopen(path, "rb");
	if (f) {
		static uint8_t blob[65536];
		size_t n = fread(blob, 1, sizeof blob, f);
		fclose(f);
		if (n) slog("hmd: eye calibration upload %s", usb_bulk_out(fd, EP_GAZE_OUT, blob, (int)n, 1000) >= 0 ? "ok" : "failed");
	}
	hmd_ctrl_set(RPT_GAZE_STREAM, 1, NULL, 0);
	H.gaze_keepalive_us = now_us();
	for (int i = URB_STATUS0; i <= URB_GAZE; i++)
		if (!hmd_submit(i)) slog("hmd: URB %d submit failed: %s", i, strerror(errno));
	atomic_store(&H.connected, true);
	atomic_store(&H.ready, true);
	slog("hmd: connected");
	return true;
}

static void hmd_close(void)
{
	if (H.fd < 0) return;
	hmd_ctrl_set(RPT_GAZE_STREAM, 2, NULL, 0);
	if (H.optical) hmd_ctrl_set(RPT_LEDDET, 2, NULL, 0);
	if (H.camera_mode) { uint32_t cmd[2] = { 1, 0 }; hmd_ctrl_set(RPT_CAMERA_MODE, 1, cmd, sizeof cmd); }
	for (int i = 0; i < N_URB; i++) ioctl(H.fd, USBDEVFS_DISCARDURB, &H.urb[i]);
	while (usb_reap(H.fd)) {}
	int ifs[] = { IF_STATUS, IF_SLAM, IF_GAZE, IF_CAM, IF_LEDDET, IF_GENDATA };
	for (unsigned i = 0; i < ARRAY_LEN(ifs); i++) ioctl(H.fd, USBDEVFS_RELEASEINTERFACE, &ifs[i]);
	close(H.fd);
	H.fd = -1;
	atomic_store(&H.connected, false);
	pthread_mutex_lock(&H.lock);
	H.imu_ok = H.slam_ok = H.slam_init = false;
	hmd_submit_pose_locked();
	pthread_mutex_unlock(&H.lock);
	slog("hmd: disconnected");
}

static void *hmd_thread(void *arg)
{
	(void)arg;
	while (atomic_load(&H.run)) {
		if (H.fd < 0 && !hmd_open()) { usleep(500000); continue; }
		struct pollfd pfd = { H.fd, POLLOUT, 0 };
		int r = poll(&pfd, 1, 100);
		if (r < 0 && errno != EINTR) { hmd_close(); continue; }
		bool dead = (pfd.revents & (POLLERR | POLLHUP)) != 0;
		struct usbdevfs_urb *u;
		while ((u = usb_reap(H.fd))) {
			int i = (int)(u - H.urb);
			if (u->status == -ENODEV || u->status == -ESHUTDOWN) { dead = true; continue; }
			if (u->status == 0 || u->status == -EOVERFLOW) {
				if (g_usb_hook) g_usb_hook(u->endpoint, u->buffer, u->actual_length);
				switch (u->endpoint) {
				case EP_STATUS: hmd_on_status(u->buffer, u->actual_length); break;
				case EP_SLAM: hmd_on_slam(u->buffer, u->actual_length); break;
				case EP_GAZE: hmd_on_gaze(u->buffer, u->actual_length); break;
				case EP_LEDDET: optical_on_ld(u->buffer, u->actual_length); break;
				case EP_GENDATA: hmd_on_gendata(u->buffer, u->actual_length); break;
				default: break; /* camera frames: drained, not decoded */
				}
			} else if (u->status != -ECONNRESET && u->status != -ENOENT) {
				static int logged[N_URB];
				if (!logged[i]++) slog("hmd: URB ep 0x%02x error %d (%s)", u->endpoint, u->status, strerror(-u->status));
			}
			if (!hmd_submit(i)) dead = true;
		}
		if (dead) { hmd_close(); continue; }
		uint64_t now = now_us();
		if (now - H.gaze_keepalive_us > 1000000) { /* stream stops without a 1 Hz keepalive */
			H.gaze_keepalive_us = now;
			if (!hmd_ctrl_set(RPT_GAZE_STREAM, 1, NULL, 0) && (errno == ENODEV || errno == ENOENT)) { hmd_close(); continue; }
		}
		if (H.optical) {
			if (!optical_calib_ok() && now - H.gd_retry_us > 1500000) {
				/* calibration not answered yet: re-ask.  The headset answers the camera-mode report only when
				 * the mode changes, so a mode left set by a dead instance needs a toggle through 0 first */
				uint32_t off[2] = { 1, 0 }, cmd[2] = { 1, (uint32_t)H.camera_mode };
				H.gd_tries++; H.gd_retry_us = now;
				if (H.gd_tries % 2 == 0) hmd_ctrl_set(RPT_CAMERA_MODE, 1, off, sizeof off);
				hmd_ctrl_set(RPT_CAMERA_MODE, 1, cmd, sizeof cmd);
				if (H.gd_tries % 5 == 0) slog("hmd: still waiting for the factory camera calibration (GenData), %d requests", H.gd_tries);
			}
			if (now - H.ld_keepalive_us > 2000000 && now - optical_last_ld_us() > 2000000) { /* no LD packets: restart the stream */
				static unsigned restarts;
				if (++restarts % 5 == 1) slog("hmd: no LED detection packets, restarting the stream (%u)", restarts);
				hmd_start_leddet();
			}
		}
	}
	hmd_close();
	return NULL;
}

/* ================= ITrackedDeviceServerDriver_005 ================= */
static int32_t hmd_Activate(void *self, uint32_t id)
{
	(void)self;
	H.id = id;
	uint64_t c = H.container = ovr_props_TrackedDeviceToPropertyContainer(g_props, id);
	prop_str(c, Prop_TrackingSystemName_String, "slopvr");
	prop_str(c, Prop_ModelNumber_String, "PlayStation VR2");
	prop_str(c, Prop_SerialNumber_String, H.serial);
	prop_str(c, Prop_ManufacturerName_String, "Sony");
	prop_str(c, Prop_RenderModelName_String, "generic_hmd");
	prop_str(c, Prop_DriverVersion_String, SLOP_VERSION);
	prop_str(c, Prop_RegisteredDeviceType_String, "sony/psvr2");
	prop_str(c, Prop_InputProfilePath_String, "{slopvr}/input/slopvr_hmd_profile.json");
	prop_u64(c, Prop_FirmwareVersion_Uint64, H.fw_version);
	prop_u64(c, Prop_CurrentUniverseId_Uint64, 2);
	prop_bool(c, Prop_WillDriftInYaw_Bool, false);
	prop_bool(c, Prop_DeviceIsWireless_Bool, false);
	prop_bool(c, Prop_DeviceProvidesBatteryStatus_Bool, false);
	prop_bool(c, Prop_ContainsProximitySensor_Bool, true);
	prop_bool(c, Prop_HasDisplayComponent_Bool, true);
	prop_bool(c, Prop_HasDriverDirectModeComponent_Bool, false);
	prop_bool(c, Prop_HasVirtualDisplayComponent_Bool, false);
	prop_bool(c, Prop_HasCameraComponent_Bool, false);
	prop_bool(c, Prop_HasCamera_Bool, false);
	prop_bool(c, Prop_IsOnDesktop_Bool, false); /* direct mode: compositor DRM-leases the panel */
	prop_bool(c, Prop_DisplayDebugMode_Bool, false);
	prop_bool(c, Prop_ReportsTimeSinceVSync_Bool, false);
	prop_bool(c, Prop_DisplayAllowNightMode_Bool, true);
	/* the compositor can't read the EDID through a Wayland DRM lease and otherwise assumes a 90 Hz panel */
	{ static const float rates[2] = { 90.f, 120.f }; prop_set(c, Prop_DisplayAvailableFrameRates_Float_Array, k_unFloatPropertyTag, rates, sizeof rates); }
	prop_bool(c, Prop_DisplaySupportsMultipleFramerates_Bool, true);
	prop_bool(c, Prop_DisplaySupportsRuntimeFramerateChange_Bool, false);
	prop_bool(c, Prop_DisplaySupportsAnalogGain_Bool, true); /* brightness via steamvr/analogGain */
	prop_f32(c, Prop_DisplayMinAnalogGain_Float, 0.f);
	prop_f32(c, Prop_DisplayMaxAnalogGain_Float, 1.f);
	prop_bool(c, Prop_Hmd_SupportsDisplayPortTrainingMode_Bool, true);
	prop_bool(c, Prop_Hmd_SupportsHDCP14LegacyCompat_Bool, false);
	prop_bool(c, Prop_Hmd_SupportsAppThrottling_Bool, true);
	prop_bool(c, Prop_Hmd_AllowSupersampleFiltering_Bool, true);
	prop_bool(c, Prop_DriverIsDrawingControllers_Bool, false);
	prop_bool(c, Prop_DriverProvidedChaperoneVisibility_Bool, true);
	prop_i32(c, Prop_EdidVendorID_Int32, 0x4DD9); /* "SNY" */
	prop_i32(c, Prop_EdidProductID_Int32, 0xA205);
	prop_i32(c, Prop_ExpectedControllerCount_Int32, 2);
	prop_i32(c, Prop_ExpectedTrackingReferenceCount_Int32, 0);
	prop_i32(c, Prop_DistortionMeshResolution_Int32, 64);
	prop_f32(c, Prop_DisplayFrequency_Float, H.display_hz);
	prop_f32(c, Prop_UserIpdMeters_Float, (H.ipd_mm >= 50 && H.ipd_mm <= 80 ? H.ipd_mm : 65) / 1000.f);
	prop_f32(c, Prop_MinimumIpdStepMeters_Float, 0.001f);
	prop_f32(c, Prop_UserHeadToEyeDepthMeters_Float, (float)T.eye_depth);
	/* rolling scanout top->bottom over 2040/2200 lines of the frame; mid-panel photons */
	prop_f32(c, Prop_SecondsFromVsyncToPhotons_Float, (float)(0.5 * (2040.0 / 2200.0) / H.display_hz));
	prop_str(c, Prop_NamedIconPathDeviceOff_String, "{indexhmd}/icons/headset_status_off.png");
	prop_str(c, Prop_NamedIconPathDeviceSearching_String, "{indexhmd}/icons/headset_status_searching.gif");
	prop_str(c, Prop_NamedIconPathDeviceSearchingAlert_String, "{indexhmd}/icons/headset_status_searching_alert.gif");
	prop_str(c, Prop_NamedIconPathDeviceReady_String, "{indexhmd}/icons/headset_status_ready.png");
	prop_str(c, Prop_NamedIconPathDeviceReadyAlert_String, "{indexhmd}/icons/headset_status_ready_alert.png");
	prop_str(c, Prop_NamedIconPathDeviceNotReady_String, "{indexhmd}/icons/headset_status_error.png");
	prop_str(c, Prop_NamedIconPathDeviceStandby_String, "{indexhmd}/icons/headset_status_standby.png");
	prop_str(c, Prop_NamedIconPathDeviceAlertLow_String, "{indexhmd}/icons/headset_status_ready_alert.png");

	ovr_input_CreateBooleanComponent(g_input, c, "/proximity", &H.h_prox);
	ovr_input_CreateBooleanComponent(g_input, c, "/input/system/click", &H.h_system);
	if (ovr_input_CreateEyeTrackingComponent(g_input, c, "/eyetracking", &H.h_gaze) != 0) H.h_gaze = 0;

	HmdRect2_t l = { { { -TAN_OUT, -TAN_V } }, { { TAN_IN, TAN_V } } };
	HmdRect2_t r = { { { -TAN_IN, -TAN_V } }, { { TAN_OUT, TAN_V } } };
	ovr_host_SetDisplayProjectionRaw(g_host, id, &l, &r);
	atomic_store(&H.active, true);
	return VRInitError_None;
}

static void hmd_Deactivate(void *self) { (void)self; atomic_store(&H.active, false); }
static void hmd_EnterStandby(void *self) { (void)self; }
static void *hmd_GetComponent(void *self, const char *name)
{
	(void)self;
	return strcmp(name, OVR_STR_IVRDisplayComponent) == 0 ? &H.display : NULL;
}
static void hmd_DebugRequest(void *self, const char *req, char *resp, uint32_t len)
{
	(void)self; (void)req;
	snprintf(resp, len, "{\"connected\":%d,\"slam\":%d,\"prox\":%d,\"ipd_mm\":%d,\"dprx\":%d,\"fw\":\"%08x\"}",
		 atomic_load(&H.connected), H.slam_ok, H.prox, H.ipd_mm, H.dprx, H.fw_version);
}
static void hmd_GetPose(DriverPose_t *out, void *self)
{
	(void)self;
	*out = pose_default();
	pthread_mutex_lock(&H.lock);
	out->qRotation = H.slam_ok ? H.slam_q : H.imu_q;
	memcpy(out->vecPosition, H.slam_p, sizeof out->vecPosition);
	out->poseIsValid = H.imu_ok;
	out->deviceIsConnected = atomic_load(&H.connected);
	out->result = H.slam_ok ? TrackingResult_Running_OK : TrackingResult_Running_OutOfRange;
	pthread_mutex_unlock(&H.lock);
}

OVR_VTABLE(hmd_vt, OVR_FN(ovr_dev_Activate, hmd_Activate), OVR_FN(ovr_dev_Deactivate, hmd_Deactivate), OVR_FN(ovr_dev_EnterStandby, hmd_EnterStandby),
	   OVR_FN(ovr_dev_GetComponent, hmd_GetComponent), OVR_FN(ovr_dev_DebugRequest, hmd_DebugRequest), OVR_FN(ovr_dev_GetPose, hmd_GetPose));

/* ================= IVRDisplayComponent_003 ================= */
static void disp_GetWindowBounds(void *self, int32_t *x, int32_t *y, uint32_t *w, uint32_t *h)
{ (void)self; *x = 0; *y = 0; *w = PANEL_W; *h = PANEL_H; }
static bool disp_IsDisplayOnDesktop(void *self) { (void)self; return false; }
static bool disp_IsDisplayRealDisplay(void *self) { (void)self; return true; }
static void disp_GetRecommendedRenderTargetSize(void *self, uint32_t *w, uint32_t *h) { (void)self; *w = H.render_w; *h = H.render_h; }
static void disp_GetEyeOutputViewport(void *self, int32_t eye, uint32_t *x, uint32_t *y, uint32_t *w, uint32_t *h)
{ (void)self; *x = eye == Eye_Left ? 0 : PANEL_W / 2; *y = 0; *w = PANEL_W / 2; *h = PANEL_H; }
static void disp_GetProjectionRaw(void *self, int32_t eye, float *l, float *r, float *t, float *b)
{
	(void)self;
	*l = eye == Eye_Left ? -TAN_OUT : -TAN_IN;
	*r = eye == Eye_Left ? TAN_IN : TAN_OUT;
	*t = -TAN_V; *b = TAN_V;
}
/* Display UV (per eye, 0..1, origin top-left) -> render-target UV per colour channel. */
static void disp_ComputeDistortion(DistortionCoordinates_t *out, void *self, int32_t eye, float u, float v)
{
	(void)self;
	const float *c = H.calib;
	float k1, k2, k3, k4, cx, cy = 0.5f, fx = 0.3585564f, fy = 0.3762281f;
	if (eye == Eye_Left) { k1 = c[0]; k2 = c[1]; k3 = c[4]; k4 = c[5]; cx = 0.6603788f; }
	else { k1 = c[2]; k2 = c[3]; k3 = c[6]; k4 = c[7]; cx = 0.3396213f; }
	float lx = k1 + (2.f * u - 1.f) * 0.9803922f * 0.9394987f;
	float ly = k2 + (2.f * v - 1.f) * 0.9394987f;
	float R = (lx * lx + ly * ly) * 1023.f;
	int i = (int)R;
	float s[3];
	if (i >= 0 && i + 1 < 1024) {
		float f = R - (float)i;
		for (int ch = 0; ch < 3; ch++) s[ch] = f * slop_lut[i + 1][ch] + (1.f - f) * slop_lut[i][ch];
	} else {
		float d = R - 1023.f;
		s[0] = d * 0.003648639f + 1.592586f; s[1] = d * 0.004862547f + 1.692979f; s[2] = d * 0.007425308f + 1.841771f;
	}
	float *o[3] = { out->rfRed, out->rfGreen, out->rfBlue };
	for (int ch = 0; ch < 3; ch++) {
		float y = ch == 1 ? ly - 0.0002302693f : ly;
		float a = lx * s[ch], b = y * s[ch];
		o[ch][0] = (k3 * a - k4 * b) * fx + cx;
		o[ch][1] = (k4 * a + k3 * b) * fy + cy;
	}
}
static bool disp_ComputeInverseDistortion(void *self, HmdVector2_t *out, int32_t eye, uint32_t ch, float u, float v)
{ (void)self; (void)out; (void)eye; (void)ch; (void)u; (void)v; return false; }

OVR_VTABLE(disp_vt, OVR_FN(ovr_disp_GetWindowBounds, disp_GetWindowBounds), OVR_FN(ovr_disp_IsDisplayOnDesktop, disp_IsDisplayOnDesktop),
	   OVR_FN(ovr_disp_IsDisplayRealDisplay, disp_IsDisplayRealDisplay), OVR_FN(ovr_disp_GetRecommendedRenderTargetSize, disp_GetRecommendedRenderTargetSize),
	   OVR_FN(ovr_disp_GetEyeOutputViewport, disp_GetEyeOutputViewport), OVR_FN(ovr_disp_GetProjectionRaw, disp_GetProjectionRaw),
	   OVR_FN(ovr_disp_ComputeDistortion, disp_ComputeDistortion), OVR_FN(ovr_disp_ComputeInverseDistortion, disp_ComputeInverseDistortion));

/* ---- provider hooks ---- */
static void hmd_init(void)
{
	H.vt = hmd_vt;
	H.display.vt = disp_vt;
	H.fd = -1;
	H.imu_q = QID;
	H.slam_q = QID;
	H.ipd_mm = 65;
	pthread_mutex_init(&H.lock, NULL);
	H.display_hz = cfg_f32("displayFrequency", 120.f);
	/* vrcompositor picks the DRM mode closest to steamvr.preferredRefreshRate (Valve default 90) */
	{ int32_t e = 0; ovr_settings_SetFloat(g_settings, "steamvr", "preferredRefreshRate", H.display_hz, &e); }
	H.render_w = (uint32_t)cfg_i32("renderWidth", 2400);
	H.render_h = (uint32_t)cfg_i32("renderHeight", 2448);
	H.camera_mode = SLOP_DBG ? cfg_i32("cameraMode", 0) : 0; /* camera frames: the test tool's view; the driver only needs the LED detector */
	H.optical = cfg_bool("opticalTracking", true);
	H.ld_gain = (int)T.led_gain; H.ld_exposure = (int)T.led_exp_us; /* blob detector analog gain / exposure (tunables) */
	{ int32_t e = 0; float g = ovr_settings_GetFloat(g_settings, "steamvr", "analogGain", &e); H.analog_gain = e ? 1.f : CLAMP(g, 0.f, 1.f); }
	snprintf(H.serial, sizeof H.serial, "PSVR2");
	H.calib[0] = -0.09919293f; H.calib[2] = 0.09919293f; H.calib[4] = H.calib[6] = 1.f;
	/* fakeHmd: expose the display/tracking surface without hardware (debugging, ABI tests) */
	H.fake = SLOP_DBG && (cfg_bool("fakeHmd", false) || getenv("SLOPVR_FAKE_HMD"));
	if (H.fake) { snprintf(H.serial, sizeof H.serial, "PSVR2-FAKE"); atomic_store(&H.ready, true); return; }
	if (!hmd_open()) slog("hmd: PSVR2 (054c:0cde) not found — waiting for hotplug");
	atomic_store(&H.run, true);
	pthread_create(&H.thread, NULL, hmd_thread, NULL);
}

/* main thread: register the device once the USB side has it (serial known) */
static void hmd_frame(void)
{
	if (!H.added && atomic_load(&H.ready))
		H.added = ovr_host_TrackedDeviceAdded(g_host, H.serial, TrackedDeviceClass_HMD, &H);
	if (H.fake && atomic_load(&H.active)) {
		DriverPose_t p = pose_default();
		p.poseIsValid = p.deviceIsConnected = true;
		p.result = TrackingResult_Running_OK;
		pose_submit(H.id, &p);
	}
}

static void hmd_settings_changed(void)
{
	int32_t e = 0;
	float g = ovr_settings_GetFloat(g_settings, "steamvr", "analogGain", &e);
	if (e || fabsf(g - H.analog_gain) < 1e-3f) return;
	H.analog_gain = CLAMP(g, 0.f, 1.f);
	hmd_set_brightness(H.analog_gain);
}

static void hmd_cleanup(void)
{
	if (atomic_exchange(&H.run, false)) pthread_join(H.thread, NULL);
	pthread_mutex_destroy(&H.lock);
	/* the URB buffers stay allocated (arena) and are reused by the next open */
}
