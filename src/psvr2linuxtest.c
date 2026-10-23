/* psvr2linuxtest — camera view + 6DoF controller tracking debugger for slopvr.
 *
 * Runs the real driver (slopvr.c, unity build) against the headset and the Sense controllers with a
 * fake vrserver host, taps the raw USB streams (tracking-camera frames, "LD" blob packets) and the
 * optical worker's per-frame fits, and draws everything in an SDL window:
 *   4 grey camera views (mode 4: 2x 508x508 binned fisheye pairs, alternating top/bottom cameras),
 *   the headset's blob detections, the LEDs the fit explained, the projected LED model of each fitted
 *   controller, the IMU-predicted constellation, the pose the driver would publish to SteamVR,
 *   LED flash sync state, clocks, rates, per-frame history charts, live tunables (same table as the
 *   driver), and the driver log.
 * Nothing in the tracking math is reimplemented here: what you see is what driver_slopvr.so does.
 *
 * Build/run:  ./build.sh test && build/psvr2linuxtest      (dev shell; or nix run .#test)
 *   - quit SteamVR first: the headset interfaces and the controllers' hidraw nodes can have one owner
 *   - needs the same udev rule as the driver (70-slopvr.rules) and the Sense controllers paired
 *   - settings come from resources/settings/default.vrsettings + ~/.local/share/Steam/config/steamvr.vrsettings
 *     (driver_slopvr section), i.e. exactly what the driver sees; -s key=value overrides
 *   - offline: ./psvr2linuxtest --replay DIR --calib item10.bin   (captured frame_*.bin / ld_*.bin, e.g. from 's')
 * Keys: h (help overlay lists every symbol and key). */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#define SDL_MAIN_HANDLED
#include <SDL2/SDL.h>
#include <signal.h>
#include <sys/stat.h>
#include "slopvr.c"
#include "font.h"

/* ---- layout (fixed canvas, scaled to the window) ---- */
#define CW 1900
#define CH 1070
#define Q_W 508              /* quadrant image size (1:1 with the binned 508x508 tracking frames) */
#define Q_X0 6
#define Q_Y0 20
#define Q_PX 516
#define Q_PY 530
#define PANEL_X (Q_X0 + 2 * Q_PX + 6)
#define PNL_W (CW - PANEL_X - 6)
#define ROW FONT_H

#define NVIEW 4
#define VIEW_MAX 640
#define LD_RING 16
#define MAX_LDBLOB 128
#define HIST_N 840
#define RAW_MAX 1040640
#define MAX_IBLOB 64
#define LOG_N 14
#define REPLAY_MAX_FILES 4096 /* replay of a packet dump directory: files listed into the arena */

#define C_BG 0xff0e1116
#define C_PANEL 0xff161b22
#define C_TEXT 0xffd4d4d4
#define C_DIM 0xff6e7781
#define C_L 0xff3fc1ff
#define C_R 0xffff5fc8
#define C_LD 0xff3fff6a
#define C_IMG 0xffffe13f
#define C_WARN 0xffff9040
#define C_OK 0xff5fff5f
#define C_BAD 0xffff5050
#define C_GRID 0xff2a313a
#define C_SEL 0xff2d4a6a
static const uint32_t C_HAND[2] = { C_L, C_R };

/* ================= fake vrserver host ================= */
struct kv { char sec[32], key[48], val[96]; int type; }; /* 0 number, 1 bool, 2 string */
static struct fake {
	pthread_mutex_t lock;
	struct fdev { bool used; char serial[64]; int cls; DriverPose_t pose; uint64_t pose_us; unsigned acc, rate; } dev[4];
	int ndev;
	struct comp { char name[48]; uint64_t container; int kind; bool b; float f; } comp[64]; int ncomp; /* kind 0 bool 1 scalar 2 haptic 3 eye */
	VREyeTrackingData_t gaze; uint64_t gaze_us;
	char log[LOG_N][100]; unsigned log_n;
	struct kv kv[256]; int nkv;
} F;
static uint64_t app_t0;

static struct kv *kv_find(const char *sec, const char *key)
{
	for (int i = 0; i < F.nkv; i++) if (!strcmp(F.kv[i].sec, sec) && !strcmp(F.kv[i].key, key)) return &F.kv[i];
	return NULL;
}
static void kv_set(const char *sec, const char *key, const char *val, int type)
{
	struct kv *v = kv_find(sec, key);
	if (!v) { if (F.nkv >= (int)ARRAY_LEN(F.kv)) return; v = &F.kv[F.nkv++]; snprintf(v->sec, sizeof v->sec, "%s", sec); snprintf(v->key, sizeof v->key, "%s", key); }
	snprintf(v->val, sizeof v->val, "%s", val); v->type = type;
}
/* value from the command line / env: "true"/"false" are bools, numbers are numbers, the rest strings */
static void kv_set_auto(const char *sec, const char *key, const char *val)
{
	char *e; strtod(val, &e);
	kv_set(sec, key, val, !strcmp(val, "true") || !strcmp(val, "false") ? 1 : e != val && !*e ? 0 : 2);
}

/* tolerant JSON: { "section": { "key": value, ... }, ... }; scalars two levels down land in the store */
struct jp { const char *s; };
static void js_ws(struct jp *p) { while (*p->s && strchr(" \t\r\n", *p->s)) p->s++; }
static bool js_str(struct jp *p, char *out, size_t n)
{
	if (*p->s != '"') return false;
	p->s++; size_t i = 0;
	while (*p->s && *p->s != '"') {
		char c = *p->s++;
		if (c == '\\' && *p->s) { c = *p->s++; if (c == 'n') c = '\n'; }
		if (i + 1 < n) out[i++] = c;
	}
	if (*p->s == '"') p->s++;
	out[i < n ? i : n - 1] = 0;
	return true;
}
static bool js_value(struct jp *p, const char *sec, const char *key)
{
	js_ws(p);
	char c = *p->s;
	if (c == '{') {
		p->s++;
		for (;;) {
			js_ws(p);
			if (*p->s == '}') { p->s++; return true; }
			if (*p->s == ',') { p->s++; continue; }
			char k[48];
			if (!js_str(p, k, sizeof k)) return false;
			js_ws(p);
			if (*p->s != ':') return false;
			p->s++;
			if (!js_value(p, sec ? sec : k, sec ? k : NULL)) return false;
		}
	}
	if (c == '[') {
		p->s++;
		for (;;) {
			js_ws(p);
			if (*p->s == ']') { p->s++; return true; }
			if (*p->s == ',') { p->s++; continue; }
			if (!js_value(p, NULL, NULL)) return false;
		}
	}
	char val[96]; int type;
	if (c == '"') { if (!js_str(p, val, sizeof val)) return false; type = 2; }
	else if (!strncmp(p->s, "true", 4)) { strcpy(val, "true"); type = 1; p->s += 4; }
	else if (!strncmp(p->s, "false", 5)) { strcpy(val, "false"); type = 1; p->s += 5; }
	else if (!strncmp(p->s, "null", 4)) { p->s += 4; return true; }
	else { char *e; double d = strtod(p->s, &e); if (e == p->s) return false; snprintf(val, sizeof val, "%.17g", d); type = 0; p->s = e; }
	if (sec && key) kv_set(sec, key, val, type);
	return true;
}
static bool json_load(const char *path)
{
	FILE *f = fopen(path, "rb");
	if (!f) return false;
	fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
	struct arena_temp sc = mem_scratch();
	char *b = sc.a ? arena_push_zero(sc.a, (size_t)n + 1) : NULL;
	if (!b || fread(b, 1, (size_t)n, f) != (size_t)n) { fclose(f); mem_scratch_end(sc); return false; }
	fclose(f);
	struct jp p = { b };
	bool ok = js_value(&p, NULL, NULL);
	mem_scratch_end(sc);
	return ok;
}

static void fk_log(void *self, const char *msg)
{
	(void)self;
	int n = (int)strcspn(msg, "\n");
	fprintf(stderr, "[%8.3f] %.*s\n", (now_us() - app_t0) * 1e-6, n, msg);
	pthread_mutex_lock(&F.lock);
	snprintf(F.log[F.log_n % LOG_N], sizeof F.log[0], "%.*s", n, msg); F.log_n++;
	pthread_mutex_unlock(&F.lock);
}
static bool fk_added(void *self, const char *serial, int32_t cls, void *drv)
{
	(void)self;
	pthread_mutex_lock(&F.lock);
	int id = F.ndev < 4 ? F.ndev++ : -1;
	if (id >= 0) { F.dev[id].used = true; F.dev[id].cls = cls; snprintf(F.dev[id].serial, sizeof F.dev[id].serial, "%s", serial); }
	pthread_mutex_unlock(&F.lock);
	if (id < 0) return false;
	char m[160]; snprintf(m, sizeof m, "host: device %d added (%s, class %d), activating", id, serial, cls); fk_log(NULL, m);
	int32_t r = ((ovr_dev_Activate)((ovr_obj *)drv)->vt[0])(drv, (uint32_t)id); /* what vrserver does next */
	if (r) { snprintf(m, sizeof m, "host: Activate(%d) failed: %d", id, r); fk_log(NULL, m); }
	return true;
}
static void fk_pose(void *self, uint32_t id, const DriverPose_t *p, uint32_t size)
{
	(void)self; (void)size;
	if (id >= 4) return;
	pthread_mutex_lock(&F.lock);
	F.dev[id].pose = *p; F.dev[id].pose_us = now_us(); F.dev[id].acc++;
	pthread_mutex_unlock(&F.lock);
}
static bool fk_exiting(void *self) { (void)self; return false; }
static bool fk_poll(void *self, VREvent_t *e, uint32_t size) { (void)self; (void)e; (void)size; return false; }
static void fk_proj(void *self, uint32_t id, const HmdRect2_t *l, const HmdRect2_t *r) { (void)self; (void)id; (void)l; (void)r; }
static int32_t fk_props_write(void *self, uint64_t c, PropertyWrite_t *b, uint32_t n) { (void)self; (void)c; (void)b; (void)n; return 0; }
static uint64_t fk_props_container(void *self, uint32_t id) { (void)self; return id + 1; }
static void fk_set_i32(void *self, const char *s, const char *k, int32_t v, int32_t *err) { (void)self; char b[32]; snprintf(b, sizeof b, "%d", v); kv_set(s, k, b, 0); if (err) *err = 0; }
static void fk_set_f32(void *self, const char *s, const char *k, float v, int32_t *err) { (void)self; char b[32]; snprintf(b, sizeof b, "%.9g", v); kv_set(s, k, b, 0); if (err) *err = 0; }
static bool fk_get_bool(void *self, const char *s, const char *k, int32_t *err)
{
	(void)self; const struct kv *v = kv_find(s, k);
	if (!v || v->type == 2) { if (err) *err = 4; return false; }
	if (err) *err = 0;
	return v->type == 1 ? !strcmp(v->val, "true") : atof(v->val) != 0;
}
static int32_t fk_get_i32(void *self, const char *s, const char *k, int32_t *err)
{
	(void)self; const struct kv *v = kv_find(s, k);
	if (!v || v->type != 0) { if (err) *err = 4; return 0; }
	if (err) *err = 0;
	return (int32_t)lround(atof(v->val));
}
static float fk_get_f32(void *self, const char *s, const char *k, int32_t *err)
{
	(void)self; const struct kv *v = kv_find(s, k);
	if (!v || v->type != 0) { if (err) *err = 4; return 0; }
	if (err) *err = 0;
	return (float)atof(v->val);
}
static int32_t fk_comp_new(uint64_t c, const char *n, uint64_t *h, int kind)
{
	pthread_mutex_lock(&F.lock);
	int i = F.ncomp < (int)ARRAY_LEN(F.comp) ? F.ncomp++ : 0;
	snprintf(F.comp[i].name, sizeof F.comp[i].name, "%s", n); F.comp[i].container = c; F.comp[i].kind = kind;
	pthread_mutex_unlock(&F.lock);
	*h = (uint64_t)i + 1;
	return 0;
}
static int32_t fk_in_cbool(void *self, uint64_t c, const char *n, uint64_t *h) { (void)self; return fk_comp_new(c, n, h, 0); }
static int32_t fk_in_cscalar(void *self, uint64_t c, const char *n, uint64_t *h, int32_t type, int32_t units) { (void)self; (void)type; (void)units; return fk_comp_new(c, n, h, 1); }
static int32_t fk_in_chaptic(void *self, uint64_t c, const char *n, uint64_t *h) { (void)self; return fk_comp_new(c, n, h, 2); }
static int32_t fk_in_ceye(void *self, uint64_t c, const char *n, uint64_t *h) { (void)self; return fk_comp_new(c, n, h, 3); }
static int32_t fk_in_ubool(void *self, uint64_t h, bool v, double t) { (void)self; (void)t; if (h && h <= ARRAY_LEN(F.comp)) F.comp[h - 1].b = v; return 0; }
static int32_t fk_in_uscalar(void *self, uint64_t h, float v, double t) { (void)self; (void)t; if (h && h <= ARRAY_LEN(F.comp)) F.comp[h - 1].f = v; return 0; }
static int32_t fk_in_ueye(void *self, uint64_t h, const VREyeTrackingData_t *d, double t) { (void)self; (void)h; (void)t; F.gaze = *d; F.gaze_us = now_us(); return 0; }
static void fk_nop(void) {}

static ovr_fn vt_host[16], vt_props[8], vt_settings[12], vt_input[12], vt_log[2], vt_ctx[2];
static ovr_obj o_host = { vt_host }, o_props = { vt_props }, o_settings = { vt_settings }, o_input = { vt_input }, o_log = { vt_log }, o_ctx = { vt_ctx };
static void *fk_ctx_get(void *self, const char *v, int32_t *err)
{
	(void)self;
	if (err) *err = 0;
	if (!strcmp(v, OVR_STR_IVRServerDriverHost)) return &o_host;
	if (!strcmp(v, OVR_STR_IVRProperties)) return &o_props;
	if (!strcmp(v, OVR_STR_IVRSettings)) return &o_settings;
	if (!strcmp(v, OVR_STR_IVRDriverInput)) return &o_input;
	if (!strcmp(v, OVR_STR_IVRDriverLog)) return &o_log;
	if (err) *err = VRInitError_Init_InterfaceNotFound;
	return NULL;
}
static uint64_t fk_ctx_handle(void *self) { (void)self; return 1; }
static void fake_init(void)
{
	pthread_mutex_init(&F.lock, NULL);
	ovr_fn *all[] = { vt_host, vt_props, vt_settings, vt_input, vt_log, vt_ctx };
	size_t len[] = { 16, 8, 12, 12, 2, 2 };
	for (unsigned i = 0; i < ARRAY_LEN(all); i++) for (size_t k = 0; k < len[i]; k++) all[i][k] = fk_nop;
	vt_host[0] = (ovr_fn)fk_added; vt_host[1] = (ovr_fn)fk_pose; vt_host[4] = (ovr_fn)fk_exiting; vt_host[5] = (ovr_fn)fk_poll; vt_host[10] = (ovr_fn)fk_proj;
	vt_props[1] = (ovr_fn)fk_props_write; vt_props[3] = (ovr_fn)fk_props_container;
	vt_settings[2] = (ovr_fn)fk_set_i32; vt_settings[3] = (ovr_fn)fk_set_f32; vt_settings[5] = (ovr_fn)fk_get_bool; vt_settings[6] = (ovr_fn)fk_get_i32; vt_settings[7] = (ovr_fn)fk_get_f32;
	vt_input[0] = (ovr_fn)fk_in_cbool; vt_input[1] = (ovr_fn)fk_in_ubool; vt_input[2] = (ovr_fn)fk_in_cscalar; vt_input[3] = (ovr_fn)fk_in_uscalar;
	vt_input[4] = (ovr_fn)fk_in_chaptic; vt_input[9] = (ovr_fn)fk_in_ceye; vt_input[10] = (ovr_fn)fk_in_ueye;
	vt_log[0] = (ovr_fn)fk_log;
	vt_ctx[0] = (ovr_fn)fk_ctx_get; vt_ctx[1] = (ovr_fn)fk_ctx_handle;
}

/* ================= taps: camera frames, LD packets, optical results ================= */
struct ldblob { uint8_t cam, idx, peak; float x, y; uint16_t xmin, xmax, ymin, ymax; uint32_t m00; };
struct ldpkt { uint32_t ts, frame; uint64_t host_us; int n, ncam[4]; float gain, exp_us; struct ldblob b[4 * MAX_LDBLOB]; };
struct view { int w, h, vw; float ld_scale; uint32_t ts; uint64_t host_us; unsigned seq; int type; char name[40]; uint8_t px[VIEW_MAX * VIEW_MAX]; };
struct optsnap {
	bool valid; struct opt_frame_dbg f; struct opt_pose P[2]; bool ok[2], acq[2]; uint64_t host_us; struct opt_evid ev[2]; int par;
	struct opt_ray rays[OPT_MAX_RAY]; int nray; struct opt_cand cand[OPT_MAX_CAND]; int ncand; struct opt_cl cl[OPT_MAX_CL]; int ncl;
};
struct hist { uint64_t t; uint8_t n[2], ok[2], acq[2], nray; int8_t par; uint16_t ms; float res[2], pos[2][3]; };

#define RAW_RING 4 /* camera packets come in pairs (two frame types back to back): keep a few so the render loop sees both */
static struct app {
	pthread_mutex_t lock;
	uint8_t raw[RAW_RING][RAW_MAX]; int raw_len[RAW_RING]; uint64_t raw_us[RAW_RING]; unsigned raw_w;
	uint8_t ldraw[OPT_LD_SIZE]; int ldraw_len;
	struct ldpkt ld[LD_RING]; unsigned ld_n;
	struct optsnap opt;
	struct hist hist[HIST_N]; unsigned hist_n;
	unsigned acc[6], rate[6]; /* 0 status 1 slam 2 gaze 3 cam 4 ld 5 gendata */
	int cam_lens[6], ncam_lens;
} A;

static int ep_slot(uint8_t ep)
{
	switch (ep) { case EP_STATUS: return 0; case EP_SLAM: return 1; case EP_GAZE: return 2; case EP_CAM: return 3; case EP_LEDDET: return 4; case EP_GENDATA: return 5; default: return -1; }
}
static void app_on_ld(const uint8_t *b, int len, uint64_t now)
{
	if (len < 0x40 + 4 || b[0] != 'L' || b[1] != 'D') return;
	pthread_mutex_lock(&A.lock);
	struct ldpkt *p = &A.ld[A.ld_n % LD_RING];
	memset(p, 0, offsetof(struct ldpkt, b));
	memcpy(&p->ts, b + 8, 4); memcpy(&p->frame, b + 20, 4); memcpy(&p->gain, b + 32, 4); memcpy(&p->exp_us, b + 36, 4);
	p->host_us = now;
	for (int cam = 0; cam < 4; cam++) {
		const uint8_t *blk = b + 0x40 + cam * OPT_LD_CAM_BLOCK;
		if (blk + 4 > b + len) break;
		uint32_t n; memcpy(&n, blk, 4);
		if (n > MAX_LDBLOB) n = MAX_LDBLOB;
		for (uint32_t i = 0; i < n && p->n < (int)ARRAY_LEN(p->b); i++) {
			const uint8_t *r = blk + 4 + i * OPT_LD_REC;
			if (r + OPT_LD_REC > b + len) break;
			uint16_t valid; uint32_t m00, m10, m01;
			memcpy(&valid, r, 2); memcpy(&m00, r + 12, 4); memcpy(&m10, r + 16, 4); memcpy(&m01, r + 20, 4);
			if (!valid || !m00) continue;
			struct ldblob *o = &p->b[p->n++];
			o->cam = (uint8_t)cam; o->idx = (uint8_t)i; o->peak = r[2];
			memcpy(&o->xmin, r + 4, 2); memcpy(&o->xmax, r + 6, 2); memcpy(&o->ymin, r + 8, 2); memcpy(&o->ymax, r + 10, 2);
			o->m00 = m00; o->x = o->xmin + (float)m10 / m00; o->y = o->ymin + (float)m01 / m00;
			p->ncam[cam]++;
		}
	}
	A.ld_n++;
	A.ldraw_len = len < OPT_LD_SIZE ? len : OPT_LD_SIZE; memcpy(A.ldraw, b, (size_t)A.ldraw_len);
	pthread_mutex_unlock(&A.lock);
}
static void app_usb_hook(uint8_t ep, const uint8_t *b, int len)
{
	uint64_t now = now_us();
	int s = ep_slot(ep);
	if (s >= 0) __atomic_fetch_add(&A.acc[s], 1, __ATOMIC_RELAXED);
	if (ep == EP_CAM && len >= 256 && len <= RAW_MAX) {
		pthread_mutex_lock(&A.lock);
		unsigned w = A.raw_w % RAW_RING;
		memcpy(A.raw[w], b, (size_t)len); A.raw_len[w] = len; A.raw_us[w] = now; A.raw_w++;
		int i; for (i = 0; i < A.ncam_lens && A.cam_lens[i] != len; i++) {}
		if (i == A.ncam_lens && i < (int)ARRAY_LEN(A.cam_lens)) A.cam_lens[A.ncam_lens++] = len;
		pthread_mutex_unlock(&A.lock);
	} else if (ep == EP_LEDDET) app_on_ld(b, len, now);
}
static void calflow_on_frame(const struct opt_frame_dbg *f); /* calflow.c: guided calibration metrics */
static void calflow_follow(const char *text, uint64_t now);      /* calflow.c: replay follows the recorded step markers */
static void app_opt_hook(const struct opt_frame_dbg *f)
{
	uint64_t now = now_us();
	calflow_on_frame(f);
	pthread_mutex_lock(&A.lock);
	struct optsnap *s = &A.opt;
	s->valid = true; s->f = *f; s->host_us = now; s->par = -1;
	for (unsigned i = A.ld_n > 8 ? A.ld_n - 8 : 0; i < A.ld_n; i++) if (A.ld[i % LD_RING].ts == f->ts) s->par = (int)(A.ld[i % LD_RING].frame & 1);
	for (int h = 0; h < 2; h++) { s->P[h] = f->P[h]; s->ok[h] = f->ok[h]; s->acq[h] = f->acq[h]; s->ev[h] = f->ok[h] ? opt_evidence(&f->P[h], O.mirror[h]) : (struct opt_evid){ 0, 0, 0 }; }
	s->f.P = s->P; s->f.ok = s->ok; s->f.acq = s->acq;
	s->nray = O.nray; memcpy(s->rays, O.rays, (size_t)O.nray * sizeof *O.rays);
	s->ncand = O.ncand; memcpy(s->cand, O.cand, (size_t)O.ncand * sizeof *O.cand);
	s->ncl = O.ncl; memcpy(s->cl, O.cl, (size_t)O.ncl * sizeof *O.cl);
	struct hist *hh = &A.hist[A.hist_n++ % HIST_N];
	memset(hh, 0, sizeof *hh);
	hh->t = f->now; hh->nray = (uint8_t)(O.nray > 255 ? 255 : O.nray); hh->ms = (uint16_t)((now - f->now) / 1000); hh->par = (int8_t)s->par;
	for (int h = 0; h < 2; h++) {
		hh->ok[h] = f->ok[h]; hh->acq[h] = f->acq[h]; hh->n[h] = f->ok[h] ? (uint8_t)f->P[h].n : 0; hh->res[h] = f->ok[h] ? (float)f->P[h].res : 0;
		for (int k = 0; k < 3; k++) hh->pos[h][k] = (float)f->P[h].t[k];
	}
	pthread_mutex_unlock(&A.lock);
}

/* ================= software canvas ================= */
static uint32_t *cv;
static int clip_x0, clip_y0, clip_x1 = CW, clip_y1 = CH;
static void clip(int x0, int y0, int x1, int y1) { clip_x0 = x0 < 0 ? 0 : x0; clip_y0 = y0 < 0 ? 0 : y0; clip_x1 = x1 > CW ? CW : x1; clip_y1 = y1 > CH ? CH : y1; }
static inline void px(int x, int y, uint32_t c) { if (x >= clip_x0 && x < clip_x1 && y >= clip_y0 && y < clip_y1) cv[y * CW + x] = c; }
static void fill_rect(int x, int y, int w, int h, uint32_t c)
{
	if (x < 0) { w += x; x = 0; }
	if (y < 0) { h += y; y = 0; }
	if (x < clip_x0) { w -= clip_x0 - x; x = clip_x0; }
	if (y < clip_y0) { h -= clip_y0 - y; y = clip_y0; }
	if (x + w > clip_x1) w = clip_x1 - x;
	if (y + h > clip_y1) h = clip_y1 - y;
	for (int j = 0; j < h; j++) for (int i = 0; i < w; i++) cv[(y + j) * CW + x + i] = c;
}
static void rect(int x, int y, int w, int h, uint32_t c)
{
	for (int i = 0; i < w; i++) { px(x + i, y, c); px(x + i, y + h - 1, c); }
	for (int j = 0; j < h; j++) { px(x, y + j, c); px(x + w - 1, y + j, c); }
}
static void line(int x0, int y0, int x1, int y1, uint32_t c)
{
	int dx = abs(x1 - x0), dy = -abs(y1 - y0), sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1, e = dx + dy;
	for (int n = 0; n < 4096; n++) {
		px(x0, y0, c);
		if (x0 == x1 && y0 == y1) break;
		int e2 = 2 * e;
		if (e2 >= dy) { e += dy; x0 += sx; }
		if (e2 <= dx) { e += dx; y0 += sy; }
	}
}
static void circle(int cx, int cy, int r, uint32_t c)
{
	int x = r, y = 0, err = 1 - r;
	while (x >= y) {
		px(cx + x, cy + y, c); px(cx - x, cy + y, c); px(cx + x, cy - y, c); px(cx - x, cy - y, c);
		px(cx + y, cy + x, c); px(cx - y, cy + x, c); px(cx + y, cy - x, c); px(cx - y, cy - x, c);
		y++;
		if (err < 0) err += 2 * y + 1; else { x--; err += 2 * (y - x) + 1; }
	}
}
static void cross(int x, int y, int r, uint32_t c) { line(x - r, y - r, x + r, y + r, c); line(x - r, y + r, x + r, y - r, c); }
static void plus(int x, int y, int r, uint32_t c) { line(x - r, y, x + r, y, c); line(x, y - r, x, y + r, c); }
static void diamond(int x, int y, int r, uint32_t c) { line(x - r, y, x, y - r, c); line(x, y - r, x + r, y, c); line(x + r, y, x, y + r, c); line(x, y + r, x - r, y, c); }
static int glyphs(int x, int y, uint32_t c, const char *s)
{
	for (; *s; s++, x += FONT_W) {
		unsigned ch = (unsigned char)*s;
		if (ch < 32 || ch > 126) ch = '?';
		const unsigned char *g = font8x14[ch - 32];
		for (int r = 0; r < FONT_H; r++) for (int b = 0; b < FONT_W; b++) if (g[r] & (0x80 >> b)) px(x + b, y + r, c);
	}
	return x;
}
static int txt(int x, int y, uint32_t c, const char *fmt, ...)
{
	char b[256]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
	return glyphs(x, y, c, b);
}
static uint32_t dim(uint32_t c) { return 0xff000000 | ((c >> 1) & 0x7f7f7f); }

/* ================= frame decoding (render thread) ================= */
static struct view V[NVIEW];
static void view_put(int slot, const uint8_t *src, int stride, int w, int h, int vw, float ld_scale, uint32_t ts, uint64_t host_us, int type, const char *name)
{
	if (slot < 0 || slot >= NVIEW || w > VIEW_MAX || h > VIEW_MAX) return;
	struct view *v = &V[slot];
	for (int y = 0; y < h; y++) memcpy(v->px + (size_t)y * w, src + (size_t)y * stride, (size_t)w);
	v->w = w; v->h = h; v->vw = vw; v->ld_scale = ld_scale; v->ts = ts; v->host_us = host_us; v->seq++; v->type = type;
	snprintf(v->name, sizeof v->name, "%s", name);
}
/* BC4 (one 8-bit channel, 4x4 blocks of 8 bytes) -> 8-bit rows */
static void bc4_decode(const uint8_t *blk, int bw, int bh, uint8_t *out, int stride)
{
	for (int by = 0; by < bh; by++) for (int bx = 0; bx < bw; bx++, blk += 8) {
		int a0 = blk[0], a1 = blk[1], pal[8] = { a0, a1 };
		if (a0 > a1) for (int i = 1; i < 7; i++) pal[i + 1] = ((7 - i) * a0 + i * a1) / 7;
		else { for (int i = 1; i < 5; i++) pal[i + 1] = ((5 - i) * a0 + i * a1) / 5; pal[6] = 0; pal[7] = 255; }
		uint64_t idx = 0; for (int i = 0; i < 6; i++) idx |= (uint64_t)blk[2 + i] << (8 * i);
		for (int t = 0; t < 16; t++) out[(by * 4 + t / 4) * stride + bx * 4 + t % 4] = (uint8_t)pal[(idx >> (3 * t)) & 7];
	}
}
static void decode_raw(const uint8_t *b, int len, uint64_t host_us)
{
	uint32_t ts = 0; if (len >= 12) memcpy(&ts, b + 8, 4);
	switch (len) {
	case 520448: { /* mode 4: 2x 508x508 (stride 512) binned fisheye, type 4 = one camera pair, type 5 = the other */
		int type = b[16], base = type == 5 ? 2 : 0; char nm[40]; snprintf(nm, sizeof nm, "VI type %d", type);
		for (int i = 0; i < 2; i++) view_put(base + i, b + 256 + (size_t)i * 512 * 508, 512, 512, 508, 508, 1.f, ts, host_us, type, nm);
		break; }
	case 260352: /* mode 0xc: all four cameras 256x254 (stride 256, 254 valid), half the LD resolution */
		for (int i = 0; i < 4; i++) view_put(i, b + 256 + (size_t)i * 256 * 254, 256, 256, 254, 254, 0.5f, ts, host_us, b[16], "mode 0xc 4x 256x254");
		break;
	case 819456: /* mode 1: bottom cameras 640x640 side by side, crop (no LD overlay: unknown mapping) */
		for (int i = 0; i < 2; i++) view_put(i, b + 256 + (size_t)i * 640, 1280, 640, 640, 640, 0.f, ts, host_us, b[16], "mode 1 640x640 crop");
		break;
	case 1040640: { /* mode 0x10: two 1024x1016 BC4 images; shown 2x downscaled (508 binned px = LD space) */
		static uint8_t full[1024 * 1016], half[512 * 508];
		for (int i = 0; i < 2; i++) {
			bc4_decode(b + 256 + (size_t)i * 254 * 256 * 8, 256, 254, full, 1024);
			for (int y = 0; y < 508; y++) for (int x = 0; x < 512; x++) {
				const uint8_t *s = full + (size_t)(2 * y) * 1024 + 2 * x;
				half[y * 512 + x] = (uint8_t)((s[0] + s[1] + s[1024] + s[1025] + 2) >> 2);
			}
			view_put(i, half, 512, 512, 508, 508, 1.f, ts, host_us, b[16], "mode 0x10 BC4 1024x1016/2");
		}
		break; }
	default: break;
	}
}

/* ---- our own blob detector on the camera image: cross-check of the headset's LD detector ---- */
struct iblob { float x, y; int area, peak, xmin, xmax, ymin, ymax; };
static int detect_blobs(const struct view *v, int thr, struct iblob *out, int max)
{
	static uint8_t vis[VIEW_MAX * VIEW_MAX]; static int stack[1 << 16];
	int w = v->w, h = v->h, vw = v->vw < w ? v->vw : w, n = 0;
	memset(vis, 0, (size_t)w * h);
	for (int y = 0; y < h && n < max; y++) for (int x = 0; x < vw && n < max; x++) {
		int i = y * w + x;
		if (vis[i] || v->px[i] < thr) continue;
		int sp = 0; stack[sp++] = i; vis[i] = 1;
		double sx = 0, sy = 0, sw = 0; struct iblob b = { 0, 0, 0, 0, x, x, y, y };
		while (sp) {
			int j = stack[--sp], jx = j % w, jy = j / w, val = v->px[j];
			sx += (double)val * jx; sy += (double)val * jy; sw += val; b.area++;
			if (val > b.peak) b.peak = val;
			if (jx < b.xmin) b.xmin = jx;
			if (jx > b.xmax) b.xmax = jx;
			if (jy < b.ymin) b.ymin = jy;
			if (jy > b.ymax) b.ymax = jy;
			int nb[4] = { j - 1, j + 1, j - w, j + w };
			for (int k = 0; k < 4; k++) {
				int q = nb[k];
				if (q < 0 || q >= w * h || (k < 2 && q / w != jy) || q % w >= vw || vis[q] || v->px[q] < thr || sp >= (int)ARRAY_LEN(stack)) continue;
				vis[q] = 1; stack[sp++] = q;
			}
		}
		if (b.area < 2 || b.area > 4000) continue;
		b.x = (float)(sx / sw); b.y = (float)(sy / sw);
		out[n++] = b;
	}
	return n;
}

/* ================= UI state ================= */
static volatile sig_atomic_t g_quit;
static void on_signal(int s) { (void)s; g_quit = 1; }
static struct ui {
	float gain; bool show_img_blobs, show_imu, show_cl, show_labels, help, paused, auto_map, auto_gain; float vgain[NVIEW];
	int map[NVIEW];              /* view slot -> LD camera index */
	int votes[NVIEW][4];         /* auto mapping evidence: image blobs coincident with LD blobs */
	float off[NVIEW][2]; int noff[NVIEW]; /* mean LD - image blob offset (binned px) for the mapped camera */
	int sel;                     /* selected tunable */
	bool replay; char replay_dir[256]; int cam_mode; double autoshot_s, led_toggle_s; bool calib;
	int hmd_view; bool no_hmd; const char *hmd_conn, *hmd_mode; /* hmd_view: 1 see-through, 2 grid, 0 off */
	struct iblob ib[NVIEW][MAX_IBLOB]; int nib[NVIEW]; unsigned ib_seq[NVIEW];
	unsigned frames; uint64_t rate_t0; int fps, fps_acc;
	int shots;
} U = { .gain = 2.5f, .show_img_blobs = true, .show_imu = true, .show_labels = true, .auto_map = true, .auto_gain = true, .map = { 0, 1, 2, 3 }, .hmd_view = 1 };

static int age_ms(uint64_t now, uint64_t t) { return t && t <= now ? (int)((now - t) / 1000) : t > now ? 0 : -1; }
static const char *phase_name(int ph, bool freerun) { return ph == LED_PHASE_PRESCAN ? (freerun ? "FREE" : "sync") : ph == LED_PHASE_OFF ? "OFF" : ph == LED_PHASE_ON ? "ON" : ph == 0 ? "-" : "?"; }
static const char *cam_name(int c) { static const char *n[4] = { "front-left", "front-right", "lower-left", "lower-right" }; return (unsigned)c < 4 ? n[c] : "?"; }

/* LD / calibration space (binned 508 px) -> quadrant pixel */
struct qctx { int ox, oy; const struct view *v; float s; };
static int qx(const struct qctx *q, double x) { return q->ox + (int)lrint(x * q->s); }
static int qy(const struct qctx *q, double y) { return q->oy + (int)lrint(y * q->s); }
/* full-resolution calibration pixel -> binned LD pixel (inverse of opt_parse_ld's mapping) */
static void full2bin(double u, double v, double *x, double *y) { *x = (u - 0.5 - OPT_LD_XOFF) * 0.5; *y = (v - 0.5 - OPT_LD_YOFF) * 0.5; }
/* undistorted normalized coordinates -> distorted full-res pixel */
static void norm2full(const struct opt_cam *c, const double n[2], double *u, double *v)
{
	double r = hypot(n[0], n[1]), s = r > 1e-9 ? opt_distort_r(c, r) / r : 1.0;
	*u = n[0] * s * c->fx + c->cx; *v = n[1] * s * c->fy + c->cy;
}
/* cam0-frame point -> binned pixel of camera cam; false if behind or off the calibrated range */
static bool cam0_to_bin(int cam, const double X[3], double *x, double *y)
{
	const struct opt_cam *c = &O.cam[cam]; double n[2], u, v;
	if (!c->ok || !opt_project(c, X, n) || hypot(n[0], n[1]) > 3.0) return false;
	norm2full(c, n, &u, &v); full2bin(u, v, x, y);
	return *x > -50 && *x < 560 && *y > -50 && *y < 560;
}

static void resolve_map(void)
{
	if (!U.auto_map) return;
	bool us[NVIEW] = { 0 }, uc[4] = { 0 }; int map[NVIEW] = { -1, -1, -1, -1 };
	for (int k = 0; k < NVIEW; k++) {
		int bs = -1, bc = -1, bv = 0;
		for (int s = 0; s < NVIEW; s++) for (int c = 0; c < 4; c++) if (!us[s] && !uc[c] && U.votes[s][c] > bv) { bv = U.votes[s][c]; bs = s; bc = c; }
		if (bs < 0) break;
		map[bs] = bc; us[bs] = uc[bc] = true;
	}
	for (int s = 0; s < NVIEW; s++) if (map[s] < 0) for (int c = 0; c < 4; c++) if (!uc[c]) { map[s] = c; uc[c] = true; break; }
	memcpy(U.map, map, sizeof map);
}
/* vote with a freshly decoded view: LD blobs (of each camera, packet nearest in time) coincident with image blobs */
static void vote_map(int slot, const struct ldpkt *L)
{
	const struct view *v = &V[slot];
	if (!L || U.nib[slot] < 3 || v->ld_scale <= 0) return;
	float gate = 3.0f;
	for (int c = 0; c < 4; c++) {
		if (L->ncam[c] < 3) continue;
		int hits = 0; double ox = 0, oy = 0;
		for (int i = 0; i < L->n; i++) {
			if (L->b[i].cam != c) continue;
			float bx = L->b[i].x * v->ld_scale, by = L->b[i].y * v->ld_scale, bd = gate;
			int best = -1;
			for (int k = 0; k < U.nib[slot]; k++) { float d = hypotf(U.ib[slot][k].x - bx, U.ib[slot][k].y - by); if (d < bd) { bd = d; best = k; } }
			if (best >= 0) { hits++; ox += (bx - U.ib[slot][best].x) / v->ld_scale; oy += (by - U.ib[slot][best].y) / v->ld_scale; }
		}
		if (hits >= 3) {
			U.votes[slot][c] += hits;
			if (U.votes[slot][c] > 100000) for (int s = 0; s < NVIEW; s++) for (int cc = 0; cc < 4; cc++) U.votes[s][cc] /= 2;
			if (c == U.map[slot]) { float k = U.noff[slot] < 30 ? 1.f / (U.noff[slot] + 1) : 1.f / 30; U.off[slot][0] += k * ((float)(ox / hits) - U.off[slot][0]); U.off[slot][1] += k * ((float)(oy / hits) - U.off[slot][1]); U.noff[slot]++; }
		}
	}
	resolve_map();
}

#include "hmdout.c" /* DRM lease: lights the panel (wakes the headset) and renders the see-through / grid view into it */
#include "calflow.c" /* guided tracking calibration / test flow (Options button or F5) */

/* ================= driver controls ================= */
static bool cam_urbs; /* hmd_open submits the camera URBs only when it starts with a non-zero mode */
static void set_camera_mode(int mode)
{
	uint32_t cmd[2] = { 1, (uint32_t)mode };
	U.cam_mode = mode; H.camera_mode = mode;
	if (H.fd >= 0) {
		if (mode && !cam_urbs && usb_claim(H.fd, IF_CAM, 0) == 0) { hmd_submit(URB_CAM0); hmd_submit(URB_CAM1); cam_urbs = true; }
		hmd_ctrl_set(RPT_CAMERA_MODE, 1, cmd, sizeof cmd);
	}
	for (int i = 0; i < NVIEW; i++) { V[i].w = V[i].h = 0; V[i].seq++; }
	char m[64]; snprintf(m, sizeof m, "test: camera mode 0x%x", mode); fk_log(NULL, m);
}
static void restart_ld(void)
{
	H.ld_gain = (int)T.led_gain; H.ld_exposure = (int)T.led_exp_us;
	if (H.fd >= 0) hmd_start_leddet();
	char m[96]; snprintf(m, sizeof m, "test: LED detector restarted: gain %d exposure %d us", H.ld_gain, H.ld_exposure); fk_log(NULL, m);
}
static void cycle_led_mode(void)
{
	pthread_mutex_lock(&O.lock);
	O.led_mode = O.led_mode == 0 ? LED_PHASE_ON : O.led_mode == LED_PHASE_ON ? LED_PHASE_OFF : 0;
	for (int h = 0; h < 2; h++) O.led[h].dirty = true;
	int m = O.led_mode;
	pthread_mutex_unlock(&O.lock);
	char s[96]; snprintf(s, sizeof s, "test: LED mode %s", m == 0 ? "synced flashes" : m == LED_PHASE_ON ? "free-running flash (sweeps through the exposure ~1x/s)" : "OFF"); fk_log(NULL, s);
}
static void reset_led_sync(void)
{
	pthread_mutex_lock(&O.lock);
	for (int h = 0; h < 2; h++) { struct led_sync *L = &O.led[h]; L->lat_ok = false; L->scan = false; L->probe = 0; L->scan_done_us = 0; L->scan_wait_us = 0; L->dirty = true; O.lock_us[h] = 0; }
	pthread_mutex_unlock(&O.lock);
	fk_log(NULL, "test: LED sync latencies dropped, scans will restart");
}
static void tune_changed(const struct param *p)
{
	if (!strncmp(p->name, "cam_", 4) || !strncmp(p->name, "mirror_", 7) || !strcmp(p->name, "led_period")) optical_tune_apply();
	if (!strcmp(p->name, "led_gain") || !strcmp(p->name, "led_exp_us")) restart_ld();
}
static void tune_step(int dir, bool big)
{
	const struct param *p = &params[U.sel];
	double range = p->max - p->min, step = range >= 100 ? 1 : range >= 10 ? 0.1 : range >= 2 ? 0.01 : 0.001;
	if (p->def == floor(p->def) && p->min == floor(p->min) && p->max == floor(p->max) && range >= 5) step = 1; /* counts, ms, px */
	if (big) step *= 10;
	if (!strncmp(p->name, "mirror_", 7) || !strcmp(p->name, "qc_on")) step = 1;
	*p->v = CLAMP(*p->v + dir * step, p->min, p->max);
	tune_changed(p);
}
static void write_tune(void)
{
	char path[300]; snprintf(path, sizeof path, "%s/.config/slopvr/psvr2linuxtest_tune.json", getenv("HOME") ? getenv("HOME") : "/tmp");
	FILE *f = fopen(path, "w");
	if (!f) { fk_log(NULL, "test: cannot write tune file"); return; }
	fprintf(f, "{\n\t\"driver_slopvr\": {\n");
	int n = 0;
	for (unsigned i = 0; i < ARRAY_LEN(params); i++) {
		const struct param *p = &params[i];
		if (fabs(*p->v - p->def) < 1e-9) continue;
		fprintf(f, "%s\t\t\"%s\": %.6g", n++ ? ",\n" : "", p->cfg, *p->v);
	}
	fprintf(f, "\n\t}\n}\n"); fclose(f);
	char m[400]; snprintf(m, sizeof m, "test: %d non-default tunables written to %s (merge: jq -s '.[0] * .[1]' steamvr.vrsettings that-file)", n, path); fk_log(NULL, m);
}
static void save_snapshot(SDL_Renderer *r)
{
	pthread_mutex_lock(&O.lock); O.state_req = true; pthread_mutex_unlock(&O.lock); /* the whole tracker state goes to the log with the next LD frame */
	(void)r;
	char dir[300], path[400]; const char *home = getenv("HOME") ? getenv("HOME") : "/tmp";
	snprintf(dir, sizeof dir, "%s/.cache", home); mkdir(dir, 0755);
	snprintf(dir, sizeof dir, "%s/.cache/slopvr_test", home); mkdir(dir, 0755);
	for (;;) { snprintf(path, sizeof path, "%s/shot_%03d.bmp", dir, U.shots); if (access(path, F_OK)) break; U.shots++; } /* never overwrite */
	pthread_mutex_lock(&A.lock);
	for (unsigned k = A.raw_w > 2 ? A.raw_w - 2 : 0; k < A.raw_w; k++) { /* the last pair */
		unsigned w = k % RAW_RING;
		snprintf(path, sizeof path, "%s/frame_%03d_%u_%d.bin", dir, U.shots, k, A.raw_len[w]);
		FILE *f = fopen(path, "wb"); if (f) { fwrite(A.raw[w], 1, (size_t)A.raw_len[w], f); fclose(f); }
	}
	snprintf(path, sizeof path, "%s/ld_%03d_%d.bin", dir, U.shots, A.ldraw_len);
	FILE *f = fopen(path, "wb"); if (f) { fwrite(A.ldraw, 1, (size_t)A.ldraw_len, f); fclose(f); }
	pthread_mutex_unlock(&A.lock);
	SDL_Surface *s = SDL_CreateRGBSurfaceWithFormatFrom(cv, CW, CH, 32, CW * 4, SDL_PIXELFORMAT_ARGB8888);
	snprintf(path, sizeof path, "%s/shot_%03d.bmp", dir, U.shots);
	if (s) { SDL_SaveBMP(s, path); SDL_FreeSurface(s); }
	if (g_ov && CF.state != CF_IDLE) { /* the headset HUD overlay bitmap */
		SDL_Surface *p3 = SDL_CreateRGBSurfaceWithFormatFrom(g_ov, OV_W, OV_H, 32, OV_W * 4, SDL_PIXELFORMAT_ARGB8888);
		snprintf(path, sizeof path, "%s/hud_%03d.bmp", dir, U.shots);
		if (p3) { SDL_SaveBMP(p3, path); SDL_FreeSurface(p3); }
	}
	if (HO.active && HO.map[HO.back ^ 1]) { /* what the headset panel shows (the last presented buffer), 1/4 size */
		int w = HO.w / 4, h = HO.h / 4; struct arena_temp sc = mem_scratch(); uint32_t *small = sc.a ? arena_push(sc.a, (size_t)w * h * 4) : NULL;
		if (small) {
			for (int y = 0; y < h; y++) for (int x = 0; x < w; x++) small[y * w + x] = *(const uint32_t *)(HO.map[HO.back ^ 1] + (size_t)(4 * y) * HO.pitch + (size_t)(4 * x) * 4) | 0xff000000u;
			SDL_Surface *p2 = SDL_CreateRGBSurfaceWithFormatFrom(small, w, h, 32, w * 4, SDL_PIXELFORMAT_ARGB8888);
			snprintf(path, sizeof path, "%s/panel_%03d.bmp", dir, U.shots);
			if (p2) { SDL_SaveBMP(p2, path); SDL_FreeSurface(p2); }
		}
		mem_scratch_end(sc);
	}
	char m[400]; snprintf(m, sizeof m, "test: saved frame/LD/screenshot %03d to %s", U.shots, dir); fk_log(NULL, m);
	U.shots++;
}

/* ================= replay (offline: captured camera frames + LD packets through the same taps) ================= */
static struct replay { char **cam, **ld; int ncam, nld, icam, ild; uint64_t next_us;
	/* session file (.slop, see rec.c): records are replayed in real time, mapped onto this process's clock */
	FILE *slop; uint64_t first_us, base_us; bool have_first; uint8_t ld_pending[OPT_LD_SIZE]; bool ld_have; unsigned nrec, nld_rec, nimu_rec;
	struct { bool ok, acq; int n; float res; float pw[3]; uint32_t frame; } recfit[2]; /* the fits the recording session got (comparison) */
} RP;
static bool replay_is_slop(const char *path) { size_t l = strlen(path); struct stat st; return l > 5 && !strcmp(path + l - 5, ".slop") && !stat(path, &st) && S_ISREG(st.st_mode); }
static bool replay_open_slop(const char *path)
{
	RP.slop = fopen(path, "rb"); if (!RP.slop) return false;
	char magic[8]; if (fread(magic, 1, 8, RP.slop) != 8 || memcmp(magic, "SLOPREC\1", 8)) { fclose(RP.slop); RP.slop = NULL; return false; }
	return true;
}
/* one record of the session file -> the same entry points the live USB/Bluetooth data takes */
static void replay_slop_record(uint8_t type, uint8_t sub, uint64_t host_us, const uint8_t *d, int len, uint64_t now)
{
	uint64_t t = host_us - RP.first_us + RP.base_us; /* the recording's clock mapped onto ours */
	switch (type) {
	case REC_CALIB: opt_set_calib(d, len); break;
	case REC_TUNE: { char line[300]; snprintf(line, sizeof line, "rec: tunables of the recording: %.*s", len > 250 ? 250 : len, (const char *)d); fk_log(NULL, line); break; }
	case REC_LD: { /* rebuild the detector packet */
		if (len < 16) break;
		uint8_t *b = RP.ld_pending; memset(b, 0, OPT_LD_SIZE); b[0] = 'L'; b[1] = 'D';
		memcpy(b + 8, d, 4); memcpy(b + 20, d + 4, 4); memcpy(b + 32, d + 8, 4); memcpy(b + 36, d + 12, 4);
		const uint8_t *p = d + 16, *end = d + len;
		for (int cam = 0; cam < 4 && p < end; cam++) {
			uint32_t n = *p++; uint8_t *blk = b + 0x40 + cam * 0x2404; if (n > 128) n = 128;
			memcpy(blk, &n, 4);
			for (uint32_t i = 0; i < n && p + 21 <= end; i++) {
				uint8_t *r = blk + 4 + i * 36; uint16_t one = 1;
				memcpy(r, &one, 2); memcpy(r + 4, p, 8); r[2] = p[8]; memcpy(r + 12, p + 9, 12); p += 21;
			}
		}
		RP.ld_have = true; RP.nld_rec++;
		break; }
	case REC_HEAD: { /* follows its LD record: head pose into the SLAM ring, then the packet goes in */
		if (len < 4 + 28 + 1) break;
		uint32_t ts; float q[4], pos[3]; memcpy(&ts, d, 4); memcpy(q, d + 4, 16); memcpy(pos, d + 20, 12); bool valid = d[32];
		uint64_t texp = 0; if (len >= 33 + 8) { memcpy(&texp, d + 33, 8); texp = texp - RP.first_us + RP.base_us; } /* recorded exposure time, else the arrival time less the usual 6 ms */
		if (!texp || texp > t) texp = t - 6000;
		pthread_mutex_lock(&H.lock);
		H.slam_ok = H.slam_init = valid; H.slam_host_us = now;
		H.sring[H.sring_n % SLAM_RING].t = ts; H.sring[H.sring_n % SLAM_RING].q = (quat){ q[0], q[1], q[2], q[3] };
		for (int k = 0; k < 3; k++) H.sring[H.sring_n % SLAM_RING].p[k] = pos[k];
		H.sring_n++;
		pthread_mutex_unlock(&H.lock);
		if (RP.ld_have) { RP.ld_have = false; app_usb_hook(EP_LEDDET, RP.ld_pending, OPT_LD_SIZE); optical_on_ld_at(RP.ld_pending, OPT_LD_SIZE, t, (int64_t)texp); } /* the recording's own timing, not the feed time */
		break; }
	case REC_IMU: { /* the fused orientation of the recording feeds the tracker, as sense.c would */
		if (len < 4 + 12 + 32 + 1 || sub > 1) break;
		float q[4], qc[4]; memcpy(q, d + 16, 16); memcpy(qc, d + 32, 16);
		uint8_t battery = d[49];
		pthread_mutex_lock(&O.lock); O.led[sub].in_us = t; O.led[sub].sample_host_us = t; pthread_mutex_unlock(&O.lock);
		optical_imu_update(sub, (quat){ q[0], q[1], q[2], q[3] });
		S[sub].battery = battery; S[sub].q = (quat){ q[0], q[1], q[2], q[3] }; S[sub].q_ok = true; S[sub].last_in_us = t;
		RP.nimu_rec++;
		break; }
	case REC_FIT: {
		if (len < 4 + 3 + 4 + 36 + 12 + 12 || sub > 1) break;
		memcpy(&RP.recfit[sub].frame, d, 4); RP.recfit[sub].ok = d[4]; RP.recfit[sub].acq = d[5]; RP.recfit[sub].n = d[6]; memcpy(&RP.recfit[sub].res, d + 7, 4); memcpy(RP.recfit[sub].pw, d + 59, 12);
		break; }
	case REC_TEXT: { char line[600]; snprintf(line, sizeof line, "rec: %.*s", len > 580 ? 580 : len, (const char *)d); fputs(line, stderr); fputc('\n', stderr); calflow_follow(line + 5, now_us()); break; }
	default: break; /* LEDCMD, CLK, PUB, SENSECAL: informational */
	}
	(void)t;
}
static void replay_slop_tick(uint64_t now)
{
	static uint8_t buf[70000];
	for (int guard = 0; guard < 4000; guard++) { /* all records due by now */
		long pos = ftell(RP.slop);
		uint8_t h[12]; if (fread(h, 1, 12, RP.slop) != 12) { fseek(RP.slop, 8, SEEK_SET); RP.have_first = false; fk_log(NULL, "replay: end of the session file, looping"); return; }
		uint16_t len; uint64_t host; memcpy(&len, h + 2, 2); memcpy(&host, h + 4, 8);
		if (!RP.have_first) { RP.first_us = host; RP.base_us = now; RP.have_first = true; }
		if (host - RP.first_us + RP.base_us > now) { fseek(RP.slop, pos, SEEK_SET); return; } /* not yet */
		if (len && fread(buf, 1, len, RP.slop) != len) return;
		RP.nrec++;
		replay_slop_record(h[0], h[1], host, buf, len, now);
	}
}
static int cmp_str(const void *a, const void *b) { return strcmp(*(char *const *)a, *(char *const *)b); }
static uint32_t ld_file_frame(const char *path) { uint8_t h[24]; FILE *f = fopen(path, "rb"); if (!f) return 0; size_t n = fread(h, 1, sizeof h, f); fclose(f); uint32_t c = 0; if (n == sizeof h) memcpy(&c, h + 20, 4); return c; }
static int cmp_ld_frame(const void *a, const void *b) { uint32_t fa = ld_file_frame(*(char *const *)a), fb = ld_file_frame(*(char *const *)b); return fa < fb ? -1 : fa > fb; }
static bool replay_scan(const char *dir)
{
	DIR *d = opendir(dir); struct dirent *e;
	if (!d) return false;
	while ((e = readdir(d))) {
		char p[600]; struct stat st;
		snprintf(p, sizeof p, "%s/%s", dir, e->d_name);
		if (stat(p, &st) || !S_ISREG(st.st_mode)) continue;
		char ***list = NULL; int *n = NULL;
		if (st.st_size == 520448 || st.st_size == 260352 || st.st_size == 819456 || st.st_size == 1040640) { list = &RP.cam; n = &RP.ncam; }
		else if (st.st_size == OPT_LD_SIZE || st.st_size == OPT_LD_SIZE + 136) { list = &RP.ld; n = &RP.nld; }
		if (!list) continue;
		if (!*list) *list = mem_alloc(sizeof **list * REPLAY_MAX_FILES);
		if (*n < REPLAY_MAX_FILES) (*list)[(*n)++] = mem_strdup(p);
	}
	closedir(d);
	if (RP.ncam) qsort(RP.cam, (size_t)RP.ncam, sizeof *RP.cam, cmp_str);
	if (RP.nld) qsort(RP.ld, (size_t)RP.nld, sizeof *RP.ld, cmp_ld_frame); /* by the headset frame counter: the dump is a ring, file names wrap */
	return RP.ncam || RP.nld;
}
static int read_file(const char *path, uint8_t *buf, int max)
{
	FILE *f = fopen(path, "rb"); if (!f) return -1;
	int n = (int)fread(buf, 1, (size_t)max, f); fclose(f); return n;
}
static void replay_tick(uint64_t now)
{
	static uint8_t buf[RAW_MAX];
	if (RP.slop) { replay_slop_tick(now); return; }
	if (!RP.next_us || now > RP.next_us + 500000) RP.next_us = now;
	if (now < RP.next_us) return;
	RP.next_us += 16683;
	if (RP.ncam) { int n = read_file(RP.cam[RP.icam++ % RP.ncam], buf, sizeof buf); if (n > 0) app_usb_hook(EP_CAM, buf, n); }
	if (RP.nld) {
		int n = read_file(RP.ld[RP.ild++ % RP.nld], buf, sizeof buf);
		if (n >= OPT_LD_SIZE) {
			uint32_t ts; memcpy(&ts, buf + 8, 4);
			/* the driver's dump trailer (opticalDump): head pose and controller IMU orientations at the frame */
			struct { char magic[4]; quat hq; double hp[3]; quat iq[2]; int iok[2]; } tr = { "SLO", QID, { 0, 0, 0 }, { QID, QID }, { 0, 0 } };
			if (n >= OPT_LD_SIZE + (int)sizeof tr && !memcmp(buf + OPT_LD_SIZE, "SLOP", 4)) memcpy(&tr, buf + OPT_LD_SIZE, sizeof tr);
			pthread_mutex_lock(&H.lock); /* the recorded (else a frozen) head pose at the frame time so the worker can fit */
			H.slam_ok = H.slam_init = true; H.slam_host_us = now;
			H.sring[H.sring_n % SLAM_RING].t = ts; H.sring[H.sring_n % SLAM_RING].q = tr.hq; memcpy(H.sring[H.sring_n % SLAM_RING].p, tr.hp, sizeof(double[3])); H.sring_n++;
			pthread_mutex_unlock(&H.lock);
			pthread_mutex_lock(&O.lock); for (int h = 0; h < 2; h++) { O.led[h].in_us = now; O.led[h].sample_host_us = now; } pthread_mutex_unlock(&O.lock); /* "controllers connected" */
			for (int h = 0; h < 2; h++) if (tr.iok[h]) optical_imu_update(h, tr.iq[h]);
			app_usb_hook(EP_LEDDET, buf, OPT_LD_SIZE);
			optical_on_ld(buf, OPT_LD_SIZE);
		}
	}
}

/* ================= drawing ================= */
/* LD packet for a camera frame: nearest device time stamp if within 25 ms (same clock), else the newest packet */
static const struct ldpkt *ld_nearest(const struct ldpkt *ring, unsigned n, uint32_t ts, int32_t *dts)
{
	const struct ldpkt *best = NULL; int32_t bd = INT32_MAX;
	*dts = 0;
	if (!n) return NULL;
	for (unsigned i = n > LD_RING ? n - LD_RING : 0; i < n; i++) {
		const struct ldpkt *p = &ring[i % LD_RING]; int32_t d = (int32_t)(p->ts - ts);
		if (abs(d) < bd) { bd = abs(d); best = p; *dts = d; }
	}
	if (bd > 25000) { best = &ring[(n - 1) % LD_RING]; *dts = (int32_t)(best->ts - ts); }
	return best;
}
static bool imu_ok(int h) { return S[h].fd >= 0 && S[h].q_ok; }
static void draw_view(int quadrant, int slot, const struct ldpkt *ld, unsigned ld_n, const struct optsnap *snap, const double R_cw[9], const double p_wc[3], bool head_ok, const DriverPose_t pub[2], const bool pub_ok[2], uint64_t now)
{
	int ox = Q_X0 + (quadrant % 2) * Q_PX, oy = Q_Y0 + (quadrant / 2) * Q_PY;
	int cam = slot >= 0 ? U.map[slot] : quadrant;
	const struct view *v = slot >= 0 ? &V[slot] : NULL;
	clip(ox - 2, oy - 18, ox + Q_W + 2, oy + Q_W + 2);
	fill_rect(ox, oy, Q_W, Q_W, 0xff000000);
	struct qctx q = { ox, oy, v, 1.f }; /* without camera frames the overlays are still drawn in the 508 px LD space */
	bool have_img = v && v->w > 0;
	if (have_img) {
		static uint8_t lut[256];
		float gain = U.gain;
		if (U.auto_gain && slot >= 0) { /* stretch the 99.5th percentile to 230 (LEDs are < 0.5 % of the pixels) */
			unsigned hst[256] = { 0 }, tot = 0; int vw = v->vw < v->w ? v->vw : v->w;
			for (int yy = 0; yy < v->h; yy += 2) for (int xx = 0; xx < vw; xx += 2) { hst[v->px[yy * v->w + xx]]++; tot++; }
			unsigned acc = 0; int p = 255; for (int i = 0; i < 256; i++) { acc += hst[i]; if (acc >= tot * 0.995) { p = i; break; } }
			float g = p > 4 ? 230.f / (float)p : 16.f; if (g > 16) g = 16; if (g < 1) g = 1;
			U.vgain[slot] = U.vgain[slot] > 0 ? U.vgain[slot] + 0.1f * (g - U.vgain[slot]) : g;
			gain = U.vgain[slot];
		}
		for (int i = 0; i < 256; i++) { float g = (float)i * gain; lut[i] = (uint8_t)(g > 255 ? 255 : g); }
		for (int dy = 0; dy < Q_W; dy++) {
			const uint8_t *row = v->px + (size_t)(dy * v->h / Q_W) * v->w; uint32_t *out = cv + (oy + dy) * CW + ox;
			for (int dx = 0; dx < Q_W; dx++) { uint8_t g = lut[row[dx * v->w / Q_W]]; out[dx] = 0xff000000 | (uint32_t)g << 16 | (uint32_t)g << 8 | g; }
		}
		q.s = v->ld_scale > 0 ? v->ld_scale * (float)Q_W / (float)v->w : 0;
	}
	/* title */
	int tx = txt(ox, oy - 16, C_TEXT, "cam%d %s", cam, cam_name(cam));
	int32_t dts = 0; const struct ldpkt *L = have_img ? ld_nearest(ld, ld_n, v->ts, &dts) : ld_n ? &ld[(ld_n - 1) % LD_RING] : NULL;
	if (have_img) {
		tx = txt(tx + 8, oy - 16, C_DIM, "t%d/s%d %dx%d age %3dms", v->type, slot, v->vw, v->h, age_ms(now, v->host_us));
		if (L) tx = txt(tx + 8, oy - 16, abs(dts) < 25000 ? C_DIM : C_WARN, "LDdt %+dus", dts);
		if (U.auto_map) txt(tx + 8, oy - 16, U.votes[slot][cam] >= 20 ? C_OK : C_WARN, U.votes[slot][cam] >= 20 ? "map ok" : "map?");
	} else txt(tx + 8, oy - 16, C_WARN, "no camera frame (LD overlay only)");
	if (have_img && q.s <= 0) { txt(ox + 4, oy + 4, C_WARN, "no LD overlay for this camera mode"); }
	/* principal point and image centre */
	if (O.cam[cam].ok && q.s > 0) { double x, y; full2bin(O.cam[cam].cx, O.cam[cam].cy, &x, &y); plus(qx(&q, x), qy(&q, y), 6, dim(C_DIM)); }
	/* our own image blobs */
	if (have_img && U.show_img_blobs && slot >= 0) {
		float is = (float)Q_W / (float)v->w;
		for (int i = 0; i < U.nib[slot]; i++) {
			const struct iblob *b = &U.ib[slot][i];
			int x = ox + (int)lrintf(b->x * is), y = oy + (int)lrintf(b->y * is), r = (int)lrintf(((b->xmax - b->xmin) + 2) * is / 2) + 2;
			rect(x - r, y - r, 2 * r + 1, 2 * r + 1, dim(C_IMG));
		}
	}
	if (q.s <= 0) { clip(0, 0, CW, CH); return; }
	/* headset blob detections (LD packet nearest in time to this frame) */
	if (L) for (int i = 0; i < L->n; i++) {
		const struct ldblob *b = &L->b[i];
		if (b->cam != cam) continue;
		int x = qx(&q, b->x), y = qy(&q, b->y), r = (int)lrintf(((b->xmax - b->xmin) + 3) * q.s / 2) + 3;
		circle(x, y, r, C_LD);
		if (U.show_labels) txt(x + r + 2, y - 6, dim(C_LD), "%d", b->idx);
	}
	/* fit results: blobs explained by each hand (rings), projected LED model (crosses), model axes */
	if (snap->valid) {
		for (int h = 0; h < 2; h++) {
			if (!snap->ok[h]) continue;
			const struct opt_pose *P = &snap->P[h]; uint32_t col = C_HAND[h];
			for (int i = 0; i < P->nobs; i++) {
				const struct opt_ray *ray = &snap->rays[P->obs[i].ray];
				if (ray->cam != cam) continue;
				double u, vv, x, y; norm2full(&O.cam[cam], ray->n, &u, &vv); full2bin(u, vv, &x, &y);
				circle(qx(&q, x), qy(&q, y), 7, col);
				if (U.show_labels) txt(qx(&q, x) - 24, qy(&q, y) - 6, col, "%d", P->obs[i].led);
			}
			double Q[OPT_NLED][3], N[OPT_NLED][3];
			opt_model(P->R, O.mirror[h], Q, N);
			for (int i = 0; i < OPT_NLED; i++) {
				double X[3] = { Q[i][0] + P->t[0], Q[i][1] + P->t[1], Q[i][2] + P->t[2] }, x, y;
				if (!cam0_to_bin(cam, X, &x, &y)) continue;
				bool facing = opt_facing(N[i], X, O.cam[cam].o);
				cross(qx(&q, x), qy(&q, y), facing ? 4 : 2, facing ? col : dim(col));
			}
			double x0, y0;
			if (cam0_to_bin(cam, P->t, &x0, &y0)) {
				static const double ax[3][3] = { { 0.04, 0, 0 }, { 0, 0.04, 0 }, { 0, 0, 0.04 } }; static const uint32_t ac[3] = { 0xffff4040, 0xff40ff40, 0xff4080ff };
				for (int a = 0; a < 3; a++) {
					double d[3], X[3], x, y; m3_mulv(P->R, ax[a], d);
					for (int k = 0; k < 3; k++) X[k] = P->t[k] + d[k];
					if (cam0_to_bin(cam, X, &x, &y)) line(qx(&q, x0), qy(&q, y0), qx(&q, x), qy(&q, y), ac[a]);
				}
				if (U.show_labels) txt(qx(&q, x0) + 6, qy(&q, y0) + 6, col, "%c n=%d %.1fpx %s  ev %d/%d/%d", h ? 'R' : 'L', P->n, P->res, snap->acq[h] ? "ACQ" : "lock", snap->ev[h].n, snap->ev[h].missing, snap->ev[h].stray);
			}
		}
		/* stereo clusters (triangulated LED candidates) */
		if (U.show_cl) for (int i = 0; i < snap->ncl; i++) {
			double x, y;
			if (!cam0_to_bin(cam, snap->cl[i].x, &x, &y)) continue;
			uint32_t c = snap->cl[i].used ? dim(C_DIM) : snap->cl[i].n >= 3 ? 0xffffffff : 0xff9090a0;
			rect(qx(&q, x) - 2, qy(&q, y) - 2, 5, 5, c);
			if (U.show_labels && !snap->cl[i].used) txt(qx(&q, x) + 4, qy(&q, y) - 14, c, "%d", snap->cl[i].n);
		}
	}
	/* the pose the driver publishes (diamond) and the IMU-predicted constellation at that position (dots) */
	if (head_ok) for (int h = 0; h < 2; h++) {
		if (!pub_ok[h] || !pub[h].poseIsValid) continue;
		double d[3], pc[3], x, y;
		for (int k = 0; k < 3; k++) d[k] = pub[h].vecPosition[k] - p_wc[k];
		m3_mulv(R_cw, d, pc);
		if (cam0_to_bin(cam, pc, &x, &y)) {
			uint32_t col = pub[h].result == TrackingResult_Running_OK ? C_HAND[h] : dim(C_HAND[h]);
			diamond(qx(&q, x), qy(&q, y), 9, col);
			if (U.show_imu && imu_ok(h)) {
				double Rw[9], Rc[9], Q[OPT_NLED][3], N[OPT_NLED][3];
				m3_from_q(sense_world_q(&S[h]), Rw); m3_mul(R_cw, Rw, Rc);
				opt_model(Rc, O.mirror[h], Q, N);
				for (int i = 0; i < OPT_NLED; i++) {
					double X[3] = { Q[i][0] + pc[0], Q[i][1] + pc[1], Q[i][2] + pc[2] }, lx, ly;
					if (!cam0_to_bin(cam, X, &lx, &ly) || !opt_facing(N[i], X, O.cam[cam].o)) continue;
					fill_rect(qx(&q, lx) - 1, qy(&q, ly) - 1, 3, 3, dim(C_HAND[h]));
				}
			}
		}
	}
	rect(ox - 1, oy - 1, Q_W + 2, Q_W + 2, C_GRID);
	clip(0, 0, CW, CH);
}

/* ---- history charts: one pixel per LD frame, oldest left ---- */
static void chart_frame(int x, int y, int w, int h, const char *title)
{
	fill_rect(x, y, w, h, 0xff0a0d12); rect(x - 1, y - 1, w + 2, h + 2, C_GRID);
	txt(x + 2, y + 1, C_DIM, "%s", title);
}
static int draw_charts(int x, int y, const struct hist *hist, int nh)
{
	int w = HIST_N, cx0 = x + 2;
	/* LEDs fitted per frame, per hand */
	for (int h = 0; h < 2; h++) {
		int ch = 40;
		char t[96]; snprintf(t, sizeof t, "%c LEDs fitted per LD frame (17 max; orange = acquisition, red tick = no fit)", h ? 'R' : 'L');
		chart_frame(cx0, y, w, ch, t);
		for (int i = 0; i < nh; i++) {
			int px0 = cx0 + w - nh + i;
			if (!hist[i].ok[h]) { if (T.led_mux == 0 || hist[i].par < 0 || hist[i].par == h) { px(px0, y + ch - 1, C_BAD); px(px0, y + ch - 2, C_BAD); } continue; }
			int bh = 1 + hist[i].n[h] * (ch - 15) / 17;
			for (int k = 0; k < bh; k++) px(px0, y + ch - 1 - k, hist[i].acq[h] ? C_WARN : C_HAND[h]);
		}
		y += ch + 6;
	}
	/* cam0-frame position traces (x red, y green, z blue), both hands, centred on the latest fix */
	{
		int ch = 90; chart_frame(cx0, y, w, ch, "position in cam0 frame, last fix centred, box = +-0.25 m (x red, y green, z blue; R dotted)");
		for (int h = 0; h < 2; h++) {
			float c[3] = { 0, 0, 0 }; bool have = false;
			for (int i = nh - 1; i >= 0; i--) if (hist[i].ok[h]) { memcpy(c, hist[i].pos[h], sizeof c); have = true; break; }
			if (!have) continue;
			static const uint32_t col[3] = { 0xffff5050, 0xff50ff50, 0xff5090ff };
			for (int i = 0; i < nh; i++) {
				if (!hist[i].ok[h] || (h == 1 && (i & 1))) continue;
				for (int k = 0; k < 3; k++) {
					float d = (hist[i].pos[h][k] - c[k]) / 0.25f; if (d < -1) d = -1; if (d > 1) d = 1;
					px(cx0 + w - nh + i, y + 14 + (int)((1 - d) * 0.5f * (ch - 16)), h ? dim(col[k]) : col[k]);
				}
			}
		}
		y += ch + 6;
	}
	/* blobs per frame and worker time */
	{
		int ch = 40; chart_frame(cx0, y, w, ch, "blobs per LD frame (grey, 64 max)   optical worker time per frame (orange dot, 30 ms max)");
		for (int i = 0; i < nh; i++) {
			int px0 = cx0 + w - nh + i, bh = hist[i].nray * (ch - 15) / 64, mh = (hist[i].ms > 30 ? 30 : hist[i].ms) * (ch - 15) / 30;
			for (int k = 0; k < bh; k++) px(px0, y + ch - 1 - k, 0xff5a6470);
			if (mh) px(px0, y + ch - 1 - mh, C_WARN);
		}
		y += ch + 6;
	}
	return y;
}

static void comp_summary(uint64_t container, char *out, size_t n)
{
	size_t len = 0; out[0] = 0;
	for (int i = 0; i < F.ncomp && len + 24 < n; i++) {
		const struct comp *c = &F.comp[i];
		if (c->container != container) continue;
		const char *nm = c->name; if (!strncmp(nm, "/input/", 7)) nm += 7;
		char s[48]; s[0] = 0;
		if (c->kind == 1 && fabsf(c->f) > 0.02f) snprintf(s, sizeof s, "%.*s=%.2f ", (int)strcspn(nm, "/"), nm, c->f);
		else if (c->kind == 0 && c->b) snprintf(s, sizeof s, "%.*s%s ", (int)strcspn(nm, "/"), nm, strstr(nm, "touch") ? "~" : "");
		len += (size_t)snprintf(out + len, n - len, "%s", s);
	}
}

static void draw_panel(const struct optsnap *snap, const struct hist *hist, int nh, const struct ldpkt *ldl, const struct ldpkt *ldring, unsigned ldn, uint64_t now, const DriverPose_t pub[2], const bool pub_ok[2])
{
	int x = PANEL_X, y = 4;
	fill_rect(PANEL_X - 3, 0, PNL_W + 6, CH, C_PANEL);
	clip(PANEL_X - 3, 0, CW, CH);
	txt(x, y, C_TEXT, "psvr2linuxtest  slopvr %s  %s  cam mode 0x%x  %d fps  gain %.1f", SLOP_VERSION, U.replay ? "REPLAY" : "live", U.cam_mode, U.fps, U.gain);
	if (U.paused) txt(x + 560, y, C_WARN, "PAUSED");
	y += ROW + 4;
	/* ---- headset ---- */
	{
		bool conn = atomic_load(&H.connected); quat sq; double sp[3]; bool sok, sinit, iok; uint64_t s_us, i_us; uint8_t prox, btn, ipd, dprx;
		pthread_mutex_lock(&H.lock);
		sq = H.slam_q; memcpy(sp, H.slam_p, sizeof sp); sok = H.slam_ok; sinit = H.slam_init; iok = H.imu_ok; s_us = H.slam_host_us; i_us = H.imu_host_us;
		prox = H.prox; btn = H.button; ipd = H.ipd_mm; dprx = H.dprx;
		pthread_mutex_unlock(&H.lock);
		txt(x, y, C_TEXT, "HMD"); txt(x + 40, y, conn ? C_OK : C_BAD, conn ? "connected" : (H.fake ? "fake" : "NOT CONNECTED"));
		txt(x + 152, y, C_DIM, "fw %08x %-16.16s dprx %d prox %d btn %d ipd %d mm", H.fw_version, H.serial, dprx, prox, btn, ipd); y += ROW;
		txt(x, y, C_DIM, "panel"); txt(x + 48, y, HO.active ? C_OK : U.no_hmd || U.replay ? C_DIM : C_BAD, "%.104s", HO.status[0] ? HO.status : "not leased");
		y += ROW;
		if (HO.active) { txt(x, y, C_DIM, "view"); txt(x + 48, y, C_TEXT, "%s  %.0f fps  flips %u busy %u  lut %s (%llu ms)   v: see-through / grid / off   a: auto-gain %s", U.hmd_view == 1 ? "see-through (L<-cam0, R<-cam1)" : U.hmd_view == 2 ? "2x2 grid" : "off", HP.fps, HO.flips, HO.busy, HP.lut_ok ? "ok" : "building", (unsigned long long)HP.lut_ms, U.auto_gain ? "on" : "off"); y += ROW; }
		txt(x, y, C_DIM, "SLAM"); txt(x + 40, y, sok ? C_OK : C_WARN, sok ? "ok" : sinit ? "lost" : "init");
		if (sok) txt(x + 72, y, C_TEXT, "p %+.3f %+.3f %+.3f  q %+.3f %+.3f %+.3f %+.3f  %3d ms  IMU %s %d ms", sp[0], sp[1], sp[2], sq.w, sq.x, sq.y, sq.z, age_ms(now, s_us), iok ? "ok" : "-", iok ? age_ms(now, i_us) : 0);
		else txt(x + 72, y, C_DIM, "IMU %s  (SLAM needs the headset awake: panel lit / status stream running)", iok ? "ok" : "-");
		y += ROW;
		int tx = txt(x, y, C_DIM, "USB/s"); tx = txt(x + 48, y, C_TEXT, "status %u  slam %u  gaze %u  cam %u  LD %u  gendata %u", A.rate[0], A.rate[1], A.rate[2], A.rate[3], A.rate[4], A.rate[5]);
		if (A.ncam_lens) { tx = txt(tx + 10, y, C_DIM, "cam pkt"); for (int i = 0; i < A.ncam_lens; i++) tx = txt(tx + 6, y, C_DIM, "%d", A.cam_lens[i]); }
		y += ROW;
		bool cal; pthread_mutex_lock(&O.lock); cal = O.calib_ok; pthread_mutex_unlock(&O.lock);
		txt(x, y, C_DIM, "calib"); txt(x + 48, y, cal ? C_OK : C_BAD, cal ? "ok" : "MISSING (GenData)");
		if (cal) txt(x + 72, y, C_DIM, "cam0 f %.1f c %.1f,%.1f  cam1 @ %+.3f %+.3f %+.3f  cam2 @ %+.3f %+.3f %+.3f", O.cam[0].fx, O.cam[0].cx, O.cam[0].cy, O.cam[1].o[0], O.cam[1].o[1], O.cam[1].o[2], O.cam[2].o[0], O.cam[2].o[1], O.cam[2].o[2]);
		y += ROW;
		int ldd, ldmin, ldmax; unsigned ldf; int lmode; bool dump;
		pthread_mutex_lock(&O.lock); ldd = O.ld_delay; ldmin = O.ld_delay_min; ldmax = O.ld_delay_max; ldf = O.ld_frames; lmode = O.led_mode; dump = O.dump; pthread_mutex_unlock(&O.lock);
		txt(x, y, C_DIM, "LD"); txt(x + 48, y, C_TEXT, "frames %u  gain %d exp %d us (pkt %.0f/%.0f)  frame->IMU %d (%d..%d) us  LEDs %s%s", ldf, H.ld_gain, H.ld_exposure,
		    ldl ? ldl->gain : 0.f, ldl ? ldl->exp_us : 0.f, ldd, ldmin, ldmax, lmode == 0 ? "synced" : lmode == LED_PHASE_ON ? "ON" : lmode == LED_PHASE_OFF ? "OFF" : "?", dump ? " verbose" : "");
		y += ROW;
		txt(x, y, C_DIM, "blobs");
		if (ldl) txt(x + 48, y, C_TEXT, "cam0 %2d cam1 %2d cam2 %2d cam3 %2d  frame %u %s  %3d ms   worker: %d rays %d pairs %d clusters", ldl->ncam[0], ldl->ncam[1], ldl->ncam[2], ldl->ncam[3], ldl->frame, T.led_mux != 0 ? (ldl->frame & 1 ? "(R frame)" : "(L frame)") : "", age_ms(now, ldl->host_us), snap->valid ? snap->nray : 0, snap->valid ? snap->ncand : 0, snap->valid ? snap->ncl : 0);
		else txt(x + 48, y, C_DIM, "no LD packets yet (LED detector stream, interface 8)");
		y += ROW;
		if (ldl && T.led_mux != 0) { /* does the controller honour the 2-frame cycle?  cover one controller: its parity should go to 0 */
			int n_par[2] = { -1, -1 }; unsigned f_par[2] = { 0, 0 };
			for (unsigned i = ldn > LD_RING ? ldn - LD_RING : 0; i < ldn; i++) { const struct ldpkt *p = &ldring[i % LD_RING]; n_par[p->frame & 1] = p->n; f_par[p->frame & 1] = p->frame; }
			txt(x, y, C_DIM, "mux"); txt(x + 48, y, C_TEXT, "L frames (even): %d blobs (#%u)   R frames (odd): %d blobs (#%u)   -> with ledMux each hand's LEDs should appear only on its own parity", n_par[0], f_par[0], n_par[1], f_par[1]);
			y += ROW;
		}
		y += 4;
	}
	/* ---- controllers ---- */
	for (int h = 0; h < 2; h++) {
		const struct sense *s = &S[h]; uint32_t col = C_HAND[h];
		struct opt_fix fx; struct led_sync L; struct clk ldclk; unsigned sl, sa, sf; int tlock; bool qcok;
		pthread_mutex_lock(&O.lock);
		fx = O.fix[h]; L = O.led[h]; ldclk = O.ldclk; sl = O.stat_lock[h]; sa = O.stat_acq[h]; sf = O.stat_fail[h]; tlock = O.tl_last_lock[h]; qcok = O.qc_ok[h];
		pthread_mutex_unlock(&O.lock);
		txt(x, y, col, "%c", h ? 'R' : 'L');
		if (s->fd < 0) txt(x + 16, y, C_BAD, "not connected (pair the Sense controller over Bluetooth / plug USB)");
		else { txt(x + 16, y, C_OK, "%-9s", s->usb ? "usb" : "bluetooth"); txt(x + 96, y, C_DIM, "%-22.22s bat %02x cal %s IMU %s yaw %d qc %s in %3d ms", s->serial, s->battery, s->cal.ok ? "ok" : "--", s->q_ok ? "ok" : "--", s->yaw_aligned, s->qc_ok && qcok ? "ok" : "--", age_ms(now, s->last_in_us)); }
		y += ROW;
		if (s->fd >= 0) { /* one line that says why there is or is not a position */
			double bl[2] = { 0, 0 }; int bc[2] = { 0, 0 };
			for (int i = nh - 30 > 0 ? nh - 30 : 0; i < nh; i++) if (hist[i].par >= 0) { bl[hist[i].par] += hist[i].nray; bc[hist[i].par]++; }
			double own_blobs = L.par_ok ? (bc[L.par_meas] ? bl[L.par_meas] / bc[L.par_meas] : 0) : ((bc[0] + bc[1]) ? (bl[0] + bl[1]) / (bc[0] + bc[1]) : 0);
			int fage = fx.valid ? age_ms(now, fx.host_us) : -1; uint32_t clkn = L.clk.n; uint64_t next_check = L.scan_done_us + L.scan_wait_us;
			const char *why; uint32_t wc = C_WARN; char buf[160];
			uint8_t bst = s->battery >> 4, blvl = s->battery & 0xF;
			if (s->battery != 0xFF && bst == 0 && blvl <= 1 && own_blobs < 1.0) { snprintf(buf, sizeof buf, "BATTERY EMPTY (%d%%): the Sense stops flashing its tracking LEDs when nearly flat - charge it", blvl * 10 + 5); why = buf; wc = C_BAD; }
			else if (clkn < 3) { snprintf(buf, sizeof buf, "syncing the controller clock (%u/3 s): LEDs dark until then", clkn); why = buf; }
			else if (L.scan && L.step < 0) { why = "checking whether the flash is visible with the stored timing"; }
			else if (L.scan) { snprintf(buf, sizeof buf, "scanning the flash timing: step %d of %d (%s), a few seconds", L.scan_k + 1, 21, L.on ? "flashing" : "LEDs off"); why = buf; }
			else if (fx.valid && fage >= 0 && fage < 100) { snprintf(buf, sizeof buf, "TRACKED  n=%d LEDs  res %.2f px", fx.n, fx.res); why = buf; wc = C_OK; }
			else if (own_blobs < 1.0) { snprintf(buf, sizeof buf, "no LEDs seen by the cameras: out of view, or the flash misses the exposure (timing re-check in %d s)", (int)(next_check > now ? (next_check - now) / 1000000 : 0)); why = buf; wc = C_BAD; }
			else if (fx.valid && fage >= 0 && fage < 1500) { snprintf(buf, sizeof buf, "LOST %d ms ago (dead-reckoning on the IMU), %.0f blobs on its frames but no fit", fage, own_blobs); why = buf; }
			else { snprintf(buf, sizeof buf, "LEDs visible (%.0f blobs on its frames) but not fitted: acquisition failing, see log (p)", own_blobs); why = buf; }
			txt(x + 16, y, wc, "%s", why);
			y += ROW;
		}
		{ /* qc = world <- IMU-world yaw correction; if the LED model frame == IMU frame it is a pure yaw (tilt ~0) */
			quat qc = s->qc; double ang = 2 * acos(fabs(qc.w) > 1 ? 1 : fabs(qc.w));
			double tilt; pthread_mutex_lock(&O.lock); tilt = O.qc_tilt[h]; pthread_mutex_unlock(&O.lock);
			int imq; double imsp, imrms; bool mlive; pthread_mutex_lock(&O.lock); imq = O.imf[h].quality; imsp = O.imf[h].spread; imrms = O.imf[h].rms; mlive = O.mux_live[h]; pthread_mutex_unlock(&O.lock);
			txt(x + 16, y, C_DIM, "LED"); txt(x + 48, y, L.scan ? C_WARN : C_TEXT, "%-4s lat %5d us%s scan %s probe %+d  clk %2d %+5.1f us/s rtt %5u  hmd %2d %+5.1f", phase_name(L.phase, L.freerun), L.lat, L.lat_ok ? " " : "?",
			    L.scan ? (L.on ? "ON " : "off") : "-  ", L.probe, L.clk.n, L.clk.b, L.clk.cur_rtt, ldclk.n, ldclk.b);
			txt(x + 48 + 8 * 81, y, mlive ? C_OK : C_WARN, "mux %s", mlive ? "live" : "DEAD (lit on both parities)"); /* with ledMux: this hand's LEDs only on its own frame parity? */
			if (mlive) txt(x + 48 + 8 * 90, y, L.par_ok ? C_OK : C_WARN, "par %s", L.par_ok ? (L.par_meas ? "odd" : "even") : "?"); /* measured frame parity of this hand's flash (until known the hand is fitted on every frame) */
			y += ROW;
			/* IMU <-> LED model: yaw correction (pure yaw once the model<->IMU rotation is calibrated), the residual tilt the
			 * calibration leaves, the calibration itself (tilt-only until the controller was seen at orientations > 25 deg apart) */
			txt(x + 16, y, C_DIM, "IMU"); txt(x + 48, y, C_TEXT, "qc %s yaw %+4.0f deg", s->qc_ok ? "ok" : "--", s->qc_ok ? qyaw(qc) * 180 / M_PI : 0);
			if (qcok) txt(x + 48 + 8 * 20, y, tilt < 20 ? C_OK : C_BAD, "resid %2.0f deg", tilt);
			txt(x + 48 + 8 * 34, y, imq == 2 ? C_OK : imq == 1 ? C_WARN : C_DIM, "model<->IMU frame %s (spread %.0f deg, rms %.1f)", imq == 2 ? "calibrated" : imq == 1 ? "tilt only - turn it" : "unknown", imsp, imrms);
			(void)ang;
			y += ROW;
		}
		int age = fx.valid ? age_ms(now, fx.host_us) : -1;
		txt(x + 16, y, C_DIM, "fix"); txt(x + 48, y, fx.valid && age < 100 ? col : C_DIM, "%s age %4d ms  n=%2d  res %.2f px  streak %-4d  world %+.3f %+.3f %+.3f  v %.2f m/s", fx.valid ? "ok  " : "none", age, fx.n, fx.res, fx.streak, fx.p[0], fx.p[1], fx.p[2], sqrt(fx.v[0] * fx.v[0] + fx.v[1] * fx.v[1] + fx.v[2] * fx.v[2])); y += ROW;
		int tx = txt(x + 16, y, C_DIM, "fit"); tx = txt(x + 48, y, C_TEXT, "lock %u acq %u miss %u (5 s window) %2d/s", sl, sa, sf, tlock);
		if (snap->valid && snap->ok[h]) { const double *t = snap->P[h].t; txt(tx + 10, y, col, "cam0-rel %+.3f %+.3f %+.3f  %.2f m  n=%d %.2fpx %s ev %d/%d/%d", t[0], t[1], t[2], sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]), snap->P[h].n, snap->P[h].res, snap->acq[h] ? "ACQ" : "lock", snap->ev[h].n, snap->ev[h].missing, snap->ev[h].stray); }
		else if (snap->valid && T.led_mux != 0 && snap->par >= 0 && snap->par != h) txt(tx + 10, y, C_DIM, "latest frame belongs to the other hand (ledMux)");
		else if (snap->valid) txt(tx + 10, y, C_DIM, "no fit in the latest frame (%d rays)", snap->nray);
		y += ROW;
		tx = txt(x + 16, y, C_DIM, "pub");
		if (pub_ok[h]) {
			const DriverPose_t *p = &pub[h]; char in[128]; comp_summary(s->container, in, sizeof in);
			tx = txt(x + 48, y, p->result == TrackingResult_Running_OK ? C_TEXT : C_WARN, "%+.3f %+.3f %+.3f  res %d %s", p->vecPosition[0], p->vecPosition[1], p->vecPosition[2], p->result, p->poseIsValid ? "valid" : "INVALID");
			txt(tx + 10, y, C_DIM, "%s", in);
		} else txt(x + 48, y, C_DIM, "not published yet");
		y += ROW + 4;
	}
	/* ---- camera mapping ---- */
	{
		txt(x, y, C_DIM, "map"); int tx = txt(x + 48, y, U.auto_map ? C_TEXT : C_WARN, U.auto_map ? "auto" : "MANUAL");
		for (int s = 0; s < NVIEW; s++) tx = txt(tx + 10, y, V[s].w ? C_TEXT : C_DIM, "slot%d(t%d%c)>cam%d", s, s < 2 ? 4 : 5, s & 1 ? 'b' : 'a', U.map[s]);
		y += ROW;
		tx = txt(x, y, C_DIM, "votes");
		for (int s = 0; s < NVIEW; s++) tx = txt(tx + 10, y, C_DIM, "s%d[%d %d %d %d]", s, U.votes[s][0], U.votes[s][1], U.votes[s][2], U.votes[s][3]);
		y += ROW;
		tx = txt(x, y, C_DIM, "LD-img");
		for (int s = 0; s < NVIEW; s++) tx = txt(tx + 10, y, U.noff[s] ? C_TEXT : C_DIM, "s%d %+.1f,%+.1f", s, U.off[s][0], U.off[s][1]);
		txt(tx + 10, y, C_DIM, "binned px (LD blob - image blob)");
		y += ROW + 4;
	}
	/* ---- charts ---- */
	y = draw_charts(x, y, hist, nh);
	/* ---- tunables ---- */
	{
		int n = (int)ARRAY_LEN(params), rows = 15, first = U.sel - rows / 2;
		if (first > n - rows) first = n - rows;
		if (first < 0) first = 0;
		txt(x, y, C_DIM, "tunables (up/down select, left/right adjust, shift = x10, w = write non-defaults)"); y += ROW;
		for (int i = first; i < first + rows && i < n; i++, y += ROW) {
			const struct param *p = &params[i];
			if (i == U.sel) fill_rect(x - 2, y - 1, PNL_W, ROW, C_SEL);
			txt(x, y, i == U.sel ? C_TEXT : C_DIM, "%-13s", p->name);
			txt(x + 112, y, fabs(*p->v - p->def) > 1e-9 ? C_WARN : C_TEXT, "%10.4g", *p->v);
			txt(x + 208, y, C_DIM, "[%g .. %g]  def %g  (%s)", p->min, p->max, p->def, p->cfg);
		}
		y += 4;
	}
	/* ---- driver log ---- */
	{
		pthread_mutex_lock(&F.lock);
		unsigned n = F.log_n < LOG_N ? F.log_n : LOG_N;
		for (unsigned i = 0; i < n; i++, y += ROW) txt(x, y, i == n - 1 ? C_TEXT : C_DIM, "%.106s", F.log[(F.log_n - n + i) % LOG_N]);
		pthread_mutex_unlock(&F.lock);
	}
	txt(x, CH - ROW - 2, C_DIM, "h help  q quit  +/- gain  space pause  t i k b overlays  m/M map  l LED  c cam  r resync  y yaw  p log  s save");
	clip(0, 0, CW, CH);
}

static void draw_help(void)
{
	static const char *lines[] = {
		"psvr2linuxtest - what you are looking at",
		"",
		"Quadrants: cam0 front-left, cam1 front-right, cam2 lower-left, cam3 lower-right (mapping from LD",
		"  camera index to image slot is learnt by matching image blobs to the headset's LD blobs).",
		"green circle + number      headset LD blob detection (record index), packet nearest to the frame time",
		"yellow square              blob found by this tool in the image (threshold 128; LD-img = mean offset)",
		"cyan/magenta ring + number LD blob the fit explained, with the LED index (L cyan, R magenta)",
		"cyan/magenta cross         projected LED model of the fitted pose (small/dim = LED facing away)",
		"rgb lines                  model axes (x red, y green, z blue) at the fitted origin",
		"white/grey square + n      stereo cluster (triangulated candidate, n = supporting pairs); grey = used",
		"diamond                    position the driver publishes to SteamVR (dim = OutOfRange/extrapolated)",
		"dots                       IMU-predicted constellation at the published position (orientation check)",
		"",
		"keys",
		"  +/-  image gain (a: auto-gain)   t image blobs   i IMU dots   k clusters   b labels   space pause",
		"  v    headset view: see-through (L eye<-cam0, R eye<-cam1 through the lens model) / 2x2 grid / off",
		"  m    auto/manual map     M next manual permutation",
		"  l    LED mode: synced flashes -> free-running flash (blinks through the exposure ~1x/s: 'is it in view?') -> OFF",
		"  c    camera mode 4 -> 0xc -> 1 -> 0x10 -> off (mode 4 is the one the LED detector sees)",
		"  g/G  e/E  LED detector gain / exposure x1.4 (also the camera exposure: raise for see-through; Sony uses 1000 us)",
		"  r    drop LED sync latencies (rescan)",
		"  y    re-align controller yaw to the head   p verbose optical log   w write tunables   s save frame+LD+bmp",
		"  arrows: select/adjust tunable (shift x10) - cam_*/mirror_* re-apply immediately, as do all thresholds",
		"  F5 (or hold Options/Create on a controller 1 s): guided calibration flow - 8 single-action steps with a live checklist shown in the",
		"       headset; trigger = next step, n key = next, hold Options 2 s / F5 = abort. Steps are marked in the recording.",
		"",
		"Lag diagnosis: fix age (ms) is how old the newest optical position is when the pose is published;",
		"the worker ms chart shows fitting time per LD frame; red ticks in the LED charts are frames without a fit.",
		"IMU line: the LED-model<->IMU rotation is calibrated from fits while you turn the controller (>25 deg apart);",
		"once 'calibrated' fits on the wrong controller / ring slot are rejected by gravity. 'mux DEAD' = this hand's",
		"LEDs show on both frame parities (no clock sync yet, or the 2-frame cycle ignored): it is fitted every frame.",
		"stderr/log: 'state:' lines every 10 s dump the whole tracker (both hands, LED timing, parity, tunables).",
	};
	int w = 108 * FONT_W, h = (int)ARRAY_LEN(lines) * ROW + 16, x = Q_X0 + (2 * Q_PX - w) / 2, y = Q_Y0 + (2 * Q_PY - h) / 2;
	fill_rect(x, y, w, h, 0xf0101820); rect(x, y, w, h, C_TEXT);
	for (unsigned i = 0; i < ARRAY_LEN(lines); i++) txt(x + 8, y + 8 + (int)i * ROW, i == 0 ? C_TEXT : C_TEXT, "%s", lines[i]);
}

/* ================= main ================= */
static bool vrserver_running(void)
{
	DIR *d = opendir("/proc"); struct dirent *e; bool found = false;
	if (!d) return false;
	while (!found && (e = readdir(d))) {
		if (e->d_name[0] < '0' || e->d_name[0] > '9') continue;
		char p[300], comm[64] = { 0 }; snprintf(p, sizeof p, "/proc/%s/comm", e->d_name);
		FILE *f = fopen(p, "r"); if (!f) continue;
		if (fgets(comm, sizeof comm, f) && !strncmp(comm, "vrserver", 8)) found = true;
		fclose(f);
	}
	closedir(d);
	return found;
}
/* 0 ok, 1 not plugged in, 2 no permission on the usbfs node (udev rule) */
static int check_usb_access(char *node, size_t n)
{
	DIR *d = opendir("/sys/bus/usb/devices"); struct dirent *e; int r = 1;
	if (!d) return 1;
	while (r == 1 && (e = readdir(d))) {
		char p[320];
		if (e->d_name[0] == '.' || strchr(e->d_name, ':')) continue;
		snprintf(p, sizeof p, "/sys/bus/usb/devices/%s/idVendor", e->d_name); if (usb_read_hex(p) != PSVR2_VID) continue;
		snprintf(p, sizeof p, "/sys/bus/usb/devices/%s/idProduct", e->d_name); if (usb_read_hex(p) != PSVR2_PID) continue;
		snprintf(p, sizeof p, "/sys/bus/usb/devices/%s/busnum", e->d_name); int bus = usb_read_dec(p);
		snprintf(p, sizeof p, "/sys/bus/usb/devices/%s/devnum", e->d_name); int dev = usb_read_dec(p);
		snprintf(node, n, "/dev/bus/usb/%03d/%03d", bus, dev);
		r = access(node, R_OK | W_OK) == 0 ? 0 : 2;
	}
	closedir(d);
	return r;
}
static bool find_default_settings(char *out, size_t n)
{
	char exe[512] = { 0 }; ssize_t l = readlink("/proc/self/exe", exe, sizeof exe - 1);
	if (l > 0) { char *s = strrchr(exe, '/'); if (s) *s = 0; }
	const char *cands[] = { "%s/resources/settings/default.vrsettings", "%s/../share/slopvr/resources/settings/default.vrsettings", "%s/build/slopvr/resources/settings/default.vrsettings", "%s/../resources/settings/default.vrsettings" };
	for (unsigned i = 0; i < ARRAY_LEN(cands); i++) { snprintf(out, n, cands[i], exe); if (!access(out, R_OK)) return true; }
	snprintf(out, n, "resources/settings/default.vrsettings");
	return !access(out, R_OK);
}
static void usage(const char *a0)
{
	fprintf(stderr,
		"usage: %s [options]\n"
		"  --mode N          camera mode to stream (default 4: 2x 508x508 binned fisheye pairs; 0xc: 4x 256x254; 1; 0x10)\n"
		"  --settings FILE   extra .vrsettings to load after SteamVR's (driver_slopvr section)\n"
		"  --no-user-settings  do not read ~/.local/share/Steam/config/steamvr.vrsettings\n"
		"  -s key=value      override a driver_slopvr setting (repeatable), e.g. -s ledPhase=6 -s ledGain=48\n"
		"  --scale F         window scale (default: fit the screen)\n"
		"  --force           run even if vrserver is running (they would fight over the USB interfaces)\n"
		"  --replay DIR      offline: play captured camera frames (frame_*.bin) and LD packets (ld_*.bin) from DIR\n"
		"  --calib FILE      camera calibration for --replay (GenData item 10 \"SIE\" blob or a whole \"VD\" dump)\n"
		"  --autoshot SEC    save frame+LD+screenshot to ~/.cache/slopvr_test after SEC seconds and quit (headless check)\n"
		"  --led-toggle SEC  LED command response test: free-running flash / OFF every SEC seconds (blob counts in the 1 Hz tl lines)\n"
		"  --guide           start the guided calibration flow 4 s after launch (also: F5, or hold Options/Create on a controller)\n"
		"  --no-hmd          do not lease/light the headset panel (the headset then stays asleep: no cameras/SLAM/LD)\n"
		"  --hmd-connector N DRM connector to lease (default: the one that looks like a PSVR2, e.g. DP-9)\n"
		"  --hmd-mode WxH@R  panel mode (default 4000x2040 at the highest rate, then 1920x1080)\n", a0);
}

int main(int argc, char **argv)
{
	const char *extra_settings = NULL, *calib = NULL; bool user_settings = true, force = false; float scale = 0;
	struct { char k[48], v[96]; } over[32]; int nover = 0;
	U.cam_mode = 4;
	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--mode") && i + 1 < argc) U.cam_mode = (int)strtol(argv[++i], NULL, 0);
		else if (!strcmp(argv[i], "--settings") && i + 1 < argc) extra_settings = argv[++i];
		else if (!strcmp(argv[i], "--no-user-settings")) user_settings = false;
		else if (!strcmp(argv[i], "--force")) force = true;
		else if (!strcmp(argv[i], "--scale") && i + 1 < argc) scale = (float)atof(argv[++i]);
		else if (!strcmp(argv[i], "--replay") && i + 1 < argc) { U.replay = true; snprintf(U.replay_dir, sizeof U.replay_dir, "%s", argv[++i]); snprintf(O.dump_dir, sizeof O.dump_dir, "slopvr_ld_replay"); }
		else if (!strcmp(argv[i], "--calib") && i + 1 < argc) calib = argv[++i];
		else if (!strcmp(argv[i], "--autoshot") && i + 1 < argc) U.autoshot_s = atof(argv[++i]);
		else if (!strcmp(argv[i], "--led-toggle") && i + 1 < argc) U.led_toggle_s = atof(argv[++i]);
		else if (!strcmp(argv[i], "--guide")) U.calib = true;
		else if (!strcmp(argv[i], "--show-help")) U.help = true;
		else if (!strcmp(argv[i], "--no-hmd")) U.no_hmd = true;
		else if (!strcmp(argv[i], "--hmd-connector") && i + 1 < argc) U.hmd_conn = argv[++i];
		else if (!strcmp(argv[i], "--hmd-mode") && i + 1 < argc) U.hmd_mode = argv[++i];
		else if (!strcmp(argv[i], "-s") && i + 1 < argc && nover < 32) {
			const char *kv = argv[++i], *eq = strchr(kv, '=');
			if (!eq) { usage(argv[0]); return 2; }
			snprintf(over[nover].k, sizeof over[nover].k, "%.*s", (int)(eq - kv), kv); snprintf(over[nover].v, sizeof over[nover].v, "%s", eq + 1); nover++;
		} else { usage(argv[0]); return !strcmp(argv[i], "--help") || !strcmp(argv[i], "-h") ? 0 : 2; }
	}
	app_t0 = now_us();
	fake_init();
	pthread_mutex_init(&A.lock, NULL);
	signal(SIGINT, on_signal); signal(SIGTERM, on_signal);

	/* settings: the driver's defaults, the user's SteamVR config, extras, forced test values, -s overrides */
	char path[600];
	if (find_default_settings(path, sizeof path)) { json_load(path); fprintf(stderr, "settings: %s\n", path); }
	else fprintf(stderr, "settings: default.vrsettings not found (run from the slopvr directory); using built-in defaults\n");
	if (user_settings) {
		snprintf(path, sizeof path, "%s/.local/share/Steam/config/steamvr.vrsettings", getenv("HOME") ? getenv("HOME") : "");
		if (json_load(path)) fprintf(stderr, "settings: %s\n", path);
	}
	if (extra_settings && !json_load(extra_settings)) fprintf(stderr, "settings: cannot read %s\n", extra_settings);
	{ char m[16]; snprintf(m, sizeof m, "%d", U.cam_mode); kv_set(SECTION, "cameraMode", m, 0); }
	kv_set(SECTION, "fakeHmd", U.replay ? "true" : "false", 1);
	kv_set(SECTION, "opticalTracking", "true", 1);
	if (U.replay) kv_set(SECTION, "enableControllers", "false", 1);
	if (U.replay) kv_set(SECTION, "record", "false", 1);
	for (int i = 0; i < nover; i++) kv_set_auto(SECTION, over[i].k, over[i].v);

	if (!U.replay) {
		if (!force && vrserver_running()) { fprintf(stderr, "vrserver is running: quit SteamVR first (or --force).\n"); return 1; }
		char node[64] = "?"; int r = check_usb_access(node, sizeof node);
		if (r == 1) fprintf(stderr, "warning: PSVR2 (054c:0cde) not on the USB bus - plug it in; the driver waits for hotplug\n");
		if (r == 2) {
			char rules[600] = "70-slopvr.rules", exe[512] = { 0 }; ssize_t l = readlink("/proc/self/exe", exe, sizeof exe - 1);
			if (l > 0) { char *sl = strrchr(exe, '/'); if (sl) *sl = 0; const char *c[] = { "%s/70-slopvr.rules", "%s/../share/slopvr/70-slopvr.rules" }; for (unsigned i = 0; i < 2; i++) { snprintf(rules, sizeof rules, c[i], exe); if (!access(rules, R_OK)) break; } }
			fprintf(stderr, "cannot open %s: no permission. Install the udev rule (until reboot):\n"
				"  pkexec sh -c 'mkdir -p /run/udev/rules.d && cp %s /run/udev/rules.d/ && udevadm control --reload && udevadm trigger'\n"
				"or on NixOS: imports = [ slopvr.nixosModules.default ];  (the driver waits for the device meanwhile)\n", node, rules);
		}
	} else if (replay_is_slop(U.replay_dir)) { if (!replay_open_slop(U.replay_dir)) { fprintf(stderr, "replay: cannot read session file %s\n", U.replay_dir); return 1; } fprintf(stderr, "replay: session file %s (calibration, head poses and controller IMU come from the file)\n", U.replay_dir); }
	else if (!replay_scan(U.replay_dir)) { fprintf(stderr, "replay: no camera frames / LD packets in %s\n", U.replay_dir); return 1; }
	else fprintf(stderr, "replay: %d camera frames, %d LD packets\n", RP.ncam, RP.nld);

	g_usb_hook = app_usb_hook; g_opt_hook = app_opt_hook;
	cam_urbs = U.cam_mode != 0;
	int32_t err = 0;
	ovr_obj *prov = HmdDriverFactory(OVR_STR_IServerTrackedDeviceProvider, &err);
	if (!prov) { fprintf(stderr, "HmdDriverFactory failed (%d)\n", err); return 1; }
	if (((ovr_provider_Init)prov->vt[0])(prov, &o_ctx) != VRInitError_None) { fprintf(stderr, "driver Init failed\n"); return 1; }
	if (U.replay) {
		static uint8_t cal[0x10000];
		int n = calib ? read_file(calib, cal, sizeof cal) : -1;
		if (n > 0 && cal[0] == 'V' && cal[1] == 'D') hmd_on_gendata(cal, n);
		else if (n > 0) opt_set_calib(cal, n);
		else if (!RP.slop) fk_log(NULL, "replay: no --calib: blobs are shown but nothing can be fitted");
	}

	if (!U.replay && !U.no_hmd) hmdout_start(U.hmd_conn, U.hmd_mode); /* DP link up -> the headset wakes its cameras/SLAM/LED detector */

	if (SDL_Init(SDL_INIT_VIDEO) != 0) { fprintf(stderr, "SDL_Init: %s\n", SDL_GetError()); return 1; }
	if (scale <= 0) { SDL_Rect db; if (!SDL_GetDisplayUsableBounds(0, &db)) { scale = fminf((float)(db.w - 40) / CW, (float)(db.h - 80) / CH); if (scale > 1) scale = 1; } else scale = 0.8f; }
	SDL_Window *win = SDL_CreateWindow("psvr2linuxtest - slopvr camera view", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, (int)(CW * scale), (int)(CH * scale), SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
	SDL_Renderer *ren = win ? SDL_CreateRenderer(win, -1, SDL_RENDERER_PRESENTVSYNC) : NULL;
	if (!ren && win) ren = SDL_CreateRenderer(win, -1, 0);
	if (!ren) { fprintf(stderr, "SDL window/renderer: %s\n", SDL_GetError()); return 1; }
	SDL_RenderSetLogicalSize(ren, CW, CH);
	SDL_Texture *tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, CW, CH);
	cv = mem_alloc((size_t)CW * CH * 4);

	static uint8_t raw[RAW_RING][RAW_MAX]; int rawlen[RAW_RING]; uint64_t rawus[RAW_RING]; static struct ldpkt ldring[LD_RING]; static struct optsnap snap; static struct hist hist[HIST_N];
	unsigned raw_r = 0; uint64_t rate_t0 = now_us();
	while (!g_quit) {
		uint64_t now = now_us();
		SDL_Event ev;
		while (SDL_PollEvent(&ev)) {
			if (ev.type == SDL_QUIT) g_quit = 1;
			if (ev.type != SDL_KEYDOWN) continue;
			bool shift = (ev.key.keysym.mod & KMOD_SHIFT) != 0;
			switch (ev.key.keysym.sym) {
			case SDLK_ESCAPE: case SDLK_q: g_quit = 1; break;
			case SDLK_h: case SDLK_F1: U.help = !U.help; break;
			case SDLK_SPACE: U.paused = !U.paused; break;
			case SDLK_PLUS: case SDLK_EQUALS: case SDLK_KP_PLUS: U.gain = fminf(U.gain * 1.25f, 16.f); break;
			case SDLK_MINUS: case SDLK_KP_MINUS: U.gain = fmaxf(U.gain / 1.25f, 0.5f); break;
			case SDLK_t: U.show_img_blobs = !U.show_img_blobs; break;
			case SDLK_i: U.show_imu = !U.show_imu; break;
			case SDLK_k: U.show_cl = !U.show_cl; break;
			case SDLK_b: U.show_labels = !U.show_labels; break;
			case SDLK_v: U.hmd_view = U.hmd_view == 1 ? 2 : U.hmd_view == 2 ? 0 : 1; hmdout_set_mode(U.hmd_view); break;
			case SDLK_a: U.auto_gain = !U.auto_gain; break;
			case SDLK_m:
				if (shift) { /* next permutation of the manual mapping */
					U.auto_map = false; int *m = U.map, i = NVIEW - 2;
					while (i >= 0 && m[i] > m[i + 1]) i--;
					if (i < 0) { for (int k = 0; k < NVIEW; k++) m[k] = k; }
					else { int j = NVIEW - 1; while (m[j] < m[i]) j--; int t = m[i]; m[i] = m[j]; m[j] = t; for (int a = i + 1, b = NVIEW - 1; a < b; a++, b--) { t = m[a]; m[a] = m[b]; m[b] = t; } }
				} else { U.auto_map = !U.auto_map; if (U.auto_map) resolve_map(); }
				break;
			case SDLK_l: cycle_led_mode(); break;
			case SDLK_c: { static const int modes[] = { 4, 0xc, 1, 0x10, 0 }; int i = 0; while (i < 4 && modes[i] != U.cam_mode) i++; set_camera_mode(modes[(i + 1) % 5]); break; }
			case SDLK_g: T.led_gain = CLAMP(round(T.led_gain * (shift ? 1.4 : 1 / 1.4)), 1, 255); restart_ld(); break;
			case SDLK_e: T.led_exp_us = CLAMP(round(T.led_exp_us * (shift ? 1.4 : 1 / 1.4)), 30, 4000); restart_ld(); break;
			case SDLK_r: reset_led_sync(); break;
			case SDLK_y: for (int h = 0; h < 2; h++) S[h].yaw_aligned = false; fk_log(NULL, "test: controller yaw re-aligns to the head on the next report"); break;
			case SDLK_p: pthread_mutex_lock(&O.lock); O.dump = !O.dump; O.state_req = true; pthread_mutex_unlock(&O.lock); break;
			case SDLK_UP: if (U.sel > 0) U.sel--; break;
			case SDLK_DOWN: if (U.sel + 1 < (int)ARRAY_LEN(params)) U.sel++; break;
			case SDLK_LEFT: tune_step(-1, shift); break;
			case SDLK_RIGHT: tune_step(1, shift); break;
			case SDLK_w: write_tune(); break;
			case SDLK_s: save_snapshot(ren); break;
			case SDLK_F5: if (CF.state == CF_RUN) cf_finish(now, true); else calflow_start(now); break;
			case SDLK_n: calflow_next(now, "skipped by key"); break;
			default: break;
			}
		}
		((ovr_provider_RunFrame)prov->vt[3])(prov); /* what vrserver does ~90x/s: device registration, events */
		if (U.replay) replay_tick(now);
		calflow_tick(now);
		if (U.calib && CF.state == CF_IDLE && now - app_t0 > 4000000) { U.calib = false; calflow_start(now); }
		if (U.led_toggle_s > 0) { /* LED command response test: continuous ON / OFF alternating, watch the blob counts in the 1 Hz "tl" lines */
			static uint64_t last; static int st = -1;
			if (!last || now - last > (uint64_t)(U.led_toggle_s * 1e6)) {
				static const int seq[4] = { LED_PHASE_OFF, 1, LED_PHASE_OFF, LED_PHASE_ON }; static const char *name[4] = { "OFF", "INIT", "OFF", "free-running flash" };
				last = now; st = (st + 1) & 3;
				pthread_mutex_lock(&O.lock); O.led_mode = seq[st]; for (int h = 0; h < 2; h++) O.led[h].dirty = true; pthread_mutex_unlock(&O.lock);
				char msg[64]; snprintf(msg, sizeof msg, "test: LED toggle -> %s", name[st]); fk_log(NULL, msg);
			}
		}
		hmdout_pump();

		/* snapshot the tapped state */
		int nraw = 0; unsigned ldn; int nh;
		pthread_mutex_lock(&A.lock);
		if (!U.paused) {
			if (A.raw_w - raw_r > RAW_RING) raw_r = A.raw_w - RAW_RING; /* fell behind: drop the oldest */
			for (; raw_r < A.raw_w; raw_r++, nraw++) { unsigned w = raw_r % RAW_RING; rawlen[nraw] = A.raw_len[w]; rawus[nraw] = A.raw_us[w]; memcpy(raw[nraw], A.raw[w], (size_t)rawlen[nraw]); }
		}
		memcpy(ldring, A.ld, sizeof ldring); ldn = A.ld_n;
		if (!U.paused) snap = A.opt;
		nh = A.hist_n < HIST_N ? (int)A.hist_n : HIST_N;
		for (int i = 0; i < nh; i++) hist[i] = A.hist[(A.hist_n - (unsigned)nh + (unsigned)i) % HIST_N];
		if (now - rate_t0 >= 1000000) { for (int k = 0; k < 6; k++) A.rate[k] = __atomic_exchange_n(&A.acc[k], 0, __ATOMIC_RELAXED); rate_t0 = now; U.fps = U.fps_acc; U.fps_acc = 0; }
		pthread_mutex_unlock(&A.lock);
		const struct ldpkt *ldl = ldn ? &ldring[(ldn - 1) % LD_RING] : NULL;
		if (nraw) {
			for (int i = 0; i < nraw; i++) decode_raw(raw[i], rawlen[i], rawus[i]);
			for (int s = 0; s < NVIEW; s++) {
				if (!V[s].w || V[s].seq == U.ib_seq[s]) continue;
				U.ib_seq[s] = V[s].seq;
				U.nib[s] = detect_blobs(&V[s], 128, U.ib[s], MAX_IBLOB);
				int32_t dts; const struct ldpkt *L = ld_nearest(ldring, ldn, V[s].ts, &dts);
				if (L && (abs(dts) < 25000 || (L->host_us <= V[s].host_us + 40000 && L->host_us + 40000 >= V[s].host_us))) vote_map(s, L);
			}
		}
		{ /* stderr diagnostic every 5 s: blobs and fits per frame parity (does each hand flash only on its own frames?) */
			static uint64_t last; 
			if (now - last > 5000000 && nh > 30) {
				double nb[2] = { 0, 0 }; int cnt[2] = { 0, 0 }, fit[2][2] = { { 0, 0 }, { 0, 0 } };
				for (int i = nh - 300 > 0 ? nh - 300 : 0; i < nh; i++) { int p = hist[i].par; if (p < 0) continue; nb[p] += hist[i].nray; cnt[p]++; for (int h = 0; h < 2; h++) fit[p][h] += hist[i].ok[h]; }
				struct ldpkt lp[2] = { 0 }; bool have[2] = { 0, 0 };
				for (unsigned i = ldn > LD_RING ? ldn - LD_RING : 0; i < ldn; i++) { const struct ldpkt *q = &ldring[i % LD_RING]; lp[q->frame & 1] = *q; have[q->frame & 1] = true; }
				bool live[2]; double ema[2]; int imq[2]; double sp[2];
				pthread_mutex_lock(&O.lock); for (int h = 0; h < 2; h++) { live[h] = O.mux_live[h]; ema[h] = O.mux_ema[h]; imq[h] = O.imf[h].quality; sp[h] = O.imf[h].spread; } pthread_mutex_unlock(&O.lock);
				fprintf(stderr, "[%8.3f] mux: even frames %d avg %.1f blobs (fits L %d R %d) | odd frames %d avg %.1f blobs (fits L %d R %d) | latest even cams %d %d %d %d, odd cams %d %d %d %d | mux L %s %.2f R %s %.2f | imu frame L q%d %.0fdeg R q%d %.0fdeg\n",
					(now - app_t0) * 1e-6, cnt[0], cnt[0] ? nb[0] / cnt[0] : 0, fit[0][0], fit[0][1], cnt[1], cnt[1] ? nb[1] / cnt[1] : 0, fit[1][0], fit[1][1],
					have[0] ? lp[0].ncam[0] : -1, have[0] ? lp[0].ncam[1] : -1, have[0] ? lp[0].ncam[2] : -1, have[0] ? lp[0].ncam[3] : -1, have[1] ? lp[1].ncam[0] : -1, have[1] ? lp[1].ncam[1] : -1, have[1] ? lp[1].ncam[2] : -1, have[1] ? lp[1].ncam[3] : -1,
					live[0] ? "live" : "dead", ema[0], live[1] ? "live" : "dead", ema[1], imq[0], sp[0], imq[1], sp[1]);
				last = now;
			}
		}
		/* head pose now (world <- cam0) for the published-pose markers */
		quat hq = QID; double hp[3] = { 0, 0, 0 }; bool head_ok = false;
		pthread_mutex_lock(&H.lock);
		if (H.slam_ok && H.sring_n) { unsigned i = (H.sring_n - 1) % SLAM_RING; hq = H.sring[i].q; memcpy(hp, H.sring[i].p, sizeof hp); head_ok = true; }
		pthread_mutex_unlock(&H.lock);
		double R_wc[9], R_cw[9], hr[9], p_wc[3];
		m3_from_q(qmul(hq, O.q_hc), R_wc);
		for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) R_cw[i * 3 + j] = R_wc[j * 3 + i];
		m3_from_q(hq, hr); m3_mulv(hr, O.p_hc, p_wc); for (int k = 0; k < 3; k++) p_wc[k] += hp[k];
		DriverPose_t pub[2]; bool pub_ok[2];
		pthread_mutex_lock(&F.lock);
		for (int h = 0; h < 2; h++) { pub_ok[h] = S[h].added && S[h].id < 4 && F.dev[S[h].id].pose_us; if (pub_ok[h]) pub[h] = F.dev[S[h].id].pose; }
		pthread_mutex_unlock(&F.lock);

		/* draw */
		for (int i = 0; i < CW * CH; i++) cv[i] = C_BG;
		for (int q = 0; q < 4; q++) {
			int slot = -1;
			for (int s = 0; s < NVIEW; s++) if (U.map[s] == q) slot = s;
			draw_view(q, slot, ldring, ldn, &snap, R_cw, p_wc, head_ok, pub, pub_ok, now);
		}
		draw_panel(&snap, hist, nh, ldl, ldring, ldn, now, pub, pub_ok);
		calflow_draw(now);
		if (U.help) draw_help();
		SDL_UpdateTexture(tex, NULL, cv, CW * 4);
		SDL_RenderClear(ren);
		SDL_RenderCopy(ren, tex, NULL, NULL);
		SDL_RenderPresent(ren);
		hmdout_submit(cv, CW, calflow_overlay(now)); /* the presentation thread warps it into the headset at <= 30 Hz, HUD overlay on top */
		{ /* lens/camera geometry changed (tunables, headset calibration arrived): rebuild the see-through LUT */
			static double sig; static uint64_t last; 
			if (now - last > 500000) {
				double g = T.cam_x * 1e3 + T.cam_y * 1e4 + T.cam_z * 1e5 + T.cam_yaw * 7 + T.cam_pitch * 11 + T.cam_roll * 13 + T.ld_xoff * 17 + T.ld_yoff * 19 + (O.cam[0].ok + 2 * O.cam[1].ok) * 1e6;
				for (int i = 0; i < 8; i++) g += H.calib[i] * (100 + i);
				if (g != sig) { sig = g; if (HO.active) hmdout_relut(); }
				last = now;
			}
		}
		U.fps_acc++;
		if (U.autoshot_s > 0 && now - app_t0 > (uint64_t)(U.autoshot_s * 1e6)) { save_snapshot(ren); fprintf(stderr, "autoshot: %d fps\n", U.fps); g_quit = 1; }
		uint64_t spent = now_us() - now;
		if (spent < 15000) SDL_Delay((Uint32)((15000 - spent) / 1000));
	}
	fk_log(NULL, "test: shutting down");
	((ovr_provider_Cleanup)prov->vt[1])(prov);
	hmdout_stop();
	SDL_DestroyTexture(tex); SDL_DestroyRenderer(ren); SDL_DestroyWindow(win); SDL_Quit();
	return 0;
}
