/* slopvr — PlayStation VR2 driver for SteamVR on Linux. Pure C, single .so.
 *
 * Unity build: this file pulls in hmd.c (headset: display, 6DoF, gaze) and
 * sense.c (Sense controllers over hidraw). Everything talks to vrserver via
 * the C++ vtable ABI declared in ovr.h. */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#ifndef SLOP_DEBUG
#include "glibc.h" /* the driver build binds to glibc 2.31 era symbol versions (Steam runtime); the test tool runs on the host */
#endif
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <ctype.h>
#include <time.h>
#include <unistd.h>

#include "ovr.h"

/* Build flavour.  SLOP_DEBUG (the test tool, `./build.sh test`) keeps the per-frame diagnostics: optical dump logging, packet dumps to
 * disk, the periodic tracker state dumps, session recording on by default.  The driver build (production) compiles those paths
 * out: it logs events only, records only when "record" is set in steamvr.vrsettings, and runs with -O2 -DNDEBUG. */
#ifdef SLOP_DEBUG
#define SLOP_DBG 1
#else
#define SLOP_DBG 0
#endif

/* Memory: one arena for everything that lives as long as the driver (USB buffers, lookup tables, canvases), thread-local scratch
 * arenas for temporaries (file reads, snapshots).  Nothing is freed individually; the arena is released when the driver unloads. */
#include "arena.h"

#define SLOP_VERSION "0.1.0"
#define SECTION "driver_slopvr"
#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))
#define CLAMP(x, lo, hi) ((x) < (lo) ? (lo) : (x) > (hi) ? (hi) : (x))
#define DEG(x) ((x) * (M_PI / 180.0))

static ovr_obj *g_host, *g_props, *g_settings, *g_input, *g_log;

static void rec_text(const char *line);
static void slog(const char *fmt, ...);

static struct arena *g_mem; static pthread_mutex_t g_mem_lock = PTHREAD_MUTEX_INITIALIZER;
static void *mem_alloc(size_t n) /* zeroed, lives until the driver unloads; thread-safe */
{
	pthread_mutex_lock(&g_mem_lock);
	if (!g_mem) g_mem = arena_create((size_t)1 << 30, 4u << 20); /* 1 GiB reserved, committed 4 MiB at a time */
	void *p = g_mem ? arena_push_zero(g_mem, n) : NULL;
	pthread_mutex_unlock(&g_mem_lock);
	if (!p) { slog("mem: arena allocation of %zu bytes failed", n); abort(); }
	return p;
}
static char *mem_strdup(const char *s) { size_t n = strlen(s) + 1; char *d = mem_alloc(n); memcpy(d, s, n); return d; }
static void mem_release(void) { pthread_mutex_lock(&g_mem_lock); if (g_mem) { arena_destroy(g_mem); g_mem = NULL; } pthread_mutex_unlock(&g_mem_lock); }
/* scratch: struct arena_temp t = mem_scratch(); ... mem_scratch_end(t); (per thread, nests, 64 MiB) */
static struct arena_temp mem_scratch(void) { return arena_scratch_begin(); }
static void mem_scratch_end(struct arena_temp t) { arena_scratch_end(t); }

static void slog(const char *fmt, ...)
{
	char b[512];
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(b, sizeof b - 2, fmt, ap);
	va_end(ap);
	if (n < 0) return;
	if (n > (int)sizeof b - 2) n = sizeof b - 2;
	b[n] = '\n'; b[n + 1] = 0;
	if (g_log) ovr_log_Log(g_log, b); else fputs(b, stderr);
	b[n] = 0; rec_text(b);
}

static uint64_t now_us(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC_RAW, &ts); /* RAW: NTP frequency slews would show up as drift of every device clock we model */
	return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

/* ---- properties / settings ---- */
static void prop_set(uint64_t c, int32_t prop, uint32_t tag, const void *buf, uint32_t size)
{
	PropertyWrite_t w = { prop, PropertyWrite_Set, 0, (void *)buf, size, tag, 0 };
	ovr_props_WritePropertyBatch(g_props, c, &w, 1);
}
static void prop_str(uint64_t c, int32_t p, const char *s) { prop_set(c, p, k_unStringPropertyTag, s, (uint32_t)strlen(s) + 1); }
static void prop_f32(uint64_t c, int32_t p, float v) { prop_set(c, p, k_unFloatPropertyTag, &v, sizeof v); }
static void prop_i32(uint64_t c, int32_t p, int32_t v) { prop_set(c, p, k_unInt32PropertyTag, &v, sizeof v); }
static void prop_u64(uint64_t c, int32_t p, uint64_t v) { prop_set(c, p, k_unUint64PropertyTag, &v, sizeof v); }
static void prop_bool(uint64_t c, int32_t p, bool v) { prop_set(c, p, k_unBoolPropertyTag, &v, sizeof v); }

static int32_t cfg_i32(const char *k, int32_t def) { int32_t e = 0; int32_t v = ovr_settings_GetInt32(g_settings, SECTION, k, &e); return e ? def : v; }
static float cfg_f32(const char *k, float def) { int32_t e = 0; float v = ovr_settings_GetFloat(g_settings, SECTION, k, &e); return e ? def : v; }
static bool cfg_bool(const char *k, bool def) { int32_t e = 0; bool v = ovr_settings_GetBool(g_settings, SECTION, k, &e); return e ? def : v; }

/* ---- tiny quaternion toolkit (w,x,y,z; q maps body -> world) ---- */
typedef HmdQuaternion_t quat;
static const quat QID = { 1, 0, 0, 0 };

static quat qmul(quat a, quat b)
{
	return (quat){ a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
		       a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
		       a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
		       a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w };
}
static quat qconj(quat q) { return (quat){ q.w, -q.x, -q.y, -q.z }; }
static quat qnorm(quat q)
{
	double n = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	if (n < 1e-12) return QID;
	return (quat){ q.w / n, q.x / n, q.y / n, q.z / n };
}
/* quaternion from rotation vector (axis * angle) */
static quat qrotvec(double x, double y, double z)
{
	double a = sqrt(x * x + y * y + z * z);
	if (a < 1e-9) return (quat){ 1, x * 0.5, y * 0.5, z * 0.5 };
	double s = sin(a * 0.5) / a;
	return (quat){ cos(a * 0.5), x * s, y * s, z * s };
}
static quat qaxis(double x, double y, double z, double deg) { double a = DEG(deg); return qrotvec(x * a, y * a, z * a); }
static double qangle(quat a, quat b) { double d = fabs(a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z); return 2 * acos(d > 1 ? 1 : d); }
static quat qnlerp(quat a, quat b, double k) /* shortest-path normalized lerp */
{
	if (a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z < 0) b = (quat){ -b.w, -b.x, -b.y, -b.z };
	return qnorm((quat){ a.w + k * (b.w - a.w), a.x + k * (b.x - a.x), a.y + k * (b.y - a.y), a.z + k * (b.z - a.z) });
}
static void qrot(quat q, const double v[3], double o[3])
{
	quat p = { 0, v[0], v[1], v[2] };
	quat r = qmul(qmul(q, p), qconj(q));
	o[0] = r.x; o[1] = r.y; o[2] = r.z;
}
static double qyaw(quat q) /* heading of -Z axis around Y */
{
	double f[3], fwd[3] = { 0, 0, -1 };
	qrot(q, fwd, f);
	return atan2(-f[0], -f[2]);
}
static HmdMatrix34_t mat_from(quat q, const double p[3])
{
	HmdMatrix34_t m;
	double xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z, xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z,
	       wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
	m.m[0][0] = (float)(1 - 2 * (yy + zz)); m.m[0][1] = (float)(2 * (xy - wz)); m.m[0][2] = (float)(2 * (xz + wy)); m.m[0][3] = (float)p[0];
	m.m[1][0] = (float)(2 * (xy + wz)); m.m[1][1] = (float)(1 - 2 * (xx + zz)); m.m[1][2] = (float)(2 * (yz - wx)); m.m[1][3] = (float)p[1];
	m.m[2][0] = (float)(2 * (xz - wy)); m.m[2][1] = (float)(2 * (yz + wx)); m.m[2][2] = (float)(1 - 2 * (xx + yy)); m.m[2][3] = (float)p[2];
	return m;
}

static DriverPose_t pose_default(void)
{
	DriverPose_t p = { 0 };
	p.qWorldFromDriverRotation = QID;
	p.qDriverFromHeadRotation = QID;
	p.qRotation = QID;
	p.result = TrackingResult_Uninitialized;
	return p;
}

/* Every pose goes through here: a non-finite or absurd value (a NaN from an uninitialised quaternion, an inf velocity from a
 * zero time step) must never reach vrserver - the compositor positions its overlays from these poses and aborts on the
 * resulting non-finite transforms ("Vulkan transfer ring buffer overflow").  Such a pose is published as invalid instead. */
static void pose_submit(uint32_t id, DriverPose_t *p)
{
	bool ok = true; static uint64_t warned_us[8];
	const double *v[] = { p->vecPosition, p->vecVelocity, p->vecAngularVelocity, p->vecWorldFromDriverTranslation, p->vecDriverFromHeadTranslation };
	for (size_t k = 0; k < sizeof v / sizeof v[0] && ok; k++) for (int i = 0; i < 3; i++) if (!isfinite(v[k][i]) || fabs(v[k][i]) > (k == 1 ? 50.0 : k == 2 ? 200.0 : 100.0)) ok = false;
	const quat *q[] = { &p->qRotation, &p->qWorldFromDriverRotation, &p->qDriverFromHeadRotation };
	for (size_t k = 0; k < 3 && ok; k++) { double n = q[k]->w * q[k]->w + q[k]->x * q[k]->x + q[k]->y * q[k]->y + q[k]->z * q[k]->z; if (!isfinite(n) || n < 0.9 || n > 1.1) ok = false; }
	if (!ok) {
		uint64_t now = now_us(); unsigned slot = id < 8 ? id : 7;
		if (now - warned_us[slot] > 60000000) { warned_us[slot] = now; slog("pose: device %u produced a non-finite or absurd pose (pos %g %g %g vel %g %g %g q %g %g %g %g): published as invalid", id, p->vecPosition[0], p->vecPosition[1], p->vecPosition[2], p->vecVelocity[0], p->vecVelocity[1], p->vecVelocity[2], p->qRotation.w, p->qRotation.x, p->qRotation.y, p->qRotation.z); }
		DriverPose_t d = pose_default(); d.deviceIsConnected = p->deviceIsConnected; d.result = TrackingResult_Running_OutOfRange; d.poseIsValid = false;
		ovr_host_TrackedDevicePoseUpdated(g_host, id, &d);
		return;
	}
	ovr_host_TrackedDevicePoseUpdated(g_host, id, p);
}

/* Latest head pose shared with the controller arm model. */
static pthread_mutex_t g_head_lock = PTHREAD_MUTEX_INITIALIZER;
static quat g_head_q = { 1, 0, 0, 0 };
static double g_head_p[3];
static bool g_head_valid;

/* debug taps (psvr2linuxtest.c sets them; NULL in the driver build): every reaped USB packet, and the
 * optical worker's per-frame result (poses in the cam0 frame, O.rays/O.cand/O.cl valid during the call) */
struct opt_pose;
struct opt_frame_dbg { uint32_t ts; uint64_t now, t_exp; quat hq; double hp[3]; const struct opt_pose *P; const bool *ok, *acq; uint32_t frame; };
static void (*g_usb_hook)(uint8_t ep, const uint8_t *b, int len);
static void (*g_opt_hook)(const struct opt_frame_dbg *f);

/* optical.c (between hmd.c and sense.c) needs the headset pose; hmd.c feeds it packets */
static void optical_on_ld(const uint8_t *b, int len);
static void optical_hmd_clock(uint32_t dev_us, uint64_t now);
static void opt_set_calib(const uint8_t *b, int len);
static bool optical_calib_ok(void);
static uint64_t optical_last_ld_us(void);

#include "tune.c"
#include "rec.c"
#include "hmd.c"
#include "optical.c"
#include "sense.c"

/* ================= IServerTrackedDeviceProvider_004 ================= */
static int32_t prov_Init(void *self, ovr_obj *ctx)
{
	(void)self;
	int32_t err = 0;
	g_log = ovr_ctx_GetGenericInterface(ctx, OVR_STR_IVRDriverLog, &err);
	g_host = ovr_ctx_GetGenericInterface(ctx, OVR_STR_IVRServerDriverHost, &err);
	g_props = ovr_ctx_GetGenericInterface(ctx, OVR_STR_IVRProperties, &err);
	g_settings = ovr_ctx_GetGenericInterface(ctx, OVR_STR_IVRSettings, &err);
	g_input = ovr_ctx_GetGenericInterface(ctx, OVR_STR_IVRDriverInput, &err);
	if (!g_host || !g_props || !g_settings || !g_input) return VRInitError_Init_InterfaceNotFound;
	slog("slopvr " SLOP_VERSION " init (%s build)", SLOP_DBG ? "debug" : "release");

	/* Wayland/NVIDIA: DP link-training path 1 is the one that modesets the
	 * 4000x2040 DSC panel correctly on a leased connector. */
	if (cfg_bool("forceDisplayPortTrainingMode", true)) {
		int32_t e = 0;
		ovr_settings_SetInt32(g_settings, "steamvr", "displayPortTrainingMode", 1, &e);
	}

	tune_init();
	rec_open(cfg_bool("record", SLOP_DBG), cfg_i32("recordMaxMb", 400)); /* the session file for offline replay: on by default in the tool, opt-in ("record": true) in the driver */
	{ char t[2048]; int k = 0; for (size_t i = 0; i < ARRAY_LEN(params) && k < (int)sizeof t - 40; i++) k += snprintf(t + k, sizeof t - k, "%s%s=%g", i ? " " : "", params[i].name, *params[i].v); rec_write(REC_TUNE, 0, now_us(), t, (size_t)k); }
	optical_init();
	hmd_init();
	sense_init();
	return VRInitError_None;
}

static void prov_Cleanup(void *self)
{
	(void)self;
	sense_cleanup();
	hmd_cleanup();
	optical_cleanup();
	slog("slopvr cleanup");
	rec_close();
	g_log = NULL;
#ifndef SLOP_DEBUG
	mem_release(); /* the driver's arena; the test tool keeps it until the process ends (hmdout still reads the canvases after this) */
#endif
}

static const char *const k_interface_versions[] = {
	OVR_STR_IVRSettings, OVR_STR_ITrackedDeviceServerDriver, OVR_STR_IVRDisplayComponent,
	OVR_STR_IServerTrackedDeviceProvider, OVR_STR_IVRDriverInput, OVR_STR_IVRProperties,
	OVR_STR_IVRServerDriverHost, OVR_STR_IVRDriverLog, NULL
};
static const char *const *prov_GetInterfaceVersions(void *self) { (void)self; return k_interface_versions; }

static void prov_RunFrame(void *self)
{
	(void)self;
	VREvent_t ev;
	while (ovr_host_PollNextEvent(g_host, &ev)) {
		if (ev.eventType == VREvent_Input_HapticVibration) {
			VREvent_HapticVibration_t h;
			memcpy(&h, ev.data, sizeof h);
			sense_haptic(h.containerHandle, h.fDurationSeconds, h.fFrequency, h.fAmplitude);
		} else if (ev.eventType == VREvent_SteamVRSectionSettingChanged) {
			hmd_settings_changed();
		}
	}
	hmd_frame();
	sense_frame();
}

static bool prov_ShouldBlockStandbyMode(void *self) { (void)self; return false; }
static void prov_EnterStandby(void *self) { (void)self; }
static void prov_LeaveStandby(void *self) { (void)self; }

OVR_VTABLE(prov_vt, OVR_FN(ovr_provider_Init, prov_Init), OVR_FN(ovr_provider_Cleanup, prov_Cleanup),
	   OVR_FN(ovr_provider_GetInterfaceVersions, prov_GetInterfaceVersions), OVR_FN(ovr_provider_RunFrame, prov_RunFrame),
	   OVR_FN(ovr_provider_ShouldBlockStandbyMode, prov_ShouldBlockStandbyMode),
	   OVR_FN(ovr_provider_EnterStandby, prov_EnterStandby), OVR_FN(ovr_provider_LeaveStandby, prov_LeaveStandby));
static ovr_obj g_provider = { 0 };

__attribute__((visibility("default"))) void *HmdDriverFactory(const char *name, int32_t *err)
{
	if (strcmp(name, OVR_STR_IServerTrackedDeviceProvider) == 0) {
		g_provider.vt = prov_vt;
		if (err) *err = VRInitError_None;
		return &g_provider;
	}
	if (err) *err = VRInitError_Init_InterfaceNotFound;
	return NULL;
}
