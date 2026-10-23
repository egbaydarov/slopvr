/* slopvr — PSVR2 Sense controllers over hidraw (Bluetooth or USB).
 * Inputs, battery, 3DoF IMU orientation + arm model, PCM haptics at 3 kHz.
 * Exposed to SteamVR as "oculus_touch" so every app binding works unchanged. */
#include <linux/hidraw.h>

#define SENSE_VID 0x054C
#define SENSE_PID_L 0x0E45
#define SENSE_PID_R 0x0E46
#define PCM_RATE 3000
#define PCM_CHUNK 32
#define OUT_PERIOD_US 10667 /* 32 samples @ 3 kHz */
#define IMU_TICK_S (1.0 / 3e6)

enum { HAND_L, HAND_R };

/* feature report 0x05 (two parts) -> 116 B calibration blob, offsets per Monado */
struct __attribute__((packed)) sense_calib {
	int16_t acc_px, u0[2], acc_mx, u1[3], acc_py, u2[2], acc_my, u3[3], acc_pz, u4[2], acc_mz;
	int16_t gyro_bias[3], gyro_px, u5[2], gyro_mx, u6[3], gyro_py, u7[2], gyro_my, u8[3], gyro_pz, u9[2], gyro_mz;
	int16_t ref1, ref2;
};
static_assert(offsetof(struct sense_calib, acc_mx) == 0x06 && offsetof(struct sense_calib, acc_py) == 0x0E &&
	      offsetof(struct sense_calib, acc_pz) == 0x1C && offsetof(struct sense_calib, gyro_bias) == 0x24 &&
	      offsetof(struct sense_calib, gyro_px) == 0x2A && offsetof(struct sense_calib, gyro_mz) == 0x4C &&
	      offsetof(struct sense_calib, ref2) == 0x50, "sense calib layout");

struct __attribute__((packed)) sense_in { /* common input payload (63 B) */
	uint8_t tx, ty, trigger, trigger_prox, squeeze_prox, u1[2], btn[3], u2;
	uint32_t seq; int16_t gyro[3], accel[3]; uint32_t imu_ticks;
	uint8_t temp, u3[7], trig_fb_state, trig_fb_mode, battery, plug;
	uint32_t host_ts, dev_ts; uint8_t u4[4], cmac[8];
};
struct __attribute__((packed)) sense_out_settings { /* 38 B */
	uint8_t flag1, flag2, vib_amp, u0, trig_mode, trig_param[10];
	uint32_t host_ts;
	uint8_t led_phase, led_seq, led_period; uint32_t led_pos, led_len; uint8_t led_blink[4];
	uint8_t haptic_reduction, status_led, u1[2];
};
static_assert(sizeof(struct sense_in) == 63 && sizeof(struct sense_out_settings) == 38, "sense layout");

struct haptic { float freq, amp; int samples_left; double phase; };

static struct sense {
	const ovr_fn *vt;
	int hand;
	uint32_t id;
	uint64_t container;
	atomic_bool active;
	bool added;
	int fd, usb;
	char serial[64], node[32];

	/* input components */
	uint64_t h_sys, h_a, h_a_t, h_b, h_b_t, h_trig, h_trig_t, h_grip, h_grip_t, h_jx, h_jy, h_jc, h_jt, h_haptic;
	uint8_t btn[3], battery, last_trigger, last_tx, last_ty, last_prox[2];
	unsigned tx_n, tx_fail; uint64_t tx_log_us;
	bool charging; float charge;

	/* IMU fusion (world <- IMU frame) */
	struct { float gs[3], as[3], ab[3]; int16_t gb[3]; bool ok; } cal;
	quat q; bool q_ok; uint32_t ticks; uint64_t last_in_us; double gyro_w[3];
	double gbo[3]; bool gbo_ok; double still_s; unsigned gbo_n; /* online gyro bias (LSB): the factory value is 1-3 LSB off (0.2-0.4 deg/s, 5-15 deg/min of yaw drift); learnt whenever the controller rests */
	uint64_t ps_down_us; bool yaw_aligned;
	quat qc; bool qc_ok; uint64_t qc_us; /* optical orientation correction (world <- IMU world), smoothed */
	double opt_p[3]; bool opt_p_ok; /* last optical position (hold-over while occluded) */
	double pub_p[3]; bool pub_ok;   /* last published (smoothed) position */
	/* position source: 0 optical, 1 IMU dead-reckoning from the last fix, 2 hold, 3 arm model.  A source change blends the published
	 * position over ~0.25 s instead of jumping; dead-reckoning integrates the IMU's linear acceleration (gravity removed with the fused
	 * orientation) with a damped velocity so a short occlusion keeps the motion and a long one settles instead of drifting away */
	int pos_mode; uint64_t blend_until_us, pub_us, dr_us; double dr_p[3], dr_v[3], acc_w[3]; bool acc_ok; unsigned pub_n;
	/* position filter: per axis a constant-velocity Kalman filter driven by the IMU's linear acceleration (world frame, gravity
	 * removed) at the report rate and corrected by the optical fixes (30 Hz per hand, ~40 ms old when they arrive: the innovation is
	 * taken against the state the filter had at the exposure time, kept in a short history).  Publishing the raw fix extrapolated by
	 * its velocity stepped 1-3 mm at every fix arrival, which shows as a 30 Hz shake of the hand */
	struct { double p, v, P00, P01, P11; } kf[3]; bool kf_ok; uint64_t kf_us, kf_fix_us; double kf_motion; /* 0 calm .. 1 moving (IMU), scales the process noise: a calm hand averages the fix noise away, a moving one follows the fixes */
	struct { uint64_t t; double p[3]; } kh[64]; unsigned kh_n;

	/* output */
	uint8_t seq, counter;
	struct haptic hap;
	pthread_mutex_t hap_lock;
	uint64_t next_out_us;
} S[2];

static atomic_bool g_sense_run;
static pthread_t g_sense_thread;
static uint32_t g_crc_tab[256];

/* ---- CRC32 (IEEE, reflected) over a 1-byte seed + the report ---- */
static uint32_t sense_crc(uint8_t seed, const uint8_t *p, size_t n)
{
	uint32_t c = 0xFFFFFFFFu;
	c = g_crc_tab[(c ^ seed) & 0xFF] ^ (c >> 8);
	for (size_t i = 0; i < n; i++) c = g_crc_tab[(c ^ p[i]) & 0xFF] ^ (c >> 8);
	return c ^ 0xFFFFFFFFu;
}

/* ---- device discovery via /sys/class/hidraw ---- */
static bool sense_find(int hand, char *node, size_t nlen, char *serial, size_t slen, int *usb)
{
	DIR *d = opendir("/sys/class/hidraw");
	struct dirent *e;
	bool found = false;
	char want[32];
	snprintf(want, sizeof want, ":%08X:%08X", SENSE_VID, hand == HAND_L ? SENSE_PID_L : SENSE_PID_R);
	if (!d) return false;
	while (!found && (e = readdir(d))) {
		char path[300], line[256];
		if (strncmp(e->d_name, "hidraw", 6)) continue;
		snprintf(path, sizeof path, "/sys/class/hidraw/%s/device/uevent", e->d_name);
		FILE *f = fopen(path, "r");
		if (!f) continue;
		unsigned bus = 0; bool match = false; serial[0] = 0;
		while (fgets(line, sizeof line, f)) {
			if (!strncmp(line, "HID_ID=", 7) && !strncmp(line + 11, want, strlen(want))) { bus = (unsigned)strtoul(line + 7, NULL, 16); match = true; }
			else if (!strncmp(line, "HID_UNIQ=", 9)) { snprintf(serial, slen, "%s", line + 9); serial[strcspn(serial, "\n")] = 0; }
		}
		fclose(f);
		/* also match on the other node of a multi-node device: skip those we already use */
		if (match && strcmp(e->d_name, S[!hand].node) != 0) {
			snprintf(node, nlen, "%s", e->d_name);
			*usb = bus == 3;
			found = true;
		}
	}
	closedir(d);
	return found;
}

static bool sense_read_calibration(struct sense *s)
{
	uint8_t blob[116] = { 0 };
	for (int attempt = 0; attempt < 4; attempt++) {
		bool got[2] = { false, false };
		for (int part = 0; part < 2; part++) {
			uint8_t r[64] = { 0x05 };
			if (ioctl(s->fd, HIDIOCGFEATURE(sizeof r), r) < 0) return false;
			uint32_t want; memcpy(&want, r + 60, 4);
			if (!s->usb && sense_crc(0xA3, r, 60) != want) break;
			if (r[1] == 0x00) { memcpy(blob, r + 2, 58); got[0] = true; }
			else if (r[1] == 0x81) { memcpy(blob + 58, r + 2, 58); got[1] = true; }
		}
		if (got[0] && got[1]) break;
		if (attempt == 3) return false;
	}
	const struct sense_calib *c = (const void *)blob;
	double ref = (c->ref1 + c->ref2) * 0.5 * M_PI / 180.0;
	if (!(ref > 0)) ref = 3 * M_PI;
	int16_t gp[3] = { c->gyro_px, c->gyro_py, c->gyro_pz }, gm[3] = { c->gyro_mx, c->gyro_my, c->gyro_mz };
	int16_t ap[3] = { c->acc_px, c->acc_py, c->acc_pz }, am[3] = { c->acc_mx, c->acc_my, c->acc_mz };
	for (int i = 0; i < 3; i++) {
		double span = (fabs((double)gp[i] - c->gyro_bias[i]) + fabs((double)gm[i] - c->gyro_bias[i])) * 0.5;
		s->cal.gb[i] = c->gyro_bias[i];
		s->cal.gs[i] = (float)(span > 0 ? ref / span : 2000.0 / 32767.0 * M_PI / 180.0);
		s->cal.ab[i] = (float)((ap[i] + am[i]) * 0.5);
		double d = ap[i] - s->cal.ab[i];
		s->cal.as[i] = (float)(fabs(d) > 1 ? 9.80665 / d : 9.80665 / 8192.0);
	}
	s->cal.ok = true;
	rec_sensecal(s->hand, s->cal.gs, s->cal.as, s->cal.ab, s->cal.gb);
	return true;
}

static void sense_set_polling(struct sense *s)
{
	uint8_t r[48] = { 0x08, 0x0E, 0x0C, 0x00, 0x02, 0x00 };
	uint32_t crc = sense_crc(0x53, r, 44);
	memcpy(r + 44, &crc, 4);
	ioctl(s->fd, HIDIOCSFEATURE(sizeof r), r); /* best effort: 133 Hz "PC" rate */
}

/* ---- inputs ---- */
static void upd_b(uint64_t h, bool v) { if (h) ovr_input_UpdateBooleanComponent(g_input, h, v, 0); }
static void upd_f(uint64_t h, float v) { if (h) ovr_input_UpdateScalarComponent(g_input, h, v, 0); }

/* IMU body frame -> OpenVR controller (grip) frame. Measured on hardware with gravity: held naturally the
 * accelerometer reads ~(0, 0.22, 0.97) g, pointed at the ceiling ~(0, 0.93, 0.35) g, identical on both hands.
 * So the controller points along IMU +Y, its up is IMU +Z tilted 13 deg toward +Y, right is IMU +X:
 * a rotation about X by 90 - 13 = 77 deg. driver_slopvr.gripPitchDeg adds a user pitch offset. */
static quat sense_world_q(const struct sense *s) { return s->qc_ok && T.qc_on != 0 ? qnorm(qmul(s->qc, s->q)) : s->q; } /* world <- IMU, optically corrected */
static quat sense_grip_q(const struct sense *s)
{
	quat off = qmul(qmul(qaxis(1, 0, 0, 77.0 + T.grip_pitch), qaxis(0, 1, 0, T.grip_yaw)), qaxis(0, 0, 1, T.grip_roll));
	return qnorm(qmul(sense_world_q(s), off));
}

static void sense_submit_pose(struct sense *s)
{
	if (!atomic_load(&s->active)) return;
	DriverPose_t p = pose_default();
	uint64_t now = now_us();
	p.deviceIsConnected = s->fd >= 0;
	p.poseIsValid = s->q_ok;
	p.result = s->q_ok ? TrackingResult_Running_OK : TrackingResult_Calibrating_InProgress;
	p.willDriftInYaw = !(s->qc_ok && T.qc_on != 0);
	p.poseTimeOffset = T.pose_time_ms * 1e-3;
	if (s->q_ok) {
		double sx = s->hand == HAND_L ? -1.0 : 1.0; /* OpenVR +x is right */
		quat q = sense_grip_q(s);
		p.qRotation = q;
		if (s->qc_ok && T.qc_on != 0) qrot(s->qc, s->gyro_w, p.vecAngularVelocity); else memcpy(p.vecAngularVelocity, s->gyro_w, sizeof p.vecAngularVelocity);

		struct opt_fix f = optical_fix(s->hand);
		uint64_t age = f.valid ? now - f.host_us : UINT64_MAX;
		double dt_pub = s->pub_us ? CLAMP((now - s->pub_us) * 1e-6, 0.0, 0.05) : 0; s->pub_us = now;
		double target[3], vel[3] = { 0, 0, 0 }; int mode;
		/* the position filter runs whenever it has a state: predicted with the IMU every report, corrected by each new optical fix */
		bool have_fix = age < 70000; /* a fix within ~2 LD frames */
		if (s->kf_ok && (now - s->kf_us > 500000 || (!have_fix && age > (uint64_t)(T.hold_s * 1e6)))) s->kf_ok = false; /* stale: start again at the next fix */
		if (have_fix && (!s->kf_ok || f.host_us - s->kf_fix_us > 500000)) { /* first fix, or the first after a long gap: start at the fix */
			for (int i = 0; i < 3; i++) { s->kf[i].p = f.p[i] + f.v[i] * age * 1e-6; s->kf[i].v = f.v[i]; s->kf[i].P00 = 1e-4; s->kf[i].P01 = 0; s->kf[i].P11 = 0.04; }
			s->kf_ok = true; s->kf_fix_us = f.host_us; s->kh_n = 0; s->kf_us = now;
		} else if (s->kf_ok) {
			double dt = now > s->kf_us ? CLAMP((now - s->kf_us) * 1e-6, 0.0, 0.05) : 0;
			{ /* how much is the hand moving, from the IMU (rises at once, decays in ~0.25 s) */
				double an = s->acc_ok ? sqrt(s->acc_w[0] * s->acc_w[0] + s->acc_w[1] * s->acc_w[1] + s->acc_w[2] * s->acc_w[2]) : 0, gn = sqrt(s->gyro_w[0] * s->gyro_w[0] + s->gyro_w[1] * s->gyro_w[1] + s->gyro_w[2] * s->gyro_w[2]);
				double m = CLAMP(an / 4.0 > gn / 3.0 ? an / 4.0 : gn / 3.0, 0.0, 1.0); /* 1 at 4 m/s2 or 170 deg/s */
				s->kf_motion = m > s->kf_motion ? m : s->kf_motion * exp(-dt / 0.12);
			}
			/* process noise: 0.15 m/s2 for a calm hand (the IMU says it is not moving: the 30 Hz fixes get averaged, the residual
			 * shake drops by a third) up to 1.65 in motion (follows the fixes within a frame); fix noise ~6 mm at 7 LEDs, more with
			 * fewer.  Thresholds and gains chosen by re-fusing a recorded Half-Life session offline: same lag in motion as fixed
			 * 1.5 m/s2, 2.3/1.9 mm instead of 3.2/2.7 mm 95th-percentile steps when slow */
			const double sa = 0.15 + 1.5 * s->kf_motion, sig = 0.006 * sqrt(7.0 / (f.n > 3 ? f.n : 3)), R = sig * sig;
			double q_v = sa * sa * dt * dt, q_pv = sa * sa * dt * dt * dt / 2, q_p = sa * sa * dt * dt * dt * dt / 4;
			double damp = have_fix ? 1.0 : exp(-dt / (T.dr_tau_s > 0 ? T.dr_tau_s : 0.3)); /* without fixes the velocity decays: IMU errors cannot run away */
			for (int i = 0; i < 3; i++) { /* predict */
				double a = s->acc_ok ? s->acc_w[i] : 0;
				s->kf[i].p += s->kf[i].v * dt + 0.5 * a * dt * dt; s->kf[i].v = (s->kf[i].v + a * dt) * damp;
				s->kf[i].P00 += 2 * s->kf[i].P01 * dt + s->kf[i].P11 * dt * dt + q_p; s->kf[i].P01 += s->kf[i].P11 * dt + q_pv; s->kf[i].P11 += q_v;
			}
			if (have_fix && f.host_us != s->kf_fix_us) { /* a new fix: innovation against the state at its exposure time */
				s->kf_fix_us = f.host_us;
				double pm[3] = { s->kf[0].p, s->kf[1].p, s->kf[2].p };
				for (unsigned k = 0; k < s->kh_n && k < 64; k++) { const typeof(s->kh[0]) *h = &s->kh[(s->kh_n - 1 - k) % 64]; if (h->t <= f.host_us) break; memcpy(pm, h->p, sizeof pm); }
				for (int i = 0; i < 3; i++) {
					double r = f.p[i] - pm[i], S = s->kf[i].P00 + R, K0 = s->kf[i].P00 / S, K1 = s->kf[i].P01 / S;
					if (fabs(r) > 0.08) { K0 = 1; K1 = 0; s->kf[i].v = f.v[i]; } /* far off (a re-acquisition elsewhere): take the fix, the blend below hides the jump */
					s->kf[i].p += K0 * r; s->kf[i].v += K1 * r;
					s->kf[i].P00 -= K0 * s->kf[i].P00; s->kf[i].P01 -= K0 * s->kf[i].P01; s->kf[i].P11 -= K1 * s->kf[i].P01;
					if (s->kf[i].P00 < 1e-7) s->kf[i].P00 = 1e-7; if (s->kf[i].P11 < 1e-5) s->kf[i].P11 = 1e-5;
				}
			}
			s->kf_us = now;
		}
		if (s->kf_ok) { typeof(s->kh[0]) *h = &s->kh[s->kh_n++ % 64]; h->t = now; for (int i = 0; i < 3; i++) h->p[i] = s->kf[i].p; }
		if (have_fix && s->kf_ok) {
			mode = 0;
			for (int i = 0; i < 3; i++) { target[i] = s->kf[i].p; vel[i] = s->kf[i].v * T.vel_gain; }
			memcpy(s->dr_p, target, sizeof s->dr_p); memcpy(s->dr_v, vel, sizeof s->dr_v); s->dr_us = now;
			s->opt_p_ok = true; memcpy(s->opt_p, target, sizeof s->opt_p);
		} else if (s->opt_p_ok && age < (uint64_t)(T.hold_s * 1e6)) {
			/* no fresh fix: the filter dead-reckons from the last one with the IMU for a short while (the hand keeps moving the way it did),
			 * the velocity damped towards zero, then the position holds */
			double dr_age = (now - s->dr_us) * 1e-6;
			if (s->kf_ok && dr_age < 1.5) { for (int i = 0; i < 3; i++) { s->dr_p[i] = s->kf[i].p; s->dr_v[i] = s->kf[i].v; } }
			else { memset(s->dr_v, 0, sizeof s->dr_v); if (s->kf_ok) for (int i = 0; i < 3; i++) { s->kf[i].p = s->dr_p[i]; s->kf[i].v = 0; } }
			mode = dr_age < 1.5 ? 1 : 2;
			memcpy(target, s->dr_p, sizeof target); memcpy(vel, s->dr_v, sizeof vel);
			/* still Running_OK: the pose is a usable estimate, and flapping the result at every short optical loss makes SteamVR
			 * redraw "tracking lost" overlays non-stop (the compositor died twice today on an overlay transfer-buffer overflow) */
		} else { /* long gone: arm model anchored at the head */
			mode = 3;
			pthread_mutex_lock(&g_head_lock);
			quat hq = g_head_q; double hp[3] = { g_head_p[0], g_head_p[1], g_head_p[2] }; bool hv = g_head_valid;
			pthread_mutex_unlock(&g_head_lock);
			double yaw = hv ? qyaw(hq) : 0.0;
			quat qy = qrotvec(0, yaw, 0);
			double sh[3] = { sx * 0.17, -0.22, -0.02 }, fwd[3] = { 0, -0.10, -0.28 }, hand[3] = { 0, 0, -0.14 }, a[3], b[3], c[3];
			qrot(qy, sh, a); qrot(qy, fwd, b); qrot(q, hand, c);
			for (int i = 0; i < 3; i++) target[i] = (hv ? hp[i] : 0.0) + a[i] + b[i] + c[i];
			s->opt_p_ok = false;
			p.result = TrackingResult_Running_OutOfRange;
		}
		/* publish: a source change (or an optical fix far from where we were) is blended over T.blend_ms instead of jumping */
		bool optical = mode == 0;
		if (s->pub_ok) {
			double d[3] = { target[0] - s->pub_p[0], target[1] - s->pub_p[1], target[2] - s->pub_p[2] };
			bool jump = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) > 0.05;
			/* optical -> dead-reckoning -> hold is continuous by construction; blend when a fix returns somewhere else than the
			 * dead-reckoned position, and when the arm model takes over (or hands back) */
			bool blend = mode == 0 ? (s->pos_mode != 0 && jump) : mode == 3 ? s->pos_mode != 3 : (s->pos_mode == 3);
			if (blend && now > s->blend_until_us) s->blend_until_us = now + (uint64_t)(T.blend_ms * 1000);
			if (now < s->blend_until_us) {
				double k = dt_pub / ((s->blend_until_us - now) * 1e-6 + dt_pub); /* linear blend finishing at blend_until */
				for (int i = 0; i < 3; i++) target[i] = s->pub_p[i] + d[i] * k;
			} else if (optical && T.pos_smooth > 0) for (int i = 0; i < 3; i++) target[i] = T.pos_smooth * s->pub_p[i] + (1 - T.pos_smooth) * target[i];
		}
		s->pos_mode = mode;
		memcpy(p.vecPosition, target, sizeof p.vecPosition); memcpy(p.vecVelocity, vel, sizeof p.vecVelocity);
		memcpy(s->pub_p, target, sizeof s->pub_p); s->pub_ok = true;
		if (++s->pub_n % 4 == 0) rec_pub(s->hand, target, q, vel, mode, now);
		(void)optical;
		{ /* tunable pose offset in the controller's own frame */
			double o[3] = { T.grip_x, T.grip_y, T.grip_z }, w[3]; qrot(q, o, w);
			for (int i = 0; i < 3; i++) p.vecPosition[i] += w[i];
		}
	}
	pose_submit(s->id, &p);
}

static void sense_fuse(struct sense *s, const struct sense_in *in, uint64_t now)
{
	double dt = 0;
	if (s->q_ok) {
		int32_t dticks = (int32_t)(in->imu_ticks - s->ticks);
		dt = dticks > 0 ? dticks * IMU_TICK_S : 0;
		if (dt > 0.05) dt = 0.05;
	}
	s->ticks = in->imu_ticks;
	double g[3], a[3];
	if (!s->gbo_ok) { for (int i = 0; i < 3; i++) s->gbo[i] = s->cal.gb[i]; s->gbo_ok = true; }
	for (int i = 0; i < 3; i++) {
		g[i] = (in->gyro[i] - s->gbo[i]) * s->cal.gs[i];
		a[i] = (in->accel[i] - s->cal.ab[i]) * s->cal.as[i];
	}
	{ /* at rest (all rates within 1.5 deg/s of the current bias, |a| within 3 % of 1 g, for 0.5 s) the raw gyro reading is the bias:
	   * track it with a 3 s time constant, within 12 LSB of the factory value.  A hand never holds a controller that still */
		double an = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
		bool rest = fabs(g[0]) < 0.026 && fabs(g[1]) < 0.026 && fabs(g[2]) < 0.026 && an > 9.5 && an < 10.1;
		s->still_s = rest ? s->still_s + dt : 0;
		if (rest && s->still_s > 0.5 && dt > 0) {
			double k = dt / 3.0;
			for (int i = 0; i < 3; i++) s->gbo[i] = CLAMP(s->gbo[i] + k * (in->gyro[i] - s->gbo[i]), s->cal.gb[i] - 12.0, s->cal.gb[i] + 12.0);
			if (++s->gbo_n == 2000) slog("sense: %c gyro bias at rest %.1f %.1f %.1f LSB (factory %d %d %d)", s->hand ? 'R' : 'L', s->gbo[0], s->gbo[1], s->gbo[2], s->cal.gb[0], s->cal.gb[1], s->cal.gb[2]);
			for (int i = 0; i < 3; i++) g[i] = (in->gyro[i] - s->gbo[i]) * s->cal.gs[i];
		}
	}
	if (!s->q_ok) { s->q = QID; s->q_ok = true; dt = 0; }
	/* gyro integration in the IMU body frame */
	quat q = qmul(s->q, qrotvec(g[0] * dt, g[1] * dt, g[2] * dt));
	/* gravity correction (complementary): only when |a| ~ 1 g, 0.5 s time constant.  (Re-fusing a recorded punching session offline
	 * against the LED fits: stricter gating or weaker gains are worse, 4-7 deg median tilt error vs 7-10; the quick coupling averages
	 * the motion out faster than the gyro integration accumulates error) */
	double an = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
	double gain = an > 7.5 && an < 12.0 ? 2.0 : 0.0;
	if (gain > 0) {
		double up[3] = { 0, 1, 0 }, gb[3];
		qrot(qconj(q), up, gb);
		double ax = a[0] / an, ay = a[1] / an, az = a[2] / an;
		double e[3] = { ay * gb[2] - az * gb[1], az * gb[0] - ax * gb[2], ax * gb[1] - ay * gb[0] };
		double k = dt > 0 ? CLAMP(dt * gain, 0.0, 0.2) : 1.0;
		q = qmul(q, qrotvec(e[0] * k, e[1] * k, e[2] * k));
	}
	s->q = qnorm(q);
	qrot(s->q, g, s->gyro_w);
	{ /* linear acceleration in the world frame (gravity removed) for position dead-reckoning */
		quat qw = s->qc_ok && T.qc_on != 0 ? qnorm(qmul(s->qc, s->q)) : s->q; double aw[3];
		qrot(qw, a, aw); aw[1] -= 9.80665;
		s->acc_w[0] = aw[0]; s->acc_w[1] = aw[1]; s->acc_w[2] = aw[2]; s->acc_ok = true;
	}
	/* optical orientation correction: the LED fit gives the true world orientation 60x per second; a smoothed
	 * world-frame correction on top of the IMU integration removes the gyro's yaw drift (and any pitch/roll
	 * error of the gravity estimate) without adding the fit's jitter */
	struct opt_qcorr qc = optical_imu_update(s->hand, s->q);
	if (qc.ok && qc.us != s->qc_us) {
		s->qc_us = qc.us;
		if (!s->qc_ok) { s->qc = qc.q; s->qc_ok = true; s->yaw_aligned = true; }
		else {
			double k = qangle(s->qc, qc.q) > 0.35 && qc.n >= 6 ? 0.3 : qc.n >= 6 ? T.qc_rate : T.qc_rate * 0.5; /* ~0.4 s time constant at 60 Hz */
			s->qc = qnlerp(s->qc, qc.q, k);
		}
	}

	/* yaw alignment to the headset: at first fix and on PS-button long press */
	bool ps = (in->btn[1] & 16) != 0;
	if (ps) { if (!s->ps_down_us) s->ps_down_us = now; } else s->ps_down_us = 0;
	bool want_align = !s->yaw_aligned || (s->ps_down_us && now - s->ps_down_us > 1000000);
	if (want_align) {
		pthread_mutex_lock(&g_head_lock);
		bool hv = g_head_valid; quat hq = g_head_q;
		pthread_mutex_unlock(&g_head_lock);
		if (hv) {
			double d = qyaw(hq) - qyaw(sense_grip_q(s));
			s->q = qnorm(qmul(qrotvec(0, d, 0), s->q));
			s->yaw_aligned = true; s->qc_ok = false; /* the next optical fix re-derives the correction */
			s->ps_down_us = now; /* avoid re-trigger until released */
		}
	}
}

static void sense_on_input(struct sense *s, const struct sense_in *in, uint64_t now)
{
	s->last_in_us = now;
	memcpy(s->btn, in->btn, 3); s->last_trigger = in->trigger; /* for the test tool's guided flow */
	optical_dev_clock(s->hand, in->dev_ts, in->host_ts, now);
	if (s->cal.ok) { sense_fuse(s, in, now); sense_submit_pose(s); }
	{ int16_t gy[3], ac[3]; memcpy(gy, in->gyro, sizeof gy); memcpy(ac, in->accel, sizeof ac); rec_imu(s->hand, in->dev_ts, gy, ac, s->q, s->qc, s->qc_ok, in->battery, in->btn, in->host_ts, (uint32_t)now - in->host_ts, now); }
	if (!atomic_load(&s->active)) return;

	const uint8_t *b = in->btn;
	bool left = s->hand == HAND_L;
	upd_b(s->h_sys, (b[1] & 16) || (left ? (b[1] & 1) : (b[1] & 2))); /* PS, create/options */
	upd_b(s->h_a, left ? (b[0] & 1) : (b[0] & 2));     /* X / A  = square / cross */
	upd_b(s->h_a_t, (b[2] & 2) != 0);
	upd_b(s->h_b, left ? (b[0] & 8) : (b[0] & 4));     /* Y / B  = triangle / circle */
	upd_b(s->h_b_t, (b[2] & 1) != 0);
	upd_f(s->h_trig, in->trigger / 255.f);
	upd_b(s->h_trig_t, (b[1] & 128) != 0 || in->trigger > 0);
	upd_f(s->h_grip, (left ? (b[0] & 16) : (b[0] & 32)) ? 1.f : 0.f);
	upd_b(s->h_grip_t, (b[2] & 8) != 0);
	upd_f(s->h_jx, (in->tx - 128) / 128.f);
	upd_f(s->h_jy, (in->ty - 128) / -128.f);
	upd_b(s->h_jc, left ? (b[1] & 4) : (b[1] & 8));
	upd_b(s->h_jt, (b[2] & 4) != 0);

	if (in->battery != s->battery) {
		s->battery = in->battery;
		uint8_t st = in->battery >> 4, lvl = in->battery & 0xF;
		float pct = st == 2 ? 1.f : CLAMP(lvl * 0.1f + 0.05f, 0.f, 1.f);
		if (st == 0 && lvl <= 1) slog("sense %c: battery %d%% - nearly flat; the controller stops flashing its tracking LEDs at this level (charge it)", s->hand == HAND_L ? 'L' : 'R', lvl * 10 + 5);
		else slog("sense %c: battery %d%%%s", s->hand == HAND_L ? 'L' : 'R', st == 2 ? 100 : lvl * 10 + 5, st == 1 ? " (charging)" : st == 2 ? " (full)" : "");
		prop_f32(s->container, Prop_DeviceBatteryPercentage_Float, pct);
		prop_bool(s->container, Prop_DeviceIsCharging_Bool, st == 1 || st == 2);
	}
}

/* The firmware's LED scheduler can get stuck with the LEDs on, ignoring every LED command (seen after it was handed a base time it
 * could not schedule); a Bluetooth disconnect clears it and the controller reconnects by itself within seconds.  Best effort via
 * bluetoothctl (double fork: no zombie, nothing inherited). */
static void sense_bt_reconnect(int hand)
{
	struct sense *s = &S[hand];
	const char *mac = strrchr(s->serial, '-');
	if (!mac || strlen(mac + 1) != 17) { slog("sense %c: LEDs stuck on but no Bluetooth address known (%s): power-cycle the controller (hold PS)", hand ? 'R' : 'L', s->serial); return; }
	char m[18]; for (int i = 0; i < 17; i++) m[i] = (char)toupper((unsigned char)mac[1 + i]); m[17] = 0;
	slog("sense %c: LEDs stuck on (the controller ignores LED commands): requesting a Bluetooth disconnect of %s, it reconnects by itself", hand ? 'R' : 'L', m);
	pid_t pid = fork();
	if (pid == 0) {
		if (fork() != 0) _exit(0);
		int fd = open("/dev/null", O_RDWR); if (fd >= 0) { dup2(fd, 0); dup2(fd, 1); dup2(fd, 2); if (fd > 2) close(fd); }
		execlp("bluetoothctl", "bluetoothctl", "disconnect", m, (char *)NULL);
		execl("/run/current-system/sw/bin/bluetoothctl", "bluetoothctl", "disconnect", m, (char *)NULL);
		_exit(127);
	} else if (pid > 0) { int st; waitpid(pid, &st, 0); }
}

/* ---- output: 0x31 (BT) / seq-only (USB) report every 10.67 ms ---- */
static void sense_fill_settings(struct sense *s, struct sense_out_settings *o, uint64_t now)
{
	struct led_cmd led = optical_led_cmd(s->hand, now); /* flash timing synced to the headset cameras */
	o->flag1 = 0x02;          /* tracking LED block valid */
	o->flag2 = 1 << 2;        /* status LED set enable */
	o->status_led = 1;        /* "ledEnable": gates the tracking LEDs */
	o->led_phase = led.phase; o->led_seq = led.seq; o->led_period = led.period;
	o->led_pos = led.pos; o->led_len = led.len; memset(o->led_blink, 0xFF, 4);
	o->trig_mode = 0x05;      /* adaptive trigger off */
	o->host_ts = (uint32_t)now;
	s->seq = (s->seq + 1) & 15;
}

static void sense_send_output(struct sense *s, uint64_t now)
{
	if (s->usb) {
		uint8_t r[1 + sizeof(struct sense_out_settings)] = { (uint8_t)((s->seq << 4) | 0x2) };
		sense_fill_settings(s, (void *)(r + 1), now);
		if (write(s->fd, r, sizeof r) < 0 && errno != EAGAIN) { close(s->fd); s->fd = -1; }
		return;
	}
	uint8_t r[78] = { 0x31, (uint8_t)(s->seq << 4), 0x10 };
	struct sense_out_settings *o = (void *)(r + 3);
	sense_fill_settings(s, o, now);
	r[41] = s->counter++; /* must increment for PCM haptics */
	int8_t *pcm = (int8_t *)(r + 42);
	pthread_mutex_lock(&s->hap_lock);
	struct haptic *h = &s->hap;
	if (h->samples_left > 0) {
		double over = CLAMP(300.0 / h->freq, 1.0, 8.0); /* squarer wave at low freq = stronger */
		double step = 2 * M_PI * h->freq / PCM_RATE;
		for (int i = 0; i < PCM_CHUNK && h->samples_left > 0; i++, h->samples_left--) {
			double v = CLAMP(cos(h->phase) * over, -1.0, 1.0) * h->amp * 127.0;
			pcm[i] = (int8_t)v;
			h->phase += step;
		}
		if (h->phase > 2 * M_PI) h->phase = fmod(h->phase, 2 * M_PI);
	}
	pthread_mutex_unlock(&s->hap_lock);
	uint32_t crc = sense_crc(0xA2, r, 74);
	memcpy(r + 74, &crc, 4);
	ssize_t w = write(s->fd, r, sizeof r);
	if (w < 0 && errno != EAGAIN) { close(s->fd); s->fd = -1; return; }
	s->tx_n++; if (w < 0) s->tx_fail++;
	if (now - s->tx_log_us > 5000000) { /* diagnostics: output reports that could not be queued */
		if (s->tx_fail) slog("sense %c: %u of %u output reports dropped (EAGAIN)", s->hand == HAND_L ? 'L' : 'R', s->tx_fail, s->tx_n);
		s->tx_n = s->tx_fail = 0; s->tx_log_us = now;
	}
}

static void sense_haptic(uint64_t container, float dur, float freq, float amp)
{
	for (int i = 0; i < 2; i++) {
		struct sense *s = &S[i];
		if (!s->added || s->container != container) continue;
		pthread_mutex_lock(&s->hap_lock);
		if (amp <= 0.f) { s->hap.samples_left = 0; }
		else {
			s->hap.freq = CLAMP(freq > 0 ? freq : 160.f, 20.f, 1000.f);
			if (dur <= 0.f) s->hap.freq = fmaxf(s->hap.freq, 80.f);
			s->hap.amp = CLAMP(sqrtf(amp), 0.f, 1.f);
			s->hap.samples_left = dur <= 0.f ? (int)(PCM_RATE / s->hap.freq) : (int)(dur * PCM_RATE);
			s->hap.phase = 0;
		}
		pthread_mutex_unlock(&s->hap_lock);
	}
}

/* ---- I/O thread: both controllers ---- */
static void sense_open(struct sense *s)
{
	char node[32], serial[64]; int usb;
	if (!sense_find(s->hand, node, sizeof node, serial, sizeof serial, &usb)) return;
	char path[64];
	snprintf(path, sizeof path, "/dev/%s", node);
	int fd = open(path, O_RDWR | O_NONBLOCK | O_CLOEXEC);
	if (fd < 0) { slog("sense %c: cannot open %s (%s) — check udev rules", s->hand == HAND_L ? 'L' : 'R', path, strerror(errno)); return; }
	s->fd = fd; s->usb = usb;
	snprintf(s->node, sizeof s->node, "%s", node);
	if (!s->added) snprintf(s->serial, sizeof s->serial, "PSVR2-Sense-%c-%s", s->hand == HAND_L ? 'L' : 'R', serial[0] ? serial : "0");
	/* reading feature 0x05 switches the controller into full (0x31) reports */
	if (!sense_read_calibration(s)) slog("sense %c: calibration read failed (%s)", s->hand == HAND_L ? 'L' : 'R', strerror(errno));
	if (!s->usb) sense_set_polling(s);
	s->q_ok = false; s->yaw_aligned = false; s->qc_ok = false; s->battery = 0xFF;
	s->next_out_us = now_us();
	slog("sense %c: %s via %s (%s)", s->hand == HAND_L ? 'L' : 'R', s->serial, node, usb ? "usb" : "bluetooth");
}

static void *sense_thread(void *arg)
{
	(void)arg;
	uint64_t last_scan = 0;
	while (atomic_load(&g_sense_run)) {
		uint64_t now = now_us();
		if (now - last_scan > 2000000) {
			last_scan = now;
			for (int i = 0; i < 2; i++) if (S[i].fd < 0) sense_open(&S[i]);
		}
		struct pollfd pfd[2];
		int n = 0, map[2];
		uint64_t next = now + 1000000;
		for (int i = 0; i < 2; i++) {
			if (S[i].fd < 0) continue;
			pfd[n].fd = S[i].fd; pfd[n].events = POLLIN; pfd[n].revents = 0; map[n++] = i;
			if (S[i].next_out_us < next) next = S[i].next_out_us;
		}
		int wait_ms = next > now ? (int)((next - now + 999) / 1000) : 0;
		if (poll(pfd, n, wait_ms > 100 ? 100 : wait_ms) < 0 && errno != EINTR) break;
		now = now_us();
		for (int k = 0; k < n; k++) {
			struct sense *s = &S[map[k]];
			if (pfd[k].revents & (POLLERR | POLLHUP | POLLNVAL)) { close(s->fd); s->fd = -1; s->node[0] = 0; continue; }
			if (!(pfd[k].revents & POLLIN)) continue;
			uint8_t buf[128];
			int r;
			while ((r = (int)read(s->fd, buf, sizeof buf)) > 0) {
				if (buf[0] == 0x31 && r == 78) {
					uint32_t want; memcpy(&want, buf + 74, 4);
					if (sense_crc(0xA1, buf, 74) != want) continue;
					sense_on_input(s, (const struct sense_in *)(buf + 2), now);
				} else if (buf[0] == 0x01 && r == 64) {
					sense_on_input(s, (const struct sense_in *)(buf + 1), now);
				} else if (buf[0] == 0x01 && !s->cal.ok) {
					sense_read_calibration(s);
				}
			}
			if (r < 0 && errno != EAGAIN) { slog("sense: read error %s", strerror(errno)); close(s->fd); s->fd = -1; s->node[0] = 0; }
		}
		for (int i = 0; i < 2; i++) {
			struct sense *s = &S[i];
			if (s->fd < 0) continue;
			if (now >= s->next_out_us) {
				sense_send_output(s, now);
				s->next_out_us += OUT_PERIOD_US;
				if (s->next_out_us < now) s->next_out_us = now + OUT_PERIOD_US;
			}
			if (s->fd < 0 || (s->last_in_us && now - s->last_in_us > 3000000)) { /* silent: treat as gone */
				if (s->fd >= 0) close(s->fd);
				s->fd = -1; s->node[0] = 0; s->last_in_us = 0; s->q_ok = false;
				sense_submit_pose(s);
			}
		}
	}
	return NULL;
}

/* ================= ITrackedDeviceServerDriver_005 ================= */
static int32_t sense_Activate(void *self, uint32_t id)
{
	struct sense *s = self;
	bool left = s->hand == HAND_L;
	s->id = id;
	uint64_t c = s->container = ovr_props_TrackedDeviceToPropertyContainer(g_props, id);
	prop_str(c, Prop_TrackingSystemName_String, "slopvr");
	prop_str(c, Prop_ModelNumber_String, left ? "PlayStation VR2 Sense (Left)" : "PlayStation VR2 Sense (Right)");
	prop_str(c, Prop_SerialNumber_String, s->serial);
	prop_str(c, Prop_ManufacturerName_String, "Sony");
	prop_str(c, Prop_RenderModelName_String, left ? "oculus_quest2_controller_left" : "oculus_quest2_controller_right");
	prop_str(c, Prop_RegisteredDeviceType_String, left ? "sony/psvr2_sense_left" : "sony/psvr2_sense_right");
	prop_str(c, Prop_ControllerType_String, "oculus_touch");
	prop_str(c, Prop_InputProfilePath_String, "{oculus}/input/touch_profile.json");
	prop_str(c, Prop_DriverVersion_String, SLOP_VERSION);
	prop_i32(c, Prop_ControllerRoleHint_Int32, left ? TrackedControllerRole_LeftHand : TrackedControllerRole_RightHand);
	prop_i32(c, Prop_ControllerHandSelectionPriority_Int32, 0);
	prop_bool(c, Prop_DeviceIsWireless_Bool, true);
	prop_bool(c, Prop_DeviceProvidesBatteryStatus_Bool, true);
	prop_bool(c, Prop_DeviceCanPowerOff_Bool, true);
	prop_bool(c, Prop_Identifiable_Bool, true);
	prop_bool(c, Prop_WillDriftInYaw_Bool, true);
	prop_bool(c, Prop_NeverTracked_Bool, false);
	prop_bool(c, Prop_HasControllerComponent_Bool, true);
	prop_f32(c, Prop_DeviceBatteryPercentage_Float, 1.f);
	const char *side = left ? "left" : "right";
	char icon[96];
#define ICON(prop, fmt) (snprintf(icon, sizeof icon, "{oculus}/icons/cb_%s_controller_" fmt, side), prop_str(c, prop, icon))
	ICON(Prop_NamedIconPathDeviceOff_String, "off.png");
	ICON(Prop_NamedIconPathDeviceSearching_String, "searching.gif");
	ICON(Prop_NamedIconPathDeviceSearchingAlert_String, "searching_alert.gif");
	ICON(Prop_NamedIconPathDeviceReady_String, "ready.png");
	ICON(Prop_NamedIconPathDeviceReadyAlert_String, "ready_alert.png");
	ICON(Prop_NamedIconPathDeviceNotReady_String, "error.png");
	ICON(Prop_NamedIconPathDeviceStandby_String, "off.png");
	ICON(Prop_NamedIconPathDeviceAlertLow_String, "ready_low.png");
#undef ICON

	ovr_input_CreateBooleanComponent(g_input, c, "/input/system/click", &s->h_sys);
	ovr_input_CreateBooleanComponent(g_input, c, left ? "/input/x/click" : "/input/a/click", &s->h_a);
	ovr_input_CreateBooleanComponent(g_input, c, left ? "/input/x/touch" : "/input/a/touch", &s->h_a_t);
	ovr_input_CreateBooleanComponent(g_input, c, left ? "/input/y/click" : "/input/b/click", &s->h_b);
	ovr_input_CreateBooleanComponent(g_input, c, left ? "/input/y/touch" : "/input/b/touch", &s->h_b_t);
	ovr_input_CreateScalarComponent(g_input, c, "/input/trigger/value", &s->h_trig, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
	ovr_input_CreateBooleanComponent(g_input, c, "/input/trigger/touch", &s->h_trig_t);
	ovr_input_CreateScalarComponent(g_input, c, "/input/grip/value", &s->h_grip, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);
	ovr_input_CreateBooleanComponent(g_input, c, "/input/grip/touch", &s->h_grip_t);
	ovr_input_CreateScalarComponent(g_input, c, "/input/joystick/x", &s->h_jx, VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);
	ovr_input_CreateScalarComponent(g_input, c, "/input/joystick/y", &s->h_jy, VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);
	ovr_input_CreateBooleanComponent(g_input, c, "/input/joystick/click", &s->h_jc);
	ovr_input_CreateBooleanComponent(g_input, c, "/input/joystick/touch", &s->h_jt);
	ovr_input_CreateHapticComponent(g_input, c, "/output/haptic", &s->h_haptic);
	atomic_store(&s->active, true);
	return VRInitError_None;
}
static void sense_Deactivate(void *self) { atomic_store(&((struct sense *)self)->active, false); }
static void sense_EnterStandby(void *self) { (void)self; }
static void *sense_GetComponent(void *self, const char *name) { (void)self; (void)name; return NULL; }
static void sense_DebugRequest(void *self, const char *req, char *resp, uint32_t len)
{
	struct sense *s = self; (void)req;
	snprintf(resp, len, "{\"connected\":%d,\"usb\":%d,\"battery\":%u,\"calibrated\":%d}", s->fd >= 0, s->usb, s->battery, s->cal.ok);
}
static void sense_GetPose(DriverPose_t *out, void *self)
{
	struct sense *s = self;
	*out = pose_default();
	out->deviceIsConnected = s->fd >= 0;
	out->poseIsValid = s->q_ok;
	out->qRotation = sense_grip_q(s);
	out->result = s->q_ok ? TrackingResult_Running_OK : TrackingResult_Uninitialized;
}
OVR_VTABLE(sense_vt, OVR_FN(ovr_dev_Activate, sense_Activate), OVR_FN(ovr_dev_Deactivate, sense_Deactivate), OVR_FN(ovr_dev_EnterStandby, sense_EnterStandby),
	   OVR_FN(ovr_dev_GetComponent, sense_GetComponent), OVR_FN(ovr_dev_DebugRequest, sense_DebugRequest), OVR_FN(ovr_dev_GetPose, sense_GetPose));

/* ---- provider hooks ---- */
static void sense_init(void)
{
	g_led_stuck_hook = sense_bt_reconnect;
	for (uint32_t i = 0; i < 256; i++) {
		uint32_t c = i;
		for (int k = 0; k < 8; k++) c = c & 1 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
		g_crc_tab[i] = c;
	}
	for (int i = 0; i < 2; i++) {
		S[i].vt = sense_vt;
		S[i].hand = i;
		S[i].fd = -1;
		S[i].q = QID;
		S[i].battery = 0xFF;
		pthread_mutex_init(&S[i].hap_lock, NULL);
	}
	if (!cfg_bool("enableControllers", true)) return;
	atomic_store(&g_sense_run, true);
	pthread_create(&g_sense_thread, NULL, sense_thread, NULL);
}

/* main thread: register controllers once they have been opened */
static void sense_frame(void)
{
	for (int i = 0; i < 2; i++)
		if (!S[i].added && S[i].fd >= 0)
			S[i].added = ovr_host_TrackedDeviceAdded(g_host, S[i].serial, TrackedDeviceClass_Controller, &S[i]);
}

static void sense_cleanup(void)
{
	if (atomic_exchange(&g_sense_run, false)) pthread_join(g_sense_thread, NULL);
	for (int i = 0; i < 2; i++) {
		if (S[i].fd >= 0) close(S[i].fd);
		S[i].fd = -1;
		pthread_mutex_destroy(&S[i].hap_lock);
	}
}
