#include <dirent.h>
/* slopvr — optical 6DoF for the Sense controllers.
 *
 * The headset detects the controllers' IR LEDs on-device: interface 8 streams one
 * "LD" packet per camera frame (60 Hz) with blob moments for all four tracking
 * cameras. Together with the factory camera calibration (GenData item 10: rational
 * intrinsics + camera-to-camera extrinsics) and the 17-LED constellation of the
 * Sense controller (IMU frame, mm) this gives:
 *   rays -> stereo candidates -> known-rotation RANSAC (rotation from the IMU) ->
 *   LED/blob correspondences -> Gauss-Newton on reprojection -> position + yaw fix.
 * Everything here runs on the headset USB thread; results go to sense.c via O.fix. */
#define OPT_NCAM 4
#define OPT_NLED 17
#define OPT_MAX_BLOB 128 /* per camera; the detector lists up to 256 in scan order, so a low cap drops the LEDs behind a sunlit window */
#define OPT_MAX_RAY (OPT_NCAM * OPT_MAX_BLOB)
#define OPT_MAX_CAND 256
#define OPT_MAX_OBS 64
#define OPT_MAX_CL 96
#define OPT_LD_SIZE 0x9050
#define OPT_LD_CAM_BLOCK 0x2404 /* u32 count + 0x2400 of 36-byte records */
#define OPT_LD_REC 36
#ifndef OPT_LD_XOFF
#define OPT_LD_XOFF (T.ld_xoff) /* full-res px: LD centroids are shifted in x vs. the calibration frame (tunable, default 8) */
#endif
#ifndef OPT_LD_YOFF
#define OPT_LD_YOFF (T.ld_yoff)
#endif
#ifndef OPT_CAND_GAP
#define OPT_CAND_GAP (T.gap_mm * 1e-3)   /* m: max ray-ray distance for a stereo candidate (+1 %/m) */
#endif
#ifndef OPT_INLIER
#define OPT_INLIER (T.inlier_mm * 1e-3)  /* m: LED model point to candidate */
#endif
#define OPT_PX_ASSOC (T.assoc_px)        /* px: reprojection gate for LED<->blob association */
#define OPT_FACING -0.15     /* cos: LED normal vs. direction to camera */

/* Retail ("MP") Sense LED constellation from the official driver: positions mm, unit normals, IMU frame. */
static const float opt_led_pos[OPT_NLED][3] = {
	{ -31.783f, 13.435f, -59.270f }, { 12.054f, -55.213f, 38.152f }, { 33.088f, -55.236f, 20.190f },
	{ 53.695f, -31.009f, 28.350f },  { 66.986f, -10.505f, 8.679f },  { 63.505f, 24.601f, 4.185f },
	{ 47.803f, 44.966f, 18.502f },   { -45.472f, -27.042f, 43.022f }, { -59.535f, -13.977f, 28.536f },
	{ -61.060f, 14.608f, 24.725f },  { -50.994f, 27.255f, 36.184f }, { -48.475f, 43.025f, 18.776f },
	{ -30.569f, 53.446f, 29.350f },  { -18.498f, 63.913f, 11.230f }, { 2.748f, 64.260f, 22.718f },
	{ 29.078f, 13.435f, -60.735f },  { -1.067f, 13.435f, -67.327f },
};
static const float opt_led_nrm[OPT_NLED][3] = {
	{ -0.459f, 0.259f, -0.850f }, { 0.176f, -0.812f, 0.556f }, { 0.426f, -0.892f, -0.151f },
	{ 0.788f, -0.456f, 0.412f },  { 0.980f, -0.154f, 0.126f }, { 0.931f, 0.361f, 0.060f },
	{ 0.702f, 0.660f, 0.268f },   { -0.667f, -0.397f, 0.630f }, { -0.962f, -0.273f, -0.026f },
	{ -0.984f, 0.155f, -0.084f }, { -0.748f, 0.400f, 0.530f }, { -0.796f, 0.580f, -0.173f },
	{ -0.449f, 0.784f, 0.429f },  { -0.347f, 0.893f, -0.286f }, { 0.040f, 0.942f, 0.332f },
	{ 0.418f, 0.259f, -0.871f },  { -0.014f, 0.259f, -0.966f },
};

struct opt_cam { bool ok; double fx, fy, cx, cy, k[6], kb[3]; double R[9], o[3]; }; /* kb: fisheye polynomial r_px/f = kb0 th + kb1 th^3 + kb2 th^5; k: OpenCV rational block (unused, see opt_distort_r); R: cam -> cam0 (row-major), o: origin in cam0 */
struct opt_ray { uint8_t cam, blob; bool used; float inten; double o[3], d[3], n[2], uv[2]; };        /* d: unit dir in cam0; n: undistorted normalized coords; uv: LD centroid in calibration pixels */
struct opt_cand { double x[3]; float gap; int16_t ray[2]; bool used; };
struct opt_cl { double x[3]; float gap; int n; bool used; };
struct opt_obs { int led, ray; };
#define CLK_N 120
struct clk { int64_t y[CLK_N]; uint64_t t[CLK_N]; uint32_t rtt[CLK_N]; int n, head; int64_t cur; uint32_t cur_rtt; uint64_t bucket, t0, rate_us; bool have, rtt_mode; double a, b; int64_t y0; int8_t id; }; /* id: 0/1 controller, 2 headset (recorder) */
/* Units on the wire (from the official driver, psvr2.dll: FUN_1801c90c0(report + 0x17, pos_us * 3) / (report + 0x1b, cycle_ns * 3); Monado's
 * pssense driver agrees): cyclePosition is the controller's 32-bit IMU tick counter (3 MHz, wraps every 23.86 min), cycleLength is in
 * thirds of a nanosecond (= ticks * 1000), periodId counts 150-tick (50 µs) units.  Sending µs/ns (1/3 of the right values) still produced
 * multiplexed flashes by a fluke (the base times advanced in multiples of the 2-frame period, which the firmware read as multiples of its
 * 11.1 ms cycle), but every latency step was really a third of what was commanded, the flash followed the exposure with 1/3 gain, and in
 * the last quarter of the tick epoch (values >= 2^30) the firmware saw bases far in the future: dark when started there, and a running
 * controller got stuck with its LEDs on until a Bluetooth reconnect. */
#define OPT_DUMP (SLOP_DBG && O.dump) /* per-frame diagnostics: compiled out of the driver build */
#define LED_SLOT 50                      /* µs per periodId unit (150 ticks) */
#define LED_CYCLE_NS 16683350u           /* 59.94 Hz frame cycle (ns) */
#define LED_CYCLE_US 16683
#define LED_SCAN_STEP 400                /* µs */
#define LED_SCAN_N 42                    /* steps over one frame cycle */
#define LED_SCAN_COARSE 14               /* coarse pass: every 3rd step, 14 of them cover the cycle */

struct opt_fix { bool valid; uint64_t host_us; double p[3], v[3]; quat q; quat iq; bool iq_ok; int n, streak; double res; }; /* iq: controller IMU orientation at the fix */
struct opt_pose { double R[9], t[3]; int n; double res; struct opt_obs obs[OPT_MAX_OBS]; int nobs; };

static struct optical {
	pthread_mutex_t lock;
	bool enabled, dump, state_req, mirror[2]; char dump_dir[64]; bool dump_dir_ok; int dump_every; /* state_req: dump the whole state with the next frame; dump_dir: ~/.cache/<dir> for the packet dump */ /* dump_dir: ~/.cache/<dir> for the packet dump (the replay tool redirects it away from its input) */
	struct opt_cam cam[OPT_NCAM]; bool calib_ok;
	quat q_hc; double p_hc[3];          /* head (SLAM/IMU origin) <- cam0 */
	struct opt_fix fix[2];
	/* LED-model frame <- IMU frame (constant per controller), from gravity pairs: g_model = R g_imu.  quality 0 unknown,
	 * 1 tilt only (all samples at one orientation), 2 full (samples spread > 25 deg).  Persisted per hand. */
#define IMF_N 96 /* gravity-pair window of the model<->IMU learner */
	struct imu_frame { double gi[IMF_N][3], gm[IMF_N][3]; int n, head; double R[9]; int quality; double spread, rms; uint64_t last_us; int agree; bool loaded; } imf[2]; /* agree: fits of this session the estimate explained (a loaded one is provisional until 8) */
	struct turn_ref { bool valid; uint64_t host_us; quat q, iq; int n; } tref[2]; int tdoubt[2]; /* last established track's orientation + IMU orientation (turn test); tdoubt: gravity rejections of turn-consistent fits in a row */
	bool mux_live[2]; double mux_ema[2]; uint64_t own_ok_us[2]; /* do this hand's LEDs appear only on its own frame parity? (probed while tracking; own_ok_us: last fit on its own parity) */
	double par_sum[2]; unsigned par_n[2], par_fit[2][2]; /* blobs per frame parity, fits per parity per hand (state dump) */
	unsigned rot_jumps[2]; double rot_jump_max[2];        /* tracked fits rejected for turning against the IMU (state dump) */
	uint64_t grav_rej_us[2]; int grav_rej_run[2];
	double imu_rate[2];                 /* controller angular rate at this frame's exposure (deg/s): tolerances widen with it (IMU timing + fusion errors grow with speed) */        /* last gravity rejection (the track before it had a wrong correspondence, do not continue from it) and the run of rejections without an accepted fit */
	quat imu_q[2]; bool imu_ok[2];      /* controller IMU orientation (world <- IMU), published by sense.c */
	struct imu_hist { uint64_t t; quat q; } ih[2][128]; unsigned ih_n[2]; /* recent IMU orientations by sample host time */
	uint64_t ld_host_us; unsigned ld_frames; bool logged_ts; uint32_t ld_frame_last; /* headset frame counter of the latest LD packet (parity = flashing hand with ledMux) */
	struct opt_ray rays[OPT_MAX_RAY]; int nray;
	struct opt_cand cand[OPT_MAX_CAND]; int ncand;
	struct opt_cl cl[96]; int ncl;
	pthread_t thread; bool run; pthread_mutex_t mb_lock; pthread_cond_t mb_cond; /* worker queue: a few frames deep so a burst of two packets (USB jitter, the replay's main loop) does not drop one; the oldest goes when it overflows */
	struct { uint8_t b[OPT_LD_SIZE]; int len; uint64_t now; int64_t exp; uint32_t frame; } mb[3]; int mb_r, mb_w, mb_n; unsigned mb_dropped;
	unsigned stat_acq[2], stat_lock[2], stat_fail[2]; uint32_t led_cycle_ns; int led_period;
	quat qc[2]; int qc_n[2]; uint64_t qc_us[2]; bool qc_ok[2]; double qc_tilt[2]; double qc_gi[2][3]; /* qc_gi: IMU gravity (IMU frame) when qc was last set: with a tilt-only frame estimate qc holds near that orientation */ /* optical orientation correction (world frame) for sense.c; tilt: deg off a pure yaw (0 if the LED model frame is the IMU frame) */
	uint64_t tl_sec; int tl_bmin, tl_bmax, tl_lock[2], tl_frames, tl_ms, tl_last_lock[2], tl_last_bmin, tl_last_bmax, tl_last_frames;
	/* LED flash sync (see below) */
	int led_mode;                                        /* 0 synced flashes, 5 off, 6 continuous */
	uint32_t ld_ts_last;                                 /* latest headset frame time (µs, same clock as the IMU) */
	int64_t hts64; uint32_t hts_last; bool hclk_have;    /* unwrapped headset IMU time */
	struct clk ldclk; /* headset µs - host µs, from the status stream's arrival envelope */
	int32_t ld_delay, ld_delay_min, ld_delay_max;        /* LD frame time -> arrival, in headset µs (diagnostic) */
	uint64_t lock_us[2]; int blob_recent, blob_acc; /* max blob count over the last second */
	struct led_sync {
		struct clk clk; bool ready; uint32_t ticks_last; int64_t ticks64; uint64_t in_us, sample_host_us; /* controller µs - host µs */
		int32_t lat; bool lat_ok; uint64_t lat_us;       /* tuned latency (µs into the frame cycle) and when it was found */
		uint8_t phase, seq; uint32_t pos; uint64_t anchor_us; bool dirty;
		bool scan, on; int step, base, peak; unsigned f0; uint64_t scan_done_us, scan_wait_us; int8_t cnt[LED_SCAN_N];
		int exp_par; int probe; unsigned probe_f0, probe_k; uint64_t probe_last_us; double probe_sum[2]; int probe_cnt[2]; /* edge-probe servo */
		uint64_t cmd_us, nosync_log_us, cmd_log_us; bool freerun; /* last LED command built for sense.c (its output thread must keep calling); freerun: sweeping flash */
		int par_fix; bool par_ok; int par_meas, par_votes[2]; uint64_t par_fit_us; bool par_flip_tried; /* ledMux: measured frame parity of this hand's flash (par_ok once confirmed; until then the hand is fitted on every frame) */
		int pbase[2], ppeak[2], pdelta[2]; /* blob counts per frame parity during an LED check / scan (OFF vs flashing): where does this hand's flash land? */
		double bsum, psum; int bcnt, pcnt; uint64_t lat_seen_us; /* scan/check: mean blob counts of the OFF and flashing windows; when the stored latency last showed a flash */
		bool scanned[LED_SCAN_N]; int scan_k, scan_c, scan_fine, dark_checks, dark_scans, period, period_cur, centred; uint64_t recover_until_us, par_flip_us;
		uint64_t muxdead_us, offtest_us, stuck_us; int offtest_frames, offtest_fits; /* stuck-LED test: a hand lit on both parities for 30 s is told OFF for 0.6 s; fits that continue mean the firmware ignores LED commands (seen once after a bad base time): a Bluetooth reconnect releases it */
		double cycle_ppm_unused; /* (the cycle is now scaled by the measured clock rates) the flash schedule drifts against the exposure at a controller-specific rate (its LED clock vs the headset's frame rate); the servo corrections measure it and the cycle length sent is tuned by cycle_ppm so the drift stops (Sony's "tunedCycle") */ /* period_cur: current steady window (48 -> stable in steps, only while both edge probes keep the LEDs) */ /* scan_fine: -1 coarse pass, else the coarse index the fine pass surrounds */ /* scan order: steps taken outward from the stored latency (k-th step, centre index) so a nearby window is found in ~2 s instead of 7 */
	} led[2];
} O;

/* ---- LED flash sync ----
 * The controller does not light its tracking LEDs continuously: it flashes them once per camera frame for
 * periodId slots of 105 µs (the official driver's PRESCAN phase), at a base time given in its own clock.
 * We map headset frame time -> host time -> controller time (headset: USB arrival envelope; controller:
 * NTP-style round trips, as the official driver does) and find the remaining fixed latency with a scan
 * over the frame cycle, watching the blob count.  An edge-probe servo then keeps the exposure centred. */
#define LED_SCAN_OFF 8                   /* frames per step with the LEDs off (background count from frame 4: the OFF command needs a few frames over BT to show) */
#define LED_SCAN_ON 12                   /* frames per step flashing (count from frame 6: BT + up to one 2-frame cycle of delay; a late command must not fragment the window) */
#define LED_PHASE_PRESCAN 1
#define LED_PHASE_OFF 5
#define LED_PHASE_ON 6

struct led_cmd { uint8_t phase, seq, period; uint32_t pos, len; };
static void (*g_led_stuck_hook)(int hand); /* sense.c: the controller ignores LED commands (LEDs stuck on): reconnect it */

/* Clock offset model y(t) = device µs - host µs.  One sample per second is kept: for the headset (USB) the
 * largest y (least delayed packet), for the controllers the sample with the shortest round trip (the host
 * time stamp we send is echoed back, NTP style, as the official driver does) with y taken at the midpoint of
 * the round trip.  A line through the kept samples gives offset and crystal rate difference. */
static void clk_reset(struct clk *c) { memset(c, 0, sizeof *c); }
static void clk_fit(struct clk *c)
{
	int n = c->n; double b = 0, a = 0;
	int64_t y0 = c->y[(c->head - n + CLK_N) % CLK_N];
	if (c->rtt_mode) {
		/* controllers: weighted least squares, the weight falling off with the round trip above the window
		 * minimum (longer trips = more asymmetric paths = worse midpoint estimate).  The rate term needs 40 s
		 * of samples; the offset is the weighted mean residual of the last 20 s so it follows the line, not
		 * a stale minimum (a stale offset lets the flash drift out of the exposure within minutes) */
		uint32_t rmin = UINT32_MAX;
		for (int i = 0; i < n; i++) { int k = (c->head - n + i + CLK_N) % CLK_N; if (c->rtt[k] < rmin) rmin = c->rtt[k]; }
		double sw = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
		for (int i = 0; i < n; i++) {
			int k = (c->head - n + i + CLK_N) % CLK_N;
			double x = (double)(c->t[k] - c->t0) * 1e-6, y = (double)(c->y[k] - y0), d = (c->rtt[k] - rmin) / 1000.0, w = 1.0 / (1.0 + d * d);
			sw += w; sx += w * x; sy += w * y; sxx += w * x * x; sxy += w * x * y;
		}
		double den = sw * sxx - sx * sx;
		if (n >= 40 && den > 1e-9) b = CLAMP((sw * sxy - sx * sy) / den, -60.0, 60.0);
		double rs = 0, rw = 0;
		for (int i = n - 1; i >= 0 && i >= n - 20; i--) {
			int k = (c->head - n + i + CLK_N) % CLK_N;
			double x = (double)(c->t[k] - c->t0) * 1e-6, d = (c->rtt[k] - rmin) / 1000.0, w = 1.0 / (1.0 + d * d);
			rs += w * ((double)(c->y[k] - y0) - b * x); rw += w;
		}
		a = rw > 0 ? rs / rw : 0;
	} else {
		/* headset over USB: upper envelope (least delayed packet) with a rate term once there is enough data */
		double sx = 0, sy = 0, sxx = 0, sxy = 0;
		for (int i = 0; i < n; i++) {
			int k = (c->head - n + i + CLK_N) % CLK_N;
			double x = (double)(c->t[k] - c->t0) * 1e-6, y = (double)(c->y[k] - y0);
			sx += x; sy += y; sxx += x * x; sxy += x * y;
		}
		double den = n * sxx - sx * sx;
		if (n >= 30 && den > 1e-9) b = CLAMP((n * sxy - sx * sy) / den, -100.0, 100.0);
		a = -1e300;
		for (int i = 0; i < n; i++) { int k = (c->head - n + i + CLK_N) % CLK_N; double r = (double)(c->y[k] - y0) - b * ((double)(c->t[k] - c->t0) * 1e-6); if (r > a) a = r; }
	}
	c->a = a; c->b = b; c->y0 = y0;
}
/* v: offset sample; rtt: its round trip (0 for the envelope mode) */
static int64_t clk_at(const struct clk *c, uint64_t host);
static void clk_push(struct clk *c, uint64_t now, int64_t v, uint32_t rtt)
{
	if (!c->have) { c->cur = v; c->cur_rtt = rtt; c->bucket = now; c->t0 = now; c->have = true; return; }
	if (now - c->bucket < 1000000) {
		if (c->rtt_mode ? rtt < c->cur_rtt : v > c->cur) { c->cur = v; c->cur_rtt = rtt; }
		return;
	}
	c->y[c->head] = c->cur; c->t[c->head] = c->bucket; c->rtt[c->head] = c->cur_rtt; c->head = (c->head + 1) % CLK_N; if (c->n < CLK_N) c->n++;
	c->cur = v; c->cur_rtt = rtt; c->bucket = now;
	int64_t before = c->n > 10 ? clk_at(c, now) : 0;
	clk_fit(c);
	if (c->rtt_mode && c->n > 10) { /* the controller offset drives the LED flash phase: a 5 ms window cannot take a 2 ms step from a change in the round-trip statistics; let it move 300 us/s at most (the edge servo follows the rest) */
		int64_t after = clk_at(c, now), lim = 300 * (int64_t)((now - c->rate_us) / 1000000 + 1);
		if (after - before > lim) c->a -= (double)(after - before - lim);
		else if (before - after > lim) c->a += (double)(before - after - lim);
	}
	c->rate_us = now;
	rec_clk(c->id, c->a, c->b, c->y0, c->t0, c->n, c->cur_rtt, now);
}
static bool clk_ready(const struct clk *c) { return c->n >= 3; }
static int64_t clk_at(const struct clk *c, uint64_t host) { return c->y0 + (int64_t)llround(c->a + c->b * ((double)host - (double)c->t0) * 1e-6); }

/* sense.c: controller time stamp of an input report (ticks of 1/3 µs) and its host arrival time */
/* sense.c: controller time stamp of an input report (ticks of 1/3 µs), the host time stamp it echoes
 * (the one we put into our last output report) and its host arrival time */
static void optical_dev_clock(int hand, uint32_t dev_ticks, uint32_t echo_host, uint64_t now)
{
	pthread_mutex_lock(&O.lock);
	struct led_sync *L = &O.led[hand];
	int32_t dt = (int32_t)(dev_ticks - L->ticks_last);
	if (!L->clk.have || dt < 0 || dt > 15000000 || now - L->in_us > 5000000) { clk_reset(&L->clk); L->clk.rtt_mode = true; L->clk.id = (int8_t)hand; L->ticks64 = dev_ticks; L->ready = false; } /* (re)connected */
	else L->ticks64 += dt;
	L->ticks_last = dev_ticks;
	uint32_t rtt = (uint32_t)now - echo_host;
	if (rtt < 40000) clk_push(&L->clk, now, L->ticks64 / 3 - (int64_t)(now - rtt / 2), rtt);
	if (!L->ready && clk_ready(&L->clk)) { L->ready = true; O.lock_us[hand] = now - 3500000; L->scan_done_us = 0; } /* synced flashes start now: the stored latency gets 1.5 s to produce fits before it is checked (a new connection usually shifts it) */
	L->sample_host_us = clk_ready(&L->clk) ? (uint64_t)(L->ticks64 / 3 - clk_at(&L->clk, now)) : now - 4000; /* when the report was generated */
	L->in_us = now;
	pthread_mutex_unlock(&O.lock);
}

/* hmd.c: headset IMU time stamp (device µs) of the latest status packet and its host arrival time.  The status
 * stream (small interrupt transfers at a high rate) gives a far tighter arrival envelope than the 36 KB LD
 * packets, whose processing delay also shifts with the scene. */
static void optical_hmd_clock(uint32_t dev_us, uint64_t now)
{
	pthread_mutex_lock(&O.lock);
	if (!O.hclk_have) { O.hts64 = dev_us; O.hclk_have = true; O.ldclk.id = 2; } else O.hts64 += (int32_t)(dev_us - O.hts_last);
	O.hts_last = dev_us;
	clk_push(&O.ldclk, now, O.hts64 - (int64_t)now, 0);
	pthread_mutex_unlock(&O.lock);
}
/* headset device time (32-bit µs) -> unwrapped, relative to the latest IMU sample; O.lock held */
static int64_t opt_dev64(uint32_t dev32) { return O.hts64 + (int32_t)(dev32 - O.hts_last); }

/* base time (controller µs) that centres the flash on the camera exposure, shifted by lat; O.lock held.
 * With ledMux the cycle is two frames and hand h flashes only on frames whose counter parity is h. */
static bool led_mux(void) { return T.led_mux != 0; }
/* Base time of the next flash in controller microseconds.  The firmware schedules flashes at base + n * cycle; a base that is already in
 * the past when the command arrives is NOT skipped forward: the controller waits for its 32-bit counter to come around (~24 min)
 * and ignores sync commands meanwhile (both controllers went dark for 20+ minutes today exactly after such commands: a stalled
 * detector stream, or a 22 ms lead eaten by a Bluetooth latency spike).  So the base is always >= led_lead_ms ahead of the
 * controller's clock, stepping by two frames to keep the parity, and nothing is anchored while the frame grid is stale. */
static bool led_pos(const struct led_sync *L, int hand, int32_t lat, uint32_t *pos)
{
	if (!clk_ready(&L->clk) || !clk_ready(&O.ldclk) || !O.ld_frames) return false;
	uint64_t now = now_us();
	if (now - O.ld_host_us > 500000) return false; /* no detector frames for half a second: no exposure to aim at, and the grid below would be a guess */
	int64_t frame_host = opt_dev64(O.ld_ts_last) - clk_at(&O.ldclk, now);
	lat = ((lat % LED_CYCLE_US) + LED_CYCLE_US) % LED_CYCLE_US; /* a latency pushed past a frame boundary by a probe/servo step must not move the flash to the other parity's frame */
	int k = 2; /* on a frame of this hand's parity.  Which LD frame counter parity the flash really lands on differs from the anchored one by a
	            * session-dependent frame (pipeline offset); par_fix holds the measured correction */
	if (led_mux() && (int)((O.ld_frame_last + k) & 1) != (hand ^ L->par_fix)) k++;
	int64_t lead = (int64_t)(T.led_lead_ms * 1000);
	while (frame_host + k * LED_CYCLE_US + lat - (int64_t)now < lead && k < 400) k += led_mux() ? 2 : 1; /* at least led_lead_ms ahead, same parity */
	int64_t ctrl = frame_host + k * LED_CYCLE_US + lat + clk_at(&L->clk, now); /* controller µs (64-bit, unwrapped) */
	*pos = (uint32_t)((uint64_t)(ctrl * 3) & 0xffffffffu); /* the controller's 32-bit tick counter */
	return true;
}

static void led_sync_path(int hand, char *path, size_t n)
{
	snprintf(path, n, "%s/.config/slopvr/ledsync_%c", getenv("HOME") ? getenv("HOME") : "/tmp", hand ? 'R' : 'L');
}

static void led_save(int h)
{
	char path[256]; led_sync_path(h, path, sizeof path);
	FILE *f = fopen(path, "w");
	if (f) { fprintf(f, "v2 %d\n", O.led[h].lat); fclose(f); } /* latency (v2: real microseconds; the v1 files were in thirds) */
}

static bool led_scan_finish(int h, uint64_t now) /* O.lock held; true when a flash window was found (latency kept or set) */
{
	struct led_sync *L = &O.led[h];
	L->scan = false; L->scan_done_us = now; L->dirty = true;
	int best_len = 0, best_start = 0, non = 0, best_sum = 0, nscanned = 0;
	for (int s = 0; s < LED_SCAN_N; s++) { if (!L->scanned[s]) L->cnt[s] = 0; else nscanned++; }
	for (int s = 0; s < LED_SCAN_N; s++) {
		if (L->cnt[s] >= 3) non++;
		if (L->cnt[s] < 3 || L->cnt[(s + LED_SCAN_N - 1) % LED_SCAN_N] >= 3) continue; /* start of a run */
		int len = 0, sum = 0;
		while (len < LED_SCAN_N && L->cnt[(s + len) % LED_SCAN_N] >= 3) { sum += L->cnt[(s + len) % LED_SCAN_N]; len++; }
		if (len > best_len) { best_len = len; best_start = s; best_sum = sum; }
	}
	/* a 2.1 ms window lights ~5 steps; a 2-step run is accepted when it is unmistakable (close controller: +6 blobs in both), 1 = noise.
	 * A latency that showed a flash within the last minute is only replaced by a strong window (the controller may simply be out of view) */
	bool weak = best_len < 3 || best_sum < 15;
	if (non == nscanned || best_len < 2 || (best_len == 2 && best_sum < 12) || best_len > LED_SCAN_N / 3 || (weak && L->lat_ok && now - L->lat_seen_us < 60000000)) {
		/* nothing lit (controller not in view of the cameras) or no clear window: retry, backing off to 20 s */
		L->scan_wait_us = L->scan_wait_us < 20000000 ? L->scan_wait_us * 2 : L->scan_wait_us;
		L->dark_scans++;
		if (nscanned >= LED_SCAN_COARSE && non == 0) { /* nothing at any timing (the coarse pass covers the cycle): out of view, or the LEDs are off; kick the scheduler (OFF 1 s, INIT 1 s, then PRESCAN) */
			L->recover_until_us = now + 2000000; L->dirty = true;
			slog("optical: %c LED sync: no flash at any of the %d timings (%s); LED recovery sequence (off, init, prescan)%s", h ? 'R' : 'L', nscanned,
			     L->dark_scans >= 2 ? "controller in view of the cameras? if so its LEDs are stuck: power-cycle it (hold PS)" : "controller in view of the cameras?", L->dark_scans >= 2 ? " again" : "");
		} else slog("optical: %c LED sync: no flash window found (%d/%d steps lit; controller in view of the cameras?), next try in %llu s",
		     h ? 'R' : 'L', non, nscanned, (unsigned long long)(L->scan_wait_us / 1000000));
		return false;
	}
	int32_t centre = (int32_t)((best_start + (best_len - 1) * 0.5) * LED_SCAN_STEP) % LED_CYCLE_US;
	if (L->lat_ok) { /* a stored (servo-refined) latency inside the lit run is better than the coarse scan centre */
		int32_t d = (L->lat - centre + LED_CYCLE_US + LED_CYCLE_US / 2) % LED_CYCLE_US - LED_CYCLE_US / 2;
		if (abs(d) <= (best_len - 1) * LED_SCAN_STEP / 2) {
			slog("optical: %c LED sync: scan confirms latency %d us (window centre %d, lit for %d x %d us)", h ? 'R' : 'L', L->lat, centre, best_len, LED_SCAN_STEP);
			L->scan_wait_us = 5000000; L->probe_last_us = now;
			return true;
		}
	}
	L->lat = centre;
	L->lat_ok = true; L->lat_us = now; L->scan_wait_us = 5000000; L->probe_last_us = now; /* first edge probe 15 s later */
	slog("optical: %c LED sync: latency %d us (lit for %d x %d us, %d steps scanned)", h ? 'R' : 'L', L->lat, best_len, LED_SCAN_STEP, nscanned);
	L->dark_scans = 0; L->period_cur = (int)T.led_period; L->centred = 0;
	led_save(h);
	return true;
}

/* Latency scan (O.lock held, once per LD frame, also on frames without blobs).  One controller at a time
 * steps its flash through the cycle; the blob count with the LEDs off vs. flashing tells where the exposure
 * is.  total: blobs not explained by the other controller's fit. */
/* ledMux: which frame parity carries this hand's flash?  Measured from the OFF -> flashing blob counts per parity (LED check / scan), or from
 * where its fits land.  The firmware puts the 2-frame cycle on a parity of its own choosing (per controller, per session), so this is the
 * only reliable source; when both hands end up on one parity, one of them is moved by a frame once (if the firmware lets it). */
static void led_par_set(int h, int p, uint64_t now, const char *how)
{
	struct led_sync *L = &O.led[h], *Lo = &O.led[h ^ 1];
	bool changed = !L->par_ok || L->par_meas != p;
	L->par_meas = p; L->par_ok = true; L->par_votes[0] = L->par_votes[1] = 0;
	if (changed) slog("optical: %c flash lands on frame parity %d (%s)", h ? 'R' : 'L', p, how);
	if (Lo->par_ok && Lo->par_meas == p && (int64_t)(now - Lo->in_us) <= 1000000) {
		if (!L->par_flip_tried) { /* try to separate them: anchor this hand one frame later (votes are ignored until the new anchor has taken effect) */
			L->par_flip_tried = true; L->par_fix ^= 1; L->par_ok = false; L->dirty = true; L->scan_wait_us = 3000000; L->scan_done_us = now; L->par_flip_us = now;
			slog("optical: both hands flash on frame parity %d: moving %c by one frame", p, h ? 'R' : 'L');
		} else if (changed) slog("optical: both hands flash on frame parity %d (the firmware keeps it there): no parity separation, hands told apart by the fit", p);
	}
}
static void led_par_from_counts(int h, uint64_t now, const char *how)
{
	struct led_sync *L = &O.led[h];
	int d0 = L->pdelta[0], d1 = L->pdelta[1];
	if (d0 < 3 && d1 < 3) return;
	if (d0 >= 3 && d1 >= 3 && d0 < 2 * d1 && d1 < 2 * d0) { /* lit on both parities: not multiplexed (or a mistimed flash straddling frames) */
		if (OPT_DUMP) slog("optical: %c flash seen on both frame parities (+%d / +%d blobs)", h ? 'R' : 'L', d0, d1);
		return;
	}
	led_par_set(h, d1 > d0, now, how);
}
/* Scan plan: the background is measured once with the LEDs off (10 frames), then the flash is stepped without turning it
 * off in between (12 frames per step: 6 to settle over Bluetooth, 6 measured).  First a coarse pass every 3 grid steps
 * (2.4 ms; the 5 ms window always lights at least two of them), outward from the stored latency, stopping as soon as a lit
 * coarse step is bounded by dark ones; then the fine steps around it.  ~2.5 s instead of 7 for a full sweep. */
#define LED_SCAN_BG 10
#define LED_SCAN_STEPF 12
static int led_scan_next(struct led_sync *L) /* next grid index to scan, -1 when the pass is done */
{
	if (L->scan_fine < 0) { /* coarse pass: indices c, c+3, c-3, ... that are not scanned yet */
		for (; L->scan_k < LED_SCAN_COARSE; L->scan_k++) {
			int off = (L->scan_k + 1) / 2 * 3; if (L->scan_k % 2 == 0) off = -off;
			int idx = ((L->scan_c + off) % LED_SCAN_N + LED_SCAN_N) % LED_SCAN_N;
			if (!L->scanned[idx]) { L->scan_k++; return idx; }
		}
		return -1;
	}
	for (; L->scan_k < 5; L->scan_k++) { /* fine pass: the two grid steps either side of the best coarse step */
		int off = L->scan_k == 0 ? 1 : L->scan_k == 1 ? -1 : L->scan_k == 2 ? 2 : L->scan_k == 3 ? -2 : 0;
		int idx = ((L->scan_fine + off) % LED_SCAN_N + LED_SCAN_N) % LED_SCAN_N;
		if (!L->scanned[idx]) { L->scan_k++; return idx; }
	}
	return -1;
}
static void led_scan_frame(int total, uint64_t now, int par)
{
	int scanning = -1;
	for (int h = 0; h < 2; h++) if (O.led[h].scan) scanning = h;
	if (scanning < 0) {
		if (O.led_mode || O.ld_frames < 60) return;
		for (int h = 0; h < 2; h++) {
			struct led_sync *L = &O.led[h]; uint32_t dummy;
			if ((int64_t)(now - L->in_us) > 1000000 || !led_pos(L, h, 0, &dummy)) continue; /* no controller or no clocks yet (in_us may be newer than this frame's time) */
			if (now - O.lock_us[h] < 5000000 || L->probe) continue;           /* tracking (or the stored latency's grace period) */
			if (!L->scan_wait_us) L->scan_wait_us = 5000000;
			if (now - L->scan_done_us < L->scan_wait_us) continue;
			/* with a stored latency, first check whether the flash is visible at all with it (step -1); a visible
			 * flash means the timing is fine and the missing lock has another cause (too close, occluded) */
			L->scan = true; L->on = false; L->step = L->lat_ok ? -1 : 0; L->f0 = 0; L->dirty = true;
			L->bsum = L->psum = 0; L->bcnt = L->pcnt = 0; memset(L->pbase, 0, sizeof L->pbase); memset(L->pdelta, 0, sizeof L->pdelta);
			memset(L->scanned, 0, sizeof L->scanned); memset(L->cnt, 0, sizeof L->cnt); L->scan_k = 0; L->scan_fine = -1;
			L->scan_c = L->lat_ok ? (int)((L->lat + LED_SCAN_STEP / 2) / LED_SCAN_STEP) % LED_SCAN_N : 0; /* start next to the stored latency: a new connection shifts it by a few ms */
			if (!L->lat_ok) { L->step = led_scan_next(L); slog("optical: %c LED sync scan", h ? 'R' : 'L'); }
			break;
		}
		return;
	}
	struct led_sync *L = &O.led[scanning];
	if (L->step >= 0 && L->on && L->f0 >= 4 && now - O.lock_us[scanning] < 50000) { /* the hand got fitted while flashing at this step: that latency works, take it and stop */
		L->scan = false; L->scan_done_us = now; L->scan_wait_us = 5000000; L->lat_seen_us = now; L->dark_checks = 0; L->dark_scans = 0; L->dirty = true;
		int32_t newlat = L->step * LED_SCAN_STEP;
		if (!L->lat_ok || newlat != L->lat) { L->lat = newlat; L->lat_ok = true; L->lat_us = now; led_save(scanning); }
		L->period_cur = (int)T.led_period; L->centred = 0;
		L->probe_last_us = now;
		slog("optical: %c LED sync: fitted while scanning at latency %d us, taking it", scanning ? 'R' : 'L', L->lat);
		return;
	}
	/* all frames count: the flash of a hand that has not been seen for a while may have moved to the other parity (the frame it lights
	 * depends on where the exposure sits relative to the latency, which shifts with the per-connection clock bias and with re-anchors);
	 * the other hand's fitted blobs are already subtracted from `total`, and means over the window absorb the rest */
	unsigned n = ++L->f0; /* frames since the phase began */
	if (!L->on) { /* LEDs off: background blob count (mean over the settled frames; a max is too noisy next to a moving controller) */
		if (n >= 4) { L->bsum += total; L->bcnt++; if (total > L->pbase[par]) L->pbase[par] = total; }
		if (n >= (L->step < 0 ? LED_SCAN_OFF : LED_SCAN_BG)) { L->on = true; L->f0 = 0; L->psum = 0; L->pcnt = 0; memset(L->ppeak, 0, sizeof L->ppeak); L->dirty = true; }
		return;
	}
	/* LEDs flashing (the check: at the stored latency; the scan: at the current step) */
	if (n >= 6) { L->psum += total; L->pcnt++; if (total > L->ppeak[par]) L->ppeak[par] = total; }
	if (n < (unsigned)(L->step < 0 ? LED_SCAN_ON : LED_SCAN_STEPF)) return;
	double base = L->bcnt ? L->bsum / L->bcnt : 0, peak = L->pcnt ? L->psum / L->pcnt : 0; int contrast = (int)lround(peak - base);
	L->base = (int)lround(base); L->peak = (int)lround(peak);
	for (int p = 0; p < 2; p++) { int d = L->ppeak[p] - L->pbase[p]; if (d > 0 && contrast >= 3) L->pdelta[p] += d; }
	if (L->step < 0) { /* the check */
		if (contrast >= 3) { /* visible with the stored timing: no scan */
			L->scan = false; L->scan_done_us = now; L->scan_wait_us = 20000000; L->lat_seen_us = now; L->dark_checks = 0; L->dirty = true;
			if (OPT_DUMP) slog("optical: %c LED check: flash visible with latency %d (%d -> %d blobs; parity %d: %d -> %d, parity %d: %d -> %d), no scan", scanning ? 'R' : 'L', L->lat, L->base, L->peak, 0, L->pbase[0], L->ppeak[0], 1, L->pbase[1], L->ppeak[1]);
			if (led_mux()) led_par_from_counts(scanning, now, "LED check");
			return;
		}
		if (now - L->lat_seen_us < 60000000 && L->dark_checks < 2) { /* the timing worked a moment ago: probably out of view; but the clock may also have walked the flash out of the exposure, so only twice */
			L->dark_checks++;
			L->scan = false; L->scan_done_us = now; L->scan_wait_us = 5000000; L->dirty = true;
			if (OPT_DUMP) slog("optical: %c LED check: nothing visible with latency %d (%d -> %d blobs), it worked %llu s ago: out of view? (check %d of 2 before a scan)", scanning ? 'R' : 'L', L->lat, L->base, L->peak, (unsigned long long)((now - L->lat_seen_us) / 1000000), L->dark_checks);
			return;
		}
		slog("optical: %c LED sync scan (nothing visible with the stored latency %d: %d -> %d blobs)", scanning ? 'R' : 'L', L->lat, L->base, L->peak);
		L->scan_k = 0; L->scan_fine = -1; L->step = led_scan_next(L); L->f0 = 0; L->psum = 0; L->pcnt = 0; L->dirty = true; memset(L->pdelta, 0, sizeof L->pdelta); /* the background stays: straight into the coarse pass */
		return;
	}
	L->cnt[L->step] = (int8_t)CLAMP(contrast, -100, 100); L->scanned[L->step] = true;
	if (OPT_DUMP) slog("optical: %c scan step %d lat %d pos %u base %d peak %d (coff %lld ldoff %lld)", scanning ? 'R' : 'L', L->step, L->step * LED_SCAN_STEP, L->pos, L->base, L->peak, (long long)clk_at(&L->clk, now), (long long)clk_at(&O.ldclk, now));
	L->f0 = 0; L->psum = 0; L->pcnt = 0; memset(L->ppeak, 0, sizeof L->ppeak); L->dirty = true;
	if (L->scan_fine < 0) { /* coarse pass: a lit step bounded by dark coarse neighbours (or the pass exhausted) -> fine pass around the best lit step */
		int best = -1, bestc = 2;
		for (int i = 0; i < LED_SCAN_N; i++) if (L->scanned[i] && L->cnt[i] > bestc) { bestc = L->cnt[i]; best = i; } /* the coarse indices are scan_c +- 3k */
		bool bounded = best >= 0 && L->scanned[(best + 3) % LED_SCAN_N] && L->cnt[(best + 3) % LED_SCAN_N] < 3 && L->scanned[(best + LED_SCAN_N - 3) % LED_SCAN_N] && L->cnt[(best + LED_SCAN_N - 3) % LED_SCAN_N] < 3;
		int next = led_scan_next(L);
		if (best >= 0 && (bounded || next < 0)) { L->scan_fine = best; L->scan_k = 0; next = led_scan_next(L); }
		if (next < 0 && best < 0) { led_scan_finish(scanning, now); return; } /* nothing lit anywhere */
		L->step = next;
		return;
	}
	int next = led_scan_next(L);
	if (next < 0) led_scan_finish(scanning, now); /* the parity is measured by the LED check that follows (a scan sweeps the flash across frame boundaries) */
	else L->step = next;
}

/* Edge-probe servo (O.lock held, once per LD frame for each connected controller; n: LEDs fitted this frame).
 * Every 15 s while tracking, the flash window is shifted by +-1000 µs for a few frames each.  Losing the LEDs on one side means the exposure has
 * drifted toward that edge, so the latency is corrected the other way.  While the exposure sits within
 * +-600 µs of the window centre both probes keep every LED lit and tracking is undisturbed. */
#define LED_PROBE_FRAMES 5 /* 3 frames for the new timing to show (a Bluetooth round trip and a cycle), 2 measured */
/* Flash window width.  48 slots (5 ms) while the timing is being found or the hand is not tracked; once tracked, the narrow
 * steady-state window (ledPeriodStable, Sony's BG/STABLE phases use 20/9): the LEDs run at a third of the duty cycle.  A Sense that
 * flashes 5 ms every 2 frames for minutes goes dark for a while (both controllers did today; valid commands, no flash at any timing) */
static int led_period_now(const struct led_sync *L, uint64_t now) { return (!L->lat_ok || L->scan || now - L->lat_seen_us > 2000000 || L->period_cur < 6) ? (int)T.led_period : L->period_cur; } /* narrowed step by step by the servo once the exposure is centred */
static int32_t led_probe_off(const struct led_sync *L, uint64_t now) { int w = led_period_now(L, now) * LED_SLOT; return w / 2 - 450 > 250 ? w / 2 - 450 : 250; } /* the probe must keep the exposure inside the window with 450 us to spare for clock jitter */
static void led_servo(int h, int n, uint64_t now)
{
	struct led_sync *L = &O.led[h];
	if (O.led_mode || L->scan || !L->lat_ok) return;
	if (!L->probe) {
		if (now - O.lock_us[h] > 200000 || now - L->probe_last_us < 15000000) return; /* with exact timing on the wire the drift is tiny: probe rarely (a probe at the window edge costs the LEDs for a few frames) */
		L->probe = 1; L->probe_f0 = O.ld_frames; L->probe_k = 0; memset(L->probe_sum, 0, sizeof L->probe_sum); memset(L->probe_cnt, 0, sizeof L->probe_cnt); L->dirty = true;
		return;
	}
	unsigned k = L->probe_k++; /* this hand's frames only (every other frame with ledMux) */
	int side = L->probe > 0 ? 0 : 1;
	if (k >= 3) { L->probe_sum[side] += n; L->probe_cnt[side]++; } /* the new timing takes a few frames to show */
	if (k + 1 < LED_PROBE_FRAMES) return;
	if (L->probe > 0) { L->probe = -1; L->probe_f0 = O.ld_frames; L->probe_k = 0; L->dirty = true; return; }
	double m[2]; for (int i = 0; i < 2; i++) m[i] = L->probe_cnt[i] ? L->probe_sum[i] / L->probe_cnt[i] : 0;
	int step = 0, per = L->period_cur < 6 ? (int)T.led_period : L->period_cur;
	if (m[0] >= 3 || m[1] >= 3) {
		int32_t st = led_probe_off(L, now) / 2;
		if (m[0] < 0.5 * m[1]) step = -st;     /* late window lost the LEDs: the exposure is earlier than lat */
		else if (m[1] < 0.5 * m[0]) step = st; /* early window lost them: later */
	}
	if (step) {
		L->lat = (L->lat + step + LED_CYCLE_US) % LED_CYCLE_US; L->lat_us = now; L->centred = 0;
		led_save(h);
		slog("optical: %c LED sync: exposure drifted, latency %+d -> %d us (probe %.1f / %.1f LEDs, window %d slots)", h ? 'R' : 'L', step, L->lat, m[0], m[1], per);
	} else if (m[0] >= 3 && m[1] >= 3) { /* both edges keep the LEDs: the exposure is centred; after two such probes the window can narrow one step (42 -> 36 -> 30 -> stable) */
		if (++L->centred >= 2 && per > (int)T.led_period_stable) {
			int next = per > 36 ? 36 : per > 30 ? 30 : (int)T.led_period_stable; if (next < (int)T.led_period_stable) next = (int)T.led_period_stable;
			L->period_cur = next; L->centred = 0; L->dirty = true;
			slog("optical: %c LED sync: exposure centred, flash window %d -> %d slots", h ? 'R' : 'L', per, next);
		}
	} else if (m[0] < 3 && m[1] < 3 && per < (int)T.led_period) { /* both probes dark: lost the exposure, back to the wide window */
		L->period_cur = (int)T.led_period; L->centred = 0; L->dirty = true;
		slog("optical: %c LED sync: LEDs lost at both probes, flash window back to %d slots", h ? 'R' : 'L', (int)T.led_period);
	}
	L->probe = 0; L->probe_last_us = step ? now - 12000000 : now; L->dirty = true; /* re-check 3 s after a correction, else in 15 s */
}

/* sense.c: LED block of the output report (every 10.7 ms) */
static struct led_cmd optical_led_cmd(int hand, uint64_t now)
{
	pthread_mutex_lock(&O.lock);
	struct led_sync *L = &O.led[hand];
	uint8_t phase; int32_t lat;
	if (L->probe && (L->scan || O.ld_frames - L->probe_f0 > 60)) { L->probe = 0; L->probe_last_us = now; L->dirty = true; } /* no frames: give up */
	bool freerun = false; /* PRESCAN flash with a cycle 1/60 longer than the camera's: the flash drifts through the exposure about once a second ("is the controller in view at all?").  Measured: LED_ALL_ON (6) leaves the tracking LEDs dark, and a PRESCAN base time of 0 gives no flash either, so every flash needs the clock sync */
	if (L->offtest_us && now < L->offtest_us) { phase = LED_PHASE_OFF; lat = 0; } /* stuck-LED test */
	else if (L->recover_until_us && now < L->recover_until_us) { phase = now + 1000000 < L->recover_until_us ? LED_PHASE_OFF : 0; lat = 0; } /* recovery: OFF for a second, INIT (0) for a second */
	else if (L->recover_until_us) { L->recover_until_us = 0; L->dirty = true; phase = LED_PHASE_PRESCAN; lat = L->lat; }
#if SLOP_DBG
	else if (O.led_mode == LED_PHASE_OFF) { phase = LED_PHASE_OFF; lat = 0; }
	else if (O.led_mode == 1) { phase = 0; lat = 0; } /* experiment: INIT */
	else if (O.led_mode == LED_PHASE_ON) { phase = LED_PHASE_PRESCAN; lat = L->lat; freerun = true; }
	else if (O.led_mode == 7) { /* experiment: does the controller honour re-anchors?  flip the flash by half a cycle every 2 s */
		int par = (int)(now / 2000000 % 2); phase = LED_PHASE_PRESCAN; lat = L->lat + par * (LED_CYCLE_US / 2);
		if (par != L->exp_par) { L->exp_par = par; L->dirty = true; }
	}
#endif
	else if (L->scan) { phase = !L->on ? LED_PHASE_OFF : LED_PHASE_PRESCAN; lat = L->step < 0 ? L->lat : L->step * LED_SCAN_STEP; }
	else if (O.led[!hand].scan && O.led[!hand].step >= 0) { phase = LED_PHASE_OFF; lat = 0; } /* the other hand is scanning its latency (3 s, rare): stay dark so the scan sees only its own flash.  A hand coming into view unfitted during the other's scan produced a false latency (saved, and wrong at the next start) */
	else { phase = LED_PHASE_PRESCAN; lat = L->lat + (int32_t)(hand ? T.lat_trim_R : T.lat_trim_L) + L->probe * led_probe_off(L, now); }
	int period = led_period_now(L, now);
	if (period != L->period) { L->period = period; L->dirty = true; }
	L->cmd_us = now;
	if (phase == LED_PHASE_PRESCAN && !(clk_ready(&L->clk) && clk_ready(&O.ldclk))) { /* no clock sync yet: nothing useful can be commanded (dark until synced, ~3 s) */
		phase = LED_PHASE_OFF; freerun = false;
		if (now - L->nosync_log_us > 5000000) { L->nosync_log_us = now; slog("optical: %c LED timing: no clock sync yet (controller clock %d samples, headset clock %d) -> LEDs dark until synced", hand ? 'R' : 'L', L->clk.n, O.ldclk.n); }
	}
	/* re-anchor interval: every re-anchor restarts the controller's flash schedule (a glitch of a frame or two); with the cycle length
	 * tuned to the controller's crystal (below) the schedule stays aligned for seconds, so re-anchor every 4 s, else every 1.1 s like Sony */
	bool rate_known = L->clk.n >= 40 && O.ldclk.n >= 30;
	if (phase != L->phase || L->dirty || freerun != L->freerun || (phase == LED_PHASE_PRESCAN && now - L->anchor_us > (rate_known ? 4000000u : 1100000u))) {
		uint32_t pos = 0;
		if (phase == LED_PHASE_PRESCAN && !led_pos(L, hand, lat, &pos)) { /* no exposure to aim at (no detector frames for 0.5 s): dark until they return, without a new command every report (the dirty flag stays, the PRESCAN anchor follows the first frame) */
			if (L->phase != LED_PHASE_OFF) { L->phase = LED_PHASE_OFF; L->pos = 0; L->seq++; L->anchor_us = now; L->freerun = false; rec_ledcmd(hand, LED_PHASE_OFF, L->seq, (uint8_t)O.led_period, L->par_fix, false, 0, 0, 0, now); }
			goto send;
		}
		L->phase = phase; L->pos = pos; L->seq++; L->anchor_us = now; L->dirty = false; L->freerun = freerun;
		{ uint32_t cyc = O.led_cycle_ns ? O.led_cycle_ns : LED_CYCLE_NS; rec_ledcmd(hand, phase, L->seq, (uint8_t)(phase == LED_PHASE_PRESCAN ? L->period : O.led_period), L->par_fix, freerun, pos, freerun ? cyc + cyc / 60 : led_mux() ? 2 * cyc : cyc, lat, now); }
		if (OPT_DUMP && phase == LED_PHASE_PRESCAN && now - L->cmd_log_us > 5000000) { /* where does the commanded base time sit relative to the controller's own clock? (a base far in the past gives no flash at all) */
			L->cmd_log_us = now;
			int64_t ctrl_now = (int64_t)now + clk_at(&L->clk, now); int32_t ahead_ticks = (int32_t)(pos - (uint32_t)((uint64_t)(ctrl_now * 3) & 0xffffffffu));
			slog("optical: %c LED cmd: PRESCAN seq %u base %u ticks = %+.1f ms from the controller's now (lat %d, cycle x%d, clk n=%d rtt %u)", hand ? 'R' : 'L', L->seq, pos, ahead_ticks / 3e3, lat, L->freerun ? 1 : led_mux() ? 2 : 1, L->clk.n, L->clk.cur_rtt);
		}
	}
send:;
	uint32_t cycle = O.led_cycle_ns ? O.led_cycle_ns : LED_CYCLE_NS;
	{ /* the controller's flash period in its own clock: the headset frame cycle scaled by the two crystals' rates against the host
	   * (both clock models give µs/s of drift), so the schedule stays on the exposure between re-anchors (Sony's "tunedCycle") */
		double ppm = rate_known ? L->clk.b - O.ldclk.b : 0;
		if (ppm > -300 && ppm < 300) cycle = (uint32_t)llround(cycle * (1.0 + ppm * 1e-6));
	}
	uint32_t wire = (L->freerun ? cycle + cycle / 60 : led_mux() ? 2 * cycle : cycle) * 3; /* thirds of a nanosecond */
	struct led_cmd c = { L->phase, L->seq, (uint8_t)(L->phase == LED_PHASE_PRESCAN ? L->period : O.led_period), L->pos, wire };
	pthread_mutex_unlock(&O.lock);
	return c;
}

/* ---- 3x3 row-major helpers ---- */
static void m3_from_q(quat q, double R[9])
{
	double xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z, xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z,
	       wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
	R[0] = 1 - 2 * (yy + zz); R[1] = 2 * (xy - wz); R[2] = 2 * (xz + wy);
	R[3] = 2 * (xy + wz); R[4] = 1 - 2 * (xx + zz); R[5] = 2 * (yz - wx);
	R[6] = 2 * (xz - wy); R[7] = 2 * (yz + wx); R[8] = 1 - 2 * (xx + yy);
}
static quat q_from_m3(const double R[9])
{
	double tr = R[0] + R[4] + R[8]; quat q;
	if (tr > 0) { double s = sqrt(tr + 1) * 2; q = (quat){ 0.25 * s, (R[7] - R[5]) / s, (R[2] - R[6]) / s, (R[3] - R[1]) / s }; }
	else if (R[0] > R[4] && R[0] > R[8]) { double s = sqrt(1 + R[0] - R[4] - R[8]) * 2; q = (quat){ (R[7] - R[5]) / s, 0.25 * s, (R[1] + R[3]) / s, (R[2] + R[6]) / s }; }
	else if (R[4] > R[8]) { double s = sqrt(1 + R[4] - R[0] - R[8]) * 2; q = (quat){ (R[2] - R[6]) / s, (R[1] + R[3]) / s, 0.25 * s, (R[5] + R[7]) / s }; }
	else { double s = sqrt(1 + R[8] - R[0] - R[4]) * 2; q = (quat){ (R[3] - R[1]) / s, (R[2] + R[6]) / s, (R[5] + R[7]) / s, 0.25 * s }; }
	return qnorm(q);
}
static void m3_mul(const double A[9], const double B[9], double C[9])
{
	double T[9];
	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) T[i * 3 + j] = A[i * 3] * B[j] + A[i * 3 + 1] * B[3 + j] + A[i * 3 + 2] * B[6 + j];
	memcpy(C, T, sizeof T);
}
static void m3_mulv(const double A[9], const double v[3], double o[3])
{
	double t[3] = { A[0] * v[0] + A[1] * v[1] + A[2] * v[2], A[3] * v[0] + A[4] * v[1] + A[5] * v[2], A[6] * v[0] + A[7] * v[1] + A[8] * v[2] };
	memcpy(o, t, sizeof t);
}
static void m3_tmulv(const double A[9], const double v[3], double o[3]) /* Aᵀ v */
{
	double t[3] = { A[0] * v[0] + A[3] * v[1] + A[6] * v[2], A[1] * v[0] + A[4] * v[1] + A[7] * v[2], A[2] * v[0] + A[5] * v[1] + A[8] * v[2] };
	memcpy(o, t, sizeof t);
}
static double v3_dot(const double a[3], const double b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
static double v3_norm(const double a[3]) { return sqrt(v3_dot(a, a)); }
static void v3_unit(double a[3]) { double n = v3_norm(a); if (n > 1e-12) { a[0] /= n; a[1] /= n; a[2] /= n; } }

/* ---- factory calibration (GenData item 10, "SIE\1") ---- */
static void opt_set_calib(const uint8_t *b, int len)
{
	if (len < 0x444 || memcmp(b, "SIE", 3) != 0) return;
	rec_calib(b, len);
	struct opt_cam cams[OPT_NCAM] = { 0 };
	for (int i = 0; i < OPT_NCAM; i++) {
		const uint8_t *r = b + 0x18 + i * 0xe4;
		/* record: u8 cam index, u16 w, u16 h, float K[9], then 20 doubles: OpenCV's 14 (k1 k2 p1 p2 k3 k4 k5 k6 s1..s4 tx ty),
		 * 3 per-camera values of unknown use, and the lens polynomial r/f = a th + b th^3 + c th^5 (identical for all cameras) */
		uint32_t idx; float K[9]; double d[20];
		memcpy(&idx, r, 4); memcpy(K, r + 8, sizeof K); memcpy(d, r + 44, sizeof d);
		if (idx >= OPT_NCAM || !(K[0] > 100.f)) return;
		struct opt_cam *c = &cams[idx];
		c->fx = K[0]; c->fy = K[4]; c->cx = K[2]; c->cy = K[5];
		c->k[0] = d[0]; c->k[1] = d[1]; c->k[2] = d[4]; c->k[3] = d[5]; c->k[4] = d[6]; c->k[5] = d[7];
		c->kb[0] = d[17]; c->kb[1] = d[18]; c->kb[2] = d[19];
		if (!(c->kb[0] > 0.5 && c->kb[0] < 1.5)) { c->kb[0] = 1; c->kb[1] = c->kb[2] = 0; } /* unexpected layout: plain equidistant, which is within 2 % anyway */
	}
	/* extrinsic records {u8 from, u8 to, u16, float R[9], float t[3]}: X_to = R X_from + t; chain from cam0 */
	struct { int from, to; double R[9], t[3]; } rec[3];
	for (int i = 0; i < 3; i++) {
		const uint8_t *r = b + 0x3a8 + i * 52; float R[9], t[3];
		rec[i].from = r[0]; rec[i].to = r[1];
		memcpy(R, r + 4, sizeof R); memcpy(t, r + 40, sizeof t);
		for (int k = 0; k < 9; k++) rec[i].R[k] = R[k];
		for (int k = 0; k < 3; k++) rec[i].t[k] = t[k];
	}
	cams[0].ok = true; cams[0].R[0] = cams[0].R[4] = cams[0].R[8] = 1;
	for (int pass = 0; pass < 3; pass++)
		for (int i = 0; i < 3; i++) {
			int f = rec[i].from, t = rec[i].to;
			if (f >= OPT_NCAM || t >= OPT_NCAM || cams[f].ok == cams[t].ok) continue;
			if (cams[f].ok) { /* T_c0<-to = T_c0<-from * T_from<-to, T_from<-to = (Rᵀ, -Rᵀ t) */
				double Rt[9] = { rec[i].R[0], rec[i].R[3], rec[i].R[6], rec[i].R[1], rec[i].R[4], rec[i].R[7], rec[i].R[2], rec[i].R[5], rec[i].R[8] }, o[3], tmp[3];
				m3_mulv(Rt, rec[i].t, o); for (int k = 0; k < 3; k++) o[k] = -o[k];
				m3_mul(cams[f].R, Rt, cams[t].R);
				m3_mulv(cams[f].R, o, tmp); for (int k = 0; k < 3; k++) cams[t].o[k] = cams[f].o[k] + tmp[k];
				cams[t].ok = true;
			} else {          /* T_c0<-from = T_c0<-to * T_to<-from, T_to<-from = (R, t) */
				double tmp[3];
				m3_mul(cams[t].R, rec[i].R, cams[f].R);
				m3_mulv(cams[t].R, rec[i].t, tmp); for (int k = 0; k < 3; k++) cams[f].o[k] = cams[t].o[k] + tmp[k];
				cams[f].ok = true;
			}
		}
	pthread_mutex_lock(&O.lock);
	memcpy(O.cam, cams, sizeof cams);
	O.calib_ok = cams[0].ok && cams[1].ok && cams[2].ok && cams[3].ok;
	pthread_mutex_unlock(&O.lock);
	slog("optical: camera calibration %s (cam0 fx %.1f cx %.1f cy %.1f, lens %.4f th + %.4f th^3 + %.2g th^5; cam1 at %.3f %.3f %.3f)", O.calib_ok ? "ok" : "incomplete",
	     cams[0].fx, cams[0].cx, cams[0].cy, cams[0].kb[0], cams[0].kb[1], cams[0].kb[2], cams[1].o[0], cams[1].o[1], cams[1].o[2]);
}

/* Lens model: a near-equidistant fisheye, r_px / f = a th + b th^3 + c th^5 (th = angle off the optical axis), the polynomial
 * stored at the end of each camera record (0.9716, 0.0367, 0.00023 on this unit: 76 deg at the image edge).  The OpenCV-style
 * rational block in the same record is NOT the LD camera model: applied to tan(th) it is off by 5+ degrees beyond 45 deg, which
 * is why multi-camera rays stopped meeting as soon as a controller left the centre of the images.
 * r below is the pinhole radius tan(th); r_d the normalized pixel radius r_px / f. */
static double opt_distort_r(const struct opt_cam *c, double r)
{
	double th = atan(r), t2 = th * th;
	return th * (c->kb[0] + t2 * (c->kb[1] + t2 * c->kb[2]));
}
static bool opt_undistort(const struct opt_cam *c, double u, double v, double n[2])
{
	double xd = (u - c->cx) / c->fx, yd = (v - c->cy) / c->fy, rd = sqrt(xd * xd + yd * yd);
	if (rd < 1e-9) { n[0] = n[1] = 0; return true; }
	double th = rd / c->kb[0]; /* Newton on p(th) = rd */
	for (int i = 0; i < 30; i++) {
		double t2 = th * th, f = th * (c->kb[0] + t2 * (c->kb[1] + t2 * c->kb[2])) - rd, df = c->kb[0] + t2 * (3 * c->kb[1] + 5 * t2 * c->kb[2]);
		if (df < 1e-9) return false;
		double step = f / df;
		th -= step;
		if (th < 0) th = 1e-6;
		if (fabs(step) < 1e-10) break;
	}
	if (!(th > 0) || th > 1.45) return false; /* > 83 deg off-axis: outside the lens */
	double r = tan(th);
	n[0] = xd * r / rd; n[1] = yd * r / rd;
	return true;
}
/* normalized (pinhole) coords -> fisheye image pixels.  Residuals are measured here: at 50 deg off-axis a pixel is 2.4x
 * smaller in the normalized plane, so pixel gates expressed as fx * dn would shrink towards the image edge */
static void opt_cam_px(const struct opt_cam *c, const double n[2], double uv[2])
{
	double r = hypot(n[0], n[1]), s = r > 1e-9 ? opt_distort_r(c, r) / r : c->kb[0];
	uv[0] = c->cx + c->fx * n[0] * s; uv[1] = c->cy + c->fy * n[1] * s;
}
/* project a cam0-frame point into camera c: normalized coords; false if behind */
static bool opt_project(const struct opt_cam *c, const double X[3], double n[2])
{
	double d[3] = { X[0] - c->o[0], X[1] - c->o[1], X[2] - c->o[2] }, p[3];
	m3_tmulv(c->R, d, p);
	if (p[2] < 0.03) return false;
	n[0] = p[0] / p[2]; n[1] = p[1] / p[2];
	return true;
}

/* ---- LD packet -> rays -> stereo candidates ---- */
static void opt_cluster(void);
static void opt_parse_ld(const uint8_t *b, int len)
{
	O.nray = 0;
	for (int cam = 0; cam < OPT_NCAM; cam++) {
		const struct opt_cam *c = &O.cam[cam];
		const uint8_t *blk = b + 0x40 + cam * OPT_LD_CAM_BLOCK;
		if (blk + 4 > b + len) break;
		uint32_t n; memcpy(&n, blk, 4);
		if (n > OPT_MAX_BLOB) n = OPT_MAX_BLOB;
		for (uint32_t i = 0; i < n && O.nray < OPT_MAX_RAY; i++) {
			const uint8_t *r = blk + 4 + i * OPT_LD_REC;
			if (r + OPT_LD_REC > b + len) break;
			uint16_t valid, xmin, xmax, ymin, ymax; uint32_t m00, m10, m01;
			memcpy(&valid, r, 2); memcpy(&xmin, r + 4, 2); memcpy(&xmax, r + 6, 2); memcpy(&ymin, r + 8, 2); memcpy(&ymax, r + 10, 2);
			memcpy(&m00, r + 12, 4); memcpy(&m10, r + 16, 4); memcpy(&m01, r + 20, 4);
			if (!valid || !m00) continue;
			if (xmax - xmin > T.blob_max_px || ymax - ymin > T.blob_max_px) continue; /* sunlight / lamps: far bigger than an LED */
			double u = 2.0 * (xmin + (double)m10 / m00) + 0.5 + OPT_LD_XOFF, v = 2.0 * (ymin + (double)m01 / m00) + 0.5 + OPT_LD_YOFF;
			struct opt_ray *ray = &O.rays[O.nray];
			if (!opt_undistort(c, u, v, ray->n)) continue;
			double d[3] = { ray->n[0], ray->n[1], 1.0 };
			v3_unit(d);
			m3_mulv(c->R, d, ray->d);
			memcpy(ray->o, c->o, sizeof ray->o);
			ray->cam = (uint8_t)cam; ray->blob = (uint8_t)i; ray->used = false; ray->inten = (float)m00; ray->uv[0] = u; ray->uv[1] = v;
			O.nray++;
		}
	}
	O.ncand = 0;
	for (int a = 0; a < O.nray && O.ncand < OPT_MAX_CAND; a++)
		for (int bb = a + 1; bb < O.nray && O.ncand < OPT_MAX_CAND; bb++) {
			const struct opt_ray *ra = &O.rays[a], *rb = &O.rays[bb];
			if (ra->cam == rb->cam) continue;
			double w[3] = { ra->o[0] - rb->o[0], ra->o[1] - rb->o[1], ra->o[2] - rb->o[2] };
			double bd = v3_dot(ra->d, rb->d), dd = v3_dot(ra->d, w), e = v3_dot(rb->d, w), den = 1 - bd * bd;
			if (den < 1e-4) continue;
			double s = (bd * e - dd) / den, t = (e - bd * dd) / den;
			if (s < 0.08 || t < 0.08 || s > 2.5 || t > 2.5) continue;
			double pa[3], pb[3], g[3];
			for (int k = 0; k < 3; k++) { pa[k] = ra->o[k] + s * ra->d[k]; pb[k] = rb->o[k] + t * rb->d[k]; g[k] = pa[k] - pb[k]; }
			double gap = v3_norm(g);
			if (gap > OPT_CAND_GAP + 0.01 * s) continue;
			struct opt_cand *cd = &O.cand[O.ncand++];
			for (int k = 0; k < 3; k++) cd->x[k] = 0.5 * (pa[k] + pb[k]);
			cd->gap = (float)gap; cd->ray[0] = (int16_t)a; cd->ray[1] = (int16_t)bb; cd->used = false;
		}
	opt_cluster();
}

/* ---- pose fit for one controller ---- */
static void opt_model(const double R[9], bool mirror, double Q[OPT_NLED][3], double N[OPT_NLED][3])
{
	for (int i = 0; i < OPT_NLED; i++) {
		double p[3] = { (mirror ? -1 : 1) * opt_led_pos[i][0] * 1e-3, opt_led_pos[i][1] * 1e-3, opt_led_pos[i][2] * 1e-3 };
		double n[3] = { (mirror ? -1 : 1) * opt_led_nrm[i][0], opt_led_nrm[i][1], opt_led_nrm[i][2] };
		m3_mulv(R, p, Q[i]); m3_mulv(R, n, N[i]);
	}
}
static bool opt_facing(const double N[3], const double X[3], const double o[3])
{
	double d[3] = { o[0] - X[0], o[1] - X[1], o[2] - X[2] };
	v3_unit(d);
	return v3_dot(N, d) > OPT_FACING;
}
/* reprojection residuals (px) of the observation list for pose (R,t) */
static int opt_residuals(const struct opt_pose *P, bool mirror, double *res)
{
	double Q[OPT_NLED][3], N[OPT_NLED][3];
	opt_model(P->R, mirror, Q, N);
	int m = 0;
	for (int i = 0; i < P->nobs; i++) {
		const struct opt_ray *ray = &O.rays[P->obs[i].ray];
		const struct opt_cam *c = &O.cam[ray->cam];
		double X[3] = { Q[P->obs[i].led][0] + P->t[0], Q[P->obs[i].led][1] + P->t[1], Q[P->obs[i].led][2] + P->t[2] }, n[2];
		if (!opt_project(c, X, n)) { res[m++] = 50; res[m++] = 50; continue; }
		double uv[2]; opt_cam_px(c, n, uv);
		res[m++] = uv[0] - ray->uv[0];
		res[m++] = uv[1] - ray->uv[1];
	}
	return m;
}
static bool solve6(double A[6][6], double b[6], double x[6])
{
	for (int i = 0; i < 6; i++) {
		int p = i;
		for (int r = i + 1; r < 6; r++) if (fabs(A[r][i]) > fabs(A[p][i])) p = r;
		if (fabs(A[p][i]) < 1e-12) return false;
		if (p != i) { for (int k = 0; k < 6; k++) { double t = A[i][k]; A[i][k] = A[p][k]; A[p][k] = t; } double t = b[i]; b[i] = b[p]; b[p] = t; }
		for (int r = i + 1; r < 6; r++) {
			double f = A[r][i] / A[i][i];
			for (int k = i; k < 6; k++) A[r][k] -= f * A[i][k];
			b[r] -= f * b[i];
		}
	}
	for (int i = 5; i >= 0; i--) { double s = b[i]; for (int k = i + 1; k < 6; k++) s -= A[i][k] * x[k]; x[i] = s / A[i][i]; }
	return true;
}
/* Gauss-Newton over (rotation vector, translation) on reprojection error; returns RMS px */
/* Orientation prior for the refinements: the IMU knows how the controller is turned (tracked: the last fit turned by
 * the IMU's motion; acquisition: gravity + yaw).  The ring's near-symmetric correspondences sit 25.7 deg apart and a
 * free 6-DoF refinement happily walks over to one of them; the prior (px per rad) makes that cost more than it gains
 * while still letting the image data correct a few degrees of IMU drift. */
static const double *opt_prior_R; static double opt_prior_w;
static void opt_prior_term(const double R[9], double w, double A[6][6], double b[6]) /* left-increment rotation vector from the prior to R */
{
	if (!opt_prior_R || w <= 0) return;
	double Rt[9], D[9]; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) Rt[i * 3 + j] = opt_prior_R[j * 3 + i];
	m3_mul(R, Rt, D); quat q = q_from_m3(D); double n = sqrt(q.x * q.x + q.y * q.y + q.z * q.z), ang = 2 * atan2(n, fabs(q.w)), sg = q.w < 0 ? -1 : 1;
	double rv[3] = { 0, 0, 0 }; if (n > 1e-12) { rv[0] = sg * q.x / n * ang; rv[1] = sg * q.y / n * ang; rv[2] = sg * q.z / n * ang; }
	for (int p = 0; p < 3; p++) { A[p][p] += w * w; b[p] -= w * w * rv[p]; }
}
static double opt_refine(struct opt_pose *P, bool mirror, int iters)
{
	double r0[2 * OPT_MAX_OBS], r1[2 * OPT_MAX_OBS], J[2 * OPT_MAX_OBS][6];
	int m = 0;
	for (int it = 0; it < iters; it++) {
		m = opt_residuals(P, mirror, r0);
		if (m < 6) break;
		for (int p = 0; p < 6; p++) {
			struct opt_pose T = *P;
			double eps = p < 3 ? 1e-4 : 1e-4;
			if (p < 3) { double dv[3] = { 0, 0, 0 }, Rd[9]; dv[p] = eps; m3_from_q(qrotvec(dv[0], dv[1], dv[2]), Rd); m3_mul(Rd, P->R, T.R); }
			else T.t[p - 3] += eps;
			opt_residuals(&T, mirror, r1);
			for (int i = 0; i < m; i++) J[i][p] = (r1[i] - r0[i]) / eps;
		}
		double A[6][6] = { { 0 } }, b[6] = { 0 }, x[6];
		for (int i = 0; i < m; i++)
			for (int p = 0; p < 6; p++) { b[p] -= J[i][p] * r0[i]; for (int q = 0; q < 6; q++) A[p][q] += J[i][p] * J[i][q]; }
		opt_prior_term(P->R, opt_prior_w, A, b);
		for (int p = 0; p < 6; p++) A[p][p] += 1e-6 + 1e-3 * A[p][p];
		if (!solve6(A, b, x)) break;
		double Rd[9]; m3_from_q(qrotvec(x[0], x[1], x[2]), Rd); m3_mul(Rd, P->R, P->R);
		for (int k = 0; k < 3; k++) P->t[k] += x[3 + k];
	}
	m = opt_residuals(P, mirror, r0);
	double s = 0; for (int i = 0; i < m; i++) s += r0[i] * r0[i];
	return m ? sqrt(s / m) : 1e9;
}
/* associate LEDs with blobs by reprojection (<= OPT_PX_ASSOC px), one blob per LED per camera */
/* LED <-> blob gates grow at close range: the LEDs sit behind a translucent ring whose apparent centre shifts with the viewing
 * angle by a few mm, which is 1 px at 1 m but 4-6 px at 25 cm (measured: a pose explaining 6 LEDs within 3 px explains 8 within
 * 6 px with every residual systematic per camera).  gate * 0.5 m / distance, at most 2.5x */
static double opt_gate(const struct opt_pose *P, double gate)
{
	double d = v3_norm(P->t); return gate * CLAMP(0.5 / (d > 0.05 ? d : 0.05), 1.0, 2.5);
}
static void opt_associate(struct opt_pose *P, bool mirror, double gate)
{
	double Q[OPT_NLED][3], N[OPT_NLED][3];
	opt_model(P->R, mirror, Q, N);
	gate = opt_gate(P, gate);
	P->nobs = 0; P->n = 0;
	for (int i = 0; i < OPT_NLED; i++) {
		double X[3] = { Q[i][0] + P->t[0], Q[i][1] + P->t[1], Q[i][2] + P->t[2] };
		bool seen = false;
		for (int cam = 0; cam < OPT_NCAM; cam++) {
			const struct opt_cam *c = &O.cam[cam]; double n[2];
			if (!opt_facing(N[i], X, c->o) || !opt_project(c, X, n)) continue;
			int best = -1; double bd = gate, uv[2]; opt_cam_px(c, n, uv);
			for (int r = 0; r < O.nray; r++) {
				if (O.rays[r].cam != cam || O.rays[r].used) continue;
				double dx = O.rays[r].uv[0] - uv[0], dy = O.rays[r].uv[1] - uv[1], d = sqrt(dx * dx + dy * dy);
				if (d < bd) { bd = d; best = r; }
			}
			if (best >= 0 && P->nobs < OPT_MAX_OBS) { P->obs[P->nobs++] = (struct opt_obs){ i, best }; seen = true; }
		}
		if (seen) P->n++;
	}
}

struct opt_used { bool ray[OPT_MAX_RAY], cand[OPT_MAX_CAND], cl[OPT_MAX_CL]; };
static void opt_used_save(struct opt_used *u)
{
	for (int i = 0; i < O.nray; i++) u->ray[i] = O.rays[i].used;
	for (int i = 0; i < O.ncand; i++) u->cand[i] = O.cand[i].used;
	for (int i = 0; i < O.ncl; i++) u->cl[i] = O.cl[i].used;
}
static void opt_used_restore(const struct opt_used *u)
{
	for (int i = 0; i < O.nray; i++) O.rays[i].used = u->ray[i];
	for (int i = 0; i < O.ncand; i++) O.cand[i].used = u->cand[i];
	for (int i = 0; i < O.ncl; i++) O.cl[i].used = u->cl[i];
}
/* consume the blobs, candidates and clusters explained by a fitted controller so the other hand cannot grab them */
static void opt_consume(const struct opt_pose *P, bool mirror)
{
	double Q[OPT_NLED][3], N[OPT_NLED][3]; opt_model(P->R, mirror, Q, N);
	for (int i = 0; i < P->nobs; i++) {
		O.rays[P->obs[i].ray].used = true;
		for (int k = 0; k < O.ncand; k++)
			if (O.cand[k].ray[0] == P->obs[i].ray || O.cand[k].ray[1] == P->obs[i].ray) O.cand[k].used = true;
	}
	for (int c = 0; c < O.ncl; c++) for (int j = 0; j < OPT_NLED && !O.cl[c].used; j++) {
		double d[3] = { O.cl[c].x[0] - Q[j][0] - P->t[0], O.cl[c].x[1] - Q[j][1] - P->t[1], O.cl[c].x[2] - Q[j][2] - P->t[2] };
		if (v3_norm(d) < OPT_INLIER) O.cl[c].used = true;
	}
}
/* Negative evidence for a fitted pose: LEDs the pose predicts well in view of a camera (normal towards it,
 * inside the image) that have no blob, and blobs inside the model's image footprint that the pose does not
 * explain.  A ring of 17 LEDs has many near-symmetric correspondences: a wrong one still explains 5-8 LEDs
 * with sub-pixel residual, but leaves the others unexplained and predicts LEDs where there is nothing.  */
struct opt_evid { int n, missing, stray; };
static struct opt_evid opt_evidence(const struct opt_pose *P, bool mirror)
{
	double Q[OPT_NLED][3], N[OPT_NLED][3]; struct opt_evid e = { P->nobs, 0, 0 };
	opt_model(P->R, mirror, Q, N);
	double nearpx = 2 * opt_gate(P, OPT_PX_ASSOC); bool nearhit[OPT_MAX_RAY] = { false }; /* a predicted LED with an unexplained blob just outside the gate is a near miss: neither missing nor stray */
	for (int cam = 0; cam < OPT_NCAM; cam++) {
		const struct opt_cam *c = &O.cam[cam];
		double cx = 0, cy = 0, rad = 0; int np = 0; double pn[OPT_NLED][2], pu[OPT_NLED][2]; bool vis[OPT_NLED];
		for (int i = 0; i < OPT_NLED; i++) {
			double X[3] = { Q[i][0] + P->t[0], Q[i][1] + P->t[1], Q[i][2] + P->t[2] }, d[3] = { c->o[0] - X[0], c->o[1] - X[1], c->o[2] - X[2] };
			v3_unit(d);
			vis[i] = v3_dot(N[i], d) > T.miss_cos && opt_project(c, X, pn[i]) && opt_distort_r(c, hypot(pn[i][0], pn[i][1])) * c->fx < 480; /* inside the fisheye image (radius 508 px) with margin */
			if (vis[i]) { opt_cam_px(c, pn[i], pu[i]); cx += pu[i][0]; cy += pu[i][1]; np++; }
		}
		if (!np) continue;
		cx /= np; cy /= np;
		for (int i = 0; i < OPT_NLED; i++) {
			if (!vis[i]) continue;
			double r = hypot(pu[i][0] - cx, pu[i][1] - cy);
			if (r > rad) rad = r;
			bool seen = false;
			for (int k = 0; k < P->nobs && !seen; k++) seen = P->obs[k].led == i && O.rays[P->obs[k].ray].cam == cam;
			if (seen) continue;
			int nb = -1; double bd = nearpx;
			for (int r = 0; r < O.nray; r++) {
				const struct opt_ray *ray = &O.rays[r];
				if (ray->cam != cam || ray->used) continue;
				bool mine = false; for (int k = 0; k < P->nobs && !mine; k++) mine = P->obs[k].ray == r;
				if (mine) continue;
				double dd = hypot(ray->uv[0] - pu[i][0], ray->uv[1] - pu[i][1]); if (dd < bd) { bd = dd; nb = r; }
			}
			if (nb >= 0) nearhit[nb] = true; else e.missing++;
		}
		rad += 8.0;
		for (int r = 0; r < O.nray; r++) { /* unexplained blobs inside the footprint (those taken by the other hand do not count) */
			const struct opt_ray *ray = &O.rays[r];
			if (ray->cam != cam || ray->used || nearhit[r]) continue;
			bool mine = false;
			for (int k = 0; k < P->nobs && !mine; k++) mine = P->obs[k].ray == r;
			if (mine) continue;
			if (hypot(ray->uv[0] - cx, ray->uv[1] - cy) < rad) e.stray++;
		}
	}
	return e;
}
static void opt_evidence_dump(const struct opt_pose *P, bool mirror) /* dump mode: where are the missing LEDs and the stray blobs? */
{
	double Q[OPT_NLED][3], N[OPT_NLED][3]; opt_model(P->R, mirror, Q, N);
	char line[900]; int len = 0; double dist = v3_norm(P->t);
	len += snprintf(line + len, sizeof line - (size_t)len, "evidence: pose %.2f m, obs/cam", dist);
	for (int cam = 0; cam < OPT_NCAM; cam++) { int k2 = 0; for (int k = 0; k < P->nobs; k++) k2 += O.rays[P->obs[k].ray].cam == cam; len += snprintf(line + len, sizeof line - (size_t)len, " %d", k2); }
	len += snprintf(line + len, sizeof line - (size_t)len, " |");
	for (int cam = 0; cam < OPT_NCAM && len < 800; cam++) {
		const struct opt_cam *c = &O.cam[cam];
		len += snprintf(line + len, sizeof line - (size_t)len, " cam%d:", cam);
		for (int i = 0; i < OPT_NLED && len < 800; i++) {
			double X[3] = { Q[i][0] + P->t[0], Q[i][1] + P->t[1], Q[i][2] + P->t[2] }, d[3] = { c->o[0] - X[0], c->o[1] - X[1], c->o[2] - X[2] }, pn[2], pu[2];
			v3_unit(d); double cs = v3_dot(N[i], d);
			if (cs <= T.miss_cos || !opt_project(c, X, pn)) continue;
			double rpx = opt_distort_r(c, hypot(pn[0], pn[1])) * c->fx; if (rpx >= 480) continue;
			opt_cam_px(c, pn, pu);
			bool seen = false; for (int k = 0; k < P->nobs && !seen; k++) seen = P->obs[k].led == i && O.rays[P->obs[k].ray].cam == cam;
			if (seen) continue;
			double near = 1e9; int nb = -1; /* nearest blob of this camera not used by this pose */
			for (int r = 0; r < O.nray; r++) { const struct opt_ray *ray = &O.rays[r]; if (ray->cam != cam) continue; bool mine = false; for (int k = 0; k < P->nobs && !mine; k++) mine = P->obs[k].ray == r; if (mine) continue; double dd = hypot(ray->uv[0] - pu[0], ray->uv[1] - pu[1]); if (dd < near) { near = dd; nb = r; } }
			len += snprintf(line + len, sizeof line - (size_t)len, " L%d(cos%.2f r%.0f near%.0fpx%s)", i, cs, rpx, near < 1e8 ? near : -1.0, nb >= 0 && O.rays[nb].used ? "*" : "");
		}
	}
	slog("%s", line);
}
static bool opt_evidence_ok(const struct opt_pose *P, bool mirror, struct opt_evid *out)
{
	struct opt_evid e = opt_evidence(P, mirror);
	if (out) *out = e;
	return e.n >= T.evid * (e.n + e.missing + 0.5 * e.stray); /* a stray blob may be the other controller's: half weight */
}

/* world <- model now, from the model's orientation at an earlier fit (q_last) turned by the IMU's motion since: the body's rotation
 * in the IMU's own world frame is A B^T; expressed in the tracking world it is conjugated by the yaw offset qc between the two
 * worlds (it matters for turns about non-vertical axes: a 90 deg pitch with a 10 deg yaw offset would otherwise come out 14 deg off) */
static void opt_imu_turned(int h, quat iq_now, quat iq_last, quat q_last, double out[9])
{
	quat qn = iq_now, ql = iq_last;
	if (O.qc_ok[h]) { qn = qnorm(qmul(O.qc[h], iq_now)); ql = qnorm(qmul(O.qc[h], iq_last)); }
	double A[9], B[9], Bt[9], C[9], T[9];
	m3_from_q(qn, A); m3_from_q(ql, B); m3_from_q(q_last, C);
	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) Bt[i * 3 + j] = B[j * 3 + i];
	m3_mul(A, Bt, T); m3_mul(T, C, out);
}
/* Once the LED-model<->IMU rotation is calibrated a fit's model gravity must agree with the controller IMU's:
 * a fit on the wrong controller (the two LED layouts are near mirror images) or a wrong ring correspondence
 * shows up as a tilted model.  Rc: cam0 <- model. */
static bool opt_grav_ok(int h, const double Rc[9], const double R_wc[9], quat iq, bool iok, double *deg)
{
	if (deg) *deg = 0;
	if (!iok || O.imf[h].quality < 2 || T.grav_deg <= 0) return true;
	double Rw[9], up[3] = { 0, 1, 0 }, gi[3], gm[3]; m3_mul(R_wc, Rc, Rw);
	double gf[3] = { Rw[3], Rw[4], Rw[5] }; /* world up in the model frame = Rw^T e_y */
	qrot(qconj(iq), up, gi); m3_mulv(O.imf[h].R, gi, gm);
	double a = acos(CLAMP(v3_dot(gf, gm), -1.0, 1.0)) * 180 / M_PI;
	if (deg) *deg = a;
	return a < T.grav_deg + 0.03 * CLAMP(O.imu_rate[h], 0.0, 600.0); /* +9 deg at 300 deg/s: the fused tilt and the sample timing are that much worse in a swing */
}

/* Known-rotation RANSAC over stereo candidates; Rimu: IMU -> cam0. prior: predicted position or NULL. */
static bool opt_fit(const double Rimu[9], bool mirror, const double *prior, int min_inliers, struct opt_pose *out)
{
	double Q[OPT_NLED][3], N[OPT_NLED][3], zero[3] = { 0, 0, 0 };
	opt_model(Rimu, mirror, Q, N);
	int best_n = 0; double best_res = 1e9, best_t[3] = { 0, 0, 0 }, best_prior = 1e9;
	for (int k = 0; k < O.ncand; k++) {
		if (O.cand[k].used) continue;
		for (int i = 0; i < OPT_NLED; i++) {
			if (!opt_facing(N[i], O.cand[k].x, zero)) continue;
			double t[3] = { O.cand[k].x[0] - Q[i][0], O.cand[k].x[1] - Q[i][1], O.cand[k].x[2] - Q[i][2] };
			if (prior) { double dp[3] = { t[0] - prior[0], t[1] - prior[1], t[2] - prior[2] }; if (v3_norm(dp) > T.prior_m) continue; }
			int n = 0; double res = 0;
			for (int j = 0; j < OPT_NLED; j++) {
				double X[3] = { Q[j][0] + t[0], Q[j][1] + t[1], Q[j][2] + t[2] };
				if (!opt_facing(N[j], X, zero)) continue;
				double bd = OPT_INLIER;
				for (int m = 0; m < O.ncand; m++) {
					if (O.cand[m].used) continue;
					double dx = O.cand[m].x[0] - X[0], dy = O.cand[m].x[1] - X[1], dz = O.cand[m].x[2] - X[2], d = sqrt(dx * dx + dy * dy + dz * dz);
					if (d < bd) bd = d;
				}
				if (bd < OPT_INLIER) { n++; res += bd; }
			}
			double pd = prior ? sqrt((t[0] - prior[0]) * (t[0] - prior[0]) + (t[1] - prior[1]) * (t[1] - prior[1]) + (t[2] - prior[2]) * (t[2] - prior[2])) : 0;
			if (n > best_n || (n == best_n && n > 0 && (prior ? pd < best_prior : res < best_res))) { best_n = n; best_res = res; best_prior = pd; memcpy(best_t, t, sizeof best_t); }
		}
	}
	if (best_n < min_inliers) return false;
	struct opt_pose P = { 0 };
	memcpy(P.R, Rimu, sizeof P.R); memcpy(P.t, best_t, sizeof P.t);
	opt_associate(&P, mirror, OPT_PX_ASSOC);
	if (P.n < min_inliers) return false;
	P.res = opt_refine(&P, mirror, 4);
	opt_associate(&P, mirror, OPT_PX_ASSOC);
	P.res = opt_refine(&P, mirror, 3);
	struct opt_evid ev; bool evok = opt_evidence_ok(&P, mirror, &ev);
	if (P.n < min_inliers || P.res > 4.0 || !evok) { if (OPT_DUMP) slog("optical: track%c rejected: ransac %d -> n=%d res=%.2f evidence %d/%d/%d", mirror ? 'm' : ' ', best_n, P.n, P.res, ev.n, ev.missing, ev.stray); return false; }
	opt_consume(&P, mirror);
	*out = P;
	return true;
}

/* ---- IMU-free acquisition ----
 * Stereo candidates are merged into clusters (a real LED seen by 3-4 cameras yields several coincident
 * candidates, a ghost only one).  Seeds are the best-supported clusters; a triangle of seeds is matched to a
 * triangle of LEDs by pairwise distances, aligned (closed form) and scored by model points near clusters. */
#define OPT_SEEDS 40
#ifndef OPT_DTOL
#define OPT_DTOL 0.006
#endif
static double opt_led_d[OPT_NLED][OPT_NLED]; static double opt_span; static int opt_dbg[6]; static struct opt_pose opt_dbg_pose;
static void opt_cluster(void)
{
	O.ncl = 0;
	for (int k = 0; k < O.ncand; k++) {
		const double *x = O.cand[k].x; int hit = -1;
		for (int c = 0; c < O.ncl && hit < 0; c++) {
			double d[3] = { x[0] - O.cl[c].x[0], x[1] - O.cl[c].x[1], x[2] - O.cl[c].x[2] };
			if (v3_norm(d) < OPT_DTOL) hit = c;
		}
		if (hit >= 0) { struct opt_cl *c = &O.cl[hit]; for (int i = 0; i < 3; i++) c->x[i] = (c->x[i] * c->n + x[i]) / (c->n + 1); c->n++; if (O.cand[k].gap < c->gap) c->gap = O.cand[k].gap; }
		else if (O.ncl < OPT_MAX_CL) { struct opt_cl *c = &O.cl[O.ncl++]; memcpy(c->x, x, sizeof c->x); c->gap = O.cand[k].gap; c->n = 1; c->used = false; }
	}
	for (int i = 1; i < O.ncl; i++) { /* sorted by x for the windowed nearest search in opt_score */
		struct opt_cl c = O.cl[i]; int j = i;
		while (j > 0 && O.cl[j - 1].x[0] > c.x[0]) { O.cl[j] = O.cl[j - 1]; j--; }
		O.cl[j] = c;
	}
}
/* nearest unused cluster within OPT_INLIER of X (clusters sorted by x), or -1 */
static int opt_near_cl(const double X[3], double *dist)
{
	int lo = 0, hi = O.ncl;
	while (lo < hi) { int mid = (lo + hi) / 2; if (O.cl[mid].x[0] < X[0] - OPT_INLIER) lo = mid + 1; else hi = mid; }
	int best = -1; double bd = OPT_INLIER;
	for (int c = lo; c < O.ncl && O.cl[c].x[0] <= X[0] + OPT_INLIER; c++) {
		if (O.cl[c].used) continue;
		double d[3] = { O.cl[c].x[0] - X[0], O.cl[c].x[1] - X[1], O.cl[c].x[2] - X[2] }, l = v3_norm(d);
		if (l < bd) { bd = l; best = c; }
	}
	if (dist) *dist = bd;
	return best;
}
/* rotation taking triangle P onto triangle Q (frames built from the edges); false if degenerate */
static bool opt_tri_rot(const double P[3][3], const double Q[3][3], double R[9])
{
	double A[9], B[9];
	const double (*T[2])[3] = { P, Q }; double *M[2] = { A, B };
	for (int s = 0; s < 2; s++) {
		double u[3], v[3], w[3];
		for (int i = 0; i < 3; i++) { u[i] = T[s][1][i] - T[s][0][i]; v[i] = T[s][2][i] - T[s][0][i]; }
		v3_unit(u); double d = v3_dot(u, v); for (int i = 0; i < 3; i++) v[i] -= d * u[i];
		if (v3_norm(v) < 0.01) return false;
		v3_unit(v);
		w[0] = u[1] * v[2] - u[2] * v[1]; w[1] = u[2] * v[0] - u[0] * v[2]; w[2] = u[0] * v[1] - u[1] * v[0];
		for (int i = 0; i < 3; i++) { M[s][i * 3] = u[i]; M[s][i * 3 + 1] = v[i]; M[s][i * 3 + 2] = w[i]; } /* columns u v w */
	}
	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) R[i * 3 + j] = B[i * 3] * A[j * 3] + B[i * 3 + 1] * A[j * 3 + 1] + B[i * 3 + 2] * A[j * 3 + 2]; /* B Aᵀ */
	return true;
}
/* matched clusters minus half the LEDs that face the cameras but have no cluster (a wrong correspondence on
 * the near-symmetric ring matches a subset and predicts LEDs in empty space) */
static double opt_score(const double R[9], const double t[3], bool mirror, int *matched)
{
	double Q[OPT_NLED][3], N[OPT_NLED][3], zero[3] = { 0, 0, 0 }; int n = 0, miss = 0;
	opt_model(R, mirror, Q, N);
	for (int j = 0; j < OPT_NLED; j++) {
		double X[3] = { Q[j][0] + t[0], Q[j][1] + t[1], Q[j][2] + t[2] };
		if (!opt_facing(N[j], X, zero)) continue;
		if (opt_near_cl(X, NULL) >= 0) n++;
		else { double d[3] = { -X[0], -X[1], -X[2] }; v3_unit(d); if (v3_dot(N[j], d) > T.miss_cos) miss++; }
	}
	if (matched) *matched = n;
	return n - 0.5 * miss;
}
static void opt_refine3d(struct opt_pose *P, bool mirror);
/* polish a coarse pose against clusters and blobs; the common tail of both acquisition paths */
static int opt_acq_min = 0; /* acquisition LED minimum override (orientation known from the IMU: fewer LEDs suffice), 0 = T.acq_min */
static const double *opt_acq_hint; /* continuity acquisition: where the last track should be now (cam0); within 20 cm of it three LEDs suffice */
static bool opt_acquire_finish(struct opt_pose *Pz, bool mirror, const char *tag)
{
	int seed_n = opt_dbg[0], min_n = opt_acq_min ? opt_acq_min : (int)T.acq_min;
	opt_refine3d(Pz, mirror);              /* the seed alignment is coarse: polish against the clusters first */
	opt_dbg_pose = *Pz;
	opt_associate(Pz, mirror, 2 * OPT_PX_ASSOC);
	opt_dbg[3] = Pz->n;
	if (Pz->n < 4) { if (OPT_DUMP) slog("optical: acq %s%c: seed %d clusters -> only %d LEDs associate", tag, mirror ? 'm' : ' ', seed_n, Pz->n); return false; }
	Pz->res = opt_refine(Pz, mirror, 6);
	opt_associate(Pz, mirror, OPT_PX_ASSOC);
	Pz->res = opt_refine(Pz, mirror, 3);
	opt_dbg[4] = Pz->n; opt_dbg[5] = (int)(Pz->res * 100);
	struct opt_evid ev; bool evok = opt_evidence_ok(Pz, mirror, &ev);
	bool ok = Pz->n >= min_n && Pz->res <= 3.0 && evok;
	if (OPT_DUMP && !ok) {
		slog("optical: acq %s%c rejected: seed %d -> n=%d res=%.2f px, evidence %d obs / %d missing / %d stray (%s)", tag, mirror ? 'm' : ' ', seed_n, Pz->n, Pz->res, ev.n, ev.missing, ev.stray, Pz->n < min_n ? "too few LEDs" : Pz->res > 3.0 ? "residual" : "evidence");
		if (!evok && Pz->n >= min_n) opt_evidence_dump(Pz, mirror);
	}
	return ok;
}
/* ---- LED model frame vs IMU frame ----
 * The fit gives world <- model, the controller IMU world <- IMU (gravity-true, yaw arbitrary).  World up expressed in
 * both body frames gives one vector pair per fit for the constant rotation R_mi (g_model = R_mi g_imu); gravity
 * pairs from different controller orientations pin it down completely (Wahba's problem, solved by Gauss-Newton). */
/* Factory prior for the LED-model <- IMU rotation (the IMU mounting is the same in every Sense of a hand): solved offline from a
 * calibration session (joint hand-eye with the head<->cam0 rotation, both hands mirror-consistent to 2 deg).  A persisted estimate
 * learnt in this session replaces it; files from before the head<->cam0 fix ("v2" marker missing) are ignored. */
static const double imf_prior[2][9] = { /* two sessions averaged: 52 deg about the model x axis, +-2 deg of y/z, mirror images */
	{ 0.99906, -0.00818, -0.04268, 0.03868, 0.61505, 0.78754, 0.01981, -0.78845, 0.61478 },  /* L */
	{ 0.99906, 0.00818, 0.04268, -0.03868, 0.61505, 0.78754, -0.01981, -0.78845, 0.61478 },  /* R */
};
static void imf_path(int hand, char *path, size_t n) { snprintf(path, n, "%s/.config/slopvr/imuframe_%c", getenv("HOME") ? getenv("HOME") : "/tmp", hand ? 'R' : 'L'); }
static void imf_save(int h)
{
	struct imu_frame *F = &O.imf[h];
	if (F->spread <= 60 || F->rms > 6) return; /* only a well-spread, consistent estimate is worth more than the factory prior */
	char path[256]; imf_path(h, path, sizeof path); FILE *f = fopen(path, "w"); if (!f) return;
	fprintf(f, "v2 %d %.3f", F->quality, F->spread); for (int i = 0; i < 9; i++) fprintf(f, " %.9f", F->R[i]); fprintf(f, "\n"); fclose(f);
}
static void imf_load(int h)
{
	struct imu_frame *F = &O.imf[h];
	memcpy(F->R, imf_prior[h], sizeof F->R); F->quality = 2; F->spread = 60; F->loaded = true; F->agree = 0;
	char path[256]; imf_path(h, path, sizeof path); FILE *f = fopen(path, "r"); if (!f) return;
	double R[9]; int q; double sp;
	if (fscanf(f, "v2 %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &q, &sp, &R[0], &R[1], &R[2], &R[3], &R[4], &R[5], &R[6], &R[7], &R[8]) == 11 && q == 2 && sp > 60) { memcpy(F->R, R, sizeof R); F->quality = q; F->spread = sp; }
	fclose(f);
}
static double imf_resid(const struct imu_frame *F, int k) /* deg */
{
	double Rg[3]; m3_mulv(F->R, F->gi[k], Rg);
	return acos(CLAMP(v3_dot(Rg, F->gm[k]), -1.0, 1.0)) * 180 / M_PI;
}
static void imf_solve(struct imu_frame *F)
{
	int n = F->n < IMF_N ? F->n : IMF_N;
	if (!n) return;
	if (F->quality == 0) { /* initial guess: the minimal rotation taking the newest g_imu onto g_model */
		const double *a = F->gi[(F->head + IMF_N - 1) % IMF_N], *b = F->gm[(F->head + IMF_N - 1) % IMF_N];
		double ax[3] = { a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] }, sn = v3_norm(ax), cs = v3_dot(a, b), ang = atan2(sn, cs);
		if (sn > 1e-9) { for (int i = 0; i < 3; i++) ax[i] *= ang / sn; m3_from_q(qrotvec(ax[0], ax[1], ax[2]), F->R); }
		else { memset(F->R, 0, sizeof F->R); F->R[0] = F->R[4] = F->R[8] = 1; }
	}
	double w[IMF_N]; for (int k = 0; k < n; k++) w[k] = 1;
	for (int round = 0; round < 2; round++) { /* round 2 without the outliers (fits on the wrong controller / a wrong ring correspondence) */
		for (int it = 0; it < 8; it++) { /* Gauss-Newton on the rotation vector, damped so the unobservable axis stays put */
			double A[3][3] = { { 0 } }, g[3] = { 0 };
			for (int k = 0; k < n; k++) {
				if (w[k] <= 0) continue;
				double Rg[3], r[3]; m3_mulv(F->R, F->gi[k], Rg);
				for (int i = 0; i < 3; i++) r[i] = Rg[i] - F->gm[k][i];
				double J[3][3] = { { 0, Rg[2], -Rg[1] }, { -Rg[2], 0, Rg[0] }, { Rg[1], -Rg[0], 0 } }; /* d(exp(d) R g)/dd = -[Rg]x */
				for (int i = 0; i < 3; i++) for (int a = 0; a < 3; a++) { g[a] += J[i][a] * r[i]; for (int b = 0; b < 3; b++) A[a][b] += J[i][a] * J[i][b]; }
			}
			{ /* the factory prior pulls like 6 agreeing pairs: a few biased samples (a swing's centripetal tilt, a wrong fit) cannot swing the estimate, 30+ good ones can */
				double D[9], RT[9], v[3]; const double *P = imf_prior[F == &O.imf[1]];
				for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) RT[i * 3 + j] = F->R[j * 3 + i];
				m3_mul(P, RT, D); v[0] = 0.5 * (D[7] - D[5]); v[1] = 0.5 * (D[2] - D[6]); v[2] = 0.5 * (D[3] - D[1]); /* d with exp(d) R = P (small-angle log of P R^T) */
				double wp = 6.0; for (int a = 0; a < 3; a++) { A[a][a] += wp; g[a] -= wp * v[a]; } /* the step d solves A d = -g */
			}
			for (int a = 0; a < 3; a++) A[a][a] += 1e-3;
			double det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
			if (fabs(det) < 1e-12) break;
			double d[3];
			for (int c = 0; c < 3; c++) { /* Cramer */
				double M[3][3]; memcpy(M, A, sizeof M); for (int i = 0; i < 3; i++) M[i][c] = -g[i];
				d[c] = (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0])) / det;
			}
			double dR[9], Rn[9]; m3_from_q(qrotvec(d[0], d[1], d[2]), dR); m3_mul(dR, F->R, Rn); memcpy(F->R, Rn, sizeof Rn);
			if (v3_norm(d) < 1e-6) break;
		}
		if (n < 6) break;
		int kept = 0; for (int k = 0; k < n; k++) { w[k] = imf_resid(F, k) < 20 ? 1 : 0; kept += w[k] > 0; }
		if (kept < 4) break;
	}
	double ss = 0; int m = 0; for (int k = 0; k < n; k++) { double r = imf_resid(F, k); if (r < 20) { ss += r * r; m++; } }
	F->rms = m ? sqrt(ss / m) : 0;
	F->quality = (F->spread > 25 || F->loaded) ? 2 : 1; /* the factory prior keeps the full-rotation status; the samples refine its observable part */
}
/* a persisted model<->IMU rotation that keeps contradicting good-looking fits (e.g. one calibrated under a different lens model)
 * must not lock the tracker out: after 30 gravity rejections without an accepted fit it is discarded and re-learnt */
static void imf_discard(int h, const char *why)
{
	pthread_mutex_lock(&O.lock);
	struct imu_frame *F = &O.imf[h]; F->n = 0; F->head = 0; F->rms = 0; F->agree = 0; F->loaded = true;
	memcpy(F->R, imf_prior[h], sizeof F->R); F->quality = 2; F->spread = 60; /* back to the factory prior */
	pthread_mutex_unlock(&O.lock);
	O.grav_rej_run[h] = 0; O.tdoubt[h] = 0; O.qc_ok[h] = false;
	{ char path[256]; imf_path(h, path, sizeof path); unlink(path); }
	slog("optical: %c model<->IMU calibration discarded: %s (back to the factory prior)", h ? 'R' : 'L', why);
}
static void imf_contradicted(int h, uint64_t now)
{
	(void)now;
	int limit = O.imf[h].loaded && O.imf[h].agree < 8 ? 30 : 150; /* a persisted estimate nothing in this session confirmed yet is cheap to overturn */
	if (++O.grav_rej_run[h] < limit) return;
	char why[80]; snprintf(why, sizeof why, "it contradicted %d fits in a row", limit);
	imf_discard(h, why);
}
/* Calibration-free check of a fit against the last established track: LED model and IMU sit on one rigid body, so both must have
 * turned by the same angle since (a rotation's angle does not depend on the frame it is expressed in: neither the model<->IMU
 * rotation nor the IMU's yaw offset enter).  turn: the IMU's turn; diff: |model turn - IMU turn|; tol grows with the gyro drift */
static bool opt_turn_test(int h, const double Rc[9], const double R_wc[9], quat iq, bool iok, uint64_t now, double *turn, double *turn_model, double *tol)
{
	const struct turn_ref *r = &O.tref[h];
	if (!iok || !r->valid || now <= r->host_us || now - r->host_us > 20000000) return false;
	double Rw[9]; m3_mul(R_wc, Rc, Rw);
	*turn_model = qangle(q_from_m3(Rw), r->q) * 180 / M_PI; *turn = qangle(iq, r->iq) * 180 / M_PI;
	*tol = 5 + 0.4 * (now - r->host_us) * 1e-6;
	return true;
}
/* The gravity veto with its escape hatch: a fit the calibration rejects, but which turned exactly as the IMU did since an established
 * track that the same calibration accepted, is right - the calibration is wrong (learnt from swapped hands or a wrong ring
 * correspondence).  Three such fits in a row discard it; otherwise the fit is rejected and counted against the calibration. */
static bool opt_grav_veto(int h, const double Rc[9], const double R_wc[9], quat iq, bool iok, uint64_t now, const char *what, int n, double res)
{
	double gdeg; if (opt_grav_ok(h, Rc, R_wc, iq, iok, &gdeg)) return false;
	double turn, tm, tol;
	if (opt_turn_test(h, Rc, R_wc, iq, iok, now, &turn, &tm, &tol) && fabs(tm - turn) < tol) {
		if (++O.tdoubt[h] >= 3) {
			char why[160]; snprintf(why, sizeof why, "it put gravity %.0f deg off a fit that turned %.0f deg as the IMU did (%.0f deg, %d LEDs)", gdeg, tm, turn, n);
			imf_discard(h, why);
			return false;
		}
	} else O.tdoubt[h] = 0;
	if (OPT_DUMP) slog("optical: %s %c rejected: model gravity %.0f deg off the IMU's (n=%d res=%.2f)", what, h ? 'R' : 'L', gdeg, n, res);
	O.grav_rej_us[h] = now; imf_contradicted(h, now);
	return true;
}
/* a verified fit: q_fit = world <- model, q_imu = world <- IMU at the same instant */
static void imf_push(int h, quat q_fit, quat q_imu, uint64_t now)
{
	struct imu_frame *F = &O.imf[h];
	if (now - F->last_us < 200000) return;
	F->last_us = now;
	double up[3] = { 0, 1, 0 }, gi[3], gm[3];
	qrot(qconj(q_imu), up, gi); qrot(qconj(q_fit), up, gm);
	int n = F->n < IMF_N ? F->n : IMF_N;
	for (int k = 0; k < n; k++) { double a = acos(CLAMP(v3_dot(gi, F->gi[k]), -1.0, 1.0)) * 180 / M_PI; if (a > F->spread) F->spread = a; }
	if (F->quality == 2) { double Rg[3]; m3_mulv(F->R, gi, Rg); if (acos(CLAMP(v3_dot(Rg, gm), -1.0, 1.0)) * 180 / M_PI < 15) F->agree++; }
	memcpy(F->gi[F->head], gi, sizeof gi); memcpy(F->gm[F->head], gm, sizeof gm); F->head = (F->head + 1) % IMF_N; F->n++;
	int qold = F->quality;
	imf_solve(F);
	if (F->quality != qold || (F->n % (SLOP_DBG ? 16 : 128)) == 0) { imf_save(h); slog("optical: %c IMU frame %s (spread %.0f deg, %d samples)", h ? 'R' : 'L', F->quality == 2 ? "calibrated" : "tilt only", F->spread, F->n); }
}

/* ---- acquisition with the IMU ----
 * The controller IMU gives its orientation up to yaw (gravity); with a known yaw correction (qc, from an
 * earlier fix) it is fully known.  Sony's tracker does the same (P1P/P2P: rotation from the IMU, position
 * from one or two LED correspondences, then non-linear refinement).  Hypotheses: yaw sweep x (seed cluster
 * <-> LED) translation; scored by clusters explained minus LEDs predicted in empty space. */
static bool opt_acquire_imu(bool mirror, const double Rimu_w[9], const double R_cw[9], bool yaw_known, struct opt_pose *out)
{
	int seed[OPT_MAX_CL], ns = 0;
	for (int c = 0; c < O.ncl; c++) {
		if (O.cl[c].used) continue;
		int i = ns++;
		while (i > 0 && (O.cl[seed[i - 1]].n < O.cl[c].n || (O.cl[seed[i - 1]].n == O.cl[c].n && O.cl[seed[i - 1]].gap > O.cl[c].gap))) { seed[i] = seed[i - 1]; i--; }
		seed[i] = c;
	}
	if (ns > 32) ns = 32;
	if (ns < 3) return false;
	double step = yaw_known ? 5.0 : T.acq_yaw_step, span = yaw_known ? 30.0 : 180.0;
	/* the 3D cluster score ranks the (yaw, seed, LED) hypotheses; the best few are then re-scored by image association,
	 * which also counts LEDs seen by a single camera (close to the head a controller has few stereo clusters) */
#define ACQ_TOP 24
	struct { double sc, yaw, R[9], t[3]; int n; } top[ACQ_TOP]; int ntop = 0;
	int nmin = yaw_known ? 1 : 2; /* with the orientation known one cluster<->LED pair is already a pose: the image association ranks it */
	for (double yaw = -span; yaw < span; yaw += step) {
		double Ry[9], Rw[9], R[9], Q[OPT_NLED][3], N[OPT_NLED][3], zero[3] = { 0, 0, 0 };
		m3_from_q(qaxis(0, 1, 0, yaw), Ry); m3_mul(Ry, Rimu_w, Rw); m3_mul(R_cw, Rw, R); /* world yaw about gravity, then into cam0 */
		opt_model(R, mirror, Q, N);
		for (int a = 0; a < ns; a++) {
			const double *A = O.cl[seed[a]].x;
			for (int i = 0; i < OPT_NLED; i++) {
				double t[3] = { A[0] - Q[i][0], A[1] - Q[i][1], A[2] - Q[i][2] };
				if (!opt_facing(N[i], A, zero)) continue;
				int n; double sc = opt_score(R, t, mirror, &n) - (yaw_known ? 0.1 * fabs(yaw) : 0); /* the predicted yaw wins unless a 25.7 deg ring-slot neighbour explains ~3 more LEDs */
				if (n < nmin || (ntop == ACQ_TOP && sc <= top[ACQ_TOP - 1].sc)) continue;
				bool dup = false; /* the same pose from another seed/LED pair */
				for (int k = 0; k < ntop && !dup; k++) { double d[3] = { top[k].t[0] - t[0], top[k].t[1] - t[1], top[k].t[2] - t[2] }; if (top[k].yaw == yaw && v3_norm(d) < 0.01) { dup = true; if (sc > top[k].sc) { top[k].sc = sc; top[k].n = n; } } }
				if (dup) continue;
				int k = ntop < ACQ_TOP ? ntop++ : ACQ_TOP - 1;
				while (k > 0 && top[k - 1].sc < sc) { top[k] = top[k - 1]; k--; }
				top[k].sc = sc; top[k].yaw = yaw; top[k].n = n; memcpy(top[k].R, R, sizeof top[k].R); memcpy(top[k].t, t, sizeof top[k].t);
			}
		}
	}
	int best = -1, best_img = 0; double best_sc = -1e9; int img_n[ACQ_TOP];
	for (int k = 0; k < ntop; k++) {
		struct opt_pose P = { 0 }; memcpy(P.R, top[k].R, sizeof P.R); memcpy(P.t, top[k].t, sizeof P.t);
		opt_associate(&P, mirror, 2 * OPT_PX_ASSOC);
		img_n[k] = P.n;
		double sc = P.n - (yaw_known ? 0.1 * fabs(top[k].yaw) : 0);
		if (sc > best_sc) { best_sc = sc; best = k; best_img = P.n; }
	}
	struct opt_pose Pz = { 0 }; double R0[9];
	int need = yaw_known ? (int)T.fit_min : (int)T.acq_min; /* with the orientation known 4 LEDs pin the pose (the evidence check still applies) */
	if (yaw_known && opt_acq_hint && best >= 0) { double d[3] = { top[best].t[0] - opt_acq_hint[0], top[best].t[1] - opt_acq_hint[1], top[best].t[2] - opt_acq_hint[2] }; if (v3_norm(d) < 0.2) need = 3; } /* a moment after a track, next to where it went: three */
	if (yaw_known && best >= 0 && best_img < need + 3 && best_img >= 2) {
		/* the IMU orientation is good to ~5 deg during fast turns (timing, calibration): the raw hypotheses then associate only the few
		 * LEDs that happen to fall inside the gate.  Refine the best few hypotheses with the orientation held softly by the prior and
		 * re-associate: a right pose gains most of its LEDs, a wrong one does not (Sony: P1P/P2P + non-linear refinement) */
		int order[3] = { -1, -1, -1 };
		for (int k = 0; k < ntop; k++) for (int j = 0; j < 3; j++) if (order[j] < 0 || img_n[k] > img_n[order[j]]) { for (int m = 2; m > j; m--) order[m] = order[m - 1]; order[j] = k; break; }
		int bn = best_img; double bres = 9;
		for (int j = 0; j < 3; j++) {
			int k = order[j]; if (k < 0 || img_n[k] < 2) continue;
			struct opt_pose P = { 0 }; memcpy(P.R, top[k].R, sizeof P.R); memcpy(P.t, top[k].t, sizeof P.t); memcpy(R0, top[k].R, sizeof R0);
			opt_prior_R = R0; opt_prior_w = T.rot_prior;
			opt_associate(&P, mirror, 2 * OPT_PX_ASSOC); P.res = opt_refine(&P, mirror, 6); opt_associate(&P, mirror, OPT_PX_ASSOC); P.res = opt_refine(&P, mirror, 3);
			opt_prior_R = NULL;
			if (P.n > bn || (P.n == bn && P.res < bres)) { bn = P.n; bres = P.res; best = k; best_img = P.n; Pz = P; }
		}
		if (bn > img_n[best]) opt_dbg[1] = bn;
	}
	opt_dbg[0] = best >= 0 ? top[best].n : 0; opt_dbg[2] = ns;
	if (best < 0 || best_img < need) {
		if (OPT_DUMP) slog("optical: acq imu%c: best hypothesis matches only %d clusters of %d, %d LEDs in the images (yaw %s)", mirror ? 'm' : ' ', best >= 0 ? top[best].n : 0, ns, best_img, yaw_known ? "known +-30" : "swept");
		return false;
	}
	if (!Pz.n) { memcpy(Pz.R, top[best].R, sizeof Pz.R); memcpy(Pz.t, top[best].t, sizeof Pz.t); }
#undef ACQ_TOP
	opt_acq_min = yaw_known ? need : 0; /* orientation known: a wrong correspondence cannot hide behind few LEDs */
	memcpy(R0, yaw_known && Pz.n ? Pz.R : top[best].R, sizeof R0);
	if (yaw_known) { opt_prior_R = R0; opt_prior_w = T.rot_prior; }
	bool fin = opt_acquire_finish(&Pz, mirror, yaw_known ? "imu+yaw" : "imu");
	opt_prior_R = NULL; opt_acq_min = 0;
	if (!fin) return false;
	if (yaw_known) { /* the refinement may not have carried the pose away from the orientation the IMU gave (4 LEDs leave it room to) */
		double dev = qangle(q_from_m3(R0), q_from_m3(Pz.R)) * 180 / M_PI;
		if (dev > 15 + 0.03 * CLAMP(O.imu_rate[0] > O.imu_rate[1] ? O.imu_rate[0] : O.imu_rate[1], 0.0, 600.0)) { if (OPT_DUMP) slog("optical: acq imu+yaw%c rejected: refined %.0f deg away from the IMU orientation (n=%d)", mirror ? 'm' : ' ', dev, Pz.n); return false; }
	}
	opt_consume(&Pz, mirror);
	*out = Pz;
	return true;
}
/* Re-lock with a known orientation (tracking path, after the predicted position failed): every stereo cluster within 25 cm of the
 * prediction is tried as each LED, scored by image association with the IMU-propagated orientation, the best refined with the
 * rotation prior.  A fast hand moves 10 cm between its frames; the velocity prediction is good to a few cm, the orientation to a
 * few degrees - this is Sony's P1P: one correspondence plus the IMU is a pose. */
static bool opt_relock(int h, const double Rc[9], const double pred_t[3], int tmin, struct opt_pose *out)
{
	double Q[OPT_NLED][3], N[OPT_NLED][3], zero[3] = { 0, 0, 0 };
	opt_model(Rc, O.mirror[h], Q, N);
	struct opt_pose best = { 0 }; int bn = tmin - 1; double bres = 9;
	for (int c = 0; c < O.ncl; c++) {
		if (O.cl[c].used) continue;
		double dc[3] = { O.cl[c].x[0] - pred_t[0], O.cl[c].x[1] - pred_t[1], O.cl[c].x[2] - pred_t[2] };
		if (v3_norm(dc) > 0.25) continue;
		for (int i = 0; i < OPT_NLED; i++) {
			if (!opt_facing(N[i], O.cl[c].x, zero)) continue;
			struct opt_pose P = { 0 }; memcpy(P.R, Rc, sizeof P.R);
			for (int k = 0; k < 3; k++) P.t[k] = O.cl[c].x[k] - Q[i][k];
			opt_associate(&P, O.mirror[h], 2 * OPT_PX_ASSOC);
			if (P.n < tmin || P.n < bn) continue;
			opt_prior_R = Rc; opt_prior_w = T.rot_prior;
			P.res = opt_refine(&P, O.mirror[h], 5); opt_associate(&P, O.mirror[h], OPT_PX_ASSOC); P.res = opt_refine(&P, O.mirror[h], 3);
			opt_prior_R = NULL;
			if (P.n > bn || (P.n == bn && P.res < bres)) { bn = P.n; bres = P.res; best = P; }
		}
	}
	if (!best.n || best.res > 3.0) return false;
	double dp[3] = { best.t[0] - pred_t[0], best.t[1] - pred_t[1], best.t[2] - pred_t[2] };
	if (v3_norm(dp) > (best.n >= (int)T.fit_min ? 0.25 : 0.10) || qangle(q_from_m3(Rc), q_from_m3(best.R)) * 180 / M_PI > T.track_rot_deg || !opt_evidence_ok(&best, O.mirror[h], NULL)) return false;
	opt_consume(&best, O.mirror[h]);
	*out = best;
	return true;
}
/* Gauss-Newton on (rotation vector, translation) minimising 3D distances between model points and their
 * nearest clusters (within OPT_INLIER); 4 iterations with re-association */
static void opt_refine3d(struct opt_pose *P, bool mirror)
{
	for (int it = 0; it < 4; it++) {
		double Q[OPT_NLED][3], N[OPT_NLED][3], zero[3] = { 0, 0, 0 };
		opt_model(P->R, mirror, Q, N);
		int match[OPT_NLED], nm = 0;
		for (int j = 0; j < OPT_NLED; j++) {
			double X[3] = { Q[j][0] + P->t[0], Q[j][1] + P->t[1], Q[j][2] + P->t[2] }; match[j] = -1;
			if (!opt_facing(N[j], X, zero)) continue;
			match[j] = opt_near_cl(X, NULL);
			if (match[j] >= 0) nm++;
		}
		if (nm < 3) return;
		double A[6][6] = { { 0 } }, g[6] = { 0 };
		for (int j = 0; j < OPT_NLED; j++) {
			if (match[j] < 0) continue;
			double X[3] = { Q[j][0] + P->t[0], Q[j][1] + P->t[1], Q[j][2] + P->t[2] }, r[3], J[3][6];
			for (int k = 0; k < 3; k++) r[k] = X[k] - O.cl[match[j]].x[k];
			/* d(R q)/d(rotvec) = -[Rq]x ; d/dt = I */
			double q[3] = { Q[j][0], Q[j][1], Q[j][2] };
			double S[3][3] = { { 0, -q[2], q[1] }, { q[2], 0, -q[0] }, { -q[1], q[0], 0 } };
			for (int k = 0; k < 3; k++) for (int m = 0; m < 3; m++) { J[k][m] = -S[k][m]; J[k][3 + m] = k == m; }
			for (int k = 0; k < 3; k++) for (int a = 0; a < 6; a++) { g[a] += J[k][a] * r[k]; for (int b = 0; b < 6; b++) A[a][b] += J[k][a] * J[k][b]; }
		}
		double dx[6]; for (int a = 0; a < 6; a++) { A[a][a] += 1e-9; g[a] = -g[a]; }
		opt_prior_term(P->R, opt_prior_w * 0.001, A, g); /* metres here: ~1 mm per px at arm's length */
		if (!solve6(A, g, dx)) return;
		double dR[9]; m3_from_q(qrotvec(dx[0], dx[1], dx[2]), dR);
		double Rn[9]; m3_mul(dR, P->R, Rn); memcpy(P->R, Rn, sizeof Rn);
		for (int k = 0; k < 3; k++) P->t[k] += dx[3 + k];
	}
}
static bool opt_acquire(bool mirror, struct opt_pose *out)
{
	int seed[OPT_MAX_CL], ns = 0;
	for (int c = 0; c < O.ncl; c++) { /* unused clusters by support, descending */
		if (O.cl[c].used) continue;
		int i = ns++;
		while (i > 0 && (O.cl[seed[i - 1]].n < O.cl[c].n || (O.cl[seed[i - 1]].n == O.cl[c].n && O.cl[seed[i - 1]].gap > O.cl[c].gap))) { seed[i] = seed[i - 1]; i--; }
		seed[i] = c;
	}
	if (ns > OPT_SEEDS) ns = OPT_SEEDS;
	if (ns < 3) return false;
	double mx = mirror ? -1 : 1, bestR[9], bestT[3], best = 0; int best_n = 0, budget = 30000;
	for (int a = 0; a < ns && budget > 0; a++) for (int b = a + 1; b < ns && budget > 0; b++) {
		const double *A = O.cl[seed[a]].x, *B = O.cl[seed[b]].x;
		double dab[3] = { B[0] - A[0], B[1] - A[1], B[2] - A[2] }, lab = v3_norm(dab);
		if (lab < 0.015 || lab > opt_span + OPT_DTOL) continue;
		for (int i = 0; i < OPT_NLED; i++) for (int j = 0; j < OPT_NLED; j++) {
			if (i == j || fabs(opt_led_d[i][j] - lab) > OPT_DTOL) continue;
			for (int cc = b + 1; cc < ns && budget > 0; cc++) {
				const double *C = O.cl[seed[cc]].x;
				double dac[3] = { C[0] - A[0], C[1] - A[1], C[2] - A[2] }, dbc[3] = { C[0] - B[0], C[1] - B[1], C[2] - B[2] }, lac = v3_norm(dac), lbc = v3_norm(dbc);
				if (lac < 0.015 || lbc < 0.015 || lac > opt_span + OPT_DTOL || lbc > opt_span + OPT_DTOL) continue;
				for (int k = 0; k < OPT_NLED; k++) {
					if (k == i || k == j || fabs(opt_led_d[i][k] - lac) > OPT_DTOL || fabs(opt_led_d[j][k] - lbc) > OPT_DTOL) continue;
					double P[3][3] = { { mx * opt_led_pos[i][0] * 1e-3, opt_led_pos[i][1] * 1e-3, opt_led_pos[i][2] * 1e-3 },
							   { mx * opt_led_pos[j][0] * 1e-3, opt_led_pos[j][1] * 1e-3, opt_led_pos[j][2] * 1e-3 },
							   { mx * opt_led_pos[k][0] * 1e-3, opt_led_pos[k][1] * 1e-3, opt_led_pos[k][2] * 1e-3 } };
					double Qt[3][3] = { { A[0], A[1], A[2] }, { B[0], B[1], B[2] }, { C[0], C[1], C[2] } }, R[9], t[3], v[3];
					budget--;
					if (!opt_tri_rot(P, Qt, R)) continue;
					m3_mulv(R, P[0], v); for (int m = 0; m < 3; m++) t[m] = A[m] - v[m];
					int n; double sc = opt_score(R, t, mirror, &n);
					if (sc > best) { best = sc; best_n = n; memcpy(bestR, R, sizeof bestR); memcpy(bestT, t, sizeof bestT); } /* no early exit: the ring's twins score alike */
				}
			}
		}
	}
	opt_dbg[0] = best_n; opt_dbg[1] = budget; opt_dbg[2] = ns;
	if (best_n < 5) { if (OPT_DUMP) slog("optical: acq tri%c: best triangle hypothesis matches only %d clusters of %d (budget left %d)", mirror ? 'm' : ' ', best_n, ns, budget); return false; }
	struct opt_pose Pz = { 0 };
	memcpy(Pz.R, bestR, sizeof Pz.R); memcpy(Pz.t, bestT, sizeof Pz.t);
	if (!opt_acquire_finish(&Pz, mirror, "tri")) return false;
	opt_consume(&Pz, mirror);
	*out = Pz;
	return true;
}

/* ---- per-frame driver: called from the headset USB thread with one LD packet ---- */
static void optical_process(const uint8_t *b, int len, uint64_t now, int64_t exp_us, uint32_t frame);
/* controller IMU orientation at host time t (interpolated; the fits need the orientation at the exposure, not
 * at processing time: 20 ms of a fast turn moves the LEDs by several pixels); O.lock held */
static bool opt_imu_at(int hand, uint64_t t, quat *out)
{
	unsigned n = O.ih_n[hand]; if (!n) return false;
	unsigned m = n < 128 ? n : 128;
	const struct imu_hist *a = NULL, *b = NULL;
	for (unsigned i = 0; i < m; i++) {
		const struct imu_hist *h = &O.ih[hand][(n - 1 - i) % 128];
		if (h->t <= t) { a = h; break; }
		b = h;
	}
	if (!a) { *out = b->q; return (int64_t)(b->t - t) < 100000; }           /* t older than the history */
	if (!b) { *out = a->q; return (int64_t)(t - a->t) < 100000; }           /* t newer than the latest sample */
	double k = b->t > a->t ? (double)(t - a->t) / (double)(b->t - a->t) : 0;
	*out = qnlerp(a->q, b->q, k);
	return true;
}
static void opt_timeline(uint64_t now, const bool *ok) /* 1 Hz summary (log when OPT_DUMP; always kept for the tuning tool) */
{
	uint64_t sec = now / 1000000;
	if (sec != O.tl_sec) {
		O.tl_last_lock[0] = O.tl_lock[0]; O.tl_last_lock[1] = O.tl_lock[1]; O.tl_last_bmin = O.tl_bmin; O.tl_last_bmax = O.tl_bmax; O.tl_last_frames = O.tl_frames;
		if (O.tl_sec && OPT_DUMP) slog("optical: tl %d frames (max %d ms) blobs %d-%d | L %s lat %d lock %d | R %s lat %d lock %d", O.tl_frames, O.tl_ms, O.tl_bmin, O.tl_bmax,
			O.led[0].scan ? "SCAN" : O.led[0].probe ? "probe" : O.led[0].phase == LED_PHASE_PRESCAN ? (O.led[0].freerun ? "FREE" : "sync") : O.led[0].phase == LED_PHASE_ON ? "ON" : "off", O.led[0].lat, O.tl_lock[0],
			O.led[1].scan ? "SCAN" : O.led[1].probe ? "probe" : O.led[1].phase == LED_PHASE_PRESCAN ? (O.led[1].freerun ? "FREE" : "sync") : O.led[1].phase == LED_PHASE_ON ? "ON" : "off", O.led[1].lat, O.tl_lock[1]);
		O.tl_sec = sec; O.tl_bmin = O.tl_bmax = O.nray; O.tl_lock[0] = O.tl_lock[1] = 0; O.tl_frames = 0; O.tl_ms = 0;
	}
	O.tl_frames++; { int ms = (int)((now_us() - now) / 1000); if (ms > O.tl_ms) O.tl_ms = ms; }
	if (O.nray < O.tl_bmin) O.tl_bmin = O.nray; if (O.nray > O.tl_bmax) O.tl_bmax = O.nray;
	for (int h = 0; h < 2; h++) if (ok && ok[h]) O.tl_lock[h]++;
}
static void *optical_thread(void *arg)
{
	(void)arg;
	static uint8_t pkt[OPT_LD_SIZE];
	pthread_mutex_lock(&O.mb_lock);
	while (O.run) {
		while (O.run && !O.mb_n) pthread_cond_wait(&O.mb_cond, &O.mb_lock);
		if (!O.run) break;
		int len = O.mb[O.mb_r].len; uint64_t now = O.mb[O.mb_r].now; int64_t exp = O.mb[O.mb_r].exp; uint32_t frame = O.mb[O.mb_r].frame; memcpy(pkt, O.mb[O.mb_r].b, (size_t)len);
		O.mb_r = (O.mb_r + 1) % 3; O.mb_n--;
		pthread_mutex_unlock(&O.mb_lock);
		optical_process(pkt, len, now, exp, frame);
		pthread_mutex_lock(&O.mb_lock);
	}
	pthread_mutex_unlock(&O.mb_lock);
	return NULL;
}
/* headset USB thread: time stamping and LED sync here, the (possibly slow) fitting in the worker; a frame
 * arriving while the worker is busy replaces the pending one */
static void optical_on_ld_at(const uint8_t *b, int len, uint64_t now, int64_t exp_override); /* replay: recorded arrival and exposure times */
static void optical_on_ld(const uint8_t *b, int len) { optical_on_ld_at(b, len, now_us(), 0); }
static void optical_on_ld_at(const uint8_t *b, int len, uint64_t now, int64_t exp_override)
{
	if (!O.enabled || len < 0x40 + OPT_NCAM * 4 || b[0] != 'L' || b[1] != 'D') return;
	uint32_t ts, frame; memcpy(&ts, b + 8, 4); memcpy(&frame, b + 20, 4);
	int total = 0; /* blobs over all cameras (LED sync scan) */
	for (int cam = 0; cam < OPT_NCAM && 0x44 + cam * OPT_LD_CAM_BLOCK <= len; cam++) { uint32_t n; memcpy(&n, b + 0x40 + cam * OPT_LD_CAM_BLOCK, 4); total += n > 64 ? 64 : (int)n; }
	pthread_mutex_lock(&O.lock);
	O.ld_host_us = now; O.ld_frames++; O.ld_ts_last = ts; O.ld_frame_last = frame;
	if (O.hclk_have) { /* how far behind the IMU stream the frame's blobs arrive */
		O.ld_delay = (int32_t)(O.hts_last - ts);
		if (O.ld_frames % 300 == 2) O.ld_delay_min = O.ld_delay_max = O.ld_delay;
		if (O.ld_delay < O.ld_delay_min) O.ld_delay_min = O.ld_delay;
		if (O.ld_delay > O.ld_delay_max) O.ld_delay_max = O.ld_delay;
	}
	if (O.ld_frames % 60 == 0) { O.blob_recent = O.blob_acc, O.blob_acc = 0; if (O.led_mode == 7) slog("optical: anchor test: parity %d, max blobs last second %d", (int)(now / 2000000 % 2), O.blob_recent); }
	if (total > O.blob_acc) O.blob_acc = total;
	bool calib = O.calib_ok;
	pthread_mutex_unlock(&O.lock);
	if (!calib) { if (O.ld_frames % 600 == 1) slog("optical: LD frames arriving but no camera calibration yet (frame %u)", O.ld_frames); return; }
	if (len > OPT_LD_SIZE) len = OPT_LD_SIZE;
	pthread_mutex_lock(&O.mb_lock);
	if (O.mb_n == 3) { O.mb_r = (O.mb_r + 1) % 3; O.mb_n--; O.mb_dropped++; } /* the worker is behind: drop the oldest */
	memcpy(O.mb[O.mb_w].b, b, (size_t)len); O.mb[O.mb_w].len = len; O.mb[O.mb_w].now = now; O.mb[O.mb_w].frame = frame;
	O.mb[O.mb_w].exp = exp_override ? exp_override : clk_ready(&O.ldclk) && O.hclk_have ? opt_dev64(ts) - clk_at(&O.ldclk, now) : 0; /* host time of the exposure (the frame time stamp is its end) */
	O.mb_w = (O.mb_w + 1) % 3; O.mb_n++;
	pthread_cond_signal(&O.mb_cond);
	pthread_mutex_unlock(&O.mb_lock);
}

static bool opt_too_close(const double *t, const double *o)
{
	if (!o) return false;
	double d[3] = { t[0] - o[0], t[1] - o[1], t[2] - o[2] };
	return v3_norm(d) < T.sep_cm * 0.01;
}

/* keep the newest `keep` session directories (names sort by time) under the dump root, remove the rest */
static void opt_prune_dumps(const char *root, int keep)
{
	DIR *d = opendir(root); if (!d) return;
	char names[64][32]; int n = 0; struct dirent *e;
	while ((e = readdir(d)) && n < 64) if (e->d_name[0] == '2' && strlen(e->d_name) == 15) snprintf(names[n++], 32, "%s", e->d_name); /* YYYYmmdd-HHMMSS */
	closedir(d);
	for (int i = 1; i < n; i++) { char t[32]; memcpy(t, names[i], 32); int j = i; while (j > 0 && strcmp(names[j - 1], t) > 0) { memcpy(names[j], names[j - 1], 32); j--; } memcpy(names[j], t, 32); }
	for (int i = 0; i + keep < n; i++) {
		char dir[320]; snprintf(dir, sizeof dir, "%s/%s", root, names[i]);
		DIR *sd = opendir(dir); if (!sd) continue;
		while ((e = readdir(sd))) if (e->d_name[0] != '.') { char f[640]; snprintf(f, sizeof f, "%s/%s", dir, e->d_name); unlink(f); }
		closedir(sd); rmdir(dir);
	}
}
/* Whole tracker state, every 10 s: enough to reconstruct what happened from the log alone (both hands, the
 * LED timing, the frame-parity multiplexing, the model<->IMU calibration and the tunables in force). */
static void opt_state_log(uint64_t now, quat hq, const double hp[3])
{
	pthread_mutex_lock(&O.lock);
	slog("state: frames %u blobs %d cand %d | head %.2f %.2f %.2f yaw %.0f | parity even %.1f blobs/frame (%u, fits L %u R %u) odd %.1f (%u, fits L %u R %u) | mux L %s %.2f R %s %.2f | led mode %d period %d cycle %u | ldclk n=%d %.1f us/s delay %d..%d",
	     O.ld_frames, O.nray, O.ncand, hp[0], hp[1], hp[2], qyaw(hq) * 180 / M_PI,
	     O.par_n[0] ? O.par_sum[0] / O.par_n[0] : 0, O.par_n[0], O.par_fit[0][0], O.par_fit[0][1], O.par_n[1] ? O.par_sum[1] / O.par_n[1] : 0, O.par_n[1], O.par_fit[1][0], O.par_fit[1][1],
	     O.mux_live[0] ? "live" : "dead", O.mux_ema[0], O.mux_live[1] ? "live" : "dead", O.mux_ema[1], O.led_mode, O.led_period, O.led_cycle_ns ? O.led_cycle_ns : LED_CYCLE_NS, O.ldclk.n, O.ldclk.b, O.ld_delay_min, O.ld_delay_max);
	for (int h = 0; h < 2; h++) {
		const struct led_sync *L = &O.led[h]; const struct opt_fix *f = &O.fix[h]; const struct imu_frame *F = &O.imf[h];
		bool conn = (int64_t)(now - L->in_us) <= 1000000; uint64_t wall = now_us();
		slog("state: %c %s | fix %s age %lld ms n=%d res=%.2f streak %d pos %.3f %.3f %.3f v %.2f | led phase %d%s seq %u pos %u cmd %lld ms ago lat %d%s par %s/%d scan %d step %d probe %d clk n=%d %.1f us/s rtt %u in %lld ms | qc %s yaw %.0f tilt %.0f | imuframe q%d spread %.0f rms %.1f n=%d | locks/acq/fail %u/%u/%u rot-jumps %u (max %.0f deg)",
		     h ? 'R' : 'L', conn ? "connected" : "absent", f->valid ? "ok" : "none", f->valid ? (long long)((now - f->host_us) / 1000) : -1LL, f->n, f->res, f->streak, f->p[0], f->p[1], f->p[2], v3_norm(f->v),
		     L->phase, L->freerun ? "(free-run)" : "", L->seq, L->pos, L->cmd_us ? (long long)((int64_t)(wall - L->cmd_us) / 1000) : -1LL, L->lat, L->lat_ok ? "" : "?", L->par_ok ? (L->par_meas ? "odd" : "even") : "??", L->par_fix, L->scan, L->step, L->probe, L->clk.n, L->clk.b, L->clk.cur_rtt, conn ? (long long)((int64_t)(wall - L->in_us) / 1000) : -1LL,
		     O.qc_ok[h] ? "ok" : "--", O.qc_ok[h] ? qyaw(O.qc[h]) * 180 / M_PI : 0, O.qc_tilt[h], F->quality, F->spread, F->rms, F->n, O.stat_lock[h], O.stat_acq[h], O.stat_fail[h], O.rot_jumps[h], O.rot_jump_max[h]);
		O.rot_jumps[h] = 0; O.rot_jump_max[h] = 0;
	}
	slog("state: tune fit_min %.0f acq_min %.0f sep_cm %.0f prior_m %.2f evid %.2f miss_cos %.2f blob_max_px %.0f ld_off %.0f/%.0f led_mux %.0f acq_imu %.0f yaw_step %.0f grav_deg %.0f track_rot_deg %.0f rot_prior %.0f imu_lead_ms %.1f lat_trim %.0f/%.0f led_exp_us %.0f led_gain %.2f",
	     T.fit_min, T.acq_min, T.sep_cm, T.prior_m, T.evid, T.miss_cos, T.blob_max_px, T.ld_xoff, T.ld_yoff, T.led_mux, T.acq_imu, T.acq_yaw_step, T.grav_deg, T.track_rot_deg, T.rot_prior, T.imu_lead_ms, T.lat_trim_L, T.lat_trim_R, T.led_exp_us, T.led_gain);
	memset(O.par_sum, 0, sizeof O.par_sum); memset(O.par_n, 0, sizeof O.par_n); memset(O.par_fit, 0, sizeof O.par_fit);
	pthread_mutex_unlock(&O.lock);
}

static void optical_process(const uint8_t *b, int len, uint64_t now, int64_t exp_us, uint32_t frame)
{
	uint64_t t_exp = exp_us > 0 && (uint64_t)exp_us < now ? (uint64_t)exp_us : now;
	int par = (int)(frame & 1); bool own[2]; /* which hand's LEDs this frame can contain: with ledMux its own parity, unless the LEDs turned out to be lit on both */
	for (int h = 0; h < 2; h++) own[h] = !led_mux() || !O.mux_live[h] || !O.led[h].par_ok || par == O.led[h].par_meas;
	uint32_t ts; memcpy(&ts, b + 8, 4);
	quat iq[2]; bool iok[2]; int scanh = -1; /* hand whose LED timing is being scanned: its blobs are not fitted */
	pthread_mutex_lock(&O.lock);
	for (int h = 0; h < 2; h++) {
		iok[h] = opt_imu_at(h, t_exp + (int64_t)(T.imu_lead_ms * 1000), &iq[h]); if (O.led[h].scan) scanh = h;
		quat q0; O.imu_rate[h] = iok[h] && opt_imu_at(h, t_exp + (int64_t)(T.imu_lead_ms * 1000) - 20000, &q0) ? qangle(q0, iq[h]) * 180 / M_PI / 0.02 : 0;
	}
	pthread_mutex_unlock(&O.lock);

	/* head pose (world <- SLAM/IMU origin) at the frame time */
	quat hq; double hp[3];
	bool head_ok = hmd_head_at(ts, &hq, hp);
	rec_ld(b, len, now); rec_head(ts, head_ok ? hq : (quat){ 1, 0, 0, 0 }, head_ok ? hp : (double[3]){ 0, 0, 0 }, head_ok, now, t_exp);
	if (!head_ok) return;
	if (!O.logged_ts) { O.logged_ts = true; slog("optical: LD stream up (ts %u, head slam ts %u)", ts, H.slam_t); }

	if (SLOP_DBG && OPT_DUMP && O.ld_frames % (unsigned)O.dump_every == 0) { /* debugging: raw packets for the offline harness (ring; the default odd stride of 31 alternates the frame parity so both hands' flashes get recorded; opticalDumpEvery 1 records every frame for 50 s of exact replay) */
		char path[256]; const char *home = getenv("HOME") ? getenv("HOME") : "/tmp";
		if (!O.dump_dir_ok) { /* one directory per session so a recording worth keeping is not overwritten by the next run; the oldest sessions beyond 3 are removed */
			snprintf(path, sizeof path, "%s/.cache/slopvr_ld", home); mkdir(path, 0755);
			opt_prune_dumps(path, 3);
			snprintf(path, sizeof path, "%s/.cache/%s", home, O.dump_dir); mkdir(path, 0755); O.dump_dir_ok = true;
			slog("optical: packet dump -> %s (replay: psvr2linuxtest --replay %s --calib ...)", path, path);
		}
		snprintf(path, sizeof path, "%s/.cache/%s/ld_%04u.bin", home, O.dump_dir, O.ld_frames / (unsigned)O.dump_every % (O.dump_every == 1 ? 3000u : 200u));
		FILE *f = fopen(path, "wb");
		if (f) { /* trailer: head pose and controller IMU orientations for the offline harness */
			struct { char magic[4]; quat hq; double hp[3]; quat iq[2]; int iok[2]; } tr = { "SLOP", hq, { hp[0], hp[1], hp[2] }, { iq[0], iq[1] }, { iok[0], iok[1] } };
			fwrite(b, 1, (size_t)len, f); fwrite(&tr, sizeof tr, 1, f); fclose(f);
		}
	}
	opt_parse_ld(b, len);
	if (O.ld_frames % (SLOP_DBG ? 600u : 3600u) == 300 || O.state_req) { O.state_req = false; opt_state_log(now, hq, hp); } /* tracker state dump: every 10 s in the tool, every minute in the driver */
	if (OPT_DUMP && O.ld_frames % 600 == 150 && O.nray >= 6) { /* debugging (10 s): the stereo clusters of this frame (cam0 frame, metres), the candidate pairs and the rays per camera */
		char line[1024]; int k = 0; int pc[OPT_NCAM] = { 0 };
		for (int i = 0; i < O.nray; i++) pc[O.rays[i].cam]++;
		k += snprintf(line + k, sizeof line - k, "clusters: rays %d (%d %d %d %d) cand %d cl %d |", O.nray, pc[0], pc[1], pc[2], pc[3], O.ncand, O.ncl);
		for (int c = 0; c < O.ncl && k < (int)sizeof line - 48; c++) k += snprintf(line + k, sizeof line - k, " (%.3f %.3f %.3f n%d g%.1f)", O.cl[c].x[0], O.cl[c].x[1], O.cl[c].x[2], O.cl[c].n, O.cl[c].gap * 1000);
		slog("%s", line);
		k = 0; k += snprintf(line + k, sizeof line - k, "pairs:"); /* every candidate: cam/blob of both rays, depth along ray a, gap */
		for (int c = 0; c < O.ncand && k < (int)sizeof line - 40; c++) {
			const struct opt_ray *ra = &O.rays[O.cand[c].ray[0]], *rb = &O.rays[O.cand[c].ray[1]];
			double d[3] = { O.cand[c].x[0] - ra->o[0], O.cand[c].x[1] - ra->o[1], O.cand[c].x[2] - ra->o[2] };
			k += snprintf(line + k, sizeof line - k, " %d.%d-%d.%d z%.2f g%.1f", ra->cam, ra->blob, rb->cam, rb->blob, v3_norm(d), O.cand[c].gap * 1000);
		}
		slog("%s", line);
		k = 0; k += snprintf(line + k, sizeof line - k, "gaps01:"); /* front pair: closest-approach gap (mm) and depth (m) for every cam0 x cam1 ray pair, thresholds ignored */
		for (int a = 0; a < O.nray && k < (int)sizeof line - 40; a++) for (int bb = 0; bb < O.nray && k < (int)sizeof line - 40; bb++) {
			const struct opt_ray *ra = &O.rays[a], *rb = &O.rays[bb];
			if (ra->cam != 0 || rb->cam != 1) continue;
			double w[3] = { ra->o[0] - rb->o[0], ra->o[1] - rb->o[1], ra->o[2] - rb->o[2] };
			double bd = v3_dot(ra->d, rb->d), dd = v3_dot(ra->d, w), e = v3_dot(rb->d, w), den = 1 - bd * bd;
			if (den < 1e-6) continue;
			double sa = (bd * e - dd) / den, tb = (e - bd * dd) / den, g[3];
			for (int q = 0; q < 3; q++) g[q] = ra->o[q] + sa * ra->d[q] - rb->o[q] - tb * rb->d[q];
			k += snprintf(line + k, sizeof line - k, " %d-%d:%.1f@%.2f", ra->blob, rb->blob, v3_norm(g) * 1000, sa);
		}
		slog("%s", line);
		k = 0; k += snprintf(line + k, sizeof line - k, "rays:"); /* per ray: cam.blob and the normalized image coords */
		for (int i = 0; i < O.nray && k < (int)sizeof line - 40; i++) k += snprintf(line + k, sizeof line - k, " %d.%d(%.3f,%.3f)", O.rays[i].cam, O.rays[i].blob, O.rays[i].n[0], O.rays[i].n[1]);
		slog("%s", line);
	}
	if (SLOP_DBG && O.ld_frames % 300 == 1) {
		int pc[OPT_NCAM] = { 0 };
		for (int i = 0; i < O.nray; i++) pc[O.rays[i].cam]++;
		slog("optical: blobs/cam %d %d %d %d, stereo candidates %d | L lock/acq/miss %u/%u/%u R %u/%u/%u | clk L %.1f us/s (%d s, rtt %u) R %.1f us/s (%d s, rtt %u) hmd %.1f us/s, LD delay %d..%d us",
		     pc[0], pc[1], pc[2], pc[3], O.ncand, O.stat_lock[0], O.stat_acq[0], O.stat_fail[0], O.stat_lock[1], O.stat_acq[1], O.stat_fail[1],
		     O.led[0].clk.b, O.led[0].clk.n, O.led[0].clk.cur_rtt, O.led[1].clk.b, O.led[1].clk.n, O.led[1].clk.cur_rtt, O.ldclk.b, O.ld_delay_min, O.ld_delay_max);
		memset(O.stat_lock, 0, sizeof O.stat_lock); memset(O.stat_acq, 0, sizeof O.stat_acq); memset(O.stat_fail, 0, sizeof O.stat_fail);
	}
	bool conn[2], locked[2];
	for (int h = 0; h < 2; h++) {
		conn[h] = (int64_t)(now - O.led[h].in_us) <= 1000000; /* controller connected (a scanning one is still fitted when its flash hits); signed: an input report may be newer than this frame's time */
		locked[h] = conn[h] && O.fix[h].valid && now - O.fix[h].host_us < 300000;
	}
	O.par_sum[par] += O.nray; O.par_n[par]++;
	if (!O.ncand) { /* nothing to fit, but the LED timing must keep working: a mistimed flash gives exactly this */
		pthread_mutex_lock(&O.lock);
		for (int h = 0; h < 2; h++) if (conn[h] && own[h]) led_servo(h, 0, now);
		led_scan_frame(O.nray, now, par);
		pthread_mutex_unlock(&O.lock);
		opt_timeline(now, NULL);
		if (g_opt_hook) {
			struct opt_pose none[2] = { 0 }; bool no[2] = { false, false };
			g_opt_hook(&(struct opt_frame_dbg){ ts, now, t_exp, hq, { hp[0], hp[1], hp[2] }, none, no, no, frame });
		}
		return;
	}

	/* world <- cam0 */
	quat q_wc = qmul(hq, O.q_hc); double R_wc[9], R_cw[9], hr[9], p_wc[3];
	m3_from_q(q_wc, R_wc);
	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) R_cw[i * 3 + j] = R_wc[j * 3 + i];
	m3_from_q(hq, hr); m3_mulv(hr, O.p_hc, p_wc); for (int k = 0; k < 3; k++) p_wc[k] += hp[k];

	/* each tracked hand's last fix propagated to this frame (cam0 frame): IMU rotation since the fix, velocity for the position */
	double pred_t[2][3], pred_R[2][9];
	for (int h = 0; h < 2; h++) {
		if (!locked[h]) continue;
		double dt = (now - O.fix[h].host_us) * 1e-6, pw[3], Rw[9];
		for (int k = 0; k < 3; k++) pw[k] = O.fix[h].p[k] + O.fix[h].v[k] * dt - p_wc[k];
		m3_mulv(R_cw, pw, pred_t[h]);
		if (O.fix[h].iq_ok && iok[h]) opt_imu_turned(h, iq[h], O.fix[h].iq, O.fix[h].q, Rw);
		else m3_from_q(O.fix[h].q, Rw);
		m3_mul(R_cw, Rw, pred_R[h]);
	}
	/* two controllers cannot occupy the same place: the other hand's position this frame (its fit, else its propagated fix) */
	struct { bool ok; struct opt_pose P; bool acq; } res[2] = { 0 };
	/* the other hand's position this frame: its fit, else its propagated fix - but only while that prediction is trustworthy (a fix
	 * younger than 120 ms of a hand moving slower than 0.6 m/s); a prediction of a fast hand is off by 5-10 cm and vetoed good fits
	 * of the other hand throughout a punching session */
	bool other_pred_ok[2];
	bool separated = led_mux() && O.mux_live[0] && O.mux_live[1] && O.led[0].par_ok && O.led[1].par_ok && O.led[0].par_meas != O.led[1].par_meas; /* each frame carries one hand's LEDs only: a fit cannot sit on the other controller */
	for (int h = 0; h < 2; h++) other_pred_ok[h] = !separated && locked[h] && now - O.fix[h].host_us < 120000 && v3_norm(O.fix[h].v) < 0.6;
#define OTHER_POS(h) (res[!(h)].ok ? res[!(h)].P.t : other_pred_ok[!(h)] ? pred_t[!(h)] : NULL)
#define TOO_CLOSE(t, o) opt_too_close(t, o)

	/* 1. tracked hands first (continuity): last optical rotation, predicted position.  The better-established
	 * track (more LEDs last frame) goes first; a track sitting on top of the other hand's is a stolen one and
	 * the less established of the two is dropped so the blobs can be re-acquired properly */
	int first = locked[1] && (!locked[0] || O.fix[1].n > O.fix[0].n);
	for (int oi = 0; oi < 2; oi++) {
		int h = first ^ oi;
		if (!locked[h] || !own[h]) continue;
		struct opt_used before; opt_used_save(&before);
		const double *prior_c = pred_t[h], *Rc = pred_R[h];
		bool imu_held = O.fix[h].iq_ok && iok[h];
		int tmin = imu_held ? 3 : (int)T.fit_min; /* with the orientation held by the IMU three LEDs (six image constraints) fix the position; the evidence check guards the correspondence */
		if (imu_held) { opt_prior_R = Rc; opt_prior_w = T.rot_prior; } /* the IMU-turned orientation holds the ring correspondence */
		res[h].ok = opt_fit(Rc, O.mirror[h], prior_c, (int)T.fit_min, &res[h].P);
		if (!res[h].ok) { /* no stereo support (close range, edge-on): image-only tracking from the prediction */
			struct opt_pose P = { 0 }; memcpy(P.R, Rc, sizeof P.R); memcpy(P.t, prior_c, sizeof P.t);
			opt_associate(&P, O.mirror[h], 8.0);
			if (P.n >= tmin) {
				P.res = opt_refine(&P, O.mirror[h], 4); opt_associate(&P, O.mirror[h], OPT_PX_ASSOC); P.res = opt_refine(&P, O.mirror[h], 3);
				double d[3] = { P.t[0] - prior_c[0], P.t[1] - prior_c[1], P.t[2] - prior_c[2] };
				if (P.n >= tmin && P.res < 3.0 && v3_norm(d) < (P.n >= (int)T.fit_min ? 0.15 : 0.08) && opt_evidence_ok(&P, O.mirror[h], NULL)) { opt_consume(&P, O.mirror[h]); res[h].ok = true; res[h].P = P; }
			}
		}
		if (!res[h].ok && imu_held) res[h].ok = opt_relock(h, Rc, prior_c, tmin, &res[h].P); /* fast motion: the prediction is a few cm off; re-seat the known orientation on the clusters near it */
		opt_prior_R = NULL;
		if (res[h].ok && O.fix[h].iq_ok && iok[h]) { /* the IMU says how the controller turned since the last fix: a fit that turned differently jumped to another ring correspondence */
			double rj = qangle(q_from_m3(pred_R[h]), q_from_m3(res[h].P.R)) * 180 / M_PI;
			if (rj > O.rot_jump_max[h]) O.rot_jump_max[h] = rj;
			if (rj > T.track_rot_deg) {
				O.rot_jumps[h]++;
				if (OPT_DUMP) slog("optical: %c track rejected: turned %.0f deg against the IMU (n=%d res=%.2f)", h ? 'R' : 'L', rj, res[h].P.n, res[h].P.res);
				res[h].ok = false; opt_used_restore(&before);
			}
		}
		if (res[h].ok && opt_grav_veto(h, res[h].P.R, R_wc, iq[h], iok[h], now, "track", res[h].P.n, res[h].P.res)) {
			res[h].ok = false; opt_used_restore(&before); O.fix[h].valid = false;
		}
		if (res[h].ok && res[!h].ok && TOO_CLOSE(res[h].P.t, res[!h].P.t) && O.fix[h].iq_ok && iok[h] && O.fix[!h].iq_ok && iok[!h]) {
			/* both fitted this frame from distinct blobs (the first one consumed its rays) and both turned with their own IMUs: two controllers held together, not a stolen track */
		} else if (res[h].ok && TOO_CLOSE(res[h].P.t, OTHER_POS(h))) { /* stolen track: both hands on one controller */
			bool keep_me = !res[!h].ok && O.fix[h].n > O.fix[!h].n; /* the other was not fitted this frame and is the weaker track */
			if (keep_me) { O.fix[!h].valid = false; locked[!h] = false; }
			else { res[h].ok = false; opt_used_restore(&before); O.fix[h].valid = false; }
			if (OPT_DUMP) slog("optical: %c sits on %c's controller: dropping %c", h ? 'R' : 'L', h ? 'L' : 'R', keep_me ? (h ? 'L' : 'R') : (h ? 'R' : 'L'));
		}
	}
	/* 2. acquisition.  With the controller IMU: gravity-aligned orientation (+ the known yaw correction once
	 * there was a fix), yaw sweep otherwise; IMU-free triangle matching as the fallback.  With ledMux the
	 * frame parity says which hand this is; without it both chiralities are tried on the remaining blobs and
	 * the better fit decides (the two LED layouts are mirror images); a second pass serves the other hand */
	bool qc_ok[2]; quat qcq[2]; int imf_q[2]; double imf_R[2][9]; bool yaw_known[2];
	pthread_mutex_lock(&O.lock);
	for (int h = 0; h < 2; h++) {
		qc_ok[h] = O.qc_ok[h]; qcq[h] = O.qc[h]; imf_q[h] = O.imf[h].quality; memcpy(imf_R[h], O.imf[h].R, sizeof imf_R[h]);
		double up[3] = { 0, 1, 0 }, gi[3]; if (iok[h]) qrot(qconj(iq[h]), up, gi);
		/* the yaw correction holds with a full frame calibration; with a tilt-only one only near the orientation it was made at */
		yaw_known[h] = qc_ok[h] && iok[h] && (imf_q[h] == 2 || (imf_q[h] == 1 && v3_dot(gi, O.qc_gi[h]) > cos(10 * M_PI / 180)));
	}
	pthread_mutex_unlock(&O.lock);
	for (int pass = 0; pass < 2; pass++) {
		int want = 0; for (int h = 0; h < 2; h++) if (conn[h] && own[h] && !res[h].ok) want++;
		if (!want) break;
		struct opt_used saved; opt_used_save(&saved);
		struct opt_pose P[2]; bool ok[2];
		for (int h = 0; h < 2; h++) {
			ok[h] = false;
			if (!conn[h] || !own[h] || res[h].ok) continue;
			const struct opt_fix *lf = &O.fix[h]; double Rcont[9]; /* a recent track's orientation turned by the IMU's motion since (cam0 <- model) */
			bool cont = iok[h] && lf->valid && lf->iq_ok && now > lf->host_us && now - lf->host_us < 3000000 && now - O.grav_rej_us[h] > 2000000; /* not after a gravity rejection: that track had a wrong correspondence */
			if (cont) {
				double Rw[9]; opt_imu_turned(h, iq[h], lf->iq, lf->q, Rw); m3_mul(R_cw, Rw, Rcont);
				opt_acq_hint = now - lf->host_us < 1500000 ? pred_t[h] : NULL;
				if (T.acq_imu != 0) ok[h] = opt_acquire_imu(O.mirror[h], Rw, R_cw, true, &P[h]);
				opt_acq_hint = NULL;
			}
			if (!ok[h] && T.acq_imu != 0 && iok[h] && imf_q[h] >= 1) { /* world <- model = yaw * (world <- IMU) * (IMU <- model) */
				double Rwi[9], Rw[9], Rt[9]; bool yk = yaw_known[h];
				m3_from_q(yk ? qnorm(qmul(qcq[h], iq[h])) : iq[h], Rwi);
				for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) Rt[i * 3 + j] = imf_R[h][j * 3 + i]; /* IMU <- model */
				m3_mul(Rwi, Rt, Rw);
				ok[h] = opt_acquire_imu(O.mirror[h], Rw, R_cw, yk, &P[h]);
				if (!ok[h] && yk) ok[h] = opt_acquire_imu(O.mirror[h], Rw, R_cw, false, &P[h]); /* the stored yaw may be stale */
			}
			if (!ok[h]) {
				ok[h] = opt_acquire(O.mirror[h], &P[h]);
				if (ok[h] && cont) { /* the IMU-free match must agree with where the IMU took the recent track: the ring's near-symmetric correspondences differ by 25.7 deg */
					double rj = qangle(q_from_m3(Rcont), q_from_m3(P[h].R)) * 180 / M_PI;
					if (rj > T.track_rot_deg) {
						/* unless the last track was itself the dubious one: a weak (<= 5 LEDs) or stale (> 0.5 s) track vs. a match that passes the
						 * gravity check with a calibrated model<->IMU rotation.  Otherwise a track that drifted into a wrong correspondence blocks every correct re-acquisition */
						double gd; bool grav_ok = iok[h] && O.imf[h].quality == 2 && opt_grav_ok(h, P[h].R, R_wc, iq[h], iok[h], &gd);
						bool weak_track = lf->n <= 5 || now - lf->host_us > 500000;
						if (grav_ok && weak_track) {
							slog("optical: acq %c: match turned %.0f deg against the last track (n=%d, %.1f s old) but agrees with gravity: the track was wrong, taking the match (n=%d res=%.2f)", h ? 'R' : 'L', rj, lf->n, (now - lf->host_us) * 1e-6, P[h].n, P[h].res);
						} else {
							if (OPT_DUMP) slog("optical: acq %c rejected: triangle match turned %.0f deg against the IMU since the last track (n=%d res=%.2f)", h ? 'R' : 'L', rj, P[h].n, P[h].res);
							ok[h] = false; opt_used_restore(&saved);
						}
					}
				}
			}
			if (ok[h] && opt_grav_veto(h, P[h].R, R_wc, iq[h], iok[h], now, "acq", P[h].n, P[h].res)) { ok[h] = false; opt_used_restore(&saved); }
			if (ok[h] && O.imf[h].quality < 2) { /* no calibrated gravity check: the turn since the last established track is the only guard against a wrong ring correspondence */
				double turn, tm, tol;
				if (opt_turn_test(h, P[h].R, R_wc, iq[h], iok[h], now, &turn, &tm, &tol) && fabs(tm - turn) > tol + 10) {
					if (++O.tdoubt[h] >= 3) { O.tref[h].valid = false; O.tdoubt[h] = 0; slog("optical: acq %c: three matches in a row turned differently from the IMU since the last track: dropping that track as the reference", h ? 'R' : 'L'); }
					else { if (OPT_DUMP) slog("optical: acq %c rejected: turned %.0f deg against the IMU's %.0f deg since the last track (n=%d res=%.2f)", h ? 'R' : 'L', tm, turn, P[h].n, P[h].res); ok[h] = false; opt_used_restore(&saved); }
				}
			}
			if (ok[h]) { /* a controller is held by a person wearing the headset: lamps 3 m up are not controllers */
				double pw[3], tmp[3]; m3_mulv(R_wc, P[h].t, tmp); for (int k = 0; k < 3; k++) pw[k] = p_wc[k] + tmp[k];
				double d[3] = { pw[0] - hp[0], pw[1] - hp[1], pw[2] - hp[2] };
				if (v3_norm(d) > 1.3 || d[1] > 0.9) { if (OPT_DUMP) slog("optical: acq %c rejected: %.2f m from the head, %.2f m above it (n=%d)", h ? 'R' : 'L', v3_norm(d), d[1], P[h].n); ok[h] = false; opt_used_restore(&saved); }
			}
			if (ok[h] && TOO_CLOSE(P[h].t, OTHER_POS(h))) { /* on top of the other hand (fitted this frame or tracked on the other parity) */
				if (OPT_DUMP) slog("optical: acq %c rejected: on top of %c (n=%d res=%.2f)", h ? 'R' : 'L', h ? 'L' : 'R', P[h].n, P[h].res);
				ok[h] = false; opt_used_restore(&saved);
			}
			if (ok[h] && want > 1) opt_used_restore(&saved); /* both hands want this frame: compare before consuming */
		}
		int h = -1;
		if (ok[0] && ok[1]) h = P[1].n > P[0].n || (P[1].n == P[0].n && P[1].res < P[0].res) ? 1 : 0;
		else if (ok[0] || ok[1]) h = ok[1];
		if (h < 0) break;
		if (OPT_DUMP && iok[h] && O.fix[h].valid && O.fix[h].iq_ok && now - O.fix[h].host_us < 10000000) { /* how far did the re-acquired orientation turn from where the IMU says the last track went? */
			double Rw[9], Rc[9]; opt_imu_turned(h, iq[h], O.fix[h].iq, O.fix[h].q, Rw); m3_mul(R_cw, Rw, Rc);
			slog("optical: acq %c n=%d res=%.2f: %.0f deg from the IMU-propagated last track (%.1f s old)", h ? 'R' : 'L', P[h].n, P[h].res, qangle(q_from_m3(Rc), q_from_m3(P[h].R)) * 180 / M_PI, (now - O.fix[h].host_us) * 1e-6);
		}
		res[h].ok = true; res[h].acq = true; res[h].P = P[h];
		if (want > 1) opt_consume(&P[h], O.mirror[h]);
	}
	/* 3. ledMux liveness: a tracked hand's LEDs must be absent from the other parity's frames.  Probe the
	 * propagated pose against the blobs left over there; once the LEDs are lit on both parities (the
	 * controller ignored the 2-frame cycle, or runs free) the hand is fitted on every frame instead, and
	 * flips back when its foreign-parity fits dry up while the own-parity ones continue. */
	if (led_mux()) for (int h = 0; h < 2; h++) { /* where do this hand's fits land?  Confirms the flash parity while it is unknown */
		struct led_sync *L = &O.led[h];
		if (!res[h].ok) continue;
		L->par_fit_us = now;
		if (L->par_ok || now - L->par_flip_us < 1200000) continue; /* a re-anchor takes a Bluetooth round trip and a cycle to show */
		L->par_votes[par]++;
		if (L->par_votes[par] >= 5) { pthread_mutex_lock(&O.lock); led_par_set(h, par, now, "fits"); pthread_mutex_unlock(&O.lock); }
		else if (L->par_votes[0] + L->par_votes[1] >= 12) L->par_votes[0] = L->par_votes[1] = 0;
	}
	if (led_mux()) for (int h = 0; h < 2; h++) { /* a measured parity goes stale 3 s after the last fit or LED check: the flash can move to the other parity (re-anchor, clock bias), so fit on every frame again until it is re-measured */
		struct led_sync *L = &O.led[h]; uint64_t last = L->par_fit_us > L->scan_done_us ? L->par_fit_us : L->scan_done_us;
		if (L->par_ok && now - last > 3000000) { L->par_ok = false; L->par_votes[0] = L->par_votes[1] = 0; }
	}
	if (led_mux()) for (int h = 0; h < 2; h++) {
		if (par == O.led[h].par_meas || !locked[h] || !O.led[h].par_ok || now - O.own_ok_us[h] > 100000) continue;
		if (locked[h ^ 1]) { /* hands close together: the other hand's blobs would masquerade as this hand's on its parity */
			double d[3] = { O.fix[h].p[0] - O.fix[h ^ 1].p[0], O.fix[h].p[1] - O.fix[h ^ 1].p[1], O.fix[h].p[2] - O.fix[h ^ 1].p[2] };
			if (v3_norm(d) < 0.25) continue;
		}
		bool lit;
		if (own[h]) lit = res[h].ok;
		else {
			struct opt_pose P = { 0 }; memcpy(P.R, pred_R[h], sizeof P.R); memcpy(P.t, pred_t[h], sizeof P.t);
			opt_associate(&P, O.mirror[h], 6.0);
			struct opt_evid e = opt_evidence(&P, O.mirror[h]);
			lit = P.n >= 5 && P.nobs >= 0.6 * (P.nobs + e.missing);
		}
		O.mux_ema[h] = 0.95 * O.mux_ema[h] + (lit ? 0.05 : 0);
		bool live = O.mux_live[h] ? O.mux_ema[h] < 0.6 : O.mux_ema[h] < 0.2;
		if (live != O.mux_live[h]) {
			pthread_mutex_lock(&O.lock); O.mux_live[h] = live; pthread_mutex_unlock(&O.lock);
			slog("optical: %c LEDs %s", h ? 'R' : 'L', live ? "back on their own frame parity: 30 Hz multiplexed tracking" : "lit on both frame parities (ledMux not effective): tracking on every frame");
		}
	}
	for (int h = 0; h < 2; h++) if (res[h].ok) { O.par_fit[par][h]++; if (!led_mux() || !O.led[h].par_ok || par == O.led[h].par_meas) O.own_ok_us[h] = now; }
	for (int h = 0; h < 2; h++) if (conn[h]) { /* stuck LEDs: lit on both parities for 30 s -> tell the hand OFF for 0.6 s; if it keeps being fitted the firmware ignores us */
		struct led_sync *L = &O.led[h];
		if (L->offtest_us) {
			if (now < L->offtest_us) { if (++L->offtest_frames > 8) L->offtest_fits += res[h].ok; }
			else {
				int frames = L->offtest_frames - 8;
				bool stuck = frames >= 10 && L->offtest_fits >= 0.7 * frames;
				slog("optical: %c LED off-test: %d of %d frames still fitted with the LEDs told OFF: %s", h ? 'R' : 'L', L->offtest_fits, frames > 0 ? frames : 0, stuck ? "the firmware ignores LED commands (stuck on), reconnecting the controller" : "the LEDs obey");
				pthread_mutex_lock(&O.lock); L->offtest_us = 0; L->dirty = true; L->muxdead_us = stuck ? 0 : now; pthread_mutex_unlock(&O.lock);
				if (stuck) { L->stuck_us = now; if (g_led_stuck_hook) g_led_stuck_hook(h); }
			}
		} else if (!O.mux_live[h] && !O.led_mode && !L->scan) {
			if (!L->muxdead_us) L->muxdead_us = now;
			else if (now - L->muxdead_us > 30000000 && now - L->stuck_us > 300000000) { pthread_mutex_lock(&O.lock); L->offtest_us = now + 600000; L->offtest_frames = L->offtest_fits = 0; L->dirty = true; pthread_mutex_unlock(&O.lock); }
		} else L->muxdead_us = 0;
	}
#undef OTHER_POS
#undef TOO_CLOSE
	for (int h = 0; h < 2; h++) { if (!conn[h] || !own[h]) continue; if (res[h].ok) { if (res[h].acq) O.stat_acq[h]++; else O.stat_lock[h]++; } else O.stat_fail[h]++; }
	if (OPT_DUMP) for (int h = 0; h < 2; h++) { /* where do tracks get lost?  position relative to the head (head frame: x right, y up, -z forward) */
		static bool was[2]; static int lost_n[2];
		if (was[h] && !res[h].ok && ++lost_n[h] == 6) { /* 100 ms without a fit */
			double hr[9], hrt[9], d[3], rel[3]; m3_from_q(hq, hr); for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) hrt[i * 3 + j] = hr[j * 3 + i];
			for (int k = 0; k < 3; k++) d[k] = O.fix[h].p[k] - hp[k]; m3_mulv(hrt, d, rel);
			slog("optical: %c track lost at head-relative %.2f %.2f %.2f (dist %.2f m, last n=%d, blobs %d)", h ? 'R' : 'L', rel[0], rel[1], rel[2], v3_norm(rel), O.fix[h].n, O.nray);
			was[h] = false;
		}
		if (res[h].ok) { was[h] = true; lost_n[h] = 0; }
	}
	{ /* LED timing: servo with this frame's LED counts, scan fed with the blobs not explained by the other (tracked) controller */
		int unexplained = O.nray; for (int h = 0; h < 2; h++) if (res[h].ok && h != scanh) unexplained -= res[h].P.nobs;
		if (unexplained < 0) unexplained = 0;
		pthread_mutex_lock(&O.lock);
		for (int h = 0; h < 2; h++) if (conn[h] && own[h]) led_servo(h, res[h].ok ? res[h].P.n : 0, now);
		led_scan_frame(unexplained, now, par);
		pthread_mutex_unlock(&O.lock);
	}
	{ bool ok[2] = { res[0].ok, res[1].ok }; opt_timeline(now, ok); }
	pthread_mutex_lock(&O.lock);
	for (int h = 0; h < 2; h++) {
		if (!res[h].ok) continue;
		struct opt_fix *f = &O.fix[h];
		double pw[3], tmp[3];
		m3_mulv(R_wc, res[h].P.t, tmp); for (int k = 0; k < 3; k++) pw[k] = p_wc[k] + tmp[k];
		if (f->valid && t_exp > f->host_us && t_exp - f->host_us < 100000) {
			double dt = (t_exp - f->host_us) * 1e-6;
			for (int k = 0; k < 3; k++) { double v = (pw[k] - f->p[k]) / dt; f->v[k] = 0.6 * f->v[k] + 0.4 * CLAMP(v, -10.0, 10.0); }
		} else memset(f->v, 0, sizeof f->v);
		memcpy(f->p, pw, sizeof f->p);
		double Rw[9]; m3_mul(R_wc, res[h].P.R, Rw);
		f->q = q_from_m3(Rw); f->iq = iq[h]; f->iq_ok = iok[h];
		f->streak = res[h].acq ? 1 : f->streak + 1;
		f->n = res[h].P.n; f->res = res[h].P.res; f->host_us = t_exp; f->valid = true; O.lock_us[h] = now; O.led[h].scan_wait_us = 5000000; O.led[h].lat_seen_us = now; O.led[h].dark_checks = 0; O.grav_rej_run[h] = 0; O.tdoubt[h] = 0;
		if (iok[h] && f->streak >= 10) { struct turn_ref *r = &O.tref[h]; r->valid = true; r->host_us = t_exp; r->q = f->q; r->iq = iq[h]; r->n = f->n; } /* an established track: reference for the turn test */
		if (iok[h] && f->streak >= 2) { /* yaw correction: world <- IMU world.  (world <- model)(model <- IMU)(IMU <- IMU world) is a pure yaw once the model<->IMU rotation is known; only its yaw is used */
			bool apart = !O.fix[!h].valid || now - O.fix[!h].host_us > 1000000 || v3_norm((double[3]){ f->p[0] - O.fix[!h].p[0], f->p[1] - O.fix[!h].p[1], f->p[2] - O.fix[!h].p[2] }) > 0.3;
			if (f->n >= 8 && f->res < 0.6 * opt_gate(&res[h].P, OPT_PX_ASSOC) && f->streak >= 30 && apart) imf_push(h, f->q, iq[h], t_exp); /* learn only from unambiguous fits (residual well inside the distance-scaled gate, 1 s of continuous track, hands apart): a swapped hand or a wrong ring slot that slips in poisons the estimate for the whole session */
			quat qfull = qmul(qmul(f->q, O.imf[h].quality ? q_from_m3(O.imf[h].R) : (quat){ 1, 0, 0, 0 }), qconj(iq[h]));
			quat qc = qrotvec(0, qyaw(qfull), 0);
			if (O.imf[h].quality) { O.qc[h] = qc; O.qc_n[h] = f->n; O.qc_us[h] = t_exp; O.qc_ok[h] = true; double up[3] = { 0, 1, 0 }; qrot(qconj(iq[h]), up, O.qc_gi[h]); }
			O.qc_tilt[h] = 0.8 * O.qc_tilt[h] + 0.2 * qangle(qfull, qc) * 180 / M_PI; /* residual tilt: how far the frame estimate is from explaining this fit */
		}
		static unsigned logged[2];
		if (logged[h]++ % (SLOP_DBG ? 600 : 1800) == 0) slog("optical: %c %s n=%d res=%.2fpx pos %.3f %.3f %.3f (head %.3f %.3f %.3f)", h ? 'R' : 'L', res[h].acq ? "acquired" : "lock", f->n, f->res, pw[0], pw[1], pw[2], hp[0], hp[1], hp[2]);
	}
	pthread_mutex_unlock(&O.lock);
	for (int h = 0; h < 2; h++) if (conn[h] && own[h]) {
		double pw[3] = { 0, 0, 0 }, tmp[3];
		if (res[h].ok) { m3_mulv(R_wc, res[h].P.t, tmp); for (int k = 0; k < 3; k++) pw[k] = p_wc[k] + tmp[k]; }
		rec_fit(h, frame, res[h].ok, res[h].acq, res[h].P.n, res[h].P.res, res[h].P.R, res[h].P.t, pw, now);
	}
	if (g_opt_hook) {
		struct opt_pose P[2] = { res[0].P, res[1].P }; bool ok[2] = { res[0].ok, res[1].ok }, acq[2] = { res[0].acq, res[1].acq };
		g_opt_hook(&(struct opt_frame_dbg){ ts, now, t_exp, hq, { hp[0], hp[1], hp[2] }, P, ok, acq, frame });
	}
}

static bool optical_calib_ok(void) { pthread_mutex_lock(&O.lock); bool ok = O.calib_ok; pthread_mutex_unlock(&O.lock); return ok; }
static uint64_t optical_last_ld_us(void) { pthread_mutex_lock(&O.lock); uint64_t t = O.ld_host_us; pthread_mutex_unlock(&O.lock); return t; }

/* sense.c: publish the IMU orientation (world <- IMU, the IMU's own gravity-aligned world), get the latest
 * optical orientation correction: q_world = qc.q * q_imu */
struct opt_qcorr { bool ok; quat q; int n; uint64_t us; };
static struct opt_qcorr optical_imu_update(int hand, quat q)
{
	pthread_mutex_lock(&O.lock);
	O.imu_q[hand] = q; O.imu_ok[hand] = true;
	O.ih[hand][O.ih_n[hand]++ % 128] = (struct imu_hist){ O.led[hand].sample_host_us, q };
	struct opt_qcorr c = { O.qc_ok[hand], O.qc[hand], O.qc_n[hand], O.qc_us[hand] };
	pthread_mutex_unlock(&O.lock);
	return c;
}
/* sense.c: latest optical fix (copy) */
static struct opt_fix optical_fix(int hand)
{
	pthread_mutex_lock(&O.lock);
	struct opt_fix f = O.fix[hand];
	pthread_mutex_unlock(&O.lock);
	if (f.streak < 2) f.valid = false; /* a fresh acquisition is published once a following frame confirms it */
	return f;
}

static void optical_tune_apply(void);
static void optical_init(void)
{
	pthread_mutex_init(&O.lock, NULL); pthread_mutex_init(&O.mb_lock, NULL); pthread_cond_init(&O.mb_cond, NULL);
	O.enabled = cfg_bool("opticalTracking", true);
	O.lock_us[0] = O.lock_us[1] = O.led[0].probe_last_us = O.led[1].probe_last_us = now_us(); /* grace period: the stored latencies get a chance before any scan */
	if (O.enabled) { O.run = true; pthread_create(&O.thread, NULL, optical_thread, NULL); }
#if SLOP_DBG
	O.dump = cfg_bool("opticalDump", false);
	O.dump_every = cfg_i32("opticalDumpEvery", 31); if (O.dump_every < 1) O.dump_every = 1;
#endif
	if (!O.dump_dir[0]) { time_t t = time(NULL); struct tm tm; localtime_r(&t, &tm); strftime(O.dump_dir, sizeof O.dump_dir, "slopvr_ld/%Y%m%d-%H%M%S", &tm); } /* created at the first dump (OPT_DUMP can be toggled at run time) */
	for (int i = 0; i < OPT_NLED; i++) for (int j = 0; j < OPT_NLED; j++) {
		double d[3] = { (opt_led_pos[i][0] - opt_led_pos[j][0]) * 1e-3, (opt_led_pos[i][1] - opt_led_pos[j][1]) * 1e-3, (opt_led_pos[i][2] - opt_led_pos[j][2]) * 1e-3 };
		opt_led_d[i][j] = v3_norm(d); if (opt_led_d[i][j] > opt_span) opt_span = opt_led_d[i][j];
	}
#if SLOP_DBG /* experiments, test tool only: a different flash cycle; forcing the LEDs off / continuous / INIT / flipping (modes 5, 6, 1, 7) */
	O.led_cycle_ns = (uint32_t)cfg_i32("ledCycleNs", 0);
	O.led_mode = cfg_i32("ledPhase", 0);
#endif
	for (int h = 0; h < 2; h++) { O.mux_live[h] = true; imf_load(h); slog("optical: %c model<->IMU rotation: %s (spread %.0f deg)", h ? 'R' : 'L', O.imf[h].spread > 60 ? "learnt in an earlier session" : "factory prior", O.imf[h].spread); }
	for (int h = 0; h < 2; h++) { /* latency tuned by an earlier run */
		char path[256]; led_sync_path(h, path, sizeof path);
		FILE *f = fopen(path, "r");
		if (!f) continue;
		int lat; int got = fscanf(f, "v2 %d", &lat);
		if (got == 1 && lat >= 0 && lat < LED_CYCLE_US) { O.led[h].lat = lat; O.led[h].lat_ok = true; }
		fclose(f);
	}
	{ char d[256]; snprintf(d, sizeof d, "%s/.config", getenv("HOME") ? getenv("HOME") : "/tmp"); mkdir(d, 0755); strncat(d, "/slopvr", sizeof d - strlen(d) - 1); mkdir(d, 0755); }
	optical_tune_apply();
}
/* tunables that live here (tune.c): camera placement in the head frame, LED layout chirality, flash window */
static void optical_tune_apply(void)
{
	pthread_mutex_lock(&O.lock);
	O.mirror[0] = T.mirror_L != 0; O.mirror[1] = T.mirror_R != 0; /* the table is the right-hand geometry */
	O.led_period = (int)T.led_period;
	/* cam0 (front-left, toed out) in the SLAM/IMU frame; cam axes x right, y down, z forward */
	O.q_hc = qnorm(qmul(qmul(qmul(qaxis(0, 1, 0, T.cam_yaw), qaxis(1, 0, 0, T.cam_pitch)), qaxis(0, 0, 1, T.cam_roll)), qaxis(1, 0, 0, 180)));
	O.p_hc[0] = T.cam_x; O.p_hc[1] = T.cam_y; O.p_hc[2] = T.cam_z;
	pthread_mutex_unlock(&O.lock);
}

static void optical_cleanup(void)
{
	if (!O.run) return;
	pthread_mutex_lock(&O.mb_lock); O.run = false; pthread_cond_signal(&O.mb_cond); pthread_mutex_unlock(&O.mb_lock);
	pthread_join(O.thread, NULL);
}
