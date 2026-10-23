/* Guided tracking calibration / test flow (psvr2linuxtest).
 *
 * Started with the Options/Create button of either controller (hold 1 s), F5, or --guide.  Eight steps, each one concrete
 * action, shown in the headset as a HUD (text + an animated pictogram of the motion + a live checklist of what is still
 * missing); a step ends by itself when its checklist is complete, or at a timeout; the trigger (full press) skips, holding
 * Options for 2 s aborts.  A haptic pulse marks every step (on the hand the step is about).
 *
 * Every step is marked in the log and in the session recording ("calib: step ..."), so the .slop can be cut per step
 * offline, and a replay of the recording follows the markers and recomputes the per-step metrics with the current tracker
 * (regression harness).  Metrics go to the log at each step end and to ~/.cache/slopvr_rec/calib-<time>.txt. */

enum { CF_IDLE, CF_WAIT, CF_RUN, CF_DONE };
enum { ST_STILL, ST_TURN_R, ST_TURN_L, ST_REACH, ST_FAST, ST_HIDE, ST_CROSS, ST_LOOK, ST_N };
struct cf_step { const char *id, *title; const char *line[2]; double min_s, max_s; int hand; /* -1 both */ };
static const struct cf_step CF_STEPS[ST_N] = { /* lines <= 22 chars (3x font in the headset HUD) */
	{ "still", "HOLD STILL",       { "Both in front of you,",  "do not move."            }, 6, 20, -1 },
	{ "turn_R", "TURN RIGHT HAND", { "Turn the RIGHT one",      "as the picture shows."  }, 10, 40, 1 },
	{ "turn_L", "TURN LEFT HAND",  { "Turn the LEFT one",       "as the picture shows."  }, 10, 40, 0 },
	{ "reach", "REACH",            { "Move both hands to",      "each target in turn."   }, 10, 45, -1 },
	{ "fast",  "PUNCH FAST",       { "Punch forward fast,",     "both hands, 10 times."  }, 8, 25, -1 },
	{ "hide",  "HIDE AND SHOW",    { "Put it behind your back", "and bring it back, 3x." }, 8, 40, 1 },
	{ "cross", "CROSS HANDS",      { "Cross your arms and",     "open them again, 4x."   }, 6, 30, -1 },
	{ "look",  "LOOK AROUND",      { "Hands still in front;",   "turn your head only."   }, 8, 30, -1 },
};
/* checklist bits */
enum { TURN_UP, TURN_DOWN, TURN_LEFT, TURN_RIGHT, TURN_AT, TURN_AWAY, TURN_N };
static const char *turn_name[TURN_N] = { "up", "down", "left", "right", "to you", "away" };
enum { RCH_LEFT, RCH_RIGHT, RCH_UP, RCH_DOWN, RCH_NEAR, RCH_FAR, RCH_N };
static const char *reach_name[RCH_N] = { "left", "right", "up", "down", "near", "far" };
enum { LOOK_LEFT, LOOK_RIGHT, LOOK_UP, LOOK_DOWN, LOOK_N };
static const char *look_name[LOOK_N] = { "left", "right", "up", "down" };

struct cf_hand {
	unsigned frames, fits, acq, few, miss; double n_sum, res_sum; int n_min; /* frames: this hand's frames (its flash parity); miss: those with blobs but no fit */
	double pos[3], t; bool have;                 /* last fit (world) */
	double m[3], s2[3]; unsigned nstat;          /* Welford: position jitter */
	double dmin, dmax, vmax; unsigned fast_fits;
	uint64_t lost_us, vis_us; unsigned gaps; double reacq_sum, reacq_max, gap_max; /* re-acquisition after an outage >= 0.5 s */
	unsigned jumps0;
	unsigned hmove_frames; double hmove_s2;      /* position spread while the head turns fast (look step) */
	unsigned turn_bits, reach_bits; double rel[3]; uint64_t rel_us; /* checklists; rel: head-relative position (x right, y up, -z forward) of the last fit */
};
static struct calflow {
	pthread_mutex_t lock;
	int state, step; uint64_t t_start, t_step, settle_t0, motion_us, t_opt[2], t_trig[2]; bool trig_edge[2], opt_edge[2];
	struct cf_hand h[2]; uint16_t cover[4]; unsigned frames, swaps, crossings; bool crossed; quat hq_last, hq0, hq_ring[8]; unsigned hq_n; bool hq_ok, hq0_ok; double head_rate_max; unsigned look_bits;
	int hide_hand;                               /* hide step: which hand is being hidden now (R first, then L) */
	char live[8][120]; int nlive;                /* live metric lines for the panel */
	char rep[64][160]; int nrep;                 /* report lines */
	char path[320];
	bool follow;                                 /* replay: steps come from the recorded markers */
} CF = { .lock = PTHREAD_MUTEX_INITIALIZER };

static void cf_log(const char *fmt, ...)
{
	char b[300]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
	slog("%s", b); /* -> stderr/panel log and the session recording */
	if (CF.nrep < (int)ARRAY_LEN(CF.rep)) snprintf(CF.rep[CF.nrep++], sizeof CF.rep[0], "%s", b);
}
static void cf_pulse(int hand, float dur, float amp) /* hand -1: both */
{
	if (U.replay) return;
	for (int h = 0; h < 2; h++) {
		if (hand >= 0 && h != hand) continue;
		struct sense *s = &S[h];
		pthread_mutex_lock(&s->hap_lock);
		s->hap.freq = 170.f; s->hap.amp = amp; s->hap.samples_left = (int)(dur * PCM_RATE); s->hap.phase = 0;
		pthread_mutex_unlock(&s->hap_lock);
	}
}
static void cf_reset_acc(uint64_t now)
{
	pthread_mutex_lock(&CF.lock);
	memset(CF.h, 0, sizeof CF.h); memset(CF.cover, 0, sizeof CF.cover); CF.frames = 0; CF.swaps = 0; CF.crossings = 0; CF.crossed = false; CF.hq_ok = false; CF.hq0_ok = false; CF.head_rate_max = 0; CF.look_bits = 0;
	for (int h = 0; h < 2; h++) { CF.h[h].dmin = 1e9; CF.h[h].n_min = 99; CF.h[h].lost_us = now; }
	pthread_mutex_unlock(&CF.lock);
	pthread_mutex_lock(&O.lock);
	for (int h = 0; h < 2; h++) CF.h[h].jumps0 = O.rot_jumps[h];
	pthread_mutex_unlock(&O.lock);
}

/* optical thread, once per LD frame (from app_opt_hook) */
static void calflow_on_frame(const struct opt_frame_dbg *f)
{
	if (CF.state != CF_RUN) return;
	double R_wc[9], hr[9], hrt[9], p_wc[3], pw[2][3]; bool ok[2];
	m3_from_q(qmul(f->hq, O.q_hc), R_wc); m3_from_q(f->hq, hr); m3_mulv(hr, O.p_hc, p_wc); for (int k = 0; k < 3; k++) p_wc[k] += f->hp[k];
	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) hrt[i * 3 + j] = hr[j * 3 + i];
	for (int h = 0; h < 2; h++) { ok[h] = f->ok[h]; if (ok[h]) { double t[3]; m3_mulv(R_wc, f->P[h].t, t); for (int k = 0; k < 3; k++) pw[h][k] = p_wc[k] + t[k]; } }
	pthread_mutex_lock(&CF.lock);
	int step = CF.step; uint64_t now = f->now;
	CF.frames++;
	double head_rate = 0;
	if (CF.hq_ok) { head_rate = qangle(CF.hq_last, f->hq) * 180 / M_PI / 0.0167; if (head_rate > CF.head_rate_max) CF.head_rate_max = head_rate; }
	CF.hq_last = f->hq; CF.hq_ok = true;
	if (!CF.hq0_ok) { CF.hq0 = f->hq; CF.hq0_ok = true; }
	CF.hq_ring[CF.hq_n % 8] = f->hq; CF.hq_n++;
	if (CF.hq_n > 8 && qangle(CF.hq_ring[CF.hq_n % 8], f->hq) * 180 / M_PI / (7 * 0.0167) > 20) CF.motion_us = now; /* over 7 frames: the SLAM pose jitter (~15 deg/s frame to frame) averages out */
	if (step == ST_LOOK && CF.hq0_ok) { /* head yaw/pitch relative to where the step started */
		double R0[9], R0t[9], Rn[9], D[9]; m3_from_q(CF.hq0, R0); m3_from_q(f->hq, Rn);
		for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) R0t[i * 3 + j] = R0[j * 3 + i];
		m3_mul(R0t, Rn, D); /* head now in the start frame: forward (-z) of it */
		double fx = -D[2], fy = -D[5], fz = -D[8]; double yaw = atan2(-fx, -fz) * 180 / M_PI, pitch = asin(CLAMP(fy, -1.0, 1.0)) * 180 / M_PI;
		if (yaw < -35) CF.look_bits |= 1 << LOOK_LEFT; if (yaw > 35) CF.look_bits |= 1 << LOOK_RIGHT; if (pitch > 25) CF.look_bits |= 1 << LOOK_UP; if (pitch < -25) CF.look_bits |= 1 << LOOK_DOWN;
	}
	for (int r = 0; r < O.nray; r++) { /* image coverage: 4x4 cells per camera */
		const struct opt_ray *ray = &O.rays[r]; int cx = (int)(ray->uv[0] / 254), cy = (int)(ray->uv[1] / 254);
		if (ray->cam < 4 && cx >= 0 && cx < 4 && cy >= 0 && cy < 4) CF.cover[ray->cam] |= (uint16_t)(1u << (cy * 4 + cx));
	}
	/* hand swap heuristic: both fitted now and a moment ago, each closer to where the other was */
	if (ok[0] && ok[1] && CF.h[0].have && CF.h[1].have && now - CF.h[0].t < 200000 && now - CF.h[1].t < 200000) {
		double d00 = 0, d01 = 0, d11 = 0, d10 = 0;
		for (int k = 0; k < 3; k++) { d00 += pow(pw[0][k] - CF.h[0].pos[k], 2); d01 += pow(pw[0][k] - CF.h[1].pos[k], 2); d11 += pow(pw[1][k] - CF.h[1].pos[k], 2); d10 += pow(pw[1][k] - CF.h[0].pos[k], 2); }
		if (d01 < d00 && d10 < d11 && d00 > 0.05 * 0.05 && d11 > 0.05 * 0.05) CF.swaps++;
	}
	for (int h = 0; h < 2; h++) {
		struct cf_hand *H = &CF.h[h];
		bool mine = !led_mux() || !O.mux_live[h] || (int)(f->frame & 1) == O.led[h].par_meas; /* this hand's frame? (the last measured parity also while it is stale) */
		if (mine || ok[h]) H->frames++;
		if (!ok[h]) {
			if (mine && O.nray >= 4) { H->miss++; if (!H->vis_us && H->lost_us) H->vis_us = now; } /* blobs but no fit; after an outage the re-acquisition clock starts here */
			continue;
		}
		H->fits++; H->acq += f->acq[h]; H->n_sum += f->P[h].n; H->res_sum += f->P[h].res; if (f->P[h].n < H->n_min) H->n_min = f->P[h].n; if (f->P[h].n <= 5) H->few++;
		double d = v3_norm(f->P[h].t); if (d < H->dmin) H->dmin = d; if (d > H->dmax) H->dmax = d;
		if (H->have && now > H->t && now - H->t < 100000) {
			double dt = (now - H->t) * 1e-6, dp[3] = { pw[h][0] - H->pos[0], pw[h][1] - H->pos[1], pw[h][2] - H->pos[2] }, v = v3_norm(dp) / dt;
			if (v > H->vmax && v < 15) H->vmax = v;
			if (v > 1.0) H->fast_fits++;
			if (v > 0.25) CF.motion_us = now;
		}
		if (H->lost_us && now - H->lost_us > 500000) { /* an outage ended */
			double gap = (now - H->lost_us) * 1e-6, re = H->vis_us && now > H->vis_us ? (now - H->vis_us) * 1e-6 : gap;
			H->gaps++; H->reacq_sum += re; if (re > H->reacq_max) H->reacq_max = re; if (gap > H->gap_max) H->gap_max = gap;
		}
		H->lost_us = now; H->vis_us = 0;
		H->nstat++; for (int k = 0; k < 3; k++) { double dlt = pw[h][k] - H->m[k]; H->m[k] += dlt / H->nstat; H->s2[k] += dlt * (pw[h][k] - H->m[k]); }
		if (step == ST_LOOK && head_rate > 30 && H->have) { double dp[3] = { pw[h][0] - H->pos[0], pw[h][1] - H->pos[1], pw[h][2] - H->pos[2] }; H->hmove_frames++; H->hmove_s2 += v3_dot(dp, dp); }
		{ /* head-relative position: x right, y up, -z forward */
			double dw[3] = { pw[h][0] - f->hp[0], pw[h][1] - f->hp[1], pw[h][2] - f->hp[2] }; m3_mulv(hrt, dw, H->rel); H->rel_us = now;
			double dist = v3_norm(H->rel);
			if (H->rel[0] < -0.30) H->reach_bits |= 1 << RCH_LEFT; if (H->rel[0] > 0.30) H->reach_bits |= 1 << RCH_RIGHT;
			if (H->rel[1] > 0.12) H->reach_bits |= 1 << RCH_UP; if (H->rel[1] < -0.45) H->reach_bits |= 1 << RCH_DOWN;
			if (dist < 0.30) H->reach_bits |= 1 << RCH_NEAR; if (dist > 0.60) H->reach_bits |= 1 << RCH_FAR;
		}
		{ /* turn coverage: where the controller's ring faces, in the head frame (model +z is the ring's normal: the ring LEDs sit at z 4..43 mm with the three grip LEDs at z -60) */
			double Rw[9], ax[3], ah[3]; m3_mul(R_wc, f->P[h].R, Rw);
			ax[0] = Rw[2]; ax[1] = Rw[5]; ax[2] = Rw[8]; m3_mulv(hrt, ax, ah); /* model +z in head coords */
			if (ah[1] > 0.7) H->turn_bits |= 1 << TURN_UP; if (ah[1] < -0.7) H->turn_bits |= 1 << TURN_DOWN;
			if (ah[0] < -0.7) H->turn_bits |= 1 << TURN_LEFT; if (ah[0] > 0.7) H->turn_bits |= 1 << TURN_RIGHT;
			if (ah[2] > 0.7) H->turn_bits |= 1 << TURN_AT; if (ah[2] < -0.7) H->turn_bits |= 1 << TURN_AWAY; /* +z of the head frame points backwards: towards the user */
		}
		memcpy(H->pos, pw[h], sizeof H->pos); H->t = now; H->have = true;
	}
	if (step == ST_CROSS && CF.h[0].rel_us && CF.h[1].rel_us && now - CF.h[0].rel_us < 500000 && now - CF.h[1].rel_us < 500000) { /* crossed: the left hand is to the right of the right hand (each hand is fitted on its own frames: the other's position is at most a few frames old) */
		bool crossed = CF.h[0].rel[0] > CF.h[1].rel[0] + 0.05, open = CF.h[0].rel[0] < CF.h[1].rel[0] - 0.10;
		if (crossed && !CF.crossed) { CF.crossed = true; CF.crossings++; } else if (open) CF.crossed = false;
	}
	pthread_mutex_unlock(&CF.lock);
}

static int cf_bits(unsigned v) { int n = 0; while (v) { n += v & 1; v >>= 1; } return n; }
static double cf_jitter_mm(const struct cf_hand *H) { return H->nstat > 10 ? sqrt((H->s2[0] + H->s2[1] + H->s2[2]) / H->nstat) * 1e3 : 0; }
static void cf_checklist(char *out, size_t n, const char *const *names, int count, unsigned bits) /* "up ok  down --  ..." */
{
	size_t k = 0; out[0] = 0;
	for (int i = 0; i < count && k < n - 1; i++) k += (size_t)snprintf(out + k, n - k, "%s%s%s", i ? "  " : "", names[i], bits & (1u << i) ? " ok" : " --");
}

/* criteria + live lines; returns true when the step may end (after min_s) */
static bool cf_evaluate(uint64_t now, bool final)
{
	int step = CF.step; const struct cf_step *st = &CF_STEPS[step];
	int imq[2]; double sp[2], rms[2]; bool live[2]; unsigned lk[2], aq[2], fl[2], jm[2];
	pthread_mutex_lock(&O.lock);
	for (int h = 0; h < 2; h++) { imq[h] = O.imf[h].quality; sp[h] = O.imf[h].spread; rms[h] = O.imf[h].rms; live[h] = O.mux_live[h]; jm[h] = O.rot_jumps[h] >= CF.h[h].jumps0 ? O.rot_jumps[h] - CF.h[h].jumps0 : O.rot_jumps[h]; }
	pthread_mutex_unlock(&O.lock);
	pthread_mutex_lock(&CF.lock);
	for (int h = 0; h < 2; h++) { lk[h] = CF.h[h].fits; aq[h] = CF.h[h].acq; fl[h] = CF.h[h].miss; }
	double el = (now - CF.t_step) * 1e-6; bool met = false;
	double lockp[2], nmean[2], jit[2]; char cl[2][160];
	for (int h = 0; h < 2; h++) { lockp[h] = CF.h[h].frames ? 100.0 * CF.h[h].fits / CF.h[h].frames : 0; nmean[h] = CF.h[h].fits ? CF.h[h].n_sum / CF.h[h].fits : 0; jit[h] = cf_jitter_mm(&CF.h[h]); }
	CF.nlive = 0;
#define LIVE(...) do { if (CF.nlive < (int)ARRAY_LEN(CF.live)) snprintf(CF.live[CF.nlive++], sizeof CF.live[0], __VA_ARGS__); } while (0)
	LIVE("tracked  L %3.0f%%  R %3.0f%%   LEDs/fit  L %.1f  R %.1f   acquisitions L %u R %u", lockp[0], lockp[1], nmean[0], nmean[1], aq[0], aq[1]);
	switch (step) {
	case ST_STILL: {
		bool calm = now - CF.motion_us > 1500000;
		LIVE("%s   jitter  L %.1f mm  R %.1f mm   gyro bias L %.1f %.1f %.1f  R %.1f %.1f %.1f LSB", calm ? "calm" : "SETTLING (hold still)", jit[0], jit[1], S[0].gbo[0], S[0].gbo[1], S[0].gbo[2], S[1].gbo[0], S[1].gbo[1], S[1].gbo[2]);
		met = calm && lockp[0] > 85 && lockp[1] > 85 && jit[0] < 8 && jit[1] < 8; /* (the gyro bias learner needs the controllers put down; a hand is never that still) */
		break; }
	case ST_TURN_R: case ST_TURN_L: {
		int h = st->hand;
		cf_checklist(cl[h], sizeof cl[h], turn_name, TURN_N, CF.h[h].turn_bits);
		LIVE("ring faced: %s", cl[h]);
		LIVE("%c model<->IMU: %s, orientation spread %.0f deg, rms %.1f deg   LEDs min %d, few-LED fits %u, fails %u", h ? 'R' : 'L', imq[h] == 2 ? "calibrated" : "tilt only", sp[h], rms[h], CF.h[h].n_min < 99 ? CF.h[h].n_min : 0, CF.h[h].few, fl[h]);
		met = cf_bits(CF.h[h].turn_bits) >= 5 && sp[h] >= 100;
		break; }
	case ST_REACH:
		cf_checklist(cl[0], sizeof cl[0], reach_name, RCH_N, CF.h[0].reach_bits); cf_checklist(cl[1], sizeof cl[1], reach_name, RCH_N, CF.h[1].reach_bits);
		LIVE("L: %s", cl[0]); LIVE("R: %s", cl[1]);
		LIVE("coverage cam0 %2d/16 cam1 %2d/16 cam2 %2d/16 cam3 %2d/16   distance L %.2f-%.2f m  R %.2f-%.2f m   fails L %u R %u", cf_bits(CF.cover[0]), cf_bits(CF.cover[1]), cf_bits(CF.cover[2]), cf_bits(CF.cover[3]),
		     CF.h[0].dmin < 1e8 ? CF.h[0].dmin : 0, CF.h[0].dmax, CF.h[1].dmin < 1e8 ? CF.h[1].dmin : 0, CF.h[1].dmax, fl[0], fl[1]);
		met = cf_bits(CF.h[0].reach_bits) == RCH_N && cf_bits(CF.h[1].reach_bits) == RCH_N;
		break;
	case ST_FAST:
		LIVE("peak speed L %.1f R %.1f m/s (need 2.5)   fast frames L %u R %u (need 40)   fails L %u R %u   turn rejections L %u R %u", CF.h[0].vmax, CF.h[1].vmax, CF.h[0].fast_fits, CF.h[1].fast_fits, fl[0], fl[1], jm[0], jm[1]);
		met = CF.h[0].vmax > 2.5 && CF.h[1].vmax > 2.5 && CF.h[0].fast_fits >= 40 && CF.h[1].fast_fits >= 40;
		break;
	case ST_HIDE: {
		if (CF.hide_hand == 1 && CF.h[1].gaps >= 3) { CF.hide_hand = 0; cf_pulse(0, 0.12f, 0.6f); }
		LIVE("hidden and shown: R %u/3  L %u/3   re-acquire after return: mean L %.2f R %.2f s, max L %.2f R %.2f s", CF.h[1].gaps > 3 ? 3 : CF.h[1].gaps, CF.h[0].gaps > 3 ? 3 : CF.h[0].gaps,
		     CF.h[0].gaps ? CF.h[0].reacq_sum / CF.h[0].gaps : 0, CF.h[1].gaps ? CF.h[1].reacq_sum / CF.h[1].gaps : 0, CF.h[0].reacq_max, CF.h[1].reacq_max);
		met = CF.h[0].gaps >= 3 && CF.h[1].gaps >= 3;
		break; }
	case ST_CROSS:
		LIVE("crossings %u/4   hand swaps seen %u   mux L %s R %s   fails L %u R %u", CF.crossings, CF.swaps, live[0] ? "live" : "DEAD", live[1] ? "live" : "DEAD", fl[0], fl[1]);
		met = CF.crossings >= 4;
		break;
	case ST_LOOK:
		cf_checklist(cl[0], sizeof cl[0], look_name, LOOK_N, CF.look_bits);
		LIVE("head turned: %s   max rate %.0f deg/s   hand wobble while turning  L %.1f mm  R %.1f mm", cl[0], CF.head_rate_max,
		     CF.h[0].hmove_frames ? sqrt(CF.h[0].hmove_s2 / CF.h[0].hmove_frames) * 1e3 : 0, CF.h[1].hmove_frames ? sqrt(CF.h[1].hmove_s2 / CF.h[1].hmove_frames) * 1e3 : 0);
		met = cf_bits(CF.look_bits) == LOOK_N;
		break;
	}
	LIVE("tracked frames L %u R %u   fails L %u R %u   turn rejections L %u R %u   %s", lk[0], lk[1], fl[0], fl[1], jm[0], jm[1], met ? "criteria met" : el < st->min_s ? "collecting" : "waiting for the checklist (trigger skips)");
#undef LIVE
	bool end = (met && el >= st->min_s) || el >= st->max_s;
	if (final) {
		cf_log("calib: step %d/%d %s end after %.0f s: tracked L %.0f%% R %.0f%%, LEDs/fit L %.1f R %.1f, acq L %u R %u, fails L %u R %u, turn-rej L %u R %u, criteria %s",
		       step + 1, ST_N, st->id, el, lockp[0], lockp[1], nmean[0], nmean[1], aq[0], aq[1], fl[0], fl[1], jm[0], jm[1], met ? "met" : "NOT met");
		for (int i = 1; i < CF.nlive - 1; i++) cf_log("calib:   %s", CF.live[i]);
	}
	pthread_mutex_unlock(&CF.lock);
	return end;
}

static void cf_begin_step(uint64_t now)
{
	CF.t_step = now; CF.settle_t0 = now; CF.hide_hand = 1; cf_reset_acc(now);
	const struct cf_step *st = &CF_STEPS[CF.step];
	cf_log("calib: step %d/%d %s begin: %s - %s %s", CF.step + 1, ST_N, st->id, st->title, st->line[0], st->line[1]);
	cf_pulse(st->hand, 0.12f, 0.6f);
}
static void cf_finish(uint64_t now, bool aborted)
{
	CF.state = CF_DONE; CF.t_step = now;
	int imq[2]; double sp[2], rms[2];
	pthread_mutex_lock(&O.lock); for (int h = 0; h < 2; h++) { imq[h] = O.imf[h].quality; sp[h] = O.imf[h].spread; rms[h] = O.imf[h].rms; } pthread_mutex_unlock(&O.lock);
	cf_log("calib: %s after %.0f s. model<->IMU L %s %.0f deg rms %.1f, R %s %.0f deg rms %.1f; gyro bias L %.1f %.1f %.1f R %.1f %.1f %.1f; LED latency L %d R %d us",
	       aborted ? "ABORTED" : "DONE", (now - CF.t_start) * 1e-6, imq[0] == 2 ? "calibrated" : "tilt-only", sp[0], rms[0], imq[1] == 2 ? "calibrated" : "tilt-only", sp[1], rms[1],
	       S[0].gbo[0], S[0].gbo[1], S[0].gbo[2], S[1].gbo[0], S[1].gbo[1], S[1].gbo[2], O.led[0].lat, O.led[1].lat);
	cf_log("calib: recording %s", REC.path[0] ? REC.path : "(off)");
	snprintf(CF.path, sizeof CF.path, "%s/.cache/slopvr_rec/calib-%llu.txt", getenv("HOME") ? getenv("HOME") : "/tmp", (unsigned long long)(time(NULL)));
	FILE *f = fopen(CF.path, "w");
	if (f) { for (int i = 0; i < CF.nrep; i++) fprintf(f, "%s\n", CF.rep[i]); fclose(f); cf_log("calib: report %s", CF.path); }
	cf_pulse(-1, aborted ? 0.08f : 0.5f, 0.8f);
}
static void calflow_start(uint64_t now)
{
	if (CF.state == CF_RUN || CF.state == CF_WAIT) return;
	CF.state = CF_WAIT; CF.step = 0; CF.t_start = now; CF.t_step = now; CF.nrep = 0; CF.follow = false;
	cf_log("calib: ===== guided tracking calibration started (%d steps; trigger = next step, hold Options 2 s = abort) =====", ST_N);
}
static bool cf_hand_ready(int h, uint64_t now) { return (U.replay || (S[h].last_in_us && now - S[h].last_in_us < 1000000)) && O.lock_us[h] && now - O.lock_us[h] < 1000000; }
static void calflow_next(uint64_t now, const char *why)
{
	if (CF.state != CF_RUN) return;
	cf_evaluate(now, true);
	if (why) cf_log("calib:   (%s)", why);
	if (++CF.step >= ST_N) cf_finish(now, false);
	else cf_begin_step(now);
}

/* replay: the recorded "calib: step N/M id begin" markers drive the steps (by id, so older recordings with other step lists still map),
 * and the per-step metrics are recomputed over exactly the recorded segments with the current tracker */
static void calflow_follow(const char *text, uint64_t now)
{
	if (!U.replay || strncmp(text, "calib: ", 7)) return;
	int n = 0, m = 0; char id[24];
	if (sscanf(text, "calib: step %d/%d %23s", &n, &m, id) == 3 && strstr(text, " begin")) {
		static const struct { const char *old, *now; } alias[] = { { "roll_R", "turn_R" }, { "roll_L", "turn_L" }, { "sweep", "reach" }, { "head", "look" }, { "edge", "turn_L" }, { "walk", "look" } };
		for (size_t k = 0; k < ARRAY_LEN(alias); k++) if (!strcmp(id, alias[k].old)) snprintf(id, sizeof id, "%s", alias[k].now);
		int st = -1; for (int k = 0; k < ST_N; k++) if (!strcmp(id, CF_STEPS[k].id)) st = k;
		if (st < 0) return;
		if (CF.state == CF_RUN) cf_evaluate(now, true);
		if (CF.state != CF_RUN) { CF.state = CF_RUN; CF.t_start = now; CF.nrep = 0; cf_log("calib: ===== replay follows the recorded calibration steps ====="); }
		CF.step = st; CF.t_step = now; CF.hide_hand = 1; cf_reset_acc(now); CF.follow = true;
		cf_log("calib: step %d/%d %s begin (replay of the recorded step %d)", st + 1, ST_N, CF_STEPS[st].id, n);
	} else if ((strstr(text, "calib: DONE") || strstr(text, "calib: ABORTED")) && CF.state == CF_RUN) { cf_evaluate(now, true); cf_finish(now, false); }
}

/* main loop: controller buttons (Options = start / abort, trigger = next) and step timing */
static void calflow_tick(uint64_t now)
{
	for (int h = 0; h < 2; h++) {
		bool opt = (S[h].btn[1] & (h ? 2 : 1)) != 0, trig = S[h].last_trigger >= 240;
		if (opt) { if (!CF.t_opt[h]) CF.t_opt[h] = now; } else { CF.t_opt[h] = 0; CF.opt_edge[h] = false; }
		if (trig) { if (!CF.t_trig[h]) CF.t_trig[h] = now; } else { CF.t_trig[h] = 0; CF.trig_edge[h] = false; }
		if (CF.t_opt[h] && !CF.opt_edge[h]) {
			if ((CF.state == CF_RUN || CF.state == CF_WAIT) && now - CF.t_opt[h] > 2000000) { CF.opt_edge[h] = true; cf_finish(now, true); }
			else if (CF.state != CF_RUN && CF.state != CF_WAIT && now - CF.t_opt[h] > 1000000) { CF.opt_edge[h] = true; calflow_start(now); }
		}
		if (CF.state == CF_RUN && CF.t_trig[h] && !CF.trig_edge[h] && now - CF.t_trig[h] > 400000 && now - CF.t_step > 1500000) { CF.trig_edge[h] = true; calflow_next(now, "skipped by trigger"); }
	}
	if (CF.state == CF_WAIT && cf_hand_ready(0, now) && cf_hand_ready(1, now)) { CF.state = CF_RUN; cf_begin_step(now); }
	if (CF.state == CF_RUN) {
		static uint64_t last_eval;
		if (CF.step == ST_STILL && !CF.follow && now - CF.motion_us < 1500000 && now - CF.settle_t0 < 25000000 && now - CF.t_step > 1000000) { CF.t_step = now; cf_reset_acc(now); } /* the user is still settling: measure from the calm */
		if (now - last_eval > 250000) { last_eval = now; if (cf_evaluate(now, false) && !CF.follow) calflow_next(now, NULL); }
	} else if (CF.state == CF_DONE && now - CF.t_step > 15000000) CF.state = CF_IDLE;
}

/* ---- HUD overlay bitmap (OV_W x OV_H, see hmdout.c: eye angular space, composited into both eyes) ---- */
static uint32_t *g_ov;
static void ov_fill(int x, int y, int w, int h, uint32_t c)
{
	if (x < 0) { w += x; x = 0; } if (y < 0) { h += y; y = 0; }
	if (x + w > OV_W) w = OV_W - x; if (y + h > OV_H) h = OV_H - y;
	for (int j = 0; j < h; j++) for (int i = 0; i < w; i++) g_ov[(y + j) * OV_W + x + i] = c;
}
static void ov_frame(int x, int y, int w, int h, int t, uint32_t c) { ov_fill(x, y, w, t, c); ov_fill(x, y + h - t, w, t, c); ov_fill(x, y, t, h, c); ov_fill(x + w - t, y, t, h, c); }
static int ov_text(int x, int y, int sc, uint32_t c, const char *s)
{
	for (; *s; s++, x += FONT_W * sc) {
		unsigned ch = (unsigned char)*s; if (ch < 32 || ch > 126) ch = '?';
		const unsigned char *g = font8x14[ch - 32];
		for (int r = 0; r < FONT_H; r++) for (int b = 0; b < FONT_W; b++) if (g[r] & (0x80 >> b)) ov_fill(x + b * sc, y + r * sc, sc, sc, c);
	}
	return x;
}
static void ov_disc(int cx, int cy, int r, uint32_t c) { for (int y = -r; y <= r; y++) for (int x = -r; x <= r; x++) if (x * x + y * y <= r * r) ov_fill(cx + x, cy + y, 1, 1, c); }
static void ov_line(int x0, int y0, int x1, int y1, int w, uint32_t c)
{
	int dx = abs(x1 - x0), dy = -abs(y1 - y0), sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1, e = dx + dy;
	for (int n = 0; n < 4096; n++) {
		ov_fill(x0 - w / 2, y0 - w / 2, w, w, c);
		if (x0 == x1 && y0 == y1) break;
		int e2 = 2 * e; if (e2 >= dy) { e += dy; x0 += sx; } if (e2 <= dx) { e += dx; y0 += sy; }
	}
}
static void ov_arrow(int x0, int y0, int x1, int y1, uint32_t c)
{
	ov_line(x0, y0, x1, y1, 4, c);
	double a = atan2(y1 - y0, x1 - x0);
	ov_line(x1, y1, x1 - (int)(14 * cos(a - 0.5)), y1 - (int)(14 * sin(a - 0.5)), 4, c);
	ov_line(x1, y1, x1 - (int)(14 * cos(a + 0.5)), y1 - (int)(14 * sin(a + 0.5)), 4, c);
}
/* a controller seen from the side: grip (rectangle) + tracking ring (ellipse); drawn rotated by `rot` about its centre, the ring's apparent width scaled by `squash` (tumbling) */
static void ov_controller(int cx, int cy, double rot, double squash, uint32_t c)
{
	double cs = cos(rot), sn = sin(rot);
	for (int y = -52; y <= 52; y++) for (int x = -40; x <= 40; x++) {
		bool grip = x >= -9 && x <= 9 && y >= -10 && y <= 50;
		double ex = x / (36.0 * (fabs(squash) < 0.15 ? 0.15 : fabs(squash))), ey = (y + 20) / 24.0, rr = ex * ex + ey * ey;
		bool ring = rr <= 1.0 && rr >= 0.55;
		if (!grip && !ring) continue;
		int px = cx + (int)lrint(x * cs - y * sn), py = cy + (int)lrint(x * sn + y * cs);
		ov_fill(px, py, 2, 2, c);
	}
}
static void ov_head(int cx, int cy, double yaw, uint32_t c)
{
	ov_disc(cx, cy, 34, c); ov_disc(cx, cy, 30, 0xf8101418);
	int nx = cx + (int)(28 * sin(yaw)); ov_disc(nx, cy + 2, 9, c);                /* nose: where the head looks */
	ov_disc(cx - (int)(30 * cos(yaw)), cy, 6, c); ov_disc(cx + (int)(30 * cos(yaw)), cy, 6, c); /* ears */
}
static void ov_check(int x, int y, const char *label, bool done, uint32_t c) /* small checklist item: box + label (2x font) */
{
	ov_frame(x, y + 4, 20, 20, 3, done ? 0xff5fff5f : c);
	if (done) { ov_line(x + 4, y + 14, x + 9, y + 19, 3, 0xff5fff5f); ov_line(x + 9, y + 19, x + 17, y + 7, 3, 0xff5fff5f); }
	ov_text(x + 28, y, 2, done ? 0xff5fff5f : c, label);
}
/* the pictogram area: the motion, animated, plus the checklist for this step */
static void cf_pictogram(int x0, int y0, int w, int h, int step, double t, uint64_t now)
{
	uint32_t CL = 0xff3fc1ff, CR = 0xffff5fc8, W = 0xffe8e8e8, DIM = 0xff506070;
	int cx = x0 + w / 2, cy = y0 + h / 2;
	switch (step) {
	case ST_STILL: {
		ov_controller(cx - 70, cy, 0, 1, CL); ov_controller(cx + 70, cy, 0, 1, CR);
		ov_line(cx - 120, y0 + h - 8, cx + 120, y0 + h - 8, 3, DIM);
		bool calm = now - CF.motion_us > 1500000;
		ov_text(x0 + w - 230, y0 + 10, 2, calm ? 0xff5fff5f : 0xffff9040, calm ? "calm, measuring" : "settling...");
		break; }
	case ST_TURN_R: case ST_TURN_L: {
		int hnd = step == ST_TURN_R; uint32_t c = hnd ? CR : CL; int ax = x0 + 110;
		/* 4 s per demonstration: roll about the grip, nod forward/back, swing left/right, point at yourself / away */
		double ph = fmod(t, 16.0); int k = (int)(ph / 4); double u = ph - 4 * k;
		static const char *demo[4] = { "roll it like a key", "tip it forward / back", "turn it left / right", "ring to you / away" };
		double rot = 0, sq = 1;
		if (k == 0) rot = u * 1.6; else if (k == 1) sq = cos(u * 1.6); else if (k == 2) rot = 0.5 * sin(u * 1.6); else sq = 0.3 + 0.7 * fabs(cos(u * 1.6));
		ov_controller(ax, cy + 10, rot, sq, c);
		if (k == 2) ov_arrow(ax - 50, y0 + 20, ax + 50, y0 + 20, W); else if (k == 1) ov_arrow(ax + 55, cy - 30, ax + 55, cy + 40, W);
		else { for (int i = 0; i < 20; i++) { double a0 = -2.4 + i * 0.17, a1 = a0 + 0.12; ov_line(ax + (int)(70 * cos(a0)), cy + 10 + (int)(70 * sin(a0)), ax + (int)(70 * cos(a1)), cy + 10 + (int)(70 * sin(a1)), 3, W); } ov_arrow(ax + (int)(70 * cos(1.4)), cy + 10 + (int)(70 * sin(1.4)), ax + (int)(70 * cos(1.8)), cy + 10 + (int)(70 * sin(1.8)), W); }
		ov_text(x0 + 200, y0 + 4, 2, W, demo[k]);
		/* checklist: where the ring has pointed so far */
		for (int i = 0; i < TURN_N; i++) ov_check(x0 + 200 + (i % 3) * 150, y0 + 44 + (i / 3) * 34, turn_name[i], CF.h[hnd].turn_bits & (1u << i), W);
		char sp[40]; snprintf(sp, sizeof sp, "coverage %3.0f%%", CLAMP(O.imf[hnd].spread, 0.0, 100.0)); ov_text(x0 + 200, y0 + 120, 2, DIM, sp);
		break; }
	case ST_REACH: { /* targets around the head icon; the ticked ones turn green, the dot shows the next one to reach */
		int hx = cx - 150, hy = cy; ov_head(hx, hy, 0, DIM);
		static const int tx[RCH_N] = { -120, 120, 0, 0, 0, 0 }, ty[RCH_N] = { 0, 0, -75, 75, 0, 0 };
		for (int i = 0; i < RCH_N; i++) {
			bool dL = CF.h[0].reach_bits & (1u << i), dR = CF.h[1].reach_bits & (1u << i);
			int px = hx + tx[i], py = hy + ty[i]; int r = i == RCH_NEAR ? 10 : i == RCH_FAR ? 4 : 7;
			if (i == RCH_NEAR) { px = hx + 40; py = hy + 30; } if (i == RCH_FAR) { px = hx + 70; py = hy + 50; }
			ov_disc(px - 8, py, r, dL ? 0xff5fff5f : CL); ov_disc(px + 8, py, r, dR ? 0xff5fff5f : CR);
		}
		for (int i = 0; i < RCH_N; i++) ov_check(x0 + 330 + (i % 3) * 130, y0 + 20 + (i / 3) * 40, reach_name[i], (CF.h[0].reach_bits & CF.h[1].reach_bits) & (1u << i), W); /* ticked when both hands got there; the dots by the head show which hand is missing */
		ov_text(x0 + 330, y0 + 110, 2, DIM, "blue = left, pink = right,"); ov_text(x0 + 330, y0 + 136, 2, DIM, "green = done. near = at the face");
		break; }
	case ST_FAST: {
		double u = fmod(t * 2.5, 2.0); double px = u < 1 ? u : 2 - u; /* out and back */
		ov_head(x0 + 60, cy, 0, DIM);
		int fx = x0 + 110 + (int)(px * 180); ov_disc(fx, cy - 14, 12, CL); ov_disc(fx + 6, cy + 14, 12, CR);
		for (int k = 1; k <= 4; k++) ov_line(fx - 10 * k - 14, cy - 4, fx - 10 * k - 2, cy - 4, 2, DIM);
		ov_arrow(x0 + 120, y0 + 20, x0 + 300, y0 + 20, W);
		char s1[48]; snprintf(s1, sizeof s1, "speed  L %.1f  R %.1f m/s", CF.h[0].vmax, CF.h[1].vmax); ov_text(x0 + 330, y0 + 30, 2, W, s1);
		ov_check(x0 + 330, y0 + 64, "fast enough L", CF.h[0].vmax > 2.5 && CF.h[0].fast_fits >= 40, W);
		ov_check(x0 + 330, y0 + 98, "fast enough R", CF.h[1].vmax > 2.5 && CF.h[1].fast_fits >= 40, W);
		break; }
	case ST_HIDE: { /* body from above; the current hand goes behind the back and returns */
		int hh = CF.hide_hand; uint32_t c = hh ? CR : CL;
		ov_head(cx - 150, y0 + 40, 0, DIM); ov_line(cx - 210, y0 + 80, cx - 90, y0 + 80, 4, DIM);
		double ph = fmod(t, 3.0); double f = ph < 1.5 ? ph / 1.5 : (3 - ph) / 1.5; f = f * f * (3 - 2 * f);
		int sx = cx - 150 + (hh ? 60 : -60), ex = cx - 150 + (hh ? 30 : -30);
		int px = sx + (int)((ex - sx) * f), py = y0 + h - 30 - (int)((h - 60) * f);
		ov_disc(px, py, f > 0.8 ? 6 : 12, f > 0.8 ? DIM : c);
		ov_disc(cx - 150 + (hh ? -60 : 60), y0 + h - 30, 12, hh ? CL : CR);
		ov_arrow(sx, y0 + h - 60, ex, y0 + 60, W);
		char s1[48]; snprintf(s1, sizeof s1, "now: %s hand", hh ? "RIGHT" : "LEFT"); ov_text(x0 + 330, y0 + 10, 2, c, s1);
		snprintf(s1, sizeof s1, "right %u/3", CF.h[1].gaps > 3 ? 3 : CF.h[1].gaps); ov_check(x0 + 330, y0 + 50, s1, CF.h[1].gaps >= 3, W);
		snprintf(s1, sizeof s1, "left  %u/3", CF.h[0].gaps > 3 ? 3 : CF.h[0].gaps); ov_check(x0 + 330, y0 + 84, s1, CF.h[0].gaps >= 3, W);
		ov_text(x0 + 330, y0 + 124, 2, DIM, "hide it fully (>0.5 s)");
		break; }
	case ST_CROSS: {
		double ph = fmod(t, 3.0); double f = ph < 1.5 ? ph / 1.5 : (3 - ph) / 1.5; f = f * f * (3 - 2 * f);
		int hx = cx - 150; ov_head(hx, y0 + 40, 0, DIM); ov_line(hx - 60, y0 + 80, hx + 60, y0 + 80, 4, DIM);
		ov_disc(hx - 60 + (int)(120 * f), y0 + h - 40, 12, CL); ov_disc(hx + 60 - (int)(120 * f), y0 + h - 40 - 16, 12, CR);
		ov_arrow(hx - 40, y0 + h - 75, hx + 40, y0 + h - 75, W); ov_arrow(hx + 40, y0 + h - 95, hx - 40, y0 + h - 95, W);
		char s1[48]; snprintf(s1, sizeof s1, "crossings %u/4", CF.crossings); ov_check(x0 + 330, y0 + 40, s1, CF.crossings >= 4, W);
		ov_text(x0 + 330, y0 + 90, 2, DIM, "arms fully crossed,"); ov_text(x0 + 330, y0 + 116, 2, DIM, "then wide open");
		break; }
	case ST_LOOK: {
		double yaw = 0.9 * sin(t * 1.3);
		ov_head(x0 + 100, cy - 10, yaw, W);
		ov_disc(x0 + 50, cy + 60, 12, CL); ov_disc(x0 + 150, cy + 60, 12, CR);
		ov_arrow(x0 + 55, y0 + 12, x0 + 15, y0 + 22, W); ov_arrow(x0 + 145, y0 + 12, x0 + 185, y0 + 22, W);
		for (int i = 0; i < LOOK_N; i++) ov_check(x0 + 260 + (i % 2) * 160, y0 + 30 + (i / 2) * 40, look_name[i], CF.look_bits & (1u << i), W);
		ov_text(x0 + 260, y0 + 120, 2, DIM, "keep the hands where they are");
		break; }
	}
}
/* the headset HUD: title, two lines, pictogram + checklist, footer, progress.  3x font = 1.5 x 2.6 deg per character at the panel */
static const uint32_t *calflow_overlay(uint64_t now)
{
	if (CF.state == CF_IDLE) return NULL;
	if (!g_ov) g_ov = mem_alloc((size_t)OV_W * OV_H * 4);
	memset(g_ov, 0, (size_t)OV_W * OV_H * 4);
	int w = 760, hgt = 434, x0 = (OV_W - w) / 2, y0 = 8;
	ov_fill(x0, y0, w, hgt, 0xf8101418); ov_frame(x0, y0, w, hgt, 3, 0xffffffff);
	const char *title, *l[2]; uint32_t tc = 0xffffe13f; char t[64]; double frac = -1; const char *foot = NULL; int pict = -1;
	if (CF.state == CF_RUN) {
		const struct cf_step *st = &CF_STEPS[CF.step]; double el = (now - CF.t_step) * 1e-6; frac = el / st->max_s; if (frac > 1) frac = 1;
		snprintf(t, sizeof t, "%d/%d %s", CF.step + 1, ST_N, st->title); title = t; l[0] = st->line[0]; l[1] = st->line[1]; pict = CF.step;
		const char *status = CF.nlive ? CF.live[CF.nlive - 1] : "";
		foot = strstr(status, "criteria met") ? "good - next step in a moment" : strstr(status, "waiting") ? "finish the checklist (trigger = skip)" : (CF.step == ST_STILL && now - CF.motion_us < 1500000) ? "settling - hold still..." : "collecting...";
	} else if (CF.state == CF_WAIT) {
		title = "WAITING"; l[0] = "Switch both controllers"; l[1] = "on (PS), hold them in view."; tc = 0xffff9040;
		static char st[40]; snprintf(st, sizeof st, "L %s  R %s", cf_hand_ready(0, now) ? "ok" : "--", cf_hand_ready(1, now) ? "ok" : "--"); foot = st;
	} else {
		title = "DONE - thank you"; l[0] = "Data recorded."; l[1] = "Take the headset off."; tc = 0xff5fff5f;
	}
	ov_text(x0 + 18, y0 + 12, 3, tc, title); ov_text(x0 + 19, y0 + 12, 3, tc, title); /* bold */
	for (int i = 0; i < 2; i++) ov_text(x0 + 18, y0 + 64 + i * 46, 3, 0xffffffff, l[i]);
	if (pict >= 0) cf_pictogram(x0 + 18, y0 + 160, w - 36, 196, pict, (now - CF.t_step) * 1e-6, now);
	if (foot) ov_text(x0 + 18, y0 + hgt - 66, 2, 0xffb0e0b0, foot);
	if (frac >= 0) { ov_fill(x0 + 18, y0 + hgt - 26, w - 36, 12, 0xff303840); ov_fill(x0 + 18, y0 + hgt - 26, (int)((w - 36) * frac), 12, 0xff5fff5f); }
	return g_ov;
}
static void calflow_draw(uint64_t now)
{
	if (CF.state == CF_IDLE) return;
	/* desktop panel: live metrics (the headset gets the HUD overlay from calflow_overlay) */
	int x = PANEL_X, y = CH - 14 * ROW - 6;
	clip(PANEL_X - 3, y - 4, CW, CH);
	fill_rect(PANEL_X - 3, y - 4, PNL_W + 6, 9 * ROW + 8, 0xff1e2a1e); rect(PANEL_X - 3, y - 4, PNL_W + 6, 9 * ROW + 8, 0xff5fff5f);
	if (CF.state == CF_RUN) {
		const struct cf_step *st = &CF_STEPS[CF.step];
		txt(x, y, 0xffffe13f, "CALIBRATION step %d/%d %s: %s %s  (%.0f/%.0f s)", CF.step + 1, ST_N, st->id, st->line[0], st->line[1], (now - CF.t_step) * 1e-6, st->max_s); y += ROW;
		pthread_mutex_lock(&CF.lock);
		for (int i = 0; i < CF.nlive; i++) { txt(x, y, C_TEXT, "%s", CF.live[i]); y += ROW; }
		pthread_mutex_unlock(&CF.lock);
	} else if (CF.state == CF_WAIT) {
		txt(x, y, 0xffff9040, "CALIBRATION waiting for both controllers (connected + tracked): L %s  R %s   (hold Options 2 s or F5 to abort)", cf_hand_ready(0, now) ? "ok" : "--", cf_hand_ready(1, now) ? "ok" : "--");
	} else {
		txt(x, y, 0xff5fff5f, "CALIBRATION done - report %s", CF.path); y += ROW;
		for (int i = CF.nrep > 7 ? CF.nrep - 7 : 0; i < CF.nrep; i++) { txt(x, y, C_DIM, "%.118s", CF.rep[i]); y += ROW; }
	}
	clip(0, 0, CW, CH);
}
