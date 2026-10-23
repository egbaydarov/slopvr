/* slopvr — runtime parameters: one table with defaults and the steamvr.vrsettings keys that override them
 * (driver_slopvr section).  Everything else in the driver reads T.* */
static struct tune {
	double pose_time_ms, vel_gain, pos_smooth, glide_ms, hold_s;
	double acq_min, fit_min, assoc_px, prior_m, inlier_mm, gap_mm, sep_cm, dr_tau_s, blend_ms;
	double led_gain, led_exp_us, led_period, led_period_stable, led_lead_ms, lat_trim_L, lat_trim_R;
	double cam_x, cam_y, cam_z, cam_yaw, cam_pitch, cam_roll, eye_depth;
	double grip_pitch, grip_yaw, grip_roll, grip_x, grip_y, grip_z;
	double qc_on, qc_rate, imu_lead_ms, mirror_L, mirror_R, ld_xoff, ld_yoff;
	double led_mux, acq_imu, acq_yaw_step, evid, miss_cos, blob_max_px, grav_deg, track_rot_deg, rot_prior;
} T;

struct param { const char *name; double *v; double def, min, max; const char *cfg; };
static const struct param params[] = {
	{ "pose_time_ms", &T.pose_time_ms, 0, -60, 60, "pose_time_ms" },
	{ "vel_gain", &T.vel_gain, 1.0, 0, 2, "vel_gain" },
	{ "pos_smooth", &T.pos_smooth, 0, 0, 0.95, "pos_smooth" },
	{ "glide_ms", &T.glide_ms, 150, 30, 600, "glide_ms" },
	{ "hold_s", &T.hold_s, 30, 0, 120, "hold_s" },         /* without a fix: dead-reckon, then hold the position this long before the arm model takes over */
	{ "dr_tau_s", &T.dr_tau_s, 0.3, 0.05, 2, "dr_tau_s" },  /* velocity damping time constant of the IMU dead-reckoning after an optical loss */
	{ "blend_ms", &T.blend_ms, 250, 0, 1000, "blend_ms" },  /* position-source changes and re-acquisition jumps are blended over this time */
	{ "acq_min", &T.acq_min, 5, 3, 10, "acq_min" },     /* LEDs for an IMU-free acquisition; with the orientation from the IMU fit_min+1 suffices */
	{ "fit_min", &T.fit_min, 4, 3, 8, "fit_min" },
	{ "assoc_px", &T.assoc_px, 3, 1, 10, "assoc_px" },
	{ "prior_m", &T.prior_m, 0.25, 0.05, 0.6, "prior_m" },
	{ "inlier_mm", &T.inlier_mm, 10, 3, 25, "inlier_mm" },
	{ "gap_mm", &T.gap_mm, 6, 2, 15, "gap_mm" },
	{ "sep_cm", &T.sep_cm, 10, 0, 25, "sep_cm" },
	{ "led_gain", &T.led_gain, 32, 1, 255, "ledGain" },
	{ "led_exp_us", &T.led_exp_us, 150, 30, 4000, "ledExposureUs" },
	{ "led_period", &T.led_period, 42, 8, 60, "ledPeriod" },             /* flash window in 50 us units (2.1 ms, Sony's maximum) while syncing / not tracked */
	{ "led_period_stable", &T.led_period_stable, 42, 6, 60, "ledPeriodStable" }, /* flash window once tracked (42: no narrowing; the duty cycle is 6 % either way) */
	{ "led_lead_ms", &T.led_lead_ms, 80, 30, 300, "ledLeadMs" },         /* flash base time at least this far ahead of the controller clock (a base in the past is not flashed) */
	{ "lat_trim_L", &T.lat_trim_L, 0, -3000, 3000, "lat_trim_L" },
	{ "lat_trim_R", &T.lat_trim_R, 0, -3000, 3000, "lat_trim_R" },
	{ "cam_x", &T.cam_x, -0.038, -0.2, 0.2, "camPosX" },
	{ "cam_y", &T.cam_y, 0, -0.2, 0.2, "camPosY" },
	{ "cam_z", &T.cam_z, 0, -0.2, 0.2, "camPosZ" },
	{ "cam_yaw", &T.cam_yaw, 17.5, -45, 45, "camYawDeg" },     /* head (SLAM frame) <- cam0: solved from two calibration sessions (hand-eye over head turns + controller IMUs, 3-4 deg median residual); the front cameras look 27.5 deg down */
	{ "cam_pitch", &T.cam_pitch, -27.5, -45, 45, "camPitchDeg" },
	{ "cam_roll", &T.cam_roll, 2.4, -45, 45, "camRollDeg" },
	{ "eye_depth", &T.eye_depth, 0, -0.1, 0.2, "eye_depth" },
	{ "grip_pitch", &T.grip_pitch, 0, -90, 90, "gripPitchDeg" },
	{ "grip_yaw", &T.grip_yaw, 0, -90, 90, "grip_yaw" },
	{ "grip_roll", &T.grip_roll, 0, -90, 90, "grip_roll" },
	{ "grip_x", &T.grip_x, 0, -0.2, 0.2, "grip_x" },
	{ "grip_y", &T.grip_y, 0, -0.2, 0.2, "grip_y" },
	{ "grip_z", &T.grip_z, 0, -0.2, 0.2, "grip_z" },
	{ "qc_on", &T.qc_on, 1, 0, 1, "qc_on" },
	{ "qc_rate", &T.qc_rate, 0.04, 0.005, 0.5, "qc_rate" },
	{ "imu_lead_ms", &T.imu_lead_ms, 0, -30, 30, "imu_lead_ms" },
	{ "mirror_L", &T.mirror_L, 1, 0, 1, "ledMirrorLeft" },
	{ "mirror_R", &T.mirror_R, 0, 0, 1, "ledMirrorRight" },
	{ "ld_xoff", &T.ld_xoff, 0, -32, 32, "ld_xoff" },   /* full-res px: LD blob centroid -> calibration frame */
	{ "ld_yoff", &T.ld_yoff, 0, -32, 32, "ld_yoff" },
	{ "led_mux", &T.led_mux, 1, 0, 1, "ledMux" },           /* L flashes on even camera frames, R on odd: hand identity by frame parity, 30 Hz each */
	{ "acq_imu", &T.acq_imu, 1, 0, 1, "acq_imu" },          /* acquisition: IMU gravity + yaw sweep (1) before IMU-free triangle matching */
	{ "acq_yaw_step", &T.acq_yaw_step, 8, 2, 30, "acq_yaw_step" }, /* deg, full-circle yaw sweep when the yaw is unknown */
	{ "evid", &T.evid, 0.45, 0, 1, "evid_ratio" },          /* accept a fit only if explained >= evid * (explained + predicted-missing + unexplained-in-footprint) */
	{ "miss_cos", &T.miss_cos, 0.5, 0, 0.9, "miss_cos" },   /* an LED counts as predicted-visible (so missing if absent) when its normal faces the camera this well */
	{ "blob_max_px", &T.blob_max_px, 16, 4, 64, "blob_max_px" },
	{ "grav_deg", &T.grav_deg, 25, 0, 90, "grav_deg" },
	{ "track_rot_deg", &T.track_rot_deg, 15, 2, 90, "track_rot_deg" },
	{ "rot_prior", &T.rot_prior, 100, 0, 1000, "rot_prior" },     /* px per rad: how hard the refinement holds the IMU-given orientation (0 free; 100: a 25.7 deg ring-slot jump costs more than it can gain) */ /* a tracked fit may turn at most this far from the IMU-propagated orientation (a ring correspondence jump is 25.7 deg) */     /* once the LED-model<->IMU rotation is calibrated: reject fits whose model gravity is further than this from the IMU gravity (wrong controller / correspondence); 0 off */ /* binned px: wider/taller detections are windows, lamps, reflections, not LEDs */
};

static double cfg_num(const char *k, double def) /* the key may be stored as float, int or bool */
{
	int32_t e = 0; float f = ovr_settings_GetFloat(g_settings, SECTION, k, &e);
	if (!e) return f;
	e = 0; int32_t i = ovr_settings_GetInt32(g_settings, SECTION, k, &e);
	if (!e) return i;
	e = 0; bool b = ovr_settings_GetBool(g_settings, SECTION, k, &e);
	return e ? def : (double)b;
}
static void tune_init(void)
{
	for (unsigned i = 0; i < ARRAY_LEN(params); i++) {
		const struct param *p = &params[i];
		*p->v = CLAMP(cfg_num(p->cfg, p->def), p->min, p->max);
	}
}
