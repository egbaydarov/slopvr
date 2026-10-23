/* psvr2linuxtest — light the PSVR2 panel through a Wayland DRM lease (wp_drm_lease_v1) and mirror the camera
 * views into it.  The headset only runs its cameras, SLAM and LED detector while its DisplayPort link is up
 * (dprx = 1); normally vrcompositor's direct mode does that.  Here: lease the non-desktop connector from the
 * compositor, legacy KMS modeset on a dumb buffer, page flips.  Included by psvr2linuxtest.c. */
#include <sys/mman.h>
#include <wayland-client.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include "drm-lease-v1.h"
#include "drm-lease-v1.c" /* generated from wayland-protocols' drm-lease-v1.xml (wayland-scanner private-code), vendored so the build needs no scanner */

#define HO_MAX_CONN 8
struct ho_dev { struct wp_drm_lease_device_v1 *obj; int drm_fd; bool done; };
struct ho_conn { struct wp_drm_lease_connector_v1 *obj; struct ho_dev *dev; char name[32], desc[96]; uint32_t id; bool done, withdrawn, vr; };
static struct hmdout {
	bool active, revoked; char status[200];
	struct wl_display *dpy; struct wl_registry *reg;
	struct ho_dev dev[2]; int ndev;
	struct ho_conn conn[HO_MAX_CONN]; int nconn; struct ho_conn *pick;
	struct wp_drm_lease_v1 *lease; int lease_fd; bool lease_answered;
	uint32_t crtc, fb[2], handle[2], pitch, size; uint8_t *map[2]; int w, h, back; drmModeModeInfo mode;
	unsigned flips, busy;
} HO = { .lease_fd = -1 };

static void ho_status(const char *fmt, ...)
{
	va_list ap; va_start(ap, fmt); vsnprintf(HO.status, sizeof HO.status, fmt, ap); va_end(ap);
	char m[240]; snprintf(m, sizeof m, "panel: %s", HO.status); fk_log(NULL, m);
}

/* ---- wayland listeners ---- */
static void ho_conn_name(void *d, struct wp_drm_lease_connector_v1 *c, const char *s) { (void)c; snprintf(((struct ho_conn *)d)->name, sizeof ((struct ho_conn *)d)->name, "%s", s); }
static void ho_conn_desc(void *d, struct wp_drm_lease_connector_v1 *c, const char *s) { (void)c; snprintf(((struct ho_conn *)d)->desc, sizeof ((struct ho_conn *)d)->desc, "%s", s); }
static void ho_conn_id(void *d, struct wp_drm_lease_connector_v1 *c, uint32_t id) { (void)c; ((struct ho_conn *)d)->id = id; }
static void ho_conn_done(void *d, struct wp_drm_lease_connector_v1 *c) { (void)c; ((struct ho_conn *)d)->done = true; }
static void ho_conn_withdrawn(void *d, struct wp_drm_lease_connector_v1 *c) { (void)c; ((struct ho_conn *)d)->withdrawn = true; }
static const struct wp_drm_lease_connector_v1_listener ho_conn_listener = { ho_conn_name, ho_conn_desc, ho_conn_id, ho_conn_done, ho_conn_withdrawn };

static void ho_dev_fd(void *d, struct wp_drm_lease_device_v1 *dev, int fd) { (void)dev; ((struct ho_dev *)d)->drm_fd = fd; }
static void ho_dev_connector(void *d, struct wp_drm_lease_device_v1 *dev, struct wp_drm_lease_connector_v1 *c)
{
	(void)dev;
	if (HO.nconn >= HO_MAX_CONN) { wp_drm_lease_connector_v1_destroy(c); return; }
	struct ho_conn *k = &HO.conn[HO.nconn++];
	k->obj = c; k->dev = d;
	wp_drm_lease_connector_v1_add_listener(c, &ho_conn_listener, k);
}
static void ho_dev_done(void *d, struct wp_drm_lease_device_v1 *dev) { (void)dev; ((struct ho_dev *)d)->done = true; }
static void ho_dev_released(void *d, struct wp_drm_lease_device_v1 *dev) { (void)d; wp_drm_lease_device_v1_destroy(dev); }
static const struct wp_drm_lease_device_v1_listener ho_dev_listener = { ho_dev_fd, ho_dev_connector, ho_dev_done, ho_dev_released };

static void ho_lease_fd(void *d, struct wp_drm_lease_v1 *l, int fd) { (void)d; (void)l; HO.lease_fd = fd; HO.lease_answered = true; }
static void ho_lease_finished(void *d, struct wp_drm_lease_v1 *l)
{
	(void)d; (void)l;
	HO.lease_answered = true;
	if (HO.active) { HO.active = false; HO.revoked = true; ho_status("lease revoked by the compositor (another client took the headset?)"); }
}
static const struct wp_drm_lease_v1_listener ho_lease_listener = { ho_lease_fd, ho_lease_finished };

static void ho_global(void *d, struct wl_registry *r, uint32_t name, const char *iface, uint32_t ver)
{
	(void)d; (void)ver;
	if (strcmp(iface, wp_drm_lease_device_v1_interface.name) || HO.ndev >= 2) return;
	struct ho_dev *dv = &HO.dev[HO.ndev++];
	dv->drm_fd = -1;
	dv->obj = wl_registry_bind(r, name, &wp_drm_lease_device_v1_interface, 1);
	wp_drm_lease_device_v1_add_listener(dv->obj, &ho_dev_listener, dv);
}
static void ho_global_remove(void *d, struct wl_registry *r, uint32_t name) { (void)d; (void)r; (void)name; }
static const struct wl_registry_listener ho_reg_listener = { ho_global, ho_global_remove };

/* ---- KMS ---- */
static void ho_free_bufs(void)
{
	for (int i = 0; i < 2; i++) {
		if (HO.map[i]) munmap(HO.map[i], HO.size);
		if (HO.fb[i]) drmModeRmFB(HO.lease_fd, HO.fb[i]);
		if (HO.handle[i]) { struct drm_mode_destroy_dumb dd = { .handle = HO.handle[i] }; drmIoctl(HO.lease_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dd); }
		HO.map[i] = NULL; HO.fb[i] = HO.handle[i] = 0;
	}
}
static bool ho_alloc_bufs(int w, int h)
{
	for (int i = 0; i < 2; i++) {
		struct drm_mode_create_dumb cd = { .width = (uint32_t)w, .height = (uint32_t)h, .bpp = 32 };
		if (drmIoctl(HO.lease_fd, DRM_IOCTL_MODE_CREATE_DUMB, &cd)) return false;
		HO.handle[i] = cd.handle; HO.pitch = cd.pitch; HO.size = (uint32_t)cd.size;
		if (drmModeAddFB(HO.lease_fd, (uint32_t)w, (uint32_t)h, 24, 32, cd.pitch, cd.handle, &HO.fb[i])) return false;
		struct drm_mode_map_dumb md = { .handle = cd.handle };
		if (drmIoctl(HO.lease_fd, DRM_IOCTL_MODE_MAP_DUMB, &md)) return false;
		HO.map[i] = mmap(NULL, cd.size, PROT_READ | PROT_WRITE, MAP_SHARED, HO.lease_fd, (off_t)md.offset);
		if (HO.map[i] == MAP_FAILED) { HO.map[i] = NULL; return false; }
		memset(HO.map[i], 0, cd.size);
	}
	HO.w = w; HO.h = h;
	return true;
}
static bool ho_has_mode(int fd, uint32_t conn_id, int w, int h)
{
	drmModeConnector *c = drmModeGetConnector(fd, conn_id); bool ok = false;
	if (!c) return false;
	for (int i = 0; i < c->count_modes; i++) if (c->modes[i].hdisplay == w && c->modes[i].vdisplay == h) ok = true;
	drmModeFreeConnector(c);
	return ok;
}
static bool ho_modeset(const char *want_mode)
{
	int fd = HO.lease_fd;
	drmModeRes *res = drmModeGetResources(fd);
	if (!res) { ho_status("lease fd: no resources (%s)", strerror(errno)); return false; }
	drmModeConnector *c = NULL;
	for (int i = 0; i < res->count_connectors && !c; i++) { c = drmModeGetConnector(fd, res->connectors[i]); if (c && c->connector_id != HO.pick->id) { drmModeFreeConnector(c); c = NULL; } }
	if (!c) { ho_status("leased connector %u not in the lease's resources", HO.pick->id); drmModeFreeResources(res); return false; }
	uint32_t crtc = 0; /* possible_crtcs is remapped by the kernel to the lessee's crtc indices */
	for (int e = 0; e < c->count_encoders && !crtc; e++) {
		drmModeEncoder *enc = drmModeGetEncoder(fd, c->encoders[e]);
		if (!enc) continue;
		for (int k = 0; k < res->count_crtcs && !crtc; k++) if (enc->possible_crtcs & (1u << k)) crtc = res->crtcs[k];
		drmModeFreeEncoder(enc);
	}
	if (!crtc && res->count_crtcs) crtc = res->crtcs[0];
	if (!crtc) { ho_status("lease has no CRTC"); drmModeFreeConnector(c); drmModeFreeResources(res); return false; }
	HO.crtc = crtc;
	/* candidates: the requested mode, 4000x2040 (fastest first), then anything 1920x1080, then the first */
	int ww = 0, wh = 0, wr = 0; if (want_mode) sscanf(want_mode, "%dx%d@%d", &ww, &wh, &wr);
	const drmModeModeInfo *cand[16]; int nc = 0;
	for (int pass = 0; pass < 4 && nc < 16; pass++)
		for (int i = 0; i < c->count_modes && nc < 16; i++) {
			const drmModeModeInfo *m = &c->modes[i]; bool take = false;
			if (pass == 0) take = ww && m->hdisplay == ww && m->vdisplay == wh && (!wr || (int)m->vrefresh == wr);
			else if (pass == 1) take = m->hdisplay == 4000 && m->vdisplay == 2040;
			else if (pass == 2) take = m->hdisplay == 1920 && m->vdisplay == 1080;
			else take = true;
			for (int k = 0; k < nc && take; k++) if (cand[k] == m) take = false;
			if (take) cand[nc++] = m;
		}
	bool ok = false;
	for (int k = 0; k < nc && !ok; k++) {
		const drmModeModeInfo *m = cand[k];
		if (!ho_alloc_bufs(m->hdisplay, m->vdisplay)) { ho_free_bufs(); ho_status("dumb buffer %dx%d: %s", m->hdisplay, m->vdisplay, strerror(errno)); continue; }
		uint32_t conn_id = c->connector_id;
		drmModeModeInfo mode = *m;
		int r = drmModeSetCrtc(fd, crtc, HO.fb[0], 0, 0, &conn_id, 1, &mode);
		if (r) { fprintf(stderr, "hmd view: modeset %s@%d failed: %s\n", m->name, m->vrefresh, strerror(-r > 0 ? -r : errno)); ho_free_bufs(); continue; }
		HO.mode = mode; HO.back = 1; HO.active = true; ok = true;
		ho_status("panel lit: %s@%d on %s (%s), crtc %u — the headset should wake up (dprx 1) within a few seconds", m->name, m->vrefresh, HO.pick->name, HO.pick->desc, crtc);
	}
	if (!ok && !HO.status[0]) ho_status("no mode could be set on %s", HO.pick->name);
	drmModeFreeConnector(c); drmModeFreeResources(res);
	return ok;
}

/* ---- public ---- */
static void hmdout_thread_start(void);
/* want_name: connector name to lease (e.g. "DP-9") or NULL for the first PSVR2-looking one */
static bool hmdout_start(const char *want_name, const char *want_mode)
{
	HO.dpy = wl_display_connect(NULL);
	if (!HO.dpy) { ho_status("no Wayland display (WAYLAND_DISPLAY unset?): cannot lease the panel, the headset will stay asleep"); return false; }
	HO.reg = wl_display_get_registry(HO.dpy);
	wl_registry_add_listener(HO.reg, &ho_reg_listener, NULL);
	wl_display_roundtrip(HO.dpy);                 /* globals */
	if (!HO.ndev) { ho_status("compositor offers no wp_drm_lease_device_v1 (Hyprland/sway/KDE do)"); return false; }
	wl_display_roundtrip(HO.dpy);                 /* drm_fd, connectors */
	wl_display_roundtrip(HO.dpy);                 /* connector names/ids */
	char names[160] = ""; size_t nl = 0;
	for (int i = 0; i < HO.nconn; i++) {
		struct ho_conn *k = &HO.conn[i];
		if (k->withdrawn) continue;
		nl += (size_t)snprintf(names + nl, sizeof names - nl, "%s%s(%s)", nl ? " " : "", k->name, k->desc);
		k->vr = strcasestr(k->desc, "sony") || strcasestr(k->desc, "playstation") || strcasestr(k->desc, "vr2") || (k->dev->drm_fd >= 0 && ho_has_mode(k->dev->drm_fd, k->id, 4000, 2040));
		if (want_name ? !strcmp(k->name, want_name) : (k->vr && !HO.pick)) HO.pick = k;
	}
	if (!HO.pick) { ho_status("no PSVR2 connector offered for lease (offered: %s)", names[0] ? names : "none"); return false; }
	struct wp_drm_lease_request_v1 *req = wp_drm_lease_device_v1_create_lease_request(HO.pick->dev->obj);
	wp_drm_lease_request_v1_request_connector(req, HO.pick->obj);
	HO.lease = wp_drm_lease_request_v1_submit(req); /* destroys the request */
	wp_drm_lease_v1_add_listener(HO.lease, &ho_lease_listener, NULL);
	for (int i = 0; i < 50 && !HO.lease_answered; i++) if (wl_display_roundtrip(HO.dpy) < 0) break;
	if (HO.lease_fd < 0) { ho_status("compositor refused the lease of %s (is SteamVR/another VR runtime holding it?)", HO.pick->name); return false; }
	if (!ho_modeset(want_mode)) return false;
	hmdout_thread_start();
	return true;
}
/* main loop: dispatch compositor events without blocking */
static void hmdout_pump(void)
{
	if (!HO.dpy) return;
	wl_display_flush(HO.dpy);
	while (wl_display_prepare_read(HO.dpy) != 0) wl_display_dispatch_pending(HO.dpy);
	struct pollfd p = { wl_display_get_fd(HO.dpy), POLLIN, 0 };
	if (poll(&p, 1, 0) > 0) wl_display_read_events(HO.dpy); else wl_display_cancel_read(HO.dpy);
	wl_display_dispatch_pending(HO.dpy);
}
/* ---- presentation thread: see-through warp or the 2x2 grid, from a copy of the canvas's camera area ----
 * See-through: for every panel pixel the lens model (disp_ComputeDistortion, the driver's own) gives the view
 * direction; head -> cam0 (q_hc) -> cam k (factory extrinsics) -> fisheye projection (factory intrinsics) ->
 * binned LD pixel = canvas pixel of that camera's quadrant, overlays included.  Left eye <- cam0 (front-left),
 * right eye <- cam1 (front-right); parallax between eye and camera is ignored.  Built at half resolution. */
#define HO_CW 1030   /* canvas region copied for the headset: x 0..HO_CW, y 0..HO_CH */
#define HO_CH 1070
#define HO_LW 1000   /* half-resolution LUT per eye (panel 4000x2040 -> 2000x2040 per eye -> 1000x1020) */
#define HO_LH 1020
/* HUD overlay: a bitmap in the eye's angular (tan) space, +-OV_TX horizontally, +-OV_TY vertically around straight ahead, drawn by the
 * tool (guided calibration text) and composited over the see-through image per panel pixel; shifted OV_SHIFT inward per eye so it
 * fuses at ~1.5 m instead of infinity.  Text drawn into the camera images would be magnified ~6x by the fisheye-to-panel mapping. */
#define OV_W 900
#define OV_H 450
#define OV_TX 0.48
#define OV_TY 0.24
#define OV_SHIFT 0.012
struct ho_lut { int16_t x, y, ox, oy; }; /* canvas pixel (camera image) and overlay pixel for this panel pixel, -1 = none */
static struct {
	pthread_t thread; pthread_mutex_t lock; pthread_cond_t cond; bool run, started;
	uint32_t *canvas[2], *overlay[2]; bool overlay_on[2]; int fresh_buf, rendering; bool fresh; int mode; /* mode: 0 off, 1 see-through, 2 grid */
	struct ho_lut *lut[2], *lut_slot[2][2]; int16_t x0[2][HO_LH], x1[2][HO_LH]; bool lut_ok; unsigned lut_gen, want_gen; uint64_t lut_ms; /* lut_slot: two buffers per eye from the arena, rebuilt into the one not in use */
	unsigned presented; float fps; uint64_t fps_t0; unsigned fps_n;
	int quad_x[4], quad_y[4]; /* quadrant image origins in the copied region */
} HP = { .mode = 1, .rendering = -1 };

static void ho_build_lut(void)
{
	uint64_t t0 = now_us();
	static const float proj[2][4] = { { -TAN_OUT, TAN_IN, -TAN_V, TAN_V }, { -TAN_IN, TAN_OUT, -TAN_V, TAN_V } }; /* l r t b, as GetProjectionRaw */
	double R_ch[9], Rhc[9]; m3_from_q(O.q_hc, Rhc);
	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) R_ch[i * 3 + j] = Rhc[j * 3 + i];
	for (int eye = 0; eye < 2; eye++) {
		int cam = eye; /* cam0 front-left, cam1 front-right */
		struct ho_lut *L = HP.lut_slot[eye][HP.lut[eye] == HP.lut_slot[eye][0] ? 1 : 0]; /* the slot the renderer is not reading */
		for (int Y = 0; Y < HO_LH; Y++) {
			int xa = HO_LW, xb = -1;
			for (int X = 0; X < HO_LW; X++) {
				struct ho_lut e = { -1, -1, -1, -1 };
				DistortionCoordinates_t d; disp_ComputeDistortion(&d, NULL, eye, (2 * X + 1) / 2000.f, (2 * Y + 1) / 2040.f);
				float s = d.rfGreen[0], t = d.rfGreen[1];
				if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
					double dh[3] = { proj[eye][0] + s * (proj[eye][1] - proj[eye][0]), -(proj[eye][2] + t * (proj[eye][3] - proj[eye][2])), -1 }, dc[3], dk[3], n[2], u, v, bx, by;
					{ /* overlay: the eye-space tan coordinates of this panel pixel, each eye's content shifted toward the nose */
						double tx = dh[0] - (eye == 0 ? OV_SHIFT : -OV_SHIFT), ty = dh[1];
						int ox = (int)((tx + OV_TX) / (2 * OV_TX) * OV_W), oy = (int)((OV_TY - ty) / (2 * OV_TY) * OV_H);
						if (ox >= 0 && ox < OV_W && oy >= 0 && oy < OV_H) { e.ox = (int16_t)ox; e.oy = (int16_t)oy; }
					}
					if (!O.cam[cam].ok) goto store;
					m3_mulv(R_ch, dh, dc); m3_tmulv(O.cam[cam].R, dc, dk);
					if (dk[2] > 0.05) {
						n[0] = dk[0] / dk[2]; n[1] = dk[1] / dk[2];
						if (hypot(n[0], n[1]) < 3.0) {
							norm2full(&O.cam[cam], n, &u, &v); full2bin(u, v, &bx, &by);
							if (bx >= 0 && bx < Q_W && by >= 0 && by < Q_W) { e.x = (int16_t)(HP.quad_x[cam] + (int)bx); e.y = (int16_t)(HP.quad_y[cam] + (int)by); }
						}
					}
				}
store:
				L[Y * HO_LW + X] = e;
				if (e.x >= 0 || e.ox >= 0) { if (X < xa) xa = X; xb = X; }
			}
			HP.x0[eye][Y] = (int16_t)xa; HP.x1[eye][Y] = (int16_t)xb;
		}
		pthread_mutex_lock(&HP.lock);
		HP.lut[eye] = L;
		pthread_mutex_unlock(&HP.lock);
	}
	HP.lut_ok = true; HP.lut_ms = (now_us() - t0) / 1000;
}
static void ho_render_see(const uint32_t *cv, const uint32_t *ov)
{
	uint8_t *dst = HO.map[HO.back]; int eye_w = HO.w / 2;
	if (HO.w < 4000 || HO.h < 2040) return; /* the lens model is for the 4000x2040 panel */
	for (int eye = 0; eye < 2; eye++) {
		const struct ho_lut *L = HP.lut[eye];
		for (int Y = 0; Y < HO_LH; Y++) {
			int xa = HP.x0[eye][Y], xb = HP.x1[eye][Y];
			if (xb < xa) continue;
			uint32_t *r0 = (uint32_t *)(dst + (size_t)(2 * Y) * HO.pitch) + eye * eye_w + 2 * xa, *r1 = (uint32_t *)((uint8_t *)r0 + HO.pitch);
			const struct ho_lut *e = L + Y * HO_LW + xa;
			for (int X = xa; X <= xb; X++, e++, r0 += 2, r1 += 2) {
				uint32_t c = e->x >= 0 ? cv[e->y * HO_CW + e->x] : 0xff000000;
				if (ov && e->ox >= 0) { uint32_t o = ov[e->oy * OV_W + e->ox]; if (o >> 24) c = o; }
				r0[0] = r0[1] = r1[0] = r1[1] = c;
			}
		}
	}
}
static void ho_render_grid(const uint32_t *cv, const uint32_t *ov)
{
	uint8_t *dst = HO.map[HO.back]; int eye_w = HO.w / 2, cy = HO.h / 2, rw = HO_CW, rh = HO_CH < HO.h ? HO_CH : HO.h;
	int ex[2] = { (int)(eye_w * 0.6604), eye_w + (int)(eye_w * 0.3396) };
	for (int e = 0; e < 2; e++) {
		int dx0 = ex[e] - rw / 2, lo = e * eye_w, hi = lo + eye_w, sx = 0, w = rw;
		if (dx0 < lo) { sx = lo - dx0; w -= sx; dx0 = lo; }
		if (dx0 + w > hi) w = hi - dx0;
		if (w <= 0) continue;
		for (int y = 0; y < rh; y++) {
			int dy = cy - rh / 2 + y;
			if (dy < 0 || dy >= HO.h) continue;
			memcpy(dst + (size_t)dy * HO.pitch + (size_t)dx0 * 4, cv + (size_t)y * HO_CW + sx, (size_t)w * 4);
		}
		if (ov) { /* overlay centred on the eye, 2x */
			int ox0 = ex[e] - OV_W, oy0 = cy - OV_H;
			for (int y = 0; y < 2 * OV_H; y++) {
				int dy = oy0 + y; if (dy < 0 || dy >= HO.h) continue;
				uint32_t *row = (uint32_t *)(dst + (size_t)dy * HO.pitch); const uint32_t *src = ov + (size_t)(y / 2) * OV_W;
				for (int x = 0; x < 2 * OV_W; x++) { int dx = ox0 + x; if (dx < lo || dx >= hi) continue; uint32_t o = src[x / 2]; if (o >> 24) row[dx] = o; }
			}
		}
	}
}
static void ho_flip(void)
{
	int r = drmModePageFlip(HO.lease_fd, HO.crtc, HO.fb[HO.back], 0, NULL);
	if (r == 0) { HO.back ^= 1; HO.flips++; }
	else if (r == -EBUSY || errno == EBUSY) HO.busy++; /* previous flip still pending: this frame is skipped */
	else { uint32_t conn_id = HO.pick->id; if (drmModeSetCrtc(HO.lease_fd, HO.crtc, HO.fb[HO.back], 0, 0, &conn_id, 1, &HO.mode) == 0) { HO.back ^= 1; HO.flips++; } }
}
static void *ho_thread(void *arg)
{
	(void)arg;
	pthread_mutex_lock(&HP.lock);
	while (HP.run) {
		if (HP.want_gen != HP.lut_gen) { unsigned g = HP.want_gen; pthread_mutex_unlock(&HP.lock); ho_build_lut(); pthread_mutex_lock(&HP.lock); HP.lut_gen = g; continue; }
		if (!HP.fresh) { pthread_cond_wait(&HP.cond, &HP.lock); continue; }
		HP.fresh = false; int mode = HP.mode; HP.rendering = HP.fresh_buf; const uint32_t *cv = HP.canvas[HP.rendering], *ov = HP.overlay_on[HP.rendering] ? HP.overlay[HP.rendering] : NULL;
		pthread_mutex_unlock(&HP.lock);
		if (HO.active && mode) {
			uint64_t t0 = now_us();
			if (mode == 1 && HP.lut_ok) ho_render_see(cv, ov); else if (mode == 2) ho_render_grid(cv, ov);
			ho_flip();
			HP.presented++; HP.fps_n++;
			if (now_us() - HP.fps_t0 > 1000000) { HP.fps = HP.fps_n * 1e6f / (float)(now_us() - HP.fps_t0); HP.fps_n = 0; HP.fps_t0 = now_us(); }
			uint64_t spent = now_us() - t0; if (spent < 33000) usleep((useconds_t)(33000 - spent)); /* <= 30 Hz into the headset */
		}
		pthread_mutex_lock(&HP.lock);
		HP.rendering = -1;
	}
	pthread_mutex_unlock(&HP.lock);
	return NULL;
}
/* main loop: hand over the camera area of the canvas and the HUD overlay (OV_W x OV_H ARGB, alpha 0 = transparent; NULL = none).  Copied, so drawing can go on */
static void hmdout_submit(const uint32_t *canvas, int cw, const uint32_t *overlay)
{
	if (!HO.active || !HP.started || !HP.mode) return;
	if (pthread_mutex_trylock(&HP.lock)) return; /* the thread is swapping: skip this frame */
	int b = HP.rendering == 1 ? 0 : 1; /* never the buffer being warped right now */
	uint32_t *dst = HP.canvas[b];
	for (int y = 0; y < HO_CH; y++) memcpy(dst + (size_t)y * HO_CW, canvas + (size_t)y * cw, HO_CW * 4);
	HP.overlay_on[b] = overlay != NULL; if (overlay) memcpy(HP.overlay[b], overlay, (size_t)OV_W * OV_H * 4);
	HP.fresh_buf = b; HP.fresh = true;
	pthread_cond_signal(&HP.cond);
	pthread_mutex_unlock(&HP.lock);
}
static void hmdout_relut(void) { pthread_mutex_lock(&HP.lock); HP.want_gen++; pthread_cond_signal(&HP.cond); pthread_mutex_unlock(&HP.lock); }
static void hmdout_set_mode(int mode) { pthread_mutex_lock(&HP.lock); HP.mode = mode; if (HO.active && mode == 0) memset(HO.map[HO.back], 0, HO.size); pthread_mutex_unlock(&HP.lock); }
static void hmdout_thread_start(void)
{
	pthread_mutex_init(&HP.lock, NULL); pthread_cond_init(&HP.cond, NULL);
	for (int i = 0; i < 2; i++) { HP.canvas[i] = mem_alloc((size_t)HO_CW * HO_CH * 4); HP.overlay[i] = mem_alloc((size_t)OV_W * OV_H * 4); }
	for (int eye = 0; eye < 2; eye++) for (int k = 0; k < 2; k++) HP.lut_slot[eye][k] = mem_alloc(sizeof(struct ho_lut) * HO_LW * HO_LH);
	for (int q = 0; q < 4; q++) { HP.quad_x[q] = Q_X0 + (q % 2) * Q_PX; HP.quad_y[q] = Q_Y0 + (q / 2) * Q_PY; }
	HP.run = true; HP.started = true; HP.want_gen = 1; HP.fps_t0 = now_us();
	pthread_create(&HP.thread, NULL, ho_thread, NULL);
}

/* Teardown deliberately sends no destroy/release requests: Hyprland 0.55 (CDRMLeaseResource::setDestroy ->
 * destroyResource -> ~CDRMLeaseResource -> wl_resource destroy -> onDestroy -> destroyResource again) crashes
 * on an explicit wp_drm_lease_v1.destroy.  Closing the lease fd ends the kernel lease and disconnecting the
 * display lets the compositor clean up the way it does for vrcompositor (which simply exits). */
static void hmdout_stop(void)
{
	if (HP.started) { pthread_mutex_lock(&HP.lock); HP.run = false; pthread_cond_signal(&HP.cond); pthread_mutex_unlock(&HP.lock); pthread_join(HP.thread, NULL); HP.started = false; }
	if (HO.lease_fd >= 0) { if (HO.active) drmModeSetCrtc(HO.lease_fd, HO.crtc, 0, 0, 0, NULL, 0, NULL); ho_free_bufs(); }
	HO.active = false;
	if (HO.lease_fd >= 0) close(HO.lease_fd);
	for (int i = 0; i < HO.ndev; i++) if (HO.dev[i].drm_fd >= 0) close(HO.dev[i].drm_fd);
	if (HO.dpy) { wl_display_flush(HO.dpy); wl_display_disconnect(HO.dpy); }
	HO.dpy = NULL; HO.lease = NULL; HO.lease_fd = -1;
}
