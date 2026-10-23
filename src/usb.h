/* slopvr — raw Linux usbfs. No libusb: open /dev/bus/usb/BBB/DDD, claim
 * interfaces, synchronous control transfers, async URBs reaped via poll(). */
#pragma once
#include <dirent.h>
#include <fcntl.h>
#include <linux/usbdevice_fs.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

static int usb_read_hex(const char *path)
{
	char b[16] = {0};
	int fd = open(path, O_RDONLY), n;
	if (fd < 0) return -1;
	n = (int)read(fd, b, sizeof b - 1);
	close(fd);
	return n > 0 ? (int)strtol(b, NULL, 16) : -1;
}

static int usb_read_dec(const char *path)
{
	char b[16] = {0};
	int fd = open(path, O_RDONLY), n;
	if (fd < 0) return -1;
	n = (int)read(fd, b, sizeof b - 1);
	close(fd);
	return n > 0 ? atoi(b) : -1;
}

/* Scan sysfs for vid:pid; returns usbfs fd (O_RDWR|O_CLOEXEC) or -1. */
static int usb_open_vidpid(uint16_t vid, uint16_t pid)
{
	DIR *d = opendir("/sys/bus/usb/devices");
	struct dirent *e;
	int fd = -1;
	if (!d) return -1;
	while (fd < 0 && (e = readdir(d))) {
		char p[320];
		if (e->d_name[0] == '.' || strchr(e->d_name, ':')) continue;
		snprintf(p, sizeof p, "/sys/bus/usb/devices/%s/idVendor", e->d_name);
		if (usb_read_hex(p) != vid) continue;
		snprintf(p, sizeof p, "/sys/bus/usb/devices/%s/idProduct", e->d_name);
		if (usb_read_hex(p) != pid) continue;
		snprintf(p, sizeof p, "/sys/bus/usb/devices/%s/busnum", e->d_name);
		int bus = usb_read_dec(p);
		snprintf(p, sizeof p, "/sys/bus/usb/devices/%s/devnum", e->d_name);
		int dev = usb_read_dec(p);
		if (bus < 0 || dev < 0) continue;
		snprintf(p, sizeof p, "/dev/bus/usb/%03d/%03d", bus, dev);
		fd = open(p, O_RDWR | O_CLOEXEC);
	}
	closedir(d);
	return fd;
}

static int usb_claim(int fd, int iface, int alt)
{
	struct usbdevfs_disconnect_claim dc = { .interface = (unsigned)iface, .flags = 0 };
	struct usbdevfs_setinterface si = { .interface = (unsigned)iface, .altsetting = (unsigned)alt };
	if (ioctl(fd, USBDEVFS_DISCONNECT_CLAIM, &dc) < 0 && ioctl(fd, USBDEVFS_CLAIMINTERFACE, &iface) < 0) return -1;
	return ioctl(fd, USBDEVFS_SETINTERFACE, &si);
}

static int usb_ctrl(int fd, uint8_t rt, uint8_t req, uint16_t val, uint16_t idx, void *data, uint16_t len, uint32_t timeout_ms)
{
	struct usbdevfs_ctrltransfer ct = { rt, req, val, idx, len, timeout_ms, data };
	return ioctl(fd, USBDEVFS_CONTROL, &ct);
}

static int usb_submit(int fd, struct usbdevfs_urb *u, uint8_t type, uint8_t ep, void *buf, int len)
{
	memset(u, 0, sizeof *u);
	u->type = type;
	u->endpoint = ep;
	u->buffer = buf;
	u->buffer_length = len;
	return ioctl(fd, USBDEVFS_SUBMITURB, u);
}

/* Non-blocking reap; NULL when nothing completed. */
static struct usbdevfs_urb *usb_reap(int fd)
{
	struct usbdevfs_urb *u = NULL;
	return ioctl(fd, USBDEVFS_REAPURBNDELAY, &u) < 0 ? NULL : u;
}

static int usb_bulk_out(int fd, uint8_t ep, void *data, int len, uint32_t timeout_ms)
{
	struct usbdevfs_bulktransfer bt = { ep, (unsigned)len, timeout_ms, data };
	return ioctl(fd, USBDEVFS_BULK, &bt);
}
