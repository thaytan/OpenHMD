/*
 * Rift sensor USB device handling.
 * Copyright 2014-2015 Philipp Zabel
 * Copyright 2019-2021 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 */

#include <libusb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <inttypes.h>

#include "rift-sensor-usb.h"

#include "sensor/ar0134.h"
#include "sensor/mt9v034.h"
#include "sensor/esp770u.h"
#include "sensor/esp570.h"
#include "sensor/uvc.h"

#define ASSERT_MSG(_v, label, ...) if(!(_v)){ fprintf(stderr, __VA_ARGS__); goto label; }

typedef struct rift_sensor_usb_device rift_sensor_usb_device;

static bool rift_sensor_usb_start (rift_sensor_device *device, uint8_t min_frames, rift_sensor_device_frame_cb frame_cb, void *cb_data);
static void rift_sensor_usb_stop (rift_sensor_device *device);
static void rift_sensor_usb_free (rift_sensor_device *device);

static rift_sensor_device_funcs usb_device_funcs =
{
	rift_sensor_usb_start,
	rift_sensor_usb_stop,
	rift_sensor_usb_free,
};

struct rift_sensor_usb_device
{
	rift_sensor_device base;

	ohmd_context* ohmd_ctx;
	int id;
	char serial_no[32];
	uint8_t hmd_radio_id[5];

	libusb_device_handle *usb_devh;
	uint16_t usb_pid;
	bool usb2_mode;

	int stream_started;
	struct rift_sensor_uvc_stream uvc_stream;

	rift_sensor_device_frame_cb frame_cb;
	void *frame_cb_data;
};

static int
rift_sensor_read_calibration(rift_sensor_usb_device *dev)
{
	uint8_t buf[128];
	rift_sensor_camera_params *calib = &dev->base.calib;
	double * const A = calib->camera_matrix.m;
	double * const k = calib->dist_coeffs;
	double fx, fy, cx, cy;
	double k1, k2, k3, k4;
	double p1, p2;
	int ret;

	/* Copy the frame width and height from the UVC setup */
	calib->width = dev->uvc_stream.width;
	calib->height = dev->uvc_stream.height;

	LOGI("camera width %d height %d", calib->width, calib->height);

	switch (dev->usb_pid) {
		case CV1_PID:
			/* Read a 128-byte block at EEPROM address 0x1d000 */
			ret = rift_sensor_esp770u_flash_read(dev->usb_devh, 0x1d000, buf, sizeof buf);
			if (ret < 0)
				return ret;

			/* Fisheye distortion model parameters from firmware */
			/* FIXME: Need to endian swap for BE systems: */
			fx = fy = *(float *)(buf + 0x30);
			cx = *(float *)(buf + 0x34);
			cy = *(float *)(buf + 0x38);

			k1 = *(float *)(buf + 0x48);
			k2 = *(float *)(buf + 0x4c);
			k3 = *(float *)(buf + 0x50);
			k4 = *(float *)(buf + 0x54);

			LOGI("f = [ %7.3f %7.3f ], c = [ %7.3f %7.3f ]", fx, fy, cx, cy);
			LOGI("k = [ %9.6f %9.6f %9.6f %9.6f ]", k1, k2, k3, k4);

			/*
			 * k = [ k₁ k₂, k₃, k4 ] for CV1 fisheye distortion
			 */
			k[0] = k1; k[1] = k2; k[2] = k3; k[3] = k4;
			calib->dist_fisheye = true;
			calib->is_cv1 = true;
			break;
		case DK2_PID: {
			int i;

			/* Read 4 32-byte blocks at EEPROM address 0x2000 */
			for (i = 0; i < 128; i += 32) {
				ret = esp570_eeprom_read(dev->usb_devh, 0x2000 + i, buf + i, 32);
				if (ret < 0)
					return ret;
			}

			/* FIXME: Need to endian swap for BE systems: */
			fx = *(double *)(buf + 18);
			fy = *(double *)(buf + 30);
			cx = *(double *)(buf + 42);
			cy = *(double *)(buf + 54);
			k1 = *(double *)(buf + 66);
			k2 = *(double *)(buf + 78);
			p1 = *(double *)(buf + 90);
			p2 = *(double *)(buf + 102);
			k3 = *(double *)(buf + 114);

			LOGI("f = [ %7.3f %7.3f ], c = [ %7.3f %7.3f ]", fx, fy, cx, cy);
			LOGI("p = [ %9.6f %9.6f ]", p1, p2);
			LOGI("k = [ %9.6f %9.6f %9.6f ]", k1, k2, k3);

			/*
			 * k = [ k₁ k₂, p1, p2, k₃, k4 ] for DK2 distortion
			 */
			k[0] = k1; k[1] = k2;
			k[2] = p1; k[3] = p2;
			k[4] = k3;
			calib->dist_fisheye = false;
			calib->is_cv1 = false;
			break;
		}
		default:
			return -1;
	}

	/*
	 *     ⎡ fx 0  cx ⎤
	 * A = ⎢ 0  fy cy ⎥
	 *     ⎣ 0  0  1  ⎦
	 */
	A[0] = fx;  A[1] = 0.0; A[2] = cx;
	A[3] = 0.0; A[4] = fy;  A[5] = cy;
	A[6] = 0.0; A[7] = 0.0; A[8] = 1.0;

	return 0;
}

rift_sensor_device *rift_sensor_usb_new (ohmd_context* ohmd_ctx, int id, const char *serial_no,
	libusb_context *usb_ctx, libusb_device_handle *usb_devh, const uint8_t hmd_radio_id[5])
{
	rift_sensor_usb_device *dev = NULL;
	int ret, i;

	libusb_device *usb_dev = libusb_get_device(usb_devh);
	struct libusb_device_descriptor desc;
	ret = libusb_get_device_descriptor(usb_dev, &desc);
	if (ret < 0) {
		printf ("Failed to read device descriptor!\n");
		libusb_close (usb_devh);
		return NULL;
	}

	dev = ohmd_alloc(ohmd_ctx, sizeof (rift_sensor_usb_device));
	dev->base.funcs = &usb_device_funcs;

	dev->ohmd_ctx = ohmd_ctx;
	dev->id = id;

	for (i = 0; i < 5; i++)
		dev->hmd_radio_id[i] = hmd_radio_id[i];

	strcpy ((char *) dev->serial_no, (char *) serial_no);
	dev->usb_devh = usb_devh;
	dev->usb_pid = desc.idProduct;
	dev->usb2_mode = (desc.bcdUSB < 0x300);

	printf ("Opened Rift Sensor %d w/ Serial %s. Connecting to Radio address 0x%02x%02x%02x%02x%02x\n",
		id, serial_no, hmd_radio_id[0], hmd_radio_id[1], hmd_radio_id[2], hmd_radio_id[3], hmd_radio_id[4]);

	ret = rift_sensor_uvc_stream_setup (ohmd_ctx, usb_ctx, dev->usb_devh, &desc, &dev->uvc_stream);
	ASSERT_MSG(ret >= 0, fail, "could not prepare for streaming\n");

#if !HAVE_LIBJPEG
	if (dev->uvc_stream.format == OHMD_VIDEO_FRAME_FORMAT_JPEG) {
		LOGE("Sensor is connected to USB 2.0, but OpenHMD was compiled without libjpeg support. Please rebuild or connect to a USB 3.0 port");
		goto fail;
	}
#endif

	LOGV("Sensor %d - reading Calibration\n", id);
	ret = rift_sensor_read_calibration(dev);
	if (ret < 0) {
		LOGE("Failed to read Rift sensor calibration data");
		goto fail;
	}

	return (rift_sensor_device *)(dev);

fail:
	if (dev)
		rift_sensor_usb_free ((rift_sensor_device *) dev);
	return NULL;
}

static void frame_captured_cb(rift_sensor_uvc_stream *stream, ohmd_video_frame *vframe, void *cb_data)
{
	rift_sensor_device *base_dev = (rift_sensor_device *)(cb_data);
	rift_sensor_usb_device *dev = (rift_sensor_usb_device *)(cb_data);

	dev->frame_cb (base_dev, vframe, dev->frame_cb_data);
}

static bool rift_sensor_usb_start(rift_sensor_device *base_dev, uint8_t min_frames,
		rift_sensor_device_frame_cb frame_cb, void *cb_data)
{
	rift_sensor_usb_device *dev = (rift_sensor_usb_device *)(base_dev);
	int ret;

	LOGV("Sensor %d starting stream\n", dev->id);

	dev->frame_cb = frame_cb;
	dev->frame_cb_data = cb_data;
	ret = rift_sensor_uvc_stream_start (&dev->uvc_stream, min_frames, frame_captured_cb, dev);
	ASSERT_MSG(ret >= 0, fail, "could not start streaming\n");
	dev->stream_started = 1;

	/* Enable radio sync */
	switch (dev->usb_pid) {
			case CV1_PID:
			{
				LOGV("Sensor %d - enabling exposure sync\n", dev->id);
				ret = rift_sensor_ar0134_init(dev->usb_devh, dev->usb2_mode);
				if (ret < 0)
					goto fail;

				LOGV("Sensor %d - setting up radio\n", dev->id);
				ret = rift_sensor_esp770u_setup_radio(dev->usb_devh, dev->hmd_radio_id);
				if (ret < 0)
					goto fail;

				break;
			}
			case DK2_PID:
			{

				LOGV("Sensor %d - setting up\n", dev->id);
					ret = mt9v034_setup(dev->usb_devh);
				if (ret < 0)
					goto fail;

				LOGV("Sensor %d - enabling exposure sync\n", dev->id);
				ret = mt9v034_set_sync(dev->usb_devh, true);
				if (ret < 0)
					goto fail;
				break;
			}
	}


	LOGV("Sensor %d video streaming\n", dev->id);

	return true;
fail:
	return false;
}

void
rift_sensor_usb_stop (rift_sensor_device *dev)
{
	rift_sensor_usb_device *udev = (rift_sensor_usb_device *)(dev);

	if (udev->stream_started) {
		rift_sensor_uvc_stream_stop(&udev->uvc_stream);
		udev->stream_started = false;
	}
}

void
rift_sensor_usb_free (rift_sensor_device *base_dev)
{
	if (base_dev == NULL)
		return;

	rift_sensor_usb_device *dev = (rift_sensor_usb_device *)(base_dev);
	if (dev->stream_started)
		rift_sensor_uvc_stream_stop(&dev->uvc_stream);

	rift_sensor_uvc_stream_clear(&dev->uvc_stream);

	if (dev->usb_devh)
		libusb_close (dev->usb_devh);

	free (dev);
}
