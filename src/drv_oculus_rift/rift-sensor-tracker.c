/*
 * Rift position tracking
 * Copyright 2014-2015 Philipp Zabel
 * Copyright 2019 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 */

#include <libusb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "rift-sensor-tracker.h"

#include "rift-sensor-blobwatch.h"
#include "rift-sensor-ar0134.h"
#include "rift-sensor-esp770u.h"
#include "rift-sensor-uvc.h"

#include "rift-sensor-maths.h"
#include "rift-sensor-opencv.h"

#define ASSERT_MSG(_v, label, ...) if(!(_v)){ fprintf(stderr, __VA_ARGS__); goto label; }
#define min(a,b) ((a) < (b) ? (a) : (b))

#define MAX_SENSORS 4

typedef struct rift_sensor_ctx_s rift_sensor_ctx;

struct rift_sensor_ctx_s
{
  int id;
  rift_tracker_ctx *tracker;

  libusb_device_handle *usb_devh;
  int stream_started;
  struct rift_sensor_uvc_stream stream;
  struct blobwatch* bw;
	struct blobservation* bwobs;

  dmat3 camera_matrix;
  double dist_coeffs[5];
};

struct rift_tracker_ctx_s
{
  libusb_context *usb_ctx;

  rift_leds *leds;
  uint8_t led_pattern_phase;

  rift_sensor_ctx *sensors[MAX_SENSORS];
  uint8_t n_sensors;
};

void tracker_process_blobs(rift_sensor_ctx *ctx)
{
	struct blobservation* bwobs = ctx->bwobs;

  dmat3 *camera_matrix = &ctx->camera_matrix;
 	double dist_coeffs[5] = { 0, };
  dquat rot = { 0, };
	dvec3 trans = { 0, };
  int num_leds = 0;

  /*
   * Estimate initial pose without previously known [rot|trans].
   */
  if (estimate_initial_pose(bwobs->blobs, bwobs->num_blobs, ctx->tracker->leds->points,
            ctx->tracker->leds->num_points, camera_matrix, dist_coeffs, &rot, &trans,
            &num_leds, true)) {
    printf ("sensor %u Got PnP pose quat %f %f %f %f  pos %f %f %f from %d LEDs\n", ctx->id,
            rot.x, rot.y, rot.z, rot.w,
            trans.x, trans.y, trans.z,
	    num_leds);
  }
}

static int
rift_sensor_get_calibration(rift_sensor_ctx *ctx)
{
        uint8_t buf[128];
        double * const A = ctx->camera_matrix.m;
        double * const k = ctx->dist_coeffs;
        double fx, fy, cx, cy;
        double k1, k2, k3, k4;
        int ret;

        /* Read a 128-byte block at EEPROM address 0x1d000 */
        ret = rift_sensor_esp770u_flash_read(ctx->usb_devh, 0x1d000, buf, sizeof buf);
        if (ret < 0)
                return ret;

        /* Fisheye distortion model parameters from firmware */
        fx = fy = *(float *)(buf + 0x30);
        cx = *(float *)(buf + 0x34);
        cy = *(float *)(buf + 0x38);

        k1 = *(float *)(buf + 0x48);
        k2 = *(float *)(buf + 0x4c);
        k3 = *(float *)(buf + 0x50);
        k4 = *(float *)(buf + 0x54);

        printf (" f = [ %7.3f %7.3f ], c = [ %7.3f %7.3f ]\n", fx, fy, cx, cy);
        printf (" k = [ %9.6f %9.6f %9.6f %9.6f ]\n", k1, k2, k3, k4);

        /*
         *     ⎡ fx 0  cx ⎤
         * A = ⎢ 0  fy cy ⎥
         *     ⎣ 0  0  1  ⎦
         */
        A[0] = fx;  A[1] = 0.0; A[2] = cx;
        A[3] = 0.0; A[4] = fy;  A[5] = cy;
        A[6] = 0.0; A[7] = 0.0; A[8] = 1.0;

        /*
         * k = [ k₁ k₂, k₃, k4 ]
         */
        k[0] = k1; k[1] = k2; k[3] = k3; k[4] = k4;

        return 0;
}

static void dump_bin(const char* label, const unsigned char* data, int length)
{
        printf("D %s:\nD ", label);
        for(int i = 0; i < length; i++){
                printf("%02x ", data[i]);
                if((i % 16) == 15)
                        printf("\nD ");
        }
        printf("\n");
}

static void new_frame_cb(struct rift_sensor_uvc_stream *stream)
{
	int width = stream->width;
	int height = stream->height;

	if(stream->payload_size != width * height) {
		printf("bad frame: %d\n", (int)stream->payload_size);
	}

	rift_sensor_ctx *sensor_ctx = stream->user_data;
	assert (sensor_ctx);

	/* FIXME: Get led pattern phase from sensor reports */
	uint8_t led_pattern_phase = 0;

	blobwatch_process(sensor_ctx->bw, stream->frame, width, height,
		led_pattern_phase, sensor_ctx->tracker->leds->points,
		sensor_ctx->tracker->leds->num_points, &sensor_ctx->bwobs);

	if (sensor_ctx->bwobs)
	{
		if (sensor_ctx->bwobs->num_blobs > 0)
		{
			tracker_process_blobs (sensor_ctx); 
#if 0
			printf("Blobs: %d\n", sensor_ctx->bwobs->num_blobs);

			for (int index = 0; index < sensor_ctx->bwobs->num_blobs; index++)
			{
				printf("Blob[%d]: %d,%d id %d\n",
					index,
					sensor_ctx->bwobs->blobs[index].x,
					sensor_ctx->bwobs->blobs[index].y,
					sensor_ctx->bwobs->blobs[index].led_id);
			}
#endif
		}
	}
}


static void rift_sensor_free (rift_sensor_ctx *sensor_ctx);

static int
rift_sensor_init (rift_sensor_ctx **ctx, int id, libusb_device_handle *usb_devh,
    rift_tracker_ctx *tracker, const uint8_t radio_id[5])
{
  rift_sensor_ctx *sensor_ctx = NULL;
  int ret;

  sensor_ctx = calloc(1, sizeof (rift_sensor_ctx));

  sensor_ctx->id = id;
  sensor_ctx->tracker = tracker;
  sensor_ctx->usb_devh = usb_devh;

  sensor_ctx->stream.frame_cb = new_frame_cb;
  sensor_ctx->stream.user_data = sensor_ctx;

  printf ("Found Rift Sensor. Connecting to Radio address 0x%02x%02x%02x%02x%02x\n",
    radio_id[0], radio_id[1], radio_id[2], radio_id[3], radio_id[4]);

  ret = rift_sensor_uvc_stream_setup (sensor_ctx->tracker->usb_ctx, sensor_ctx->usb_devh, &sensor_ctx->stream);
  ASSERT_MSG(ret >= 0, fail, "could not prepare for streaming\n");

  ret = rift_sensor_uvc_stream_start (&sensor_ctx->stream);
  ASSERT_MSG(ret >= 0, fail, "could not start streaming\n");
  sensor_ctx->stream_started = 1;

  sensor_ctx->bw = blobwatch_new(sensor_ctx->stream.width, sensor_ctx->stream.height);

  ret = rift_sensor_ar0134_init(sensor_ctx->usb_devh);
  if (ret < 0)
    goto fail;

  ret = rift_sensor_esp770u_setup_radio(sensor_ctx->usb_devh, radio_id);
  if (ret < 0)
    goto fail;

  ret = rift_sensor_get_calibration(sensor_ctx);
  if (ret < 0) {
    LOGE("Failed to read Rift sensor calibration data");
    return ret;
  }

  *ctx = sensor_ctx;
  return 0;

fail:
	if (sensor_ctx)
		rift_sensor_free (sensor_ctx);
	return -1;
}

static void
rift_sensor_free (rift_sensor_ctx *sensor_ctx)
{
	if (sensor_ctx == NULL)
		return;

	if (sensor_ctx->stream_started)
		rift_sensor_uvc_stream_stop(&sensor_ctx->stream);
	rift_sensor_uvc_stream_clear(&sensor_ctx->stream);

	if (sensor_ctx->usb_devh)
		libusb_close (sensor_ctx->usb_devh);
	free (sensor_ctx);
}

int
rift_sensor_tracker_init (rift_tracker_ctx **ctx,
		const uint8_t radio_id[5], rift_leds *leds)
{
	rift_tracker_ctx *tracker_ctx = NULL;
	int ret, i;
	libusb_device **devs;

	tracker_ctx = calloc(1, sizeof (rift_tracker_ctx));
	tracker_ctx->leds = leds;

	ret = libusb_init(&tracker_ctx->usb_ctx);
	ASSERT_MSG(ret >= 0, fail, "could not initialize libusb\n");

	ret = libusb_get_device_list(tracker_ctx->usb_ctx, &devs);
	ASSERT_MSG(ret >= 0, fail, "Could not get USB device list\n");

	for (i = 0; devs[i]; ++i) {
		struct libusb_device_descriptor desc;
		libusb_device_handle *usb_devh;
		rift_sensor_ctx *sensor_ctx = NULL;

		ret = libusb_get_device_descriptor(devs[i], &desc);
		if (ret < 0)
			continue; /* Can't access this device */
		if (desc.idVendor != 0x2833 || desc.idProduct != CV1_PID)
			continue;
		ret = libusb_open(devs[i], &usb_devh);
		if (ret) {
			fprintf (stderr, "Failed to open Rift Sensor device. Check permissions\n");
			continue;
		}
		if (!rift_sensor_init (&sensor_ctx, tracker_ctx->n_sensors,
		    usb_devh, tracker_ctx, radio_id)) {

			tracker_ctx->sensors[tracker_ctx->n_sensors] = sensor_ctx;
			tracker_ctx->n_sensors++;
			if (tracker_ctx->n_sensors == MAX_SENSORS)
				break;
		}
	}
	libusb_free_device_list(devs, 1);

	printf ("Opened %u Rift Sensor cameras\n", tracker_ctx->n_sensors);

	*ctx = tracker_ctx;
	return 0;

fail:
	if (tracker_ctx)
		rift_sensor_tracker_free (tracker_ctx);
	return ret;
}

void rift_sensor_tracker_new_exposure (rift_tracker_ctx *ctx, uint8_t led_pattern_phase)
{
	ctx->led_pattern_phase = led_pattern_phase;
}

void
rift_sensor_tracker_free (rift_tracker_ctx *tracker_ctx)
{
	int i;

	if (!tracker_ctx)
		return;

	for (i = 0; i < tracker_ctx->n_sensors; i++) {
		rift_sensor_ctx *sensor_ctx = tracker_ctx->sensors[i];
		rift_sensor_free (sensor_ctx);
	}

	if (tracker_ctx->usb_ctx)
		libusb_exit (tracker_ctx->usb_ctx);

	free (tracker_ctx);
}
