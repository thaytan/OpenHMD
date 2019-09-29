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

#include "kalman.h"

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
  uint64_t frame_sof_ts;
  uint64_t led_pattern_sof_ts;
  uint8_t led_pattern_phase;
  struct blobwatch* bw;
	struct blobservation* bwobs;

  dmat3 camera_matrix;
  double dist_coeffs[5];

  kalman_pose *pose_filter;
  vec3f pose_pos;
  quatf pose_orient;
};

struct rift_tracker_ctx_s
{
	ohmd_context* ohmd_ctx;
  libusb_context *usb_ctx;
	ohmd_mutex *tracker_lock;

	ohmd_thread* usb_thread;
	int usb_completed;

  rift_leds *leds;
  uint8_t led_pattern_phase;
  uint64_t led_pattern_phase_ts;

  rift_sensor_ctx *sensors[MAX_SENSORS];
  uint8_t n_sensors;
};

static uint8_t rift_sensor_tracker_get_led_pattern_phase (rift_tracker_ctx *ctx, uint64_t *ts);

void tracker_process_blobs(rift_sensor_ctx *ctx)
{
	struct blobservation* bwobs = ctx->bwobs;

  dmat3 *camera_matrix = &ctx->camera_matrix;
 	double dist_coeffs[5] = { 0, };
  dquat rot = { ctx->pose_orient.x, ctx->pose_orient.y, ctx->pose_orient.z, ctx->pose_orient.w };
	dvec3 trans = { ctx->pose_pos.x, ctx->pose_pos.y, ctx->pose_pos.z };
  int num_leds = 0;

  /*
   * Estimate initial pose without previously known [rot|trans].
   */
  if (estimate_initial_pose(bwobs->blobs, bwobs->num_blobs, ctx->tracker->leds->points,
            ctx->tracker->leds->num_points, camera_matrix, dist_coeffs, &rot, &trans,
            &num_leds, true)) {
    vec3f transf = {{ trans.x, trans.y, trans.z }};
    quatf rotf = {{ rot.x, rot.y, rot.z, rot.w }};

    kalman_pose_update (ctx->pose_filter, ctx->frame_sof_ts, &transf, &rotf);
    kalman_pose_get_estimated (ctx->pose_filter, &ctx->pose_pos, &ctx->pose_orient);

    printf ("sensor %u Got PnP pose quat %f %f %f %f  pos %f %f %f from %d LEDs\n", ctx->id,
				ctx->pose_orient.x, ctx->pose_orient.y, ctx->pose_orient.z, ctx->pose_orient.w,
				ctx->pose_pos.x, ctx->pose_pos.y, ctx->pose_pos.z,
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

static void new_frame_start_cb(struct rift_sensor_uvc_stream *stream)
{
	rift_sensor_ctx *sensor_ctx = stream->user_data;
	rift_tracker_ctx *tracker_ctx = sensor_ctx->tracker;

	uint64_t now = ohmd_monotonic_get(tracker_ctx->ohmd_ctx);
	uint64_t phase_ts;
	uint8_t led_pattern_phase = rift_sensor_tracker_get_led_pattern_phase (sensor_ctx->tracker, &phase_ts);

	LOGD ("%f ms Sensor %d SOF phase %d\n",
		(double) (ohmd_monotonic_get(sensor_ctx->tracker->ohmd_ctx)) / 1000000.0,
		sensor_ctx->id, led_pattern_phase);

	sensor_ctx->led_pattern_phase = led_pattern_phase;
	sensor_ctx->led_pattern_sof_ts = phase_ts;
	sensor_ctx->frame_sof_ts = now;
}

static void new_frame_cb(struct rift_sensor_uvc_stream *stream)
{
	int width = stream->width;
	int height = stream->height;

	if(stream->payload_size != width * height) {
		LOGW("bad frame: %d\n", (int)stream->payload_size);
	}

	rift_sensor_ctx *sensor_ctx = stream->user_data;
	assert (sensor_ctx);
	rift_tracker_ctx *tracker_ctx = sensor_ctx->tracker;

	/* led pattern phase comes from sensor reports */
	uint8_t led_pattern_phase = sensor_ctx->led_pattern_phase;
	uint64_t cur_phase_ts;
	uint8_t cur_led_pattern_phase = rift_sensor_tracker_get_led_pattern_phase (sensor_ctx->tracker, &cur_phase_ts);

	if (cur_led_pattern_phase != led_pattern_phase) {
		/* The LED phase changed mid-frame. Choose which one to keep */
		uint64_t now = ohmd_monotonic_get(tracker_ctx->ohmd_ctx);
		uint64_t frame_midpoint = sensor_ctx->frame_sof_ts + (now - sensor_ctx->frame_sof_ts)/2;
		uint8_t chosen_phase;

		if (cur_phase_ts < frame_midpoint) {
			chosen_phase = cur_led_pattern_phase;
		} else {
			chosen_phase = led_pattern_phase;
		}

		LOGD ("%f Sensor %d Frame (sof %f) phase mismatch old %d new %d (@ %f) chose %d\n",
			(double) (now) / 1000000.0, sensor_ctx->id,
			(double) (sensor_ctx->led_pattern_sof_ts) / 1000000.0,
			led_pattern_phase, cur_led_pattern_phase,
			(double) (cur_phase_ts) / 1000000.0,
			chosen_phase);
		led_pattern_phase = chosen_phase;
	}

	LOGV ("Sensor %d Handling frame phase %d\n", sensor_ctx->id, led_pattern_phase);

	blobwatch_process(sensor_ctx->bw, stream->frame, width, height,
		led_pattern_phase, sensor_ctx->tracker->leds->points,
		sensor_ctx->tracker->leds->num_points, &sensor_ctx->bwobs);

	if (sensor_ctx->bwobs)
	{
		if (sensor_ctx->bwobs->num_blobs > 0)
		{
			tracker_process_blobs (sensor_ctx);
#if 0
			printf("Sensor %d phase %d Blobs: %d\n", sensor_ctx->id, led_pattern_phase, sensor_ctx->bwobs->num_blobs);

			for (int index = 0; index < sensor_ctx->bwobs->num_blobs; index++)
			{
				printf("Sensor %d Blob[%d]: %d,%d %dx%d id %d pattern %x age %u\n", sensor_ctx->id,
					index,
					sensor_ctx->bwobs->blobs[index].x,
					sensor_ctx->bwobs->blobs[index].y,
					sensor_ctx->bwobs->blobs[index].width,
					sensor_ctx->bwobs->blobs[index].height,
					sensor_ctx->bwobs->blobs[index].led_id,
					sensor_ctx->bwobs->blobs[index].pattern,
					sensor_ctx->bwobs->blobs[index].pattern_age);
			}
#endif
		}
	}
}


static void rift_sensor_free (rift_sensor_ctx *sensor_ctx);

static rift_sensor_ctx *
rift_sensor_new (ohmd_context* ohmd_ctx, int id, libusb_device_handle *usb_devh,
    rift_tracker_ctx *tracker, const uint8_t radio_id[5])
{
  rift_sensor_ctx *sensor_ctx = NULL;
  int ret;

  sensor_ctx = ohmd_alloc(ohmd_ctx, sizeof (rift_sensor_ctx));

  sensor_ctx->id = id;
  sensor_ctx->tracker = tracker;
  sensor_ctx->usb_devh = usb_devh;

  sensor_ctx->pose_filter = kalman_pose_new (ohmd_ctx);
  memset (&sensor_ctx->pose_pos, 0, sizeof(sensor_ctx->pose_pos));
  memset (&sensor_ctx->pose_orient, 0, sizeof(sensor_ctx->pose_orient));

  sensor_ctx->stream.sof_cb = new_frame_start_cb;
  sensor_ctx->stream.frame_cb = new_frame_cb;
  sensor_ctx->stream.user_data = sensor_ctx;

  printf ("Found Rift Sensor %d. Connecting to Radio address 0x%02x%02x%02x%02x%02x\n",
    id, radio_id[0], radio_id[1], radio_id[2], radio_id[3], radio_id[4]);

  ret = rift_sensor_uvc_stream_setup (sensor_ctx->tracker->usb_ctx, sensor_ctx->usb_devh, &sensor_ctx->stream);
  ASSERT_MSG(ret >= 0, fail, "could not prepare for streaming\n");

  sensor_ctx->bw = blobwatch_new(sensor_ctx->stream.width, sensor_ctx->stream.height);

  LOGI("Sensor %d starting stream\n", id);
  ret = rift_sensor_uvc_stream_start (&sensor_ctx->stream);
  ASSERT_MSG(ret >= 0, fail, "could not start streaming\n");
  sensor_ctx->stream_started = 1;

  LOGV("Sensor %d enabling exposure sync\n", id);
  ret = rift_sensor_ar0134_init(sensor_ctx->usb_devh);
  if (ret < 0)
    goto fail;

  LOGV("Sensor %d - setting up radio\n", id);
  ret = rift_sensor_esp770u_setup_radio(sensor_ctx->usb_devh, radio_id);
  if (ret < 0)
    goto fail;

  LOGV("Sensor %d - reading Calibration\n", id);
  ret = rift_sensor_get_calibration(sensor_ctx);
  if (ret < 0) {
	LOGE("Failed to read Rift sensor calibration data");
	goto fail;
  }

  LOGI("Sensor %d ready\n", id);

  return sensor_ctx;

fail:
	if (sensor_ctx)
		rift_sensor_free (sensor_ctx);
	return NULL;
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

  kalman_pose_free (sensor_ctx->pose_filter);
	free (sensor_ctx);
}

static unsigned int uvc_handle_events(void *arg)
{
	rift_tracker_ctx *tracker_ctx = arg;

	while (!tracker_ctx->usb_completed)
		libusb_handle_events_completed(tracker_ctx->usb_ctx, &tracker_ctx->usb_completed);

	return 0;
}

rift_tracker_ctx *
rift_sensor_tracker_new (ohmd_context* ohmd_ctx,
		const uint8_t radio_id[5], rift_leds *leds)
{
	rift_tracker_ctx *tracker_ctx = NULL;
	int ret, i;
	libusb_device **devs;

	tracker_ctx = ohmd_alloc(ohmd_ctx, sizeof (rift_tracker_ctx));
	tracker_ctx->ohmd_ctx = ohmd_ctx;
	tracker_ctx->leds = leds;
	tracker_ctx->tracker_lock = ohmd_create_mutex(ohmd_ctx);

	ret = libusb_init(&tracker_ctx->usb_ctx);
	ASSERT_MSG(ret >= 0, fail, "could not initialize libusb\n");

	ret = libusb_get_device_list(tracker_ctx->usb_ctx, &devs);
	ASSERT_MSG(ret >= 0, fail, "Could not get USB device list\n");

	/* Start USB event thread */
	tracker_ctx->usb_completed = false;
	tracker_ctx->usb_thread = ohmd_create_thread (ohmd_ctx, uvc_handle_events, tracker_ctx);

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
		sensor_ctx = rift_sensor_new (ohmd_ctx, tracker_ctx->n_sensors, usb_devh, tracker_ctx, radio_id);
		if (sensor_ctx != NULL) {
			tracker_ctx->sensors[tracker_ctx->n_sensors] = sensor_ctx;
			tracker_ctx->n_sensors++;
			if (tracker_ctx->n_sensors == MAX_SENSORS)
				break;
		}
	}
	libusb_free_device_list(devs, 1);

	printf ("Opened %u Rift Sensor cameras\n", tracker_ctx->n_sensors);

	return tracker_ctx;

fail:
	if (tracker_ctx)
		rift_sensor_tracker_free (tracker_ctx);
	return NULL;
}

static uint8_t
rift_sensor_tracker_get_led_pattern_phase (rift_tracker_ctx *ctx, uint64_t *ts) {
	uint8_t led_pattern_phase;

	ohmd_lock_mutex (ctx->tracker_lock);
	led_pattern_phase = ctx->led_pattern_phase;
	if (ts)
		*ts = ctx->led_pattern_phase_ts;
	ohmd_unlock_mutex (ctx->tracker_lock);

	return led_pattern_phase;
}

void rift_sensor_tracker_new_exposure (rift_tracker_ctx *ctx, uint8_t led_pattern_phase)
{
	if (ctx->led_pattern_phase != led_pattern_phase) {
		uint64_t now = ohmd_monotonic_get(ctx->ohmd_ctx);
		LOGD ("%f LED pattern phase changed to %d\n",
			(double) (now) / 1000000.0,
			led_pattern_phase);
		ohmd_lock_mutex (ctx->tracker_lock);
		ctx->led_pattern_phase = led_pattern_phase;
		ctx->led_pattern_phase_ts = now;
		ohmd_unlock_mutex (ctx->tracker_lock);
	}
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

	/* Stop USB event thread */
	tracker_ctx->usb_completed = true;
	ohmd_destroy_thread (tracker_ctx->usb_thread);

	if (tracker_ctx->usb_ctx)
		libusb_exit (tracker_ctx->usb_ctx);

	ohmd_destroy_mutex (tracker_ctx->tracker_lock);
	free (tracker_ctx);
}
