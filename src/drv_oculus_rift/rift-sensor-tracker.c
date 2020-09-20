/*
 * Rift position tracking
 * Copyright 2014-2015 Philipp Zabel
 * Copyright 2019 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 */

 #define _GNU_SOURCE

#include <libusb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "rift-sensor-tracker.h"

#include "rift-sensor-blobwatch.h"
#include "rift-sensor-ar0134.h"
#include "rift-sensor-mt9v034.h"
#include "rift-sensor-esp770u.h"
#include "rift-sensor-esp570.h"
#include "rift-sensor-uvc.h"

#include "rift-sensor-maths.h"
#include "rift-sensor-opencv.h"
#include "rift-sensor-pose-helper.h"

#include "rift-debug-draw.h"

#include "kalman.h"
#include "ohmd-pipewire.h"

#define ASSERT_MSG(_v, label, ...) if(!(_v)){ fprintf(stderr, __VA_ARGS__); goto label; }

#define MAX_SENSORS 4
#define MAX_DEVICES 3

struct rift_sensor_ctx_s
{
  int id;
  char serial_no[32];
  rift_tracker_ctx *tracker;
	bool is_cv1;

  libusb_device_handle *usb_devh;
  int stream_started;
  struct rift_sensor_uvc_stream stream;
  uint64_t frame_sof_ts;
  uint64_t led_pattern_sof_ts;
  uint8_t led_pattern_phase;
  struct blobwatch* bw;
	struct blobservation* bwobs;

  dmat3 camera_matrix;
  bool dist_fisheye;
  double dist_coeffs[5];

  correspondence_search_t *cs;

  kalman_pose *pose_filter;

	vec3f led_out_points[MAX_OBJECT_LEDS];

  ohmd_pw_video_stream *debug_vid;
  uint8_t *debug_frame;
  ohmd_pw_debug_stream *debug_metadata;

	bool have_camera_pose;
	posef camera_pose;
};

struct rift_tracker_ctx_s
{
	ohmd_context* ohmd_ctx;
  libusb_context *usb_ctx;
	ohmd_mutex *tracker_lock;

	ohmd_thread* usb_thread;
	int usb_completed;

  uint8_t led_pattern_phase;
  uint64_t led_pattern_phase_ts;

  rift_sensor_ctx *sensors[MAX_SENSORS];
  uint8_t n_sensors;

  rift_tracked_device devices[MAX_DEVICES];
};

static uint8_t rift_sensor_tracker_get_led_pattern_phase (rift_tracker_ctx *ctx, uint64_t *ts);
static void update_device_pose (rift_sensor_ctx *sensor_ctx, rift_tracked_device *dev,
        rift_pose_metrics *score);

static int tracker_process_blobs(rift_sensor_ctx *ctx)
{
	struct blobservation* bwobs = ctx->bwobs;
	int ret = 0;
	dmat3 *camera_matrix = &ctx->camera_matrix;
	double *dist_coeffs = ctx->dist_coeffs;

	int d;

	correspondence_search_set_blobs (ctx->cs, bwobs->blobs, bwobs->num_blobs);

	for (d = 0; d < 3; d++) {
		rift_tracked_device *dev = ctx->tracker->devices + d;
		rift_pose_metrics score;
		posef world_obj_pose, cam_pose;
		bool match_all_blobs = (d == 0); /* Let the HMD match whatever it can */

		if (dev->fusion == NULL)
			continue; /* No such device */

		oposef_init (&world_obj_pose, &dev->fusion->world_position, &dev->fusion->orient);

		LOGD ("Fusion provided pose for device %d, %f %f %f %f  pos %f %f %f",
			d, world_obj_pose.orient.x, world_obj_pose.orient.y, world_obj_pose.orient.z, world_obj_pose.orient.w,
			world_obj_pose.pos.x, world_obj_pose.pos.y, world_obj_pose.pos.z);

		/* If we have a camera pose, get the object->camera pose by taking
		 * our camera pose (world->camera) and applying to the inverse of
		 * the fusion pose (world->object).
		 * If not, the correspondence search will do a full search, so it doesn't
		 * matter what we feed as the initial pose */
		oposef_inverse (&world_obj_pose);
		if (ctx->have_camera_pose) {
			oposef_apply (&world_obj_pose, &ctx->camera_pose, &cam_pose);
		}
		else {
			cam_pose = world_obj_pose;
		}

		LOGD ("Sensor %d searching for matching pose for device %d, initial quat %f %f %f %f  pos %f %f %f",
			ctx->id, d, cam_pose.orient.x, cam_pose.orient.y, cam_pose.orient.z, cam_pose.orient.w,
			cam_pose.pos.x, cam_pose.pos.y, cam_pose.pos.z);

		rift_evaluate_pose (&score, &cam_pose.orient, &cam_pose.pos,
			ctx->bwobs->blobs, ctx->bwobs->num_blobs,
			dev->id, dev->leds->points, dev->leds->num_points,
			&ctx->camera_matrix, ctx->dist_coeffs, ctx->dist_fisheye);

#if 0
		if (score.good_pose_match)
			printf ("Sensor %d already had good pose match for device %d matched %u blobs of %u\n",
				ctx->id, d, score.matched_blobs, score.visible_leds);
		else
			printf ("Sensor %d needs correspondence search for device %d matched %u blobs of %u\n",
				ctx->id, d, score.matched_blobs, score.visible_leds);
#endif

		if (score.good_pose_match || correspondence_search_find_one_pose (ctx->cs, d, match_all_blobs, &cam_pose.orient, &cam_pose.pos, &score)) {
			if (score.good_pose_match) {
				ret = 1;

				/* Clear existing blob IDs for this device, then
				 * back project LED ids into blobs if we find them and the dot product
				 * shows them pointing strongly to the camera */
				for (int index = 0; index < ctx->bwobs->num_blobs; index++) {
					struct blob *b = ctx->bwobs->blobs + index;
					if (LED_OBJECT_ID (b->led_id) == d) {
						b->prev_led_id = b->led_id;
						b->led_id = LED_INVALID_ID;
					}
				}

				rift_mark_matching_blobs (&cam_pose.orient, &cam_pose.pos,
						bwobs->blobs, bwobs->num_blobs, d,
						dev->leds->points, dev->leds->num_points,
						camera_matrix, dist_coeffs, ctx->is_cv1);

				/* Refine the pose with PnP now that we've labelled the blobs */
				estimate_initial_pose (ctx->bwobs->blobs, ctx->bwobs->num_blobs,
					d, dev->leds->points, dev->leds->num_points, camera_matrix,
					dist_coeffs, ctx->is_cv1,
					&cam_pose.orient, &cam_pose.pos, NULL, NULL, true);

				kalman_pose_update (ctx->pose_filter, ctx->frame_sof_ts, &cam_pose.pos, &cam_pose.orient);
				kalman_pose_get_estimated (ctx->pose_filter, &dev->pose.pos, &dev->pose.orient);

				LOGD ("sensor %d kalman filter for device %d yielded quat %f %f %f %f  pos %f %f %f\n",
					ctx->id, d, dev->pose.orient.x, dev->pose.orient.y, dev->pose.orient.z, dev->pose.orient.w,
					dev->pose.pos.x, dev->pose.pos.y, dev->pose.pos.z);

				update_device_pose (ctx, dev, &score);
			}
		}
	}

	return ret;
}

void
rift_sensor_tracker_add_device (rift_tracker_ctx *ctx, int device_id, fusion *f, rift_leds *leds)
{
	int i;

	assert (device_id < MAX_DEVICES);
	ctx->devices[device_id].id = device_id;
	ctx->devices[device_id].fusion = f;
	ctx->devices[device_id].leds = leds;
	ctx->devices[device_id].led_search = led_search_model_new (leds);

	for (i = 0; i < ctx->n_sensors; i++) {
		rift_sensor_ctx *sensor_ctx = ctx->sensors[i];
		correspondence_search_set_model (sensor_ctx->cs, device_id, ctx->devices[device_id].led_search);
	}

	printf("device %d online. Now tracking.\n", device_id);
}

static int
rift_sensor_get_calibration(rift_sensor_ctx *ctx, uint16_t usb_idProduct)
{
        uint8_t buf[128];
        double * const A = ctx->camera_matrix.m;
        double * const k = ctx->dist_coeffs;
        double fx, fy, cx, cy;
        double k1, k2, k3, k4;
        double p1, p2;
        int ret;

        switch (usb_idProduct) {
			      case CV1_PID:
				        /* Read a 128-byte block at EEPROM address 0x1d000 */
				        ret = rift_sensor_esp770u_flash_read(ctx->usb_devh, 0x1d000, buf, sizeof buf);
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

                printf (" f = [ %7.3f %7.3f ], c = [ %7.3f %7.3f ]\n", fx, fy, cx, cy);
                printf (" k = [ %9.6f %9.6f %9.6f %9.6f ]\n", k1, k2, k3, k4);
                /*
                * k = [ k₁ k₂, k₃, k4 ] for CV1 fisheye distortion
                */
                k[0] = k1; k[1] = k2; k[2] = k3; k[3] = k4;
                ctx->dist_fisheye = true;
								break;
						case DK2_PID: {
                int i;

								/* Read 4 32-byte blocks at EEPROM address 0x2000 */
								for (i = 0; i < 128; i += 32) {
												ret = esp570_eeprom_read(ctx->usb_devh, 0x2000 + i, buf + i, 32);
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

                printf (" f = [ %7.3f %7.3f ], c = [ %7.3f %7.3f ]\n", fx, fy, cx, cy);
                printf (" p = [ %9.6f %9.6f ]\n", p1, p2);
                printf (" k = [ %9.6f %9.6f %9.6f ]\n", k1, k2, k3);

                /*
                * k = [ k₁ k₂, p1, p2, k₃, k4 ] for DK2 distortion
                */
                k[0] = k1; k[1] = k2;
                k[1] = p1; k[2] = p2;
                k[4] = k3;
                ctx->dist_fisheye = false;
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

static void
update_device_pose (rift_sensor_ctx *sensor_ctx, rift_tracked_device *dev,
        rift_pose_metrics *score)
{
	posef pose = dev->pose;

	rift_evaluate_pose (score, &pose.orient, &pose.pos,
		sensor_ctx->bwobs->blobs, sensor_ctx->bwobs->num_blobs,
		dev->id, dev->leds->points, dev->leds->num_points,
		&sensor_ctx->camera_matrix, sensor_ctx->dist_coeffs, sensor_ctx->dist_fisheye);

	if (score->good_pose_match) {
		LOGV("Found good pose match - %u LEDs matched %u visible ones\n",
			score->matched_blobs, score->visible_leds);
		if (sensor_ctx->have_camera_pose) {
			/* The input pose is the transform from object coords to camera frame.
			 * Our camera pose stores the transform from camera to world, and what we
			 * need to give the fusion is transform from world to object.
			 *
			 * To get the transform from world->object, take camera->object, and apply it
			 * to the world->camera pose - ie, apply the inverse of object->camera. */
			oposef_apply_inverse (&sensor_ctx->camera_pose, &pose, &pose);

			LOGD("Updating fusion for device %d pose quat %f %f %f %f  pos %f %f %f\n",
				dev->id, pose.orient.x, pose.orient.y, pose.orient.z, pose.orient.w,
				pose.pos.x, pose.pos.y, pose.pos.z);

			ofusion_tracker_update (dev->fusion, sensor_ctx->frame_sof_ts, &pose.pos, &pose.orient);
		}
		else if (dev->id == 0) {
			/* No camera pose yet. If this is the HMD, use it to initialise the camera (world->camera)
			 * pose using the current headset pose. Calculate the xform from world->camera by applying
			 * the fusion pose (from world->object) to our found pose (object->camera)
			 */
			posef world_object_pose;

			oposef_init(&world_object_pose, &dev->fusion->world_position,
							&dev->fusion->orient);
			oposef_apply(&pose, &world_object_pose, &sensor_ctx->camera_pose);

			LOGI("Set sensor %d pose from device %d - tracker pose quat %f %f %f %f  pos %f %f %f"
					" fusion pose quat %f %f %f %f  pos %f %f %f yielded"
					" world->camera pose quat %f %f %f %f  pos %f %f %f",
				sensor_ctx->id, dev->id,
				pose.orient.x, pose.orient.y, pose.orient.z, pose.orient.w, pose.pos.x, pose.pos.y, pose.pos.z,
				world_object_pose.orient.x, world_object_pose.orient.y, world_object_pose.orient.z, world_object_pose.orient.w,
				world_object_pose.pos.x, world_object_pose.pos.y, world_object_pose.pos.z,
				sensor_ctx->camera_pose.orient.x, sensor_ctx->camera_pose.orient.y, sensor_ctx->camera_pose.orient.z, sensor_ctx->camera_pose.orient.w,
				sensor_ctx->camera_pose.pos.x, sensor_ctx->camera_pose.pos.y, sensor_ctx->camera_pose.pos.z);

#if 0
			/* Double check the newly calculated camera pose can map from our observed camera-relative
			 * pose to the world (fusion) pose */
			oposef_apply_inverse (&sensor_ctx->camera_pose, &pose, &pose);
			LOGI("New camera xform took observed pose to world quat %f %f %f %f  pos %f %f %f",
				pose.orient.x, pose.orient.y, pose.orient.z, pose.orient.w,
				pose.pos.x, pose.pos.y, pose.pos.z);

			/* Double check the newly calculated camera pose can map from the fusion pose back into
			 * our camera-relative pose */
			oposef_inverse (&world_object_pose);
			oposef_apply (&world_object_pose, &sensor_ctx->camera_pose, &pose);

			LOGI("new camera xform took fusion pose back to %f %f %f %f  pos %f %f %f",
				pose.orient.x, pose.orient.y, pose.orient.z, pose.orient.w,
				pose.pos.x, pose.pos.y, pose.pos.z);
#endif

			sensor_ctx->have_camera_pose = true;
		}

	}
	else {
		LOGV("Failed pose match - only %u LEDs matched %u visible ones\n",
		    score->matched_blobs, score->visible_leds);
	}
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
		led_pattern_phase, NULL, 0, &sensor_ctx->bwobs);

	if (sensor_ctx->bwobs && sensor_ctx->bwobs->num_blobs > 0) {
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

	if (ohmd_pw_video_stream_connected(sensor_ctx->debug_vid)) {
		rift_debug_draw_frame (sensor_ctx->debug_frame, sensor_ctx->bwobs, sensor_ctx->cs, stream,
			tracker_ctx->devices, sensor_ctx->is_cv1, sensor_ctx->camera_matrix,
			sensor_ctx->dist_fisheye, sensor_ctx->dist_coeffs);
		ohmd_pw_video_stream_push (sensor_ctx->debug_vid, sensor_ctx->frame_sof_ts, sensor_ctx->debug_frame);
	}
#if 0
	if (ohmd_pw_debug_stream_connected(sensor_ctx->debug_metadata)) {
		char *debug_str;

		asprintf (&debug_str, "{ ts: %llu, exposure_phase: %d, position: { %f, %f, %f }, orientation: { %f, %f, %f, %f } }",
		  (unsigned long long) sensor_ctx->frame_sof_ts, led_pattern_phase,
		  sensor_ctx->pose.pos.x, sensor_ctx->pose.pos.y, sensor_ctx->pose.pos.z,
		  sensor_ctx->pose.orient.x, sensor_ctx->pose.orient.y,
		  sensor_ctx->pose.orient.z, sensor_ctx->pose.orient.w);

		ohmd_pw_debug_stream_push (sensor_ctx->debug_metadata, sensor_ctx->frame_sof_ts, debug_str);
		free(debug_str);
  }
#endif
}


static void rift_sensor_free (rift_sensor_ctx *sensor_ctx);

static rift_sensor_ctx *
rift_sensor_new (ohmd_context* ohmd_ctx, int id, const char *serial_no, libusb_device_handle *usb_devh,
    rift_tracker_ctx *tracker, const uint8_t radio_id[5])
{
  rift_sensor_ctx *sensor_ctx = NULL;
  char stream_id[64];
  int ret;

  libusb_device *dev = libusb_get_device(usb_devh);
  struct libusb_device_descriptor desc;
  ret = libusb_get_device_descriptor(dev, &desc);
  if (ret < 0) {
    printf ("Failed to read device descriptor!\n");
	  return NULL;
  }

  sensor_ctx = ohmd_alloc(ohmd_ctx, sizeof (rift_sensor_ctx));

  sensor_ctx->id = id;
  sensor_ctx->is_cv1 = (desc.idProduct == CV1_PID);
  strcpy ((char *) sensor_ctx->serial_no, (char *) serial_no);
  sensor_ctx->tracker = tracker;
  sensor_ctx->usb_devh = usb_devh;

  sensor_ctx->pose_filter = kalman_pose_new (ohmd_ctx);

  sensor_ctx->stream.sof_cb = new_frame_start_cb;
  sensor_ctx->stream.frame_cb = new_frame_cb;
  sensor_ctx->stream.user_data = sensor_ctx;
	
	sensor_ctx->have_camera_pose = false;

  printf ("Found Rift Sensor %d w/ Serial %s. Connecting to Radio address 0x%02x%02x%02x%02x%02x\n",
    id, serial_no, radio_id[0], radio_id[1], radio_id[2], radio_id[3], radio_id[4]);

  ret = rift_sensor_uvc_stream_setup (sensor_ctx->tracker->usb_ctx, sensor_ctx->usb_devh, &sensor_ctx->stream);
  ASSERT_MSG(ret >= 0, fail, "could not prepare for streaming\n");

  snprintf(stream_id,64,"openhmd-rift-sensor-%s", sensor_ctx->serial_no);
  stream_id[63] = 0;

  sensor_ctx->debug_vid = ohmd_pw_video_stream_new (stream_id, OHMD_PW_VIDEO_FORMAT_RGB, sensor_ctx->stream.width * 2, sensor_ctx->stream.height, 625, 12);
	if (sensor_ctx->debug_vid) {
		/* Allocate an RGB debug frame, twice the width of the input */
		sensor_ctx->debug_frame = ohmd_alloc(ohmd_ctx, 2 * 3 * sensor_ctx->stream.width * sensor_ctx->stream.height);
	}
  sensor_ctx->debug_metadata = ohmd_pw_debug_stream_new (stream_id);

  sensor_ctx->bw = blobwatch_new(sensor_ctx->is_cv1 ? BLOB_THRESHOLD_CV1 : BLOB_THRESHOLD_DK2, sensor_ctx->stream.width, sensor_ctx->stream.height);

	LOGV("Sensor %d - reading Calibration\n", id);
	ret = rift_sensor_get_calibration(sensor_ctx, desc.idProduct);
	if (ret < 0) {
		LOGE("Failed to read Rift sensor calibration data");
		goto fail;
	}

  sensor_ctx->cs = correspondence_search_new (&sensor_ctx->camera_matrix, sensor_ctx->dist_coeffs, sensor_ctx->dist_fisheye);

  LOGI("Sensor %d starting stream\n", id);
  ret = rift_sensor_uvc_stream_start (&sensor_ctx->stream);
  ASSERT_MSG(ret >= 0, fail, "could not start streaming\n");
  sensor_ctx->stream_started = 1;

  switch (desc.idProduct) {
			case CV1_PID:
			{
			  LOGV("Sensor %d - enabling exposure sync\n", id);
			  ret = rift_sensor_ar0134_init(sensor_ctx->usb_devh);
			  if (ret < 0)
			    goto fail;
		
			  LOGV("Sensor %d - setting up radio\n", id);
			  ret = rift_sensor_esp770u_setup_radio(sensor_ctx->usb_devh, radio_id);
			  if (ret < 0)
			    goto fail;
		
		    break;
			}
			case DK2_PID:
			{

				LOGV("Sensor %d - setting up\n", id);
		    	ret = mt9v034_setup(usb_devh);
				if (ret < 0)
					goto fail;

				LOGV("Sensor %d - enabling exposure sync\n", id);
				ret = mt9v034_set_sync(usb_devh, true);
				if (ret < 0)
					goto fail;
		    break;
			}
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

	if (sensor_ctx->bw)
  		blobwatch_free (sensor_ctx->bw);
		
	if (sensor_ctx->stream_started)
		rift_sensor_uvc_stream_stop(&sensor_ctx->stream);

	if (sensor_ctx->debug_vid != NULL)
		ohmd_pw_video_stream_free (sensor_ctx->debug_vid);
	if (sensor_ctx->debug_frame != NULL)
		free (sensor_ctx->debug_frame);
	if (sensor_ctx->debug_metadata != NULL)
		ohmd_pw_debug_stream_free (sensor_ctx->debug_metadata);

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
		const uint8_t radio_id[5])
{
	rift_tracker_ctx *tracker_ctx = NULL;
	int ret, i;
	libusb_device **devs;

	tracker_ctx = ohmd_alloc(ohmd_ctx, sizeof (rift_tracker_ctx));
	tracker_ctx->ohmd_ctx = ohmd_ctx;
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
		unsigned char serial[33];

		ret = libusb_get_device_descriptor(devs[i], &desc);
		if (ret < 0)
			continue; /* Can't access this device */
		if (desc.idVendor != 0x2833 || (desc.idProduct != CV1_PID && desc.idProduct != DK2_PID))
			continue;

		ret = libusb_open(devs[i], &usb_devh);
		if (ret) {
			fprintf (stderr, "Failed to open Rift Sensor device. Check permissions\n");
			continue;
		}

		sprintf ((char *) serial, "UNKNOWN");
		serial[32] = '\0';

		if (desc.iSerialNumber) {
			ret = libusb_get_string_descriptor_ascii(usb_devh, desc.iSerialNumber, serial, 32);
			if (ret < 0)
				fprintf (stderr, "Failed to read the Rift Sensor Serial number.\n");
		}

		sensor_ctx = rift_sensor_new (ohmd_ctx, tracker_ctx->n_sensors, (char *) serial, usb_devh, tracker_ctx, radio_id);
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

  for (i = 0; i < MAX_DEVICES; i++) {
    if (tracker_ctx->devices[i].led_search)
      led_search_model_free (tracker_ctx->devices[i].led_search);
  }

	/* Stop USB event thread */
	tracker_ctx->usb_completed = true;
	ohmd_destroy_thread (tracker_ctx->usb_thread);

	if (tracker_ctx->usb_ctx)
		libusb_exit (tracker_ctx->usb_ctx);

	ohmd_destroy_mutex (tracker_ctx->tracker_lock);
	free (tracker_ctx);
}
