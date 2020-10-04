/*
 * Rift sensor interface
 * Copyright 2014-2015 Philipp Zabel
 * Copyright 2019 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 */

 #define _GNU_SOURCE
#define __STDC_FORMAT_MACROS

#include <libusb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <inttypes.h>

#include "rift-sensor.h"

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

#define NUM_CAPTURE_BUFFERS 2

struct rift_sensor_ctx_s
{
	ohmd_context* ohmd_ctx;
	int id;
	char serial_no[32];
	rift_tracker_ctx *tracker;
	bool is_cv1;

	libusb_device_handle *usb_devh;
	int stream_started;
	struct rift_sensor_uvc_stream stream;

	rift_sensor_capture_frame *frames;

	struct blobwatch* bw;
	struct blobservation* bwobs;

	dmat3 camera_matrix;
	bool dist_fisheye;
	double dist_coeffs[5];

	correspondence_search_t *cs;

	kalman_pose *pose_filter;

	vec3f led_out_points[MAX_OBJECT_LEDS];

	ohmd_mutex *sensor_lock;
	ohmd_cond *new_frame_cond;

	/* Protected by sensor_lock */
	rift_tracked_device *devices[RIFT_MAX_TRACKED_DEVICES];
	uint8_t n_devices;

	/* ID of the currently capturing frame */
	int cur_capture_frame;
	/* ID of the next frame to submit for capturing */
	int next_capture_frame;
	int dropped_frames;

	/* FIXME: later this should be a ringbuffer to support
	 * more outstanding frames, and a frames_ready_for_capture
	 * ringbuffer to return them */
	int frame_for_fast_analysis;

	int shutdown;
	/* End protected by sensor_lock */

	ohmd_thread* fast_analysis_thread;
	ohmd_thread* long_analysis_thread;

	ohmd_pw_video_stream *debug_vid;
	uint8_t *debug_frame;
	ohmd_pw_debug_stream *debug_metadata;

	/* Updated from fast_analysis_thread */
	bool have_camera_pose;
	posef camera_pose;

	uint64_t prev_capture_ts;
};

static void update_device_pose (rift_sensor_ctx *sensor_ctx, rift_tracked_device *dev,
	rift_pose_metrics *score, uint64_t frame_ts, posef *capture_pose);

static int tracker_process_blobs(rift_sensor_ctx *ctx, rift_sensor_capture_frame *frame)
{
	struct blobservation* bwobs = ctx->bwobs;
	int ret = 0;
	dmat3 *camera_matrix = &ctx->camera_matrix;
	double *dist_coeffs = ctx->dist_coeffs;
	int d;

	correspondence_search_set_blobs (ctx->cs, bwobs->blobs, bwobs->num_blobs);

	/* Only process the devices that were available when this frame was captured */
	for (d = 0; d < frame->n_devices; d++) {
		rift_tracked_device *dev = ctx->devices[d];
		rift_pose_metrics score;
		posef obj_world_pose, obj_cam_pose;
		bool match_all_blobs = (dev->id == 0); /* Let the HMD match whatever it can */

		obj_world_pose = frame->capture_world_poses[d];

		LOGV ("Fusion provided pose for device %d, %f %f %f %f pos %f %f %f",
			dev->id, obj_world_pose.orient.x, obj_world_pose.orient.y, obj_world_pose.orient.z, obj_world_pose.orient.w,
			obj_world_pose.pos.x, obj_world_pose.pos.y, obj_world_pose.pos.z);

		/* If we have a camera pose, get the object's camera-relative pose by taking
		 * our camera pose (camera->world) and applying inverted to the
		 * the fusion pose (object->world) - which goes object->world->camera.
		 * If there's no camera pose, things won't match and the correspondence search
		 * will do a full search, so it doesn't matter what we feed as the initial pose */
		if (ctx->have_camera_pose) {
			oposef_apply_inverse(&obj_world_pose, &ctx->camera_pose, &obj_cam_pose);
		}
		else {
			obj_cam_pose = obj_world_pose;
		}

		LOGD ("Sensor %d searching for matching pose for device %d, initial quat %f %f %f %f pos %f %f %f",
			ctx->id, dev->id, obj_cam_pose.orient.x, obj_cam_pose.orient.y, obj_cam_pose.orient.z, obj_cam_pose.orient.w,
			obj_cam_pose.pos.x, obj_cam_pose.pos.y, obj_cam_pose.pos.z);

		rift_evaluate_pose (&score, &obj_cam_pose,
			ctx->bwobs->blobs, ctx->bwobs->num_blobs,
			dev->id, dev->leds->points, dev->leds->num_points,
			&ctx->camera_matrix, ctx->dist_coeffs, ctx->dist_fisheye, NULL);

		if (score.good_pose_match)
			LOGD("Sensor %d already had good pose match for device %d matched %u blobs of %u",
				ctx->id, dev->id, score.matched_blobs, score.visible_leds);
		else
			LOGD("Sensor %d needs correspondence search for device %d matched %u blobs of %u",
				ctx->id, dev->id, score.matched_blobs, score.visible_leds);

		if (score.good_pose_match || correspondence_search_find_one_pose (ctx->cs, dev->id, match_all_blobs, &obj_cam_pose, &score)) {
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

				rift_mark_matching_blobs (&obj_cam_pose, bwobs->blobs, bwobs->num_blobs, dev->id,
						dev->leds->points, dev->leds->num_points,
						camera_matrix, dist_coeffs, ctx->is_cv1);

				/* Refine the pose with PnP now that we've labelled the blobs */
				estimate_initial_pose (ctx->bwobs->blobs, ctx->bwobs->num_blobs,
					dev->id, dev->leds->points, dev->leds->num_points, camera_matrix,
					dist_coeffs, ctx->is_cv1,
					&obj_cam_pose, NULL, NULL, true);

				kalman_pose_update (ctx->pose_filter, frame->uvc.start_ts, &obj_cam_pose.pos, &obj_cam_pose.orient);
				kalman_pose_get_estimated (ctx->pose_filter, &dev->pose.pos, &dev->pose.orient);

				LOGD ("sensor %d kalman filter for device %d yielded quat %f %f %f %f pos %f %f %f",
					ctx->id, dev->id, dev->pose.orient.x, dev->pose.orient.y, dev->pose.orient.z, dev->pose.orient.w,
					dev->pose.pos.x, dev->pose.pos.y, dev->pose.pos.z);

				update_device_pose (ctx, dev, &score, frame->uvc.start_ts, &frame->capture_world_poses[d]);
			}
		}
	}

	return ret;
}

bool
rift_sensor_add_device(rift_sensor_ctx *sensor, rift_tracked_device *device)
{
	bool ret;

	ohmd_lock_mutex(sensor->sensor_lock);
	assert (sensor->n_devices < RIFT_MAX_TRACKED_DEVICES);

	ret	= correspondence_search_set_model (sensor->cs, device->id, device->led_search);

	if (ret) {
		sensor->devices[sensor->n_devices] = device;
		sensor->n_devices++;
	}

	ohmd_unlock_mutex(sensor->sensor_lock);

	return ret;
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

#if 0
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
#endif

static void new_frame_start_cb(struct rift_sensor_uvc_stream *stream, uint64_t start_time)
{
	rift_sensor_ctx *sensor = stream->user_data;
	uint64_t phase_ts;
	uint8_t led_pattern_phase = rift_tracker_get_led_pattern_phase (sensor->tracker, &phase_ts);

	LOGD("%f ms Sensor %d SOF phase %d", (double) (start_time) / 1000000.0,
			sensor->id, led_pattern_phase);

	ohmd_lock_mutex(sensor->sensor_lock);
	if (sensor->next_capture_frame != sensor->cur_capture_frame) {
		int d;
		rift_sensor_capture_frame *next_frame = sensor->frames + sensor->next_capture_frame;

		sensor->cur_capture_frame = sensor->next_capture_frame;

		next_frame->led_pattern_phase = led_pattern_phase;
		next_frame->led_pattern_sof_ts = phase_ts;

		if (sensor->dropped_frames) {
				LOGW("Sensor %d dropped %d frames", sensor->id, sensor->dropped_frames);
				sensor->dropped_frames = 0;
		}

		for (d = 0; d < sensor->n_devices; d++) {
			rift_tracked_device *dev = sensor->devices[d];

			/* FIXME: Thread safety around the fusion */
			posef tmp;
			oposef_init(&tmp, &dev->fusion->world_position, &dev->fusion->orient);

			if (dev->id == 0) {
				/* Mirror the pose in XZ to go from view-plane to device axes */
				oposef_mirror_XZ(&tmp);
			}

			/* Apply any needed global pose change */
			oposef_apply(&tmp, &dev->fusion_to_model, &tmp);
			next_frame->capture_world_poses[d] = tmp;
		}
		next_frame->n_devices = sensor->n_devices;

		rift_sensor_uvc_stream_set_frame(stream, (rift_sensor_uvc_frame *)next_frame);
	}
	ohmd_unlock_mutex(sensor->sensor_lock);
}

static void
update_device_pose (rift_sensor_ctx *sensor_ctx, rift_tracked_device *dev,
		rift_pose_metrics *score, uint64_t frame_ts, posef *capture_pose)
{
	posef pose = dev->pose;

	rift_evaluate_pose (score, &pose,
		sensor_ctx->bwobs->blobs, sensor_ctx->bwobs->num_blobs,
		dev->id, dev->leds->points, dev->leds->num_points,
		&sensor_ctx->camera_matrix, sensor_ctx->dist_coeffs,
		sensor_ctx->dist_fisheye, NULL);

	if (score->good_pose_match) {
		LOGV("Found good pose match - %u LEDs matched %u visible ones\n",
			score->matched_blobs, score->visible_leds);
		if (sensor_ctx->have_camera_pose) {
			/* The pose we found is the transform from object coords to camera-relative coords.
			 * Our camera pose stores the transform from camera to world, and what we
			 * need to give the fusion is transform from object->world.
			 *
			 * To get the transform from object->world, take object->camera pose, and apply
			 * the camera->world pose. */
			oposef_apply(&pose, &sensor_ctx->camera_pose, &pose);

			LOGD("Updating fusion for device %d pose quat %f %f %f %f  pos %f %f %f",
				dev->id, pose.orient.x, pose.orient.y, pose.orient.z, pose.orient.w,
				pose.pos.x, pose.pos.y, pose.pos.z);

			/* Undo any IMU to device conversion */
			oposef_apply_inverse(&pose, &dev->fusion_to_model, &pose);

			if (dev->id == 0) {
				/* Mirror the pose in XZ to go from device axes to view-plane */
				oposef_mirror_XZ(&pose);
			}

			ofusion_tracker_update (dev->fusion, (double)(frame_ts) / 1000000000.0, &pose.pos, &pose.orient);

#if 0
			oposef_init(&pose, &dev->fusion->world_position, &dev->fusion->orient);

			LOGD("After update, fusion for device %d pose quat %f %f %f %f  pos %f %f %f",
				dev->id, pose.orient.x, pose.orient.y, pose.orient.z, pose.orient.w,
				pose.pos.x, pose.pos.y, pose.pos.z);

			oposef_apply_inverse(&pose, &sensor_ctx->camera_pose, &pose);

			LOGD("Reversing the pose for device %d pose quat %f %f %f %f  pos %f %f %f",
				dev->id, pose.orient.x, pose.orient.y, pose.orient.z, pose.orient.w,
				pose.pos.x, pose.pos.y, pose.pos.z);
#endif
		}
		else if (dev->id == 0 && oquatf_get_length (&capture_pose->orient) > 0.9 && dev->fusion->last_gravity_vector_time > 0) {
			/* No camera pose yet. If this is the HMD, we had an IMU pose at capture time,
			 * and the fusion has a gravity vector from the IMU, use it to
			 * initialise the camera (world->camera) pose using the current headset pose.
			 * Calculate the xform from camera->world by applying
			 * the observed pose (object->camera), inverted (so camera->object) to our found
			 * fusion pose (object->world) to yield camera->world xform
			 *
			 * FIXME: Store the gravity vector in the fusion and record it with the capture too
			 * so we only do this calculation if the IMU had acquired a gravity vector
			 */
			posef camera_object_pose = pose;
			oposef_inverse(&camera_object_pose);

			oposef_apply(&camera_object_pose, capture_pose, &sensor_ctx->camera_pose);

			LOGI("Set sensor %d pose from device %d - tracker pose quat %f %f %f %f  pos %f %f %f"
					" fusion pose quat %f %f %f %f  pos %f %f %f yielded"
					" world->camera pose quat %f %f %f %f  pos %f %f %f",
				sensor_ctx->id, dev->id,
				pose.orient.x, pose.orient.y, pose.orient.z, pose.orient.w, pose.pos.x, pose.pos.y, pose.pos.z,
				capture_pose->orient.x, capture_pose->orient.y, capture_pose->orient.z, capture_pose->orient.w,
				capture_pose->pos.x, capture_pose->pos.y, capture_pose->pos.z,
				sensor_ctx->camera_pose.orient.x, sensor_ctx->camera_pose.orient.y, sensor_ctx->camera_pose.orient.z, sensor_ctx->camera_pose.orient.w,
				sensor_ctx->camera_pose.pos.x, sensor_ctx->camera_pose.pos.y, sensor_ctx->camera_pose.pos.z);

#if 0
			/* Double check the newly calculated camera pose can take the object->camera pose
			 * back to object->world pose */
			oposef_apply(&pose, &sensor_ctx->camera_pose, &pose);
			LOGI("New camera xform took observed pose to world quat %f %f %f %f  pos %f %f %f",
				pose.orient.x, pose.orient.y, pose.orient.z, pose.orient.w,
				pose.pos.x, pose.pos.y, pose.pos.z);
			/* Also double check the newly calculated camera->world pose can map from the fusion
			 * pose back to our object->camera pose */
			oposef_apply_inverse(&camera_object_pose, &sensor_ctx->camera_pose, &pose);

			LOGI("new camera xform took fusion pose back to %f %f %f %f  pos %f %f %f",
				pose.orient.x, pose.orient.y, pose.orient.z, pose.orient.w,
				pose.pos.x, pose.pos.y, pose.pos.z);
#endif

			sensor_ctx->have_camera_pose = true;
		}

	}
	else {
		LOGV("Failed pose match - only %u LEDs matched %u visible ones",
			score->matched_blobs, score->visible_leds);
	}
}

static void frame_captured_cb(rift_sensor_uvc_stream *stream, rift_sensor_uvc_frame *uvc_frame)
{
	rift_sensor_capture_frame *frame = (rift_sensor_capture_frame *)(uvc_frame);
	rift_sensor_ctx *sensor = stream->user_data;

	assert (sensor);

	uint64_t now = ohmd_monotonic_get(sensor->ohmd_ctx);

	frame->frame_delivered_ts = now;

	/* led pattern phase comes from sensor reports */
	uint8_t led_pattern_phase = frame->led_pattern_phase;
	uint64_t cur_phase_ts;
	uint8_t cur_led_pattern_phase = rift_tracker_get_led_pattern_phase (sensor->tracker, &cur_phase_ts);
	if (cur_led_pattern_phase != led_pattern_phase) {
		/* The LED phase changed mid-frame. Choose which one to keep */
		uint64_t frame_midpoint = uvc_frame->start_ts + (now - uvc_frame->start_ts)/2;
		uint8_t chosen_phase;

		if (cur_phase_ts < frame_midpoint) {
			chosen_phase = cur_led_pattern_phase;
		} else {
			chosen_phase = led_pattern_phase;
		}

		LOGV ("%f Sensor %d Frame (sof %f) phase mismatch old %d new %d (@ %f) chose %d",
			(double) (now) / 1000000.0, sensor->id,
			(double) (frame->led_pattern_sof_ts) / 1000000.0,
			led_pattern_phase, cur_led_pattern_phase,
			(double) (cur_phase_ts) / 1000000.0,
			chosen_phase);
		frame->led_pattern_phase = chosen_phase;
	}

	LOGD ("Sensor %d captured frame %d phase %d", sensor->id, frame->id, frame->led_pattern_phase);

	ohmd_lock_mutex(sensor->sensor_lock);
	/* Send this frame to the analysis thread */
	if (sensor->frame_for_fast_analysis == -1) {
		sensor->frame_for_fast_analysis = frame->id;
		sensor->next_capture_frame = (frame->id+1) % NUM_CAPTURE_BUFFERS;
		sensor->cur_capture_frame = -1;
		ohmd_cond_broadcast (sensor->new_frame_cond);
	} else {
		/* Analysis thread was still busy with the previous frame */
		sensor->dropped_frames++;
	}
	ohmd_unlock_mutex(sensor->sensor_lock);
}

static void analyse_frame_fast(rift_sensor_ctx *sensor, rift_sensor_capture_frame *frame)
{
	uint64_t now = ohmd_monotonic_get(sensor->ohmd_ctx);
	int width = frame->uvc.width;
	int height = frame->uvc.height;

	frame->image_analysis_start_ts = now;

	blobwatch_process(sensor->bw, frame->uvc.data, width, height,
		frame->led_pattern_phase, NULL, 0, &sensor->bwobs);

#if LOGLEVEL < 1
	uint64_t blob_extract_end = ohmd_monotonic_get(sensor->ohmd_ctx);
#endif

	if (sensor->bwobs && sensor->bwobs->num_blobs > 0) {
		tracker_process_blobs (sensor, frame);
#if 0
		printf("Sensor %d phase %d Blobs: %d\n", sensor->id, led_pattern_phase, sensor->bwobs->num_blobs);

		for (int index = 0; index < sensor->bwobs->num_blobs; index++)
		{
			printf("Sensor %d Blob[%d]: %d,%d %dx%d id %d pattern %x age %u\n", sensor->id,
				index,
				sensor->bwobs->blobs[index].x,
				sensor->bwobs->blobs[index].y,
				sensor->bwobs->blobs[index].width,
				sensor->bwobs->blobs[index].height,
				sensor->bwobs->blobs[index].led_id,
				sensor->bwobs->blobs[index].pattern,
				sensor->bwobs->blobs[index].pattern_age);
		}
#endif
	}

	now = ohmd_monotonic_get(sensor->ohmd_ctx);
	frame->image_analysis_finish_ts = now;

	if (ohmd_pw_video_stream_connected(sensor->debug_vid)) {
		rift_debug_draw_frame (sensor->debug_frame, sensor->bwobs, sensor->cs, frame,
			frame->n_devices, sensor->devices, sensor->is_cv1, sensor->camera_matrix,
			sensor->dist_fisheye, sensor->dist_coeffs, &sensor->camera_pose);
		ohmd_pw_video_stream_push (sensor->debug_vid, frame->uvc.start_ts, sensor->debug_frame);
	}
#if 0
	if (ohmd_pw_debug_stream_connected(sensor->debug_metadata)) {
		char *debug_str;

		asprintf (&debug_str, "{ ts: %llu, exposure_phase: %d, position: { %f, %f, %f }, orientation: { %f, %f, %f, %f } }",
		  (unsigned long long) frame->uvc.start_ts, led_pattern_phase,
		  sensor->pose.pos.x, sensor->pose.pos.y, sensor->pose.pos.z,
		  sensor->pose.orient.x, sensor->pose.orient.y,
		  sensor->pose.orient.z, sensor->pose.orient.w);

		ohmd_pw_debug_stream_push (sensor->debug_metadata, frame->uvc.start_ts, debug_str);
		free(debug_str);
  }
#endif

#if LOGLEVEL < 1
	double time_since_last_capture = 0;
	if (sensor->prev_capture_ts) {
		time_since_last_capture = (double)(frame->uvc.start_ts - sensor->prev_capture_ts) / 1000000.0;
	}
	sensor->prev_capture_ts = frame->uvc.start_ts;
#endif

	LOGD("Sensor %d Frame analysis done after %u ms. Captured %" PRIu64 " (%.1fms between frames) USB delivery %u ms thread switch %u ns analysis %ums (%ums blob extraction)",
		 sensor->id, (uint32_t) (frame->image_analysis_finish_ts - frame->uvc.start_ts) / 1000000,
		 frame->uvc.start_ts, time_since_last_capture,
		 (uint32_t) (frame->frame_delivered_ts - frame->uvc.start_ts) / 1000000,
		 (uint32_t) (frame->image_analysis_start_ts - frame->frame_delivered_ts),
		 (uint32_t) (frame->image_analysis_finish_ts - frame->image_analysis_start_ts) / 1000000,
		 (uint32_t) (blob_extract_end - frame->image_analysis_start_ts) / 1000000);
}

static unsigned int fast_analysis_thread(void *arg);
static unsigned int long_analysis_thread(void *arg);

rift_sensor_ctx *
rift_sensor_new(ohmd_context* ohmd_ctx, int id, const char *serial_no,
	libusb_context *usb_ctx, libusb_device_handle *usb_devh, rift_tracker_ctx *tracker, const uint8_t radio_id[5])
{
	rift_sensor_ctx *sensor_ctx = NULL;
	char stream_id[64];
	int ret, i;

	libusb_device *dev = libusb_get_device(usb_devh);
	struct libusb_device_descriptor desc;
	ret = libusb_get_device_descriptor(dev, &desc);
	if (ret < 0) {
		printf ("Failed to read device descriptor!\n");
		return NULL;
	}

	sensor_ctx = ohmd_alloc(ohmd_ctx, sizeof (rift_sensor_ctx));

	sensor_ctx->ohmd_ctx = ohmd_ctx;
	sensor_ctx->id = id;
	sensor_ctx->is_cv1 = (desc.idProduct == CV1_PID);
	strcpy ((char *) sensor_ctx->serial_no, (char *) serial_no);
	sensor_ctx->tracker = tracker;
	sensor_ctx->usb_devh = usb_devh;

	sensor_ctx->pose_filter = kalman_pose_new (ohmd_ctx);

	sensor_ctx->stream.sof_cb = new_frame_start_cb;
	sensor_ctx->stream.frame_cb = frame_captured_cb;
	sensor_ctx->stream.user_data = sensor_ctx;
	
	sensor_ctx->have_camera_pose = false;

	printf ("Found Rift Sensor %d w/ Serial %s. Connecting to Radio address 0x%02x%02x%02x%02x%02x\n",
		id, serial_no, radio_id[0], radio_id[1], radio_id[2], radio_id[3], radio_id[4]);

	ret = rift_sensor_uvc_stream_setup (usb_ctx, sensor_ctx->usb_devh, &sensor_ctx->stream);
	ASSERT_MSG(ret >= 0, fail, "could not prepare for streaming\n");

	/* Allocate capture frame buffers */
	int data_size = sensor_ctx->stream.frame_size;

	sensor_ctx->frames = ohmd_alloc(ohmd_ctx, NUM_CAPTURE_BUFFERS * sizeof (rift_sensor_capture_frame));
	for (i = 0; i < NUM_CAPTURE_BUFFERS; i++) {
			rift_sensor_capture_frame *frame = sensor_ctx->frames + i;

			frame->id = i;
			frame->uvc.data = ohmd_alloc(ohmd_ctx, data_size);
			frame->uvc.data_size = data_size;
	}
	/* Set up frame capture references */
	sensor_ctx->next_capture_frame = 0;
	sensor_ctx->cur_capture_frame = -1;
	sensor_ctx->frame_for_fast_analysis = -1;
	sensor_ctx->dropped_frames = 0;

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

	/* Start analysis threads */
	sensor_ctx->sensor_lock = ohmd_create_mutex(ohmd_ctx);
	sensor_ctx->new_frame_cond = ohmd_create_cond(ohmd_ctx);

	sensor_ctx->shutdown = false;
	sensor_ctx->fast_analysis_thread = ohmd_create_thread (ohmd_ctx, fast_analysis_thread, sensor_ctx);
	sensor_ctx->long_analysis_thread = ohmd_create_thread (ohmd_ctx, long_analysis_thread, sensor_ctx);

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

void
rift_sensor_free (rift_sensor_ctx *sensor_ctx)
{
	int i;

	if (sensor_ctx == NULL)
		return;

	if (sensor_ctx->bw)
			blobwatch_free (sensor_ctx->bw);
		
	if (sensor_ctx->stream_started)
		rift_sensor_uvc_stream_stop(&sensor_ctx->stream);

	/* Shut down analysis threads */
	ohmd_lock_mutex(sensor_ctx->sensor_lock);
	ohmd_cond_broadcast(sensor_ctx->new_frame_cond);
	ohmd_unlock_mutex(sensor_ctx->sensor_lock);

	ohmd_destroy_thread(sensor_ctx->fast_analysis_thread);
	ohmd_destroy_thread(sensor_ctx->long_analysis_thread);

	ohmd_destroy_mutex(sensor_ctx->sensor_lock);
	ohmd_destroy_cond(sensor_ctx->new_frame_cond);

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

	for (i = 0; i < NUM_CAPTURE_BUFFERS; i++) {
			rift_sensor_capture_frame *frame = sensor_ctx->frames + i;
			if (frame->uvc.data)
				free(frame->uvc.data);
	}
	free(sensor_ctx->frames);

	free (sensor_ctx);
}

static unsigned int long_analysis_thread(void *arg)
{
	rift_sensor_ctx *ctx = arg;

	ohmd_lock_mutex (ctx->sensor_lock);
	while (!ctx->shutdown) {
			ohmd_cond_wait(ctx->new_frame_cond, ctx->sensor_lock);
	}
	ohmd_unlock_mutex (ctx->sensor_lock);

	return 0;
}

static unsigned int fast_analysis_thread(void *arg)
{
	rift_sensor_ctx *ctx = arg;

	ohmd_lock_mutex (ctx->sensor_lock);
	while (!ctx->shutdown) {
			if (ctx->frame_for_fast_analysis != -1) {
				rift_sensor_capture_frame *frame = ctx->frames + ctx->frame_for_fast_analysis;

				ohmd_unlock_mutex (ctx->sensor_lock);
				analyse_frame_fast(ctx, frame);
				ohmd_lock_mutex (ctx->sensor_lock);

				/* Done with this frame - clear the ref */
				ctx->frame_for_fast_analysis = -1;
			}
			if (!ctx->shutdown) {
				ohmd_cond_wait(ctx->new_frame_cond, ctx->sensor_lock);
			}
	}
	ohmd_unlock_mutex (ctx->sensor_lock);

	return 0;
}
