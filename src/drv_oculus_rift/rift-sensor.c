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

#define ASSERT_MSG(_v, label, ...) if(!(_v)){ fprintf(stderr, __VA_ARGS__); goto label; }

/* We need 4 capture buffers:
 * 	1 to be capturing into
 * 	1 captured, in the fast analysis thread
 * 	1 possibly undergoing long analysis
 * 	1 pending long analysis
 */
#define NUM_CAPTURE_BUFFERS 4

#define QUEUE_SIZE (NUM_CAPTURE_BUFFERS+1)

typedef struct rift_sensor_frame_queue rift_sensor_frame_queue;

struct rift_sensor_frame_queue {
	rift_sensor_capture_frame *data[QUEUE_SIZE];
	unsigned int head, tail;
};

#define INIT_QUEUE(q) \
		(q)->head = (q)->tail = 0;

#define PUSH_QUEUE(q,f) do { \
	unsigned int next = ((q)->tail+1) % QUEUE_SIZE; \
	assert(next != (q)->head); /* Check there's room */ \
	assert ((f) != NULL); \
	(q)->data[(q)->tail] = (f); \
	(q)->tail = next; \
} while(0)

static rift_sensor_capture_frame *POP_QUEUE(rift_sensor_frame_queue *q) {
	rift_sensor_capture_frame *f;
	unsigned int next_head = (q->head+1) % QUEUE_SIZE;

	if ((q)->tail == (q)->head) /* Check there's something in the queue */
		return NULL;

	f = q->data[q->head];
	q->head = next_head;

	return f;
}

/* Rewind the queue and un-push the last element */
static rift_sensor_capture_frame *REWIND_QUEUE(rift_sensor_frame_queue *q) {
	rift_sensor_capture_frame *f;
	unsigned int prev_tail = (q->tail-1);

	if (prev_tail >= QUEUE_SIZE)
		prev_tail = QUEUE_SIZE-1;

	if (q->tail == q->head) /* Check there's something in the queue */
		return NULL;

	f = q->data[prev_tail];
	q->tail = prev_tail;

	return f;
}

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

	blobwatch* bw;

	dmat3 camera_matrix;
	bool dist_fisheye;
	double dist_coeffs[5];

	correspondence_search_t *cs;

	vec3f led_out_points[MAX_OBJECT_LEDS];

	ohmd_mutex *sensor_lock;
	ohmd_cond *new_frame_cond;

	/* Protected by sensor_lock */
	rift_tracked_device *devices[RIFT_MAX_TRACKED_DEVICES];
	uint8_t n_devices;

	rift_sensor_capture_frame *cur_capture_frame;

	/* Queue of frames being returned to the capture thread */
	rift_sensor_frame_queue capture_frame_q;
	int dropped_frames;
	/* queue of frames awaiting fast analysis */
	rift_sensor_frame_queue fast_analysis_q;
	/* queue of frames awaiting long analysis */
	rift_sensor_frame_queue long_analysis_q;

	int shutdown;

	bool long_analysis_busy;

	/* End protected by sensor_lock */

	ohmd_thread* fast_analysis_thread;
	ohmd_thread* long_analysis_thread;

	/* Pipewire output streams */
	ohmd_pw_video_stream *debug_vid_raw;
	ohmd_pw_video_stream *debug_vid;
	uint8_t *debug_frame;

	ohmd_gst_video_stream *debug_vid_raw_gst;

	/* Updated from fast_analysis_thread */
	bool have_camera_pose;
	posef camera_pose;

	uint64_t prev_capture_ts;
};

static void update_device_pose (rift_sensor_ctx *sensor_ctx, rift_tracked_device *dev,
	rift_sensor_capture_frame *frame, rift_sensor_frame_device_state *dev_state);
static void update_device_and_blobs (rift_sensor_ctx *ctx, rift_sensor_capture_frame *frame,
	rift_tracked_device *dev, rift_sensor_frame_device_state *dev_state, posef *obj_cam_pose);

static void tracker_process_blobs_fast(rift_sensor_ctx *ctx, rift_sensor_capture_frame *frame)
{
	const rift_tracker_exposure_info *exposure_info = &frame->exposure_info;
	blobservation* bwobs = frame->bwobs;
	dmat3 *camera_matrix = &ctx->camera_matrix;
	double *dist_coeffs = ctx->dist_coeffs;
	int d;

	/* Only process the devices that were available when this frame was captured */
	for (d = 0; d < frame->n_devices; d++) {
		rift_tracked_device *dev = ctx->devices[d];
		rift_sensor_frame_device_state *dev_state = frame->capture_state + d;
		const rift_tracked_device_exposure_info *exp_dev_info = exposure_info->devices + d;
		posef obj_world_pose, obj_cam_pose;

		if (exp_dev_info->fusion_slot == -1) {
			LOGV ("Skipping fast analysis of device %d. No fusion slot assigned\n", d);
			continue;
		}

		obj_world_pose = dev_state->capture_world_pose;

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

		LOGD ("Sensor %d Frame %d searching for matching pose for device %d, initial quat %f %f %f %f pos %f %f %f",
			ctx->id, frame->id, dev->id,
			obj_cam_pose.orient.x, obj_cam_pose.orient.y, obj_cam_pose.orient.z, obj_cam_pose.orient.w,
			obj_cam_pose.pos.x, obj_cam_pose.pos.y, obj_cam_pose.pos.z);

		dev_state->final_cam_pose = obj_cam_pose;

		rift_evaluate_pose_with_prior(&dev_state->score, &obj_cam_pose,
			&obj_cam_pose, &exp_dev_info->pos_error, &exp_dev_info->rot_error,
			frame->bwobs->blobs, frame->bwobs->num_blobs,
			dev->id, dev->leds->points, dev->leds->num_points,
			&ctx->camera_matrix, ctx->dist_coeffs, ctx->dist_fisheye, NULL);

		if (dev_state->score.good_pose_match)
			LOGD("Sensor %d already had good pose match for device %d matched %u blobs of %u",
				ctx->id, dev->id, dev_state->score.matched_blobs, dev_state->score.visible_leds);
		else {
			int num_blobs = 0;

			/* See if we still have enough labelled blobs to try to re-acquire the pose without a
			 * full search */
			for (int index = 0; index < bwobs->num_blobs; index++) {
				struct blob *b = bwobs->blobs + index;
				if (LED_OBJECT_ID (b->led_id) == dev->id) {
						num_blobs++;
				}
			}

			if (num_blobs > 4) {
				estimate_initial_pose (bwobs->blobs, bwobs->num_blobs,
					dev->id, dev->leds->points, dev->leds->num_points, camera_matrix,
					dist_coeffs, ctx->is_cv1,
					&obj_cam_pose, NULL, NULL, true);
				rift_evaluate_pose_with_prior(&dev_state->score, &obj_cam_pose,
					&dev_state->final_cam_pose, &exp_dev_info->pos_error, &exp_dev_info->rot_error,
					bwobs->blobs, bwobs->num_blobs,
					dev->id, dev->leds->points, dev->leds->num_points,
					&ctx->camera_matrix, ctx->dist_coeffs, ctx->dist_fisheye, NULL);

				if (dev_state->score.good_pose_match) {
					LOGD("Sensor %d re-acquired match for device %d matched %u blobs of %u",
						ctx->id, dev->id, dev_state->score.matched_blobs, dev_state->score.visible_leds);
				}
			}
		}

		if (dev_state->score.good_pose_match) {
			update_device_and_blobs (ctx, frame, dev, dev_state, &obj_cam_pose);
		} else {
			/* Didn't find this device - send the frame for long analysis */
			LOGD("Sensor %d frame %d needs full search for device %d - sending to long analysis thread",
				ctx->id, frame->id, dev->id);
			frame->need_long_analysis = true;
			continue;
		}
	}
}

static void tracker_process_blobs_long(rift_sensor_ctx *ctx, rift_sensor_capture_frame *frame)
{
	const rift_tracker_exposure_info *exposure_info = &frame->exposure_info;
	blobservation* bwobs = frame->bwobs;
	int d, pass;
	vec3f gravity_vector = {{ 0.0, 1.0, 0.0 }};
	bool had_strong_matches = false;

	LOGD("Sensor %d Frame %d - starting long search for devices", ctx->id, frame->id);

	correspondence_search_set_blobs (ctx->cs, bwobs->blobs, bwobs->num_blobs);

	if (ctx->have_camera_pose) {
		quatf cam_orient = ctx->camera_pose.orient;

		oquatf_inverse(&cam_orient);
		oquatf_get_rotated(&cam_orient, &gravity_vector, &gravity_vector);
	}

	for (pass = 0; pass < 2; pass++) {
		/* Only process the devices that were available when this frame was captured */
		for (d = 0; d < frame->n_devices; d++) {
			rift_tracked_device *dev = ctx->devices[d];
			rift_sensor_frame_device_state *dev_state = frame->capture_state + d;
			const rift_tracked_device_exposure_info *exp_dev_info = exposure_info->devices + d;
			posef obj_cam_pose;
			bool do_aligned_checks;
			CorrespondenceSearchFlags flags = CS_FLAG_STOP_FOR_STRONG_MATCH;

			if (dev_state->found_device_pose)
				continue; /* We already found a pose for this device */

			if (dev->id == 0)
				flags |= CS_FLAG_MATCH_ALL_BLOBS; /* Let the HMD match whatever it can */

			if (pass == 0)
				flags |= CS_FLAG_SHALLOW_SEARCH;
			else
				flags |= CS_FLAG_DEEP_SEARCH;

			if (exp_dev_info->fusion_slot == -1) {
				LOGV ("Skipping long analysis of device %d. No fusion slot assigned\n", d);
				continue;
			}

			/* If the gravity vector error standard deviation is small enough, try for an aligned pose from the prior,
			 * within 2 standard deviation */
			do_aligned_checks = (ctx->have_camera_pose && dev_state->gravity_error_rad < DEG_TO_RAD(45));
			if (do_aligned_checks)
				oposef_apply_inverse(&dev_state->capture_world_pose, &ctx->camera_pose, &obj_cam_pose);

			if (dev_state->score.good_pose_match && had_strong_matches) {
				/* We have a good pose match for this device, that we found on the first
				 * pass. If any other device found a strong match though, then that may
				 * have claimed blobs we were relying on  - so re-check our pose and possibly start again */
				if (do_aligned_checks) {
					rift_evaluate_pose_with_prior(&dev_state->score, &obj_cam_pose,
						&dev_state->final_cam_pose, &exp_dev_info->pos_error, &exp_dev_info->rot_error,
						bwobs->blobs, bwobs->num_blobs,
						dev->id, dev->leds->points, dev->leds->num_points,
						&ctx->camera_matrix, ctx->dist_coeffs, ctx->dist_fisheye, NULL);
				}
				else {
					rift_evaluate_pose (&dev_state->score, &obj_cam_pose,
						frame->bwobs->blobs, frame->bwobs->num_blobs,
						dev->id, dev->leds->points, dev->leds->num_points,
						&ctx->camera_matrix, ctx->dist_coeffs,
						ctx->dist_fisheye, NULL);
				}

				if (!dev_state->score.good_pose_match)
					flags |= CS_FLAG_SHALLOW_SEARCH;
			}

			if (flags & CS_FLAG_DEEP_SEARCH) {
				LOGD("Sensor %d long search for device %d\n", ctx->id, dev->id);
			}

			if (do_aligned_checks) {
#if LOGLEVEL == 0
				quatf ref_orient = obj_cam_pose.orient;
#endif
				quatf pose_gravity_swing, pose_gravity_twist;
				float pose_tolerance = OHMD_MAX(2 * dev_state->gravity_error_rad, DEG_TO_RAD(10));

				oquatf_decompose_swing_twist(&obj_cam_pose.orient, &gravity_vector, &pose_gravity_swing, &pose_gravity_twist);
				if (correspondence_search_find_one_pose_aligned (ctx->cs, dev->id, flags, &obj_cam_pose,
						&gravity_vector, &pose_gravity_swing, pose_tolerance, &dev_state->score)) {
					LOGD("Got aligned pose %f, %f, %f, %f (to %f, %f, %f, %f) for device %d with tolerance %f!",
						obj_cam_pose.orient.x, obj_cam_pose.orient.y, obj_cam_pose.orient.z, obj_cam_pose.orient.w,
						ref_orient.x, ref_orient.y, ref_orient.z, ref_orient.w,
						d, RAD_TO_DEG(pose_tolerance));
				}
				else {
					LOGD("No aligned pose (to %f, %f, %f, %f) for device %d with tolerance %f!",
						ref_orient.x, ref_orient.y, ref_orient.z, ref_orient.w,
						d, RAD_TO_DEG(pose_tolerance));
				}
			} else {
				correspondence_search_find_one_pose (ctx->cs, dev->id, flags, &obj_cam_pose, &dev_state->score);
			}

			LOGV("Sensor %d Frame %d - doing long search for device %d matched %u blobs of %u (%s match)",
				ctx->id, frame->id, dev->id, dev_state->score.matched_blobs, dev_state->score.visible_leds,
				dev_state->score.good_pose_match ? "good" : "bad");

			/* Require a strong pose match on the quick loop */
			if (pass == 0 && !dev_state->score.strong_pose_match)
				continue;

			if (dev_state->score.good_pose_match) {
				had_strong_matches |= dev_state->score.strong_pose_match;

				update_device_and_blobs (ctx, frame, dev, dev_state, &obj_cam_pose);
				frame->long_analysis_found_new_blobs = true;

				/* Transfer these blob labels to the blobwatch object */
				ohmd_lock_mutex(ctx->sensor_lock);
				blobwatch_update_labels (ctx->bw, frame->bwobs, dev->id);
				ohmd_unlock_mutex(ctx->sensor_lock);
			}
		}
	}
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

/* Called with sensor_lock held. Releases a frame back to the capture thread */
static void
release_capture_frame(rift_sensor_ctx *sensor, rift_sensor_capture_frame *frame)
{
	uint64_t now = ohmd_monotonic_get(sensor->ohmd_ctx);
	LOGD("Sensor %d Frame %d analysis done after %u ms. Captured %" PRIu64 " USB delivery %u ms fast: queued %f ms analysis %u ms (%ums blob extraction) long: queued %f ms analysis %u ms",
		 sensor->id, frame->id,
		 (uint32_t) (now - frame->uvc.start_ts) / 1000000,
		 frame->uvc.start_ts,
		 (uint32_t) (frame->frame_delivered_ts - frame->uvc.start_ts) / 1000000,
		 (double) (frame->image_analysis_start_ts - frame->frame_delivered_ts) / 1000000.0,
		 (uint32_t) (frame->image_analysis_finish_ts - frame->image_analysis_start_ts) / 1000000,
		 (uint32_t) (frame->blob_extract_finish_ts - frame->image_analysis_start_ts) / 1000000,
		 (double) (frame->long_analysis_start_ts - frame->image_analysis_finish_ts) / 1000000.0,
		 (uint32_t) (frame->long_analysis_finish_ts - frame->long_analysis_start_ts) / 1000000);

	rift_tracker_frame_release (sensor->tracker, now, frame->uvc.start_ts, frame->exposure_info_valid ? &frame->exposure_info : NULL, sensor->serial_no);

	if (frame->bwobs) {
		blobwatch_release_observation(sensor->bw, frame->bwobs);
		frame->bwobs = NULL;
	}
	PUSH_QUEUE(&sensor->capture_frame_q, frame);
}

static void new_frame_start_cb(struct rift_sensor_uvc_stream *stream, uint64_t start_time)
{
	rift_sensor_ctx *sensor = stream->user_data;
	rift_tracker_exposure_info exposure_info;
	bool exposure_info_valid;
	rift_sensor_capture_frame *next_frame;
	bool release_old_frame = false;
	uint64_t old_frame_ts;
	rift_tracker_exposure_info old_exposure_info;

	exposure_info_valid = rift_tracker_get_exposure_info (sensor->tracker, &exposure_info);

	if (exposure_info_valid) {
		LOGD("%f ms Sensor %d SOF phase %d", (double) (start_time) / 1000000.0,
			sensor->id, exposure_info.led_pattern_phase);
	}
	else {
		LOGD("%f ms Sensor %d SOF no phase info", (double) (start_time) / 1000000.0, sensor->id);
	}

	ohmd_lock_mutex(sensor->sensor_lock);
	if (sensor->cur_capture_frame != NULL) {
		/* Previous frame never completed - some USB problem,
		 * just reuse it (but update all the state for a new
		 * timestamp)
		 */
		next_frame = sensor->cur_capture_frame;

		/* This frame was announced as started, make sure to announce its release */
		release_old_frame = true;
		old_frame_ts = next_frame->uvc.start_ts;
		old_exposure_info = next_frame->exposure_info;
	}
	else {
		next_frame = POP_QUEUE(&sensor->capture_frame_q);
	}

	if (next_frame != NULL) {
		if (sensor->dropped_frames) {
				LOGW("Sensor %d dropped %d frames", sensor->id, sensor->dropped_frames);
				sensor->dropped_frames = 0;
		}
		LOGD("Sensor %d starting capture into frame %d", sensor->id, next_frame->id);
	}
	else {
		/* No frames available from the analysis threads yet - try
		 * to recover the most recent one we sent and reuse it. This must
		 * succeed, or else there's not enough capture frames in circulation */
		next_frame = REWIND_QUEUE(&sensor->fast_analysis_q);
		assert (next_frame != NULL);
		LOGD("Sensor %d reclaimed frame %d from fast analysis for capture", sensor->id, next_frame->id);
		sensor->dropped_frames++;

		/* This frame was announced as started, make sure to announce its release */
		release_old_frame = true;
		old_frame_ts = next_frame->uvc.start_ts;
		old_exposure_info = next_frame->exposure_info;
	}

	next_frame->exposure_info = exposure_info;
	next_frame->exposure_info_valid = exposure_info_valid;

	sensor->cur_capture_frame = next_frame;
	rift_sensor_uvc_stream_set_frame(stream, (rift_sensor_uvc_frame *)next_frame);
	ohmd_unlock_mutex(sensor->sensor_lock);

	if (release_old_frame)
		rift_tracker_frame_release (sensor->tracker, start_time, old_frame_ts, &old_exposure_info, sensor->serial_no);
	rift_tracker_frame_start (sensor->tracker, start_time, sensor->serial_no, exposure_info_valid ? &exposure_info : NULL);
}

static void
update_device_and_blobs (rift_sensor_ctx *ctx, rift_sensor_capture_frame *frame,
	rift_tracked_device *dev, rift_sensor_frame_device_state *dev_state, posef *obj_cam_pose)
{
	blobservation* bwobs = frame->bwobs;
	dmat3 *camera_matrix = &ctx->camera_matrix;
	double *dist_coeffs = ctx->dist_coeffs;

	/* Clear existing blob IDs for this device, then
	 * back project LED ids into blobs if we find them and the dot product
	 * shows them pointing strongly to the camera */
	for (int index = 0; index < bwobs->num_blobs; index++) {
		struct blob *b = bwobs->blobs + index;
		if (LED_OBJECT_ID (b->led_id) == dev->id) {
			b->prev_led_id = b->led_id;
			b->led_id = LED_INVALID_ID;
		}
	}

	rift_mark_matching_blobs (obj_cam_pose, bwobs->blobs, bwobs->num_blobs, dev->id,
			dev->leds->points, dev->leds->num_points,
			camera_matrix, dist_coeffs, ctx->is_cv1);

	/* Refine the pose with PnP now that we've labelled the blobs */
	estimate_initial_pose (bwobs->blobs, bwobs->num_blobs,
		dev->id, dev->leds->points, dev->leds->num_points, camera_matrix,
		dist_coeffs, ctx->is_cv1,
		obj_cam_pose, NULL, NULL, true);

	/* And label the blobs again in case we collected any more */
	rift_mark_matching_blobs (obj_cam_pose, bwobs->blobs, bwobs->num_blobs, dev->id,
			dev->leds->points, dev->leds->num_points,
			camera_matrix, dist_coeffs, ctx->is_cv1);

	dev_state->final_cam_pose.pos = obj_cam_pose->pos;
	dev_state->final_cam_pose.orient = obj_cam_pose->orient;

	LOGD ("sensor %d PnP for device %d yielded quat %f %f %f %f pos %f %f %f",
		ctx->id, dev->id, dev_state->final_cam_pose.orient.x, dev_state->final_cam_pose.orient.y, dev_state->final_cam_pose.orient.z, dev_state->final_cam_pose.orient.w,
		dev_state->final_cam_pose.pos.x, dev_state->final_cam_pose.pos.y, dev_state->final_cam_pose.pos.z);

	update_device_pose (ctx, dev, frame, dev_state);
}

static void
update_device_pose (rift_sensor_ctx *sensor_ctx, rift_tracked_device *dev,
	rift_sensor_capture_frame *frame, rift_sensor_frame_device_state *dev_state)
{
	posef pose = dev_state->final_cam_pose;
	posef *capture_pose = &dev_state->capture_world_pose;
	rift_pose_metrics *score = &dev_state->score;

	rift_evaluate_pose (score, &pose,
		frame->bwobs->blobs, frame->bwobs->num_blobs,
		dev->id, dev->leds->points, dev->leds->num_points,
		&sensor_ctx->camera_matrix, sensor_ctx->dist_coeffs,
		sensor_ctx->dist_fisheye, NULL);

	if (score->good_pose_match) {
		LOGV("Found good pose match - %u LEDs matched %u visible ones\n",
			score->matched_blobs, score->visible_leds);

		if (sensor_ctx->have_camera_pose) {
			uint64_t now = ohmd_monotonic_get(sensor_ctx->ohmd_ctx);

			/* The pose we found is the transform from object coords to camera-relative coords.
			 * Our camera pose stores the transform from camera to world, and what we
			 * need to give the fusion is transform from object->world.
			 *
			 * To get the transform from object->world, take object->camera pose, and apply
			 * the camera->world pose. */
			oposef_apply(&pose, &sensor_ctx->camera_pose, &pose);

			LOGD("TS %llu Updating fusion for device %d pose quat %f %f %f %f  pos %f %f %f",
				(unsigned long long)(now),
				dev->id, pose.orient.x, pose.orient.y, pose.orient.z, pose.orient.w,
				pose.pos.x, pose.pos.y, pose.pos.z);

			rift_tracked_device_model_pose_update(dev, now, frame->uvc.start_ts, &frame->exposure_info, &pose, sensor_ctx->serial_no);
			rift_tracked_device_frame_release(dev, &frame->exposure_info);
			dev_state->found_device_pose = true;

#if 0
		  rift_tracked_device_get_model_pose(dev, (double) (frame->uvc.start_ts) / 1000000000.0, &pose, NULL);

			LOGD("After update, fusion for device %d pose quat %f %f %f %f  pos %f %f %f",
				dev->id, pose.orient.x, pose.orient.y, pose.orient.z, pose.orient.w,
				pose.pos.x, pose.pos.y, pose.pos.z);

			oposef_apply_inverse(&pose, &sensor_ctx->camera_pose, &pose);

			LOGD("Reversing the pose for device %d pose quat %f %f %f %f  pos %f %f %f",
				dev->id, pose.orient.x, pose.orient.y, pose.orient.z, pose.orient.w,
				pose.pos.x, pose.pos.y, pose.pos.z);
#endif
		}
		/* Arbitrary 15 degree threshold for gravity vector as a random magic number */
		else if (dev->id == 0 && oquatf_get_length (&capture_pose->orient) > 0.9 && dev_state->gravity_error_rad < DEG_TO_RAD(15.0)) {
			/* No camera pose yet. If this is the HMD, we had an IMU pose at capture time,
			 * and the fusion has a good gravity vector from the IMU, use it to
			 * initialise the camera (world->camera) pose using the current headset pose.
			 * Calculate the xform from camera->world by applying
			 * the observed pose (object->camera), inverted (so camera->object) to our found
			 * fusion pose (object->world) to yield camera->world xform
			 */
			posef camera_object_pose = pose;
			oposef_inverse(&camera_object_pose);

			oposef_apply(&camera_object_pose, capture_pose, &sensor_ctx->camera_pose);

			LOGI("Set sensor %d pose from device %d - tracker pose quat %f %f %f %f  pos %f %f %f"
					" fusion pose quat %f %f %f %f  pos %f %f %f gravity error %f degrees yielded"
					" world->camera pose quat %f %f %f %f  pos %f %f %f",
				sensor_ctx->id, dev->id,
				pose.orient.x, pose.orient.y, pose.orient.z, pose.orient.w, pose.pos.x, pose.pos.y, pose.pos.z,
				capture_pose->orient.x, capture_pose->orient.y, capture_pose->orient.z, capture_pose->orient.w,
				capture_pose->pos.x, capture_pose->pos.y, capture_pose->pos.z, RAD_TO_DEG(dev_state->gravity_error_rad),
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
		else if (dev->id == 0) {
			LOGD("No camera pose yet - gravity error is %f degrees\n", RAD_TO_DEG(dev_state->gravity_error_rad));
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
	const rift_tracker_exposure_info *exposure_info = &frame->exposure_info;
	rift_sensor_ctx *sensor = stream->user_data;
	int d;

	/* If there's no exposure info for this frame, we can't use it -
	 * just release it back to the capture thread */
	ohmd_lock_mutex (sensor->sensor_lock);

	/* The frame being returned must be the most recent one
	 * we sent to the UVC capture */
	assert(sensor->cur_capture_frame == frame);
	sensor->cur_capture_frame = NULL;

	if (!frame->exposure_info_valid)
		goto release_frame;

	assert(sensor);
	uint64_t now = ohmd_monotonic_get(sensor->ohmd_ctx);

	frame->frame_delivered_ts = now;

	rift_tracker_frame_captured (sensor->tracker, now, frame->uvc.start_ts, &frame->exposure_info, sensor->serial_no);

	LOGD ("Sensor %d captured frame %d exposure counter %u phase %d", sensor->id,
		frame->id, frame->exposure_info.count, frame->exposure_info.led_pattern_phase);

	for (d = 0; d < exposure_info->n_devices; d++) {
		rift_sensor_frame_device_state *dev_state = frame->capture_state + d;
		const rift_tracked_device_exposure_info *exp_dev_info = exposure_info->devices + d;
		const vec3f *rot_error = &exp_dev_info->rot_error;

		dev_state->capture_world_pose = exp_dev_info->capture_pose;

		/* Compute gravity error from XZ error range */
		dev_state->gravity_error_rad = sqrtf(rot_error->x * rot_error->x + rot_error->z * rot_error->z);

		/* Mark the score as un-evaluated to start */
		dev_state->score.good_pose_match = false;
		dev_state->score.strong_pose_match = false;
		dev_state->found_device_pose = false;
	}
	frame->n_devices = exposure_info->n_devices;

	PUSH_QUEUE(&sensor->fast_analysis_q, frame);
	ohmd_cond_broadcast (sensor->new_frame_cond);
	ohmd_unlock_mutex(sensor->sensor_lock);
	return;

release_frame:
	release_capture_frame(sensor, frame);
	ohmd_unlock_mutex(sensor->sensor_lock);
}

static void analyse_frame_fast(rift_sensor_ctx *sensor, rift_sensor_capture_frame *frame)
{
	uint64_t now = ohmd_monotonic_get(sensor->ohmd_ctx);
	int width = frame->uvc.width;
	int height = frame->uvc.height;

	LOGD("Sensor %d Frame %d - starting fast analysis", sensor->id, frame->id);

	frame->need_long_analysis = false;
	frame->long_analysis_found_new_blobs = false;
	frame->long_analysis_start_ts = frame->long_analysis_finish_ts = 0;

	frame->image_analysis_start_ts = now;

	blobwatch_process(sensor->bw, frame->uvc.data, width, height,
		frame->exposure_info.led_pattern_phase, NULL, 0, &frame->bwobs);

	frame->blob_extract_finish_ts = ohmd_monotonic_get(sensor->ohmd_ctx);

	if (frame->bwobs && frame->bwobs->num_blobs > 0) {
		tracker_process_blobs_fast(sensor, frame);
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

	frame->image_analysis_finish_ts = ohmd_monotonic_get(sensor->ohmd_ctx);

	if (ohmd_pw_video_stream_connected(sensor->debug_vid_raw))
		ohmd_pw_video_stream_push (sensor->debug_vid_raw, frame->uvc.start_ts, frame->uvc.data);

	if (sensor->debug_vid_raw_gst)
		ohmd_gst_video_stream_push (sensor->debug_vid_raw_gst, frame->uvc.start_ts, frame->uvc.data);

	if (ohmd_pw_video_stream_connected(sensor->debug_vid)) {
		rift_debug_draw_frame (sensor->debug_frame, frame->bwobs, sensor->cs, frame,
			frame->n_devices, sensor->devices, sensor->is_cv1, sensor->camera_matrix,
			sensor->dist_fisheye, sensor->dist_coeffs, &sensor->camera_pose);
		ohmd_pw_video_stream_push (sensor->debug_vid, frame->uvc.start_ts, sensor->debug_frame);
	}
}

static unsigned int fast_analysis_thread(void *arg);
static unsigned int long_analysis_thread(void *arg);

rift_sensor_ctx *
rift_sensor_new(ohmd_context* ohmd_ctx, int id, const char *serial_no,
	libusb_context *usb_ctx, libusb_device_handle *usb_devh, rift_tracker_ctx *tracker, const uint8_t radio_id[5],
  ohmd_gst_pipeline *debug_pipe)
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

	sensor_ctx->stream.sof_cb = new_frame_start_cb;
	sensor_ctx->stream.frame_cb = frame_captured_cb;
	sensor_ctx->stream.user_data = sensor_ctx;

	sensor_ctx->have_camera_pose = false;

	sensor_ctx->long_analysis_busy = false;

	printf ("Found Rift Sensor %d w/ Serial %s. Connecting to Radio address 0x%02x%02x%02x%02x%02x\n",
		id, serial_no, radio_id[0], radio_id[1], radio_id[2], radio_id[3], radio_id[4]);

	ret = rift_sensor_uvc_stream_setup (usb_ctx, sensor_ctx->usb_devh, &sensor_ctx->stream);
	ASSERT_MSG(ret >= 0, fail, "could not prepare for streaming\n");

	/* Initialise frame queues */
	INIT_QUEUE(&sensor_ctx->capture_frame_q);
	INIT_QUEUE(&sensor_ctx->fast_analysis_q);
	INIT_QUEUE(&sensor_ctx->long_analysis_q);

	/* Allocate capture frame buffers */
	int data_size = sensor_ctx->stream.frame_size;

	sensor_ctx->frames = ohmd_alloc(ohmd_ctx, NUM_CAPTURE_BUFFERS * sizeof (rift_sensor_capture_frame));
	for (i = 0; i < NUM_CAPTURE_BUFFERS; i++) {
			rift_sensor_capture_frame *frame = sensor_ctx->frames + i;

			frame->id = i;
			frame->uvc.data = ohmd_alloc(ohmd_ctx, data_size);
			frame->uvc.data_size = data_size;

			/* Send all frames to the capture thread to start */
			PUSH_QUEUE(&sensor_ctx->capture_frame_q, frame);
	}
	sensor_ctx->dropped_frames = 0;
	sensor_ctx->cur_capture_frame = NULL;

	snprintf(stream_id,64,"openhmd-rift-sensor-%s", sensor_ctx->serial_no);
	stream_id[63] = 0;

	sensor_ctx->bw = blobwatch_new(sensor_ctx->is_cv1 ? BLOB_THRESHOLD_CV1 : BLOB_THRESHOLD_DK2, sensor_ctx->stream.width, sensor_ctx->stream.height);

	LOGV("Sensor %d - reading Calibration\n", id);
	ret = rift_sensor_get_calibration(sensor_ctx, desc.idProduct);
	if (ret < 0) {
		LOGE("Failed to read Rift sensor calibration data");
		goto fail;
	}

	/* Raw debug video stream */
	sensor_ctx->debug_vid_raw = ohmd_pw_video_stream_new (stream_id, "Rift Sensor", OHMD_PW_VIDEO_FORMAT_GRAY8, sensor_ctx->stream.width, sensor_ctx->stream.height, 625, 12);

	/* Raw debug video stream - GStreamer recording */
  if (debug_pipe) {
		char debug_str[1024];

    sensor_ctx->debug_vid_raw_gst = ohmd_gst_video_stream_new (debug_pipe, stream_id, OHMD_PW_VIDEO_FORMAT_GRAY8, sensor_ctx->stream.width, sensor_ctx->stream.height, 625, 12);

		snprintf (debug_str, 1024, "{ \"type\": \"device\", \"device-id\": \"%s\",\n"
        "\"is-cv1\": %d, "
				"\"camera-matrix\": [ %f, %f, %f, %f, %f, %f, %f, %f, %f ], \"dist-coeffs\": [ %f, %f, %f, %f, %f ]\n"
        "}\n",
        stream_id, sensor_ctx->is_cv1 ? 1 : 0,
        sensor_ctx->camera_matrix.m[0], sensor_ctx->camera_matrix.m[1], sensor_ctx->camera_matrix.m[2],
        sensor_ctx->camera_matrix.m[3], sensor_ctx->camera_matrix.m[4], sensor_ctx->camera_matrix.m[5],
        sensor_ctx->camera_matrix.m[6], sensor_ctx->camera_matrix.m[7], sensor_ctx->camera_matrix.m[8],
        sensor_ctx->dist_coeffs[0], sensor_ctx->dist_coeffs[1], sensor_ctx->dist_coeffs[2], sensor_ctx->dist_coeffs[3],
        sensor_ctx->dist_coeffs[4]);
		debug_str[1023] = '\0';

    ohmd_gst_pipeline_push_metadata (debug_pipe, 0, debug_str);
  }

	/* Annotated debug video stream */
	sensor_ctx->debug_vid = ohmd_pw_video_stream_new (stream_id, "Rift Tracking", OHMD_PW_VIDEO_FORMAT_RGB, sensor_ctx->stream.width * 2, sensor_ctx->stream.height, 625, 12);

	if (sensor_ctx->debug_vid) {
		/* Allocate an RGB debug frame, twice the width of the input */
		sensor_ctx->debug_frame = ohmd_alloc(ohmd_ctx, 2 * 3 * sensor_ctx->stream.width * sensor_ctx->stream.height);
	}

	sensor_ctx->cs = correspondence_search_new (&sensor_ctx->camera_matrix, sensor_ctx->dist_coeffs, sensor_ctx->dist_fisheye);

	/* Start analysis threads */
	sensor_ctx->sensor_lock = ohmd_create_mutex(ohmd_ctx);
	sensor_ctx->new_frame_cond = ohmd_create_cond(ohmd_ctx);

	sensor_ctx->shutdown = false;
	sensor_ctx->fast_analysis_thread = ohmd_create_thread (ohmd_ctx, fast_analysis_thread, sensor_ctx);
	sensor_ctx->long_analysis_thread = ohmd_create_thread (ohmd_ctx, long_analysis_thread, sensor_ctx);

	LOGV("Sensor %d starting stream\n", id);
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
	LOGV("Sensor %d ready\n", id);

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

	if (sensor_ctx->stream_started)
		rift_sensor_uvc_stream_stop(&sensor_ctx->stream);

	/* Shut down analysis threads */
	ohmd_lock_mutex(sensor_ctx->sensor_lock);
	sensor_ctx->shutdown = true;
	ohmd_cond_broadcast(sensor_ctx->new_frame_cond);
	ohmd_unlock_mutex(sensor_ctx->sensor_lock);

	ohmd_destroy_thread(sensor_ctx->fast_analysis_thread);
	ohmd_destroy_thread(sensor_ctx->long_analysis_thread);

	ohmd_destroy_mutex(sensor_ctx->sensor_lock);
	ohmd_destroy_cond(sensor_ctx->new_frame_cond);

	if (sensor_ctx->bw)
		blobwatch_free (sensor_ctx->bw);
	if (sensor_ctx->debug_vid_raw != NULL)
		ohmd_pw_video_stream_free (sensor_ctx->debug_vid_raw);
	if (sensor_ctx->debug_vid != NULL)
		ohmd_pw_video_stream_free (sensor_ctx->debug_vid);
	if (sensor_ctx->debug_frame != NULL)
		free (sensor_ctx->debug_frame);
	if (sensor_ctx->debug_vid_raw_gst != NULL)
		ohmd_gst_video_stream_free (sensor_ctx->debug_vid_raw_gst);

	rift_sensor_uvc_stream_clear(&sensor_ctx->stream);

	if (sensor_ctx->usb_devh)
		libusb_close (sensor_ctx->usb_devh);

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
			rift_sensor_capture_frame *frame = POP_QUEUE(&ctx->long_analysis_q);
			if (frame != NULL) {
				ctx->long_analysis_busy = true;
				ohmd_unlock_mutex (ctx->sensor_lock);

				frame->long_analysis_start_ts = ohmd_monotonic_get(ctx->ohmd_ctx);
				tracker_process_blobs_long(ctx, frame);
				frame->long_analysis_finish_ts = ohmd_monotonic_get(ctx->ohmd_ctx);

				ohmd_lock_mutex (ctx->sensor_lock);
				ctx->long_analysis_busy = false;

				/* Done with this frame - send it back to the capture thread */
				release_capture_frame(ctx, frame);
			}
			if (!ctx->shutdown) {
				ohmd_cond_wait(ctx->new_frame_cond, ctx->sensor_lock);
			}
	}
	ohmd_unlock_mutex (ctx->sensor_lock);

	return 0;
}

static unsigned int fast_analysis_thread(void *arg)
{
	rift_sensor_ctx *sensor = arg;

	ohmd_lock_mutex (sensor->sensor_lock);
	while (!sensor->shutdown) {
			rift_sensor_capture_frame *frame = POP_QUEUE(&sensor->fast_analysis_q);
			if (frame != NULL) {
				ohmd_unlock_mutex (sensor->sensor_lock);
				analyse_frame_fast(sensor, frame);
				ohmd_lock_mutex (sensor->sensor_lock);

				/* Done with this frame - either send it back to the capture thread,
				 * or to the long analysis thread (unless the thread is still busy
				 * processing something else
				 */
				if (frame->need_long_analysis && !sensor->long_analysis_busy) {
					/* If there is an un-fetched frame in the long analysis
					 * queue, steal it back and return that to the capture thread,
					 * then replace it with the new one */
					rift_sensor_capture_frame *old_frame = REWIND_QUEUE(&sensor->long_analysis_q);
					if (old_frame) {
						uint64_t now = ohmd_monotonic_get(sensor->ohmd_ctx);
						LOGD("Sensor %d reclaimed frame %d from long analysis queue",
							sensor->id, old_frame->id);
						old_frame->long_analysis_start_ts = old_frame->long_analysis_finish_ts = now;
						release_capture_frame(sensor, old_frame);
					}

					PUSH_QUEUE(&sensor->long_analysis_q, frame);
				} else {
					frame->long_analysis_start_ts = frame->long_analysis_finish_ts = frame->image_analysis_finish_ts;
					release_capture_frame(sensor, frame);
				}
			}
			if (!sensor->shutdown) {
				ohmd_cond_wait(sensor->new_frame_cond, sensor->sensor_lock);
			}
	}
	ohmd_unlock_mutex (sensor->sensor_lock);

	return 0;
}

void rift_sensor_update_exposure (rift_sensor_ctx *sensor, const rift_tracker_exposure_info *exposure_info)
{
	rift_tracker_exposure_info old_exposure_info;
	bool old_exposure_info_valid = false, exposure_changed = false;

	ohmd_lock_mutex (sensor->sensor_lock);
	rift_sensor_capture_frame *frame = (rift_sensor_capture_frame *)(sensor->cur_capture_frame);

	if (frame == NULL)
		goto done; /* No capture frame yet */

	uint64_t now = ohmd_monotonic_get(sensor->ohmd_ctx);

	if (!frame->exposure_info_valid) {
		/* There wasn't previously exposure info but is now, take it */
		LOGV ("%f Sensor %d Frame (sof %f) exposure info TS %u count %u phase %d",
			(double) (now) / 1000000.0, sensor->id,
			(double) (exposure_info->local_ts) / 1000000.0,
			exposure_info->hmd_ts,
			exposure_info->count, exposure_info->led_pattern_phase);

		frame->exposure_info = *exposure_info;
		frame->exposure_info_valid = true;
		exposure_changed = true;
	}
	else if (frame->exposure_info.count != exposure_info->count) {
		/* The exposure info changed mid-frame. Update if this exposure arrived within 5 ms
		 * of the frame start */
		uint64_t frame_ts_threshold = frame->uvc.start_ts + 5000000;

		if (exposure_info->local_ts < frame_ts_threshold) {
			old_exposure_info_valid = true;
			old_exposure_info = frame->exposure_info;

			frame->exposure_info = *exposure_info;
			exposure_changed = true;

			LOGV ("%f Sensor %d Frame (sof %f) updating exposure info TS %u count %u phase %d",
				(double) (now) / 1000000.0, sensor->id,
				(double) (exposure_info->local_ts) / 1000000.0,
				exposure_info->hmd_ts,
				exposure_info->count, exposure_info->led_pattern_phase);
		}
	}

	if (exposure_changed)
		rift_tracker_frame_changed_exposure(sensor->tracker, old_exposure_info_valid ? &old_exposure_info : NULL, &frame->exposure_info);

done:
	ohmd_unlock_mutex (sensor->sensor_lock);
}
