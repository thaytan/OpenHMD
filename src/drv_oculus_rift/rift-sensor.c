/*
 * Rift sensor interface. Processing task.
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
#include "rift-sensor-pose-search.h"

#include "rift-sensor-blobwatch.h"
#include "rift-sensor-ar0134.h"
#include "rift-sensor-mt9v034.h"
#include "rift-sensor-esp770u.h"
#include "rift-sensor-esp570.h"
#include "rift-sensor-uvc.h"

#include "rift-debug-draw.h"

#include "ohmd-pipewire.h"

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

struct rift_sensor_ctx_s
{
	ohmd_context* ohmd_ctx;
	int id;
	char serial_no[32];
	rift_sensor_camera_params calib;

	rift_tracker_ctx *tracker;

	libusb_device_handle *usb_devh;
	int stream_started;
	struct rift_sensor_uvc_stream stream;

	rift_sensor_capture_frame *frames;

	blobwatch* bw;
	rift_pose_finder pf;

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

bool
rift_sensor_add_device(rift_sensor_ctx *sensor, rift_tracked_device *device)
{
	bool ret;

	ohmd_lock_mutex(sensor->sensor_lock);
	assert (sensor->n_devices < RIFT_MAX_TRACKED_DEVICES);

	ret	= correspondence_search_set_model (sensor->pf.cs, device->id, device->led_search);

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
	rift_sensor_camera_params *calib = &ctx->calib;
	double * const A = calib->camera_matrix.m;
	double * const k = calib->dist_coeffs;
	double fx, fy, cx, cy;
	double k1, k2, k3, k4;
	double p1, p2;
	int ret;

	/* Copy the frame width and height from the UVC setup */
	calib->width = ctx->stream.width;
	calib->height = ctx->stream.height;

	LOGI("camera width %d height %d", calib->width, calib->height);

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
		LOGD("Sensor %d - new frame @ %f ms SOF phase %d", sensor->id,
		  (double) (start_time) / 1000000.0, exposure_info.led_pattern_phase);
	}
	else {
		LOGD("Sensor %d - new frame @ %f SOF no phase info", sensor->id,
		  (double) (start_time) / 1000000.0);
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
		dev_state->gravity_error_rad = OHMD_MAX(rot_error->x, rot_error->z);

		/* Mark the score as un-evaluated to start */
		dev_state->score.match_flags = 0;
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
		rift_pose_finder_process_blobs_fast(&sensor->pf, frame, sensor->devices);
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

	if (ohmd_pw_video_stream_connected(sensor->debug_vid)) {
		rift_debug_draw_frame (sensor->debug_frame, frame->bwobs, sensor->pf.cs, frame,
			frame->n_devices, sensor->devices, &sensor->calib, &sensor->pf.camera_pose);
		ohmd_pw_video_stream_push (sensor->debug_vid, frame->uvc.start_ts, sensor->debug_frame);
	}
}

static unsigned int fast_analysis_thread(void *arg);
static unsigned int long_analysis_thread(void *arg);
static bool handle_found_pose (rift_sensor_ctx *sensor_ctx,
	rift_tracked_device *dev, rift_sensor_capture_frame *frame,
	posef *obj_world_pose, rift_pose_metrics *score);

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
	sensor_ctx->pf.sensor_id = sensor_ctx->id = id;
	strcpy ((char *) sensor_ctx->serial_no, (char *) serial_no);
	sensor_ctx->tracker = tracker;
	sensor_ctx->usb_devh = usb_devh;

	sensor_ctx->stream.sof_cb = new_frame_start_cb;
	sensor_ctx->stream.frame_cb = frame_captured_cb;
	sensor_ctx->stream.user_data = sensor_ctx;

	sensor_ctx->long_analysis_busy = false;

	sensor_ctx->sensor_lock = ohmd_create_mutex(ohmd_ctx);
	sensor_ctx->new_frame_cond = ohmd_create_cond(ohmd_ctx);

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

	LOGV("Sensor %d - reading Calibration\n", id);
	ret = rift_sensor_get_calibration(sensor_ctx, desc.idProduct);
	if (ret < 0) {
		LOGE("Failed to read Rift sensor calibration data");
		goto fail;
	}

	sensor_ctx->bw = blobwatch_new(sensor_ctx->calib.is_cv1 ? BLOB_THRESHOLD_CV1 : BLOB_THRESHOLD_DK2, sensor_ctx->stream.width, sensor_ctx->stream.height);

	rift_pose_finder_init(&sensor_ctx->pf, &sensor_ctx->calib,
			(rift_pose_finder_cb) handle_found_pose, sensor_ctx);

	/* Raw debug video stream */
	sensor_ctx->debug_vid_raw = ohmd_pw_video_stream_new (stream_id, "Rift Sensor", OHMD_PW_VIDEO_FORMAT_GRAY8, sensor_ctx->stream.width, sensor_ctx->stream.height, 625, 12);

	/* Annotated debug video stream */
	sensor_ctx->debug_vid = ohmd_pw_video_stream_new (stream_id, "Rift Tracking", OHMD_PW_VIDEO_FORMAT_RGB, sensor_ctx->stream.width * 2, sensor_ctx->stream.height, 625, 12);

	if (sensor_ctx->debug_vid) {
		/* Allocate an RGB debug frame, twice the width of the input */
		sensor_ctx->debug_frame = ohmd_alloc(ohmd_ctx, 2 * 3 * sensor_ctx->stream.width * sensor_ctx->stream.height);
	}

	/* Start analysis threads */
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

	if (sensor_ctx->fast_analysis_thread)
		ohmd_destroy_thread(sensor_ctx->fast_analysis_thread);

	if (sensor_ctx->long_analysis_thread)
		ohmd_destroy_thread(sensor_ctx->long_analysis_thread);

	if (sensor_ctx->sensor_lock)
		ohmd_destroy_mutex(sensor_ctx->sensor_lock);

	if (sensor_ctx->new_frame_cond)
		ohmd_destroy_cond(sensor_ctx->new_frame_cond);

	if (sensor_ctx->bw)
		blobwatch_free (sensor_ctx->bw);
	if (sensor_ctx->debug_vid_raw != NULL)
		ohmd_pw_video_stream_free (sensor_ctx->debug_vid_raw);
	if (sensor_ctx->debug_vid != NULL)
		ohmd_pw_video_stream_free (sensor_ctx->debug_vid);
	if (sensor_ctx->debug_frame != NULL)
		free (sensor_ctx->debug_frame);

	rift_sensor_uvc_stream_clear(&sensor_ctx->stream);

	if (sensor_ctx->usb_devh)
		libusb_close (sensor_ctx->usb_devh);

	rift_pose_finder_clear (&sensor_ctx->pf);

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
				rift_pose_finder_process_blobs_long(&ctx->pf, frame, ctx->devices);
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

			LOGV ("Sensor %d TS now %f Frame (sof %f) updating exposure info TS %u count %u phase %d",
				sensor->id, (double) (now) / 1000000.0,
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

static bool handle_found_pose (rift_sensor_ctx *sensor_ctx,
	rift_tracked_device *dev, rift_sensor_capture_frame *frame,
	posef *obj_world_pose, rift_pose_metrics *score)
{
	uint64_t now = ohmd_monotonic_get(sensor_ctx->ohmd_ctx);
	LOGD("Sensor %d TS %llu Updating fusion for device %d pose quat %f %f %f %f  pos %f %f %f",
		sensor_ctx->id, (unsigned long long)(now),
		dev->id, pose.orient.x, pose.orient.y, pose.orient.z, pose.orient.w,
		pose.pos.x, pose.pos.y, pose.pos.z);

	bool ret = rift_tracked_device_model_pose_update(dev, now, frame->uvc.start_ts, &frame->exposure_info,
	       score, obj_world_pose, sensor_ctx->serial_no);

	if (ret) {
		/* If this pose was accepted by the tracker, transfer these blob labels to the blobwatch object */
		ohmd_lock_mutex(sensor_ctx->sensor_lock);
		blobwatch_update_labels (sensor_ctx->bw, frame->bwobs, dev->id);
		ohmd_unlock_mutex(sensor_ctx->sensor_lock);
	} else { /* FIXME: Only do this after a few failures? */
		rift_clear_blob_labels (frame->bwobs->blobs, frame->bwobs->num_blobs, dev->id);
	}

	rift_tracked_device_frame_release(dev, &frame->exposure_info);

	return ret;
}
