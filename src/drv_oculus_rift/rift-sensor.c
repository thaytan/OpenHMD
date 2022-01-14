/*
 * Rift sensor interface. Processing task.
 * Copyright 2014-2015 Philipp Zabel
 * Copyright 2019 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 */

#define _GNU_SOURCE
#define __STDC_FORMAT_MACROS

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <inttypes.h>

#include "rift-sensor.h"
#include "rift-sensor-pose-search.h"

#include "rift-sensor-blobwatch.h"

#include "sensor/ar0134.h"
#include "sensor/mt9v034.h"
#include "sensor/esp770u.h"
#include "sensor/esp570.h"
#include "sensor/uvc.h"

#include "rift-debug-draw.h"

#include "ohmd-jpeg.h"

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
	rift_sensor_analysis_frame *data[QUEUE_SIZE];
	unsigned int head, tail;
};

struct rift_sensor_ctx_s
{
	ohmd_context* ohmd_ctx;
	int id;
	char serial_no[32];

	rift_tracker_ctx *tracker;

	rift_sensor_device *dev;

	rift_sensor_analysis_frame *frames;
	ohmd_jpeg_decoder *jpeg_decoder;

	blobwatch* bw;
	rift_pose_finder pf;

	ohmd_mutex *sensor_lock;

	/* Protected by sensor_lock */
	rift_tracked_device *devices[RIFT_MAX_TRACKED_DEVICES];
	uint8_t n_devices;

	/* Queue of frames being returned to the capture thread */
	rift_sensor_frame_queue capture_frame_q;
	int dropped_frames;
	/* queue of frames awaiting fast analysis */
	rift_sensor_frame_queue fast_analysis_q;
	/* cond that's signalled when a frame is put in the fast queue */
	ohmd_cond *fast_analysis_q_cond;

	/* queue of frames awaiting long analysis */
	rift_sensor_frame_queue long_analysis_q;
	/* cond that's signalled when a frame is put in the long queue */
	ohmd_cond *long_analysis_q_cond;

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

static rift_sensor_analysis_frame *POP_QUEUE(rift_sensor_frame_queue *q) {
	rift_sensor_analysis_frame *f;
	unsigned int next_head = (q->head+1) % QUEUE_SIZE;

	if ((q)->tail == (q)->head) /* Check there's something in the queue */
		return NULL;

	f = q->data[q->head];
	q->head = next_head;

	return f;
}

/* Rewind the queue and un-push the last element */
static rift_sensor_analysis_frame *REWIND_QUEUE(rift_sensor_frame_queue *q) {
	rift_sensor_analysis_frame *f;
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

/* Called with sensor_lock held. Releases a frame back to the capture thread */
static void
release_capture_frame(rift_sensor_ctx *sensor, rift_sensor_analysis_frame *frame)
{
	uint64_t now = ohmd_monotonic_get(sensor->ohmd_ctx);

	LOGD("Sensor %d Frame %d analysis done after %u ms. Captured %" PRIu64 " capture %u ms fast: queued %f ms analysis %u ms (%ums blob extraction) long: queued %f ms analysis %u ms",
		 sensor->id, frame->id,
		 (uint32_t) (now - frame->vframe->start_ts) / 1000000,
		 frame->vframe->start_ts,
		 (uint32_t) (frame->frame_delivered_ts - frame->vframe->start_ts) / 1000000,
		 (double) (frame->image_analysis_start_ts - frame->frame_delivered_ts) / 1000000.0,
		 (uint32_t) (frame->image_analysis_finish_ts - frame->image_analysis_start_ts) / 1000000,
		 (uint32_t) (frame->blob_extract_finish_ts - frame->image_analysis_start_ts) / 1000000,
		 (double) (frame->long_analysis_start_ts - frame->image_analysis_finish_ts) / 1000000.0,
		 (uint32_t) (frame->long_analysis_finish_ts - frame->long_analysis_start_ts) / 1000000);

	rift_tracker_frame_release (sensor->tracker, now,
			frame->vframe->start_ts, frame->exposure_info_valid ? &frame->exposure_info : NULL,
			sensor->serial_no);

	if (frame->bwobs) {
		blobwatch_release_observation(sensor->bw, frame->bwobs);
		frame->bwobs = NULL;
	}

	ohmd_video_frame_release(frame->vframe);
	frame->vframe = NULL;

	PUSH_QUEUE(&sensor->capture_frame_q, frame);
}

/* Called by the rift_sensor_device when a frame is captured. Attach
 * it to a rift_sensor_analysis_frame, retrieve the exposure info
 * from the tracker, and push it on the fast analysis queue
 */
static void frame_captured_cb(rift_sensor_device *dev, ohmd_video_frame *vframe, void *cb_data)
{
	rift_sensor_ctx *sensor = cb_data;
	rift_sensor_analysis_frame *frame;
	uint64_t now = ohmd_monotonic_get(sensor->ohmd_ctx);

	assert(sensor);

	ohmd_lock_mutex (sensor->sensor_lock);

	frame = POP_QUEUE(&sensor->capture_frame_q);
	/* If there is no queue on the capture frame queue,
	 * then the analysis threads got blocked for too long. Find
	 * and reclaim a frame from the one of the analysis queues */

	/* Take from the fast queue first - it should have woken
	 * up and processed any frame we gave it by now, so if there's
	 * one pending things are quite blocked */
	if (frame == NULL) {
		if ((frame = REWIND_QUEUE(&sensor->fast_analysis_q)) != NULL) {
			release_capture_frame(sensor, frame);
			frame = POP_QUEUE(&sensor->capture_frame_q);
		}
	}

	/* Otherwise try and claim a pending frame from the long
	 * analysis queue */
	if (frame == NULL) {
		if ((frame = REWIND_QUEUE(&sensor->long_analysis_q)) != NULL) {
			release_capture_frame(sensor, frame);
			frame = POP_QUEUE(&sensor->capture_frame_q);
		}
	}

	if (frame == NULL)
		goto no_frame;

	frame->frame_delivered_ts = now;
	frame->vframe = vframe;
	frame->bwobs = NULL;

	frame->exposure_info_valid = rift_tracker_frame_captured (sensor->tracker, now,
		vframe->start_ts, &frame->exposure_info, sensor->serial_no);

	PUSH_QUEUE(&sensor->fast_analysis_q, frame);
	ohmd_cond_signal(sensor->fast_analysis_q_cond);
	ohmd_unlock_mutex(sensor->sensor_lock);
	return;

no_frame:
	ohmd_video_frame_release(vframe);
	ohmd_unlock_mutex(sensor->sensor_lock);
}

static void analyse_frame_fast(rift_sensor_ctx *sensor, rift_sensor_analysis_frame *frame)
{
	uint64_t now = ohmd_monotonic_get(sensor->ohmd_ctx);
	ohmd_video_frame *vframe = frame->vframe;
	int width = vframe->width;
	int height = vframe->height;

	LOGD("Sensor %d Frame %d - starting fast analysis", sensor->id, frame->id);

	frame->image_analysis_start_ts = now;

	blobwatch_process(sensor->bw, vframe->data, width, height,
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
		ohmd_pw_video_stream_push (sensor->debug_vid_raw, vframe->start_ts, vframe->data);

	if (ohmd_pw_video_stream_connected(sensor->debug_vid)) {
		rift_debug_draw_frame (sensor->debug_frame, frame->bwobs, sensor->pf.cs, frame,
			frame->n_devices, sensor->devices, &sensor->dev->calib, &sensor->pf.camera_pose);
		ohmd_pw_video_stream_push (sensor->debug_vid, vframe->start_ts, sensor->debug_frame);
	}
}

static unsigned int fast_analysis_thread(void *arg);
static unsigned int long_analysis_thread(void *arg);
static bool handle_found_pose (rift_sensor_ctx *sensor_ctx,
	rift_tracked_device *dev, rift_sensor_analysis_frame *frame,
	posef *obj_world_pose, rift_pose_metrics *score);

rift_sensor_ctx *
rift_sensor_new(ohmd_context* ohmd_ctx, int id, const char *serial_no,
	rift_sensor_device *dev, rift_tracker_ctx *tracker)
{
	rift_sensor_ctx *sensor_ctx = NULL;
	char stream_id[64];
	int i;

	sensor_ctx = ohmd_alloc(ohmd_ctx, sizeof (rift_sensor_ctx));

	sensor_ctx->ohmd_ctx = ohmd_ctx;
	sensor_ctx->dev = dev;

	sensor_ctx->pf.sensor_id = sensor_ctx->id = id;
	strcpy ((char *) sensor_ctx->serial_no, (char *) serial_no);
	sensor_ctx->tracker = tracker;

	sensor_ctx->long_analysis_busy = false;

	sensor_ctx->sensor_lock = ohmd_create_mutex(ohmd_ctx);

	/* Initialise frame queues */
	INIT_QUEUE(&sensor_ctx->capture_frame_q);
	INIT_QUEUE(&sensor_ctx->fast_analysis_q);
	INIT_QUEUE(&sensor_ctx->long_analysis_q);

	/* Conds we use to wake queue waiters */
	sensor_ctx->fast_analysis_q_cond = ohmd_create_cond(ohmd_ctx);
	sensor_ctx->long_analysis_q_cond = ohmd_create_cond(ohmd_ctx);

	/* Allocate capture frame buffers */
	sensor_ctx->frames = ohmd_alloc(ohmd_ctx, NUM_CAPTURE_BUFFERS * sizeof (rift_sensor_analysis_frame));
	for (i = 0; i < NUM_CAPTURE_BUFFERS; i++) {
			rift_sensor_analysis_frame *frame = sensor_ctx->frames + i;
			frame->id = i;
			/* Send all frames to the capture queue to start */
			PUSH_QUEUE(&sensor_ctx->capture_frame_q, frame);
	}
	sensor_ctx->dropped_frames = 0;

	snprintf(stream_id,64,"openhmd-rift-sensor-%s", sensor_ctx->serial_no);
	stream_id[63] = 0;

	rift_sensor_camera_params *calib = &sensor_ctx->dev->calib;

	sensor_ctx->bw = blobwatch_new(calib->is_cv1 ? BLOB_THRESHOLD_CV1 : BLOB_THRESHOLD_DK2);

	rift_pose_finder_init(&sensor_ctx->pf, calib, (rift_pose_finder_cb) handle_found_pose, sensor_ctx);

	/* Raw debug video stream */
	sensor_ctx->debug_vid_raw = ohmd_pw_video_stream_new (stream_id, "Rift Sensor", OHMD_PW_VIDEO_FORMAT_GRAY8,
			calib->width, calib->height, 625, 12);

	/* Annotated debug video stream */
	sensor_ctx->debug_vid = ohmd_pw_video_stream_new (stream_id, "Rift Tracking", OHMD_PW_VIDEO_FORMAT_RGB,
			calib->width * 2, calib->height, 625, 12);

	if (sensor_ctx->debug_vid) {
		/* Allocate an RGB debug frame, twice the width of the input */
		sensor_ctx->debug_frame = ohmd_alloc(ohmd_ctx, 2 * 3 * calib->width * calib->height);
	}

	/* Start analysis threads */
	sensor_ctx->shutdown = false;
	sensor_ctx->fast_analysis_thread = ohmd_create_thread (ohmd_ctx, fast_analysis_thread, sensor_ctx);
	sensor_ctx->long_analysis_thread = ohmd_create_thread (ohmd_ctx, long_analysis_thread, sensor_ctx);


	LOGV("Sensor %d ready\n", id);

	return sensor_ctx;
}

bool
rift_sensor_start(rift_sensor_ctx *sensor)
{
	return rift_sensor_device_start_video (sensor->dev, NUM_CAPTURE_BUFFERS + 1, frame_captured_cb, sensor);
}

void
rift_sensor_free (rift_sensor_ctx *sensor_ctx)
{
	if (sensor_ctx == NULL)
		return;

	if (sensor_ctx->dev)
		rift_sensor_device_stop_video(sensor_ctx->dev);

	/* Shut down analysis threads */
	ohmd_lock_mutex(sensor_ctx->sensor_lock);
	sensor_ctx->shutdown = true;
	ohmd_cond_broadcast(sensor_ctx->fast_analysis_q_cond);
	ohmd_cond_broadcast(sensor_ctx->long_analysis_q_cond);
	ohmd_unlock_mutex(sensor_ctx->sensor_lock);

	if (sensor_ctx->fast_analysis_thread)
		ohmd_destroy_thread(sensor_ctx->fast_analysis_thread);

	if (sensor_ctx->long_analysis_thread)
		ohmd_destroy_thread(sensor_ctx->long_analysis_thread);

	if (sensor_ctx->sensor_lock)
		ohmd_destroy_mutex(sensor_ctx->sensor_lock);

	if (sensor_ctx->fast_analysis_q_cond)
		ohmd_destroy_cond(sensor_ctx->fast_analysis_q_cond);
	if (sensor_ctx->long_analysis_q_cond)
		ohmd_destroy_cond(sensor_ctx->long_analysis_q_cond);

#if HAVE_LIBJPEG
	if (sensor_ctx->jpeg_decoder)
		ohmd_jpeg_decoder_free(sensor_ctx->jpeg_decoder);
#endif

	if (sensor_ctx->bw)
		blobwatch_free (sensor_ctx->bw);
	if (sensor_ctx->debug_vid_raw != NULL)
		ohmd_pw_video_stream_free (sensor_ctx->debug_vid_raw);
	if (sensor_ctx->debug_vid != NULL)
		ohmd_pw_video_stream_free (sensor_ctx->debug_vid);
	if (sensor_ctx->debug_frame != NULL)
		free (sensor_ctx->debug_frame);

	if (sensor_ctx->dev)
		rift_sensor_device_free(sensor_ctx->dev);

	rift_pose_finder_clear (&sensor_ctx->pf);

	free(sensor_ctx->frames);
	free (sensor_ctx);
}

static rift_sensor_analysis_frame *
get_latest_long_analysis_frame(rift_sensor_ctx *ctx) {
	/* Always drain all pending frames and only analyse the newest */
	rift_sensor_analysis_frame *frame = POP_QUEUE(&ctx->long_analysis_q);

	do {
		rift_sensor_analysis_frame *another_frame = POP_QUEUE(&ctx->long_analysis_q);
		if (another_frame == NULL) {
			break;
		}

		release_capture_frame(ctx, frame);
		frame = another_frame;
	} while (true);

	return frame;
}

static unsigned int long_analysis_thread(void *arg)
{
	rift_sensor_ctx *ctx = arg;

	ohmd_lock_mutex (ctx->sensor_lock);

	rift_sensor_analysis_frame *frame = get_latest_long_analysis_frame(ctx);
	while (!ctx->shutdown) {
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
				frame = NULL;
			}
			while (!ctx->shutdown && frame == NULL) {
				/* Try and fetch another frame, otherwise sleep */
				if ((frame = get_latest_long_analysis_frame(ctx)) != NULL)
					break;
				ohmd_cond_wait(ctx->long_analysis_q_cond, ctx->sensor_lock);
			}
	}
	ohmd_unlock_mutex (ctx->sensor_lock);

	return 0;
}

#if HAVE_LIBJPEG
bool decode_frame(rift_sensor_ctx *sensor, rift_sensor_analysis_frame *frame)
{
	ohmd_video_frame *vframe = frame->vframe;
	ohmd_video_frame *out_vframe = NULL;

	if (vframe->format != OHMD_VIDEO_FRAME_FORMAT_JPEG)
		return true;

	if (sensor->jpeg_decoder == NULL) {
		sensor->jpeg_decoder = ohmd_jpeg_decoder_new(sensor->ohmd_ctx);
		if (sensor->jpeg_decoder == NULL) {
			LOGE("Failed to create JPEG decoder");
			return false;
		}
	}

	if (!ohmd_jpeg_decoder_decode(sensor->jpeg_decoder, vframe, &out_vframe))
		return false;

	ohmd_video_frame_release(frame->vframe);
	frame->vframe = out_vframe;

	return true;
}
#endif

static bool init_frame_analyse(rift_sensor_ctx *sensor, rift_sensor_analysis_frame *frame)
{

	/* Do initial prep of the frame before we start analysis */
	LOGD ("Sensor %d captured frame %d exposure counter %u phase %d", sensor->id,
		frame->id, frame->exposure_info.count, frame->exposure_info.led_pattern_phase);

	int d;
	const rift_tracker_exposure_info *exposure_info = &frame->exposure_info;

	frame->need_long_analysis = false;
	frame->long_analysis_found_new_blobs = false;
	frame->long_analysis_start_ts = frame->long_analysis_finish_ts = 0;

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

#if HAVE_LIBJPEG
	if (!decode_frame(sensor, frame))
		return false;
#endif

	return true;
}

static unsigned int fast_analysis_thread(void *arg)
{
	rift_sensor_ctx *sensor = arg;

	ohmd_lock_mutex (sensor->sensor_lock);
	rift_sensor_analysis_frame *frame = POP_QUEUE(&sensor->fast_analysis_q);
	while (!sensor->shutdown) {
			if (frame != NULL) {
				ohmd_unlock_mutex (sensor->sensor_lock);
				if (init_frame_analyse(sensor, frame))
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
					rift_sensor_analysis_frame *old_frame = REWIND_QUEUE(&sensor->long_analysis_q);
					if (old_frame) {
						uint64_t now = ohmd_monotonic_get(sensor->ohmd_ctx);
						LOGD("Sensor %d reclaimed frame %d from long analysis queue",
							sensor->id, old_frame->id);
						old_frame->long_analysis_start_ts = old_frame->long_analysis_finish_ts = now;
						release_capture_frame(sensor, old_frame);
					}

					PUSH_QUEUE(&sensor->long_analysis_q, frame);
					ohmd_cond_signal(sensor->long_analysis_q_cond);
				} else {
					frame->long_analysis_start_ts = frame->long_analysis_finish_ts = frame->image_analysis_finish_ts;
					release_capture_frame(sensor, frame);
				}

				frame = NULL;
			}

			while (!sensor->shutdown && frame == NULL) {
				/* Try and fetch another frame, otherwise sleep */
				if ((frame = POP_QUEUE(&sensor->fast_analysis_q)) != NULL)
					break;
				ohmd_cond_wait(sensor->fast_analysis_q_cond, sensor->sensor_lock);
			}
	}
	ohmd_unlock_mutex (sensor->sensor_lock);

	return 0;
}

static bool handle_found_pose (rift_sensor_ctx *sensor_ctx,
	rift_tracked_device *dev, rift_sensor_analysis_frame *frame,
	posef *obj_world_pose, rift_pose_metrics *score)
{
	uint64_t now = ohmd_monotonic_get(sensor_ctx->ohmd_ctx);
	LOGD("Sensor %d TS %llu Updating fusion for device %d pose quat %f %f %f %f  pos %f %f %f",
		sensor_ctx->id, (unsigned long long)(now),
		dev->id, obj_world_pose->orient.x, obj_world_pose->orient.y, obj_world_pose->orient.z, obj_world_pose->orient.w,
		obj_world_pose->pos.x, obj_world_pose->pos.y, obj_world_pose->pos.z);

	bool ret = rift_tracked_device_model_pose_update(dev, now, frame->vframe->start_ts, &frame->exposure_info,
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

const char *rift_sensor_serial_no (rift_sensor_ctx *sensor)
{
	return sensor->serial_no;
}
