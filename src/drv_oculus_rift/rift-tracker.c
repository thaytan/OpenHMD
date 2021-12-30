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
#include <stdarg.h>
#include <time.h>

#include "ohmd-video.h"

#include "../exponential-filter.h"
#include "rift-tracker.h"
#include "rift-sensor.h"
#include "rift-sensor-usb.h"

#include "rift-sensor-maths.h"
#include "rift-sensor-opencv.h"
#include "rift-sensor-pose-helper.h"

#include "rift-debug-draw.h"

#include "ohmd-pipewire.h"

#define ASSERT_MSG(_v, label, ...) if(!(_v)){ fprintf(stderr, __VA_ARGS__); goto label; }

#define MAX_SENSORS 4

/* Number of IMU observations we accumulate before output */
#define RIFT_MAX_PENDING_IMU_OBSERVATIONS 1000

/* Number of state slots to use for quat/position updates */
#define NUM_POSE_DELAY_SLOTS 3

/* Number of exposure history slots to keep */
#define NUM_EXPOSURE_HISTORY 3

/* Length of time (milliseconds) we will interpolate position before declaring
 * tracking lost */
#define POSE_LOST_THRESHOLD 500

/* Length of time (milliseconds) we can ignore orientation from cameras before
 * we force an update */
#define POSE_LOST_ORIENT_THRESHOLD 100

/* Length of time (milliseconds) that we are allowed to lose tracking because
 * there's no free delay slots before we'll warn about it. 60ms is ~3 frames
 */
#define NO_FREE_DELAY_SLOT_THRESHOLD 60

#define MIN_ROT_ERROR DEG_TO_RAD(25)
#define MIN_POS_ERROR 0.1

typedef struct rift_tracked_device_priv rift_tracked_device_priv;
typedef struct rift_tracker_pose_report rift_tracker_pose_report;
typedef struct rift_tracker_pose_delay_slot rift_tracker_pose_delay_slot;

typedef struct rift_tracked_device_imu_observation rift_tracked_device_imu_observation;

struct rift_tracked_device_imu_observation {
	uint64_t local_ts;
	uint64_t device_ts;
	float dt;

	vec3f ang_vel;
	vec3f accel;
	vec3f mag;
};

struct rift_tracker_pose_report {
		bool report_used; /* TRUE if this report has been integrated */
		posef pose;
		rift_pose_metrics score;
};

struct rift_tracker_pose_delay_slot {
	int slot_id;		/* Index of the slot */
	bool valid;			/* true if the exposure info was set */
	int use_count;	/* Number of frames using this slot */

	uint64_t device_time_ns; /* Device time this slot is currently tracking */

	/* rift_tracked_device_model_pose_update stores the observed poses here */
	int n_pose_reports;
	rift_tracker_pose_report pose_reports[MAX_SENSORS];
	/* Number of reports we used from the supplied ones */
	int n_used_reports;
};

/* Internal full tracked device struct */
struct rift_tracked_device_priv {
	rift_tracked_device base;

	int index; /* Index of this entry in the devices array for the tracker and exposures */

	ohmd_mutex *device_lock;

	/* 6DOF Kalman Filter */
	rift_kalman_6dof_filter ukf_fusion;

	/* Account keeping for UKF fusion slots */
	int delay_slot_index;
	rift_tracker_pose_delay_slot delay_slots[NUM_POSE_DELAY_SLOTS];
	/* Track the time we last started having no free delay slots (or 0 if never) */
	uint64_t last_no_free_delay_slot;

	/* The pose of the device relative to the IMU 3D space */
	posef device_from_fusion;

	/* The pose of the IMU relative to the LED model space */
	posef fusion_from_model;
	posef model_from_fusion;

	uint32_t last_device_ts;
	uint64_t device_time_ns;

	uint64_t last_observed_orient_ts;
	uint64_t last_observed_pose_ts;
	posef last_observed_pose;

	/* Reported view pose (to the user) and model pose (for the tracking) respectively */
	uint64_t last_reported_pose;
	posef reported_pose;
	vec3f reported_ang_vel;
	vec3f reported_lin_vel;
	vec3f reported_lin_accel;

	posef model_pose;

	exp_filter_pose pose_output_filter;

	int num_pending_imu_observations;
	rift_tracked_device_imu_observation pending_imu_observations[RIFT_MAX_PENDING_IMU_OBSERVATIONS];

	ohmd_pw_debug_stream *debug_metadata;
	FILE *debug_file;
};

struct rift_tracker_ctx_s
{
	ohmd_context* ohmd_ctx;
	libusb_context *usb_ctx;
	ohmd_mutex *tracker_lock;

	ohmd_thread* usb_thread;
	int usb_completed;

	int exposure_history_index;
	int exposure_history_size;
	rift_tracker_exposure_info exposure_history[NUM_EXPOSURE_HISTORY];

	rift_sensor_ctx *sensors[MAX_SENSORS];
	uint8_t n_sensors;

	rift_tracked_device_priv devices[RIFT_MAX_TRACKED_DEVICES];
	uint8_t n_devices;
};

static void rift_tracked_device_send_imu_debug(rift_tracked_device_priv *dev);
static void rift_tracked_device_send_debug_printf(rift_tracked_device_priv *dev, uint64_t local_ts, const char *fmt, ...);

static void rift_tracked_device_on_new_exposure (rift_tracked_device_priv *dev, rift_tracked_device_exposure_info *dev_info);
static int rift_tracked_device_exposure_claim(rift_tracked_device_priv *dev, rift_tracked_device_exposure_info *dev_info);
static void rift_tracked_device_exposure_release_locked(rift_tracked_device_priv *dev, rift_tracked_device_exposure_info *dev_info);

rift_tracked_device *
rift_tracker_add_device (rift_tracker_ctx *ctx, int device_id, posef *imu_pose, posef *model_pose, rift_leds *leds,
      rift_tracked_device_imu_calibration *imu_calib)
{
	int i, s;
	rift_tracked_device_priv *next_dev;
	char device_name[64];
	/* Rotate our initial pose 180 deg to point along the -Z axis */
	posef init_pose = { .pos = {{ 0.0, 0.0, 0.0 }}, .orient = {{ 0.0, 1.0, 0.0, 0.0 }}};

	snprintf(device_name,64,"openhmd-rift-device-%d", device_id);
	device_name[63] = 0;

	assert (ctx->n_devices < RIFT_MAX_TRACKED_DEVICES);

	ohmd_lock_mutex (ctx->tracker_lock);
	next_dev = ctx->devices + ctx->n_devices;

	next_dev->base.id = device_id;
	rift_kalman_6dof_init(&next_dev->ukf_fusion, &init_pose, NUM_POSE_DELAY_SLOTS);
	next_dev->last_reported_pose = next_dev->last_observed_orient_ts = next_dev->last_observed_pose_ts = next_dev->device_time_ns = 0;

	exp_filter_pose_init(&next_dev->pose_output_filter);

	/* Init delay slot bookkeeping */
	for (s = 0; s < NUM_POSE_DELAY_SLOTS; s++) {
		rift_tracker_pose_delay_slot *slot = next_dev->delay_slots + s;

		slot->slot_id = s;
		slot->valid = false;
	}

	/* Compute the device->IMU conversion from the imu->device pose passed */
	next_dev->device_from_fusion = *imu_pose;
	oposef_inverse(&next_dev->device_from_fusion);

	/* Compute the IMU->model transform by composing imu->device->model */
	oposef_apply(imu_pose, model_pose, &next_dev->fusion_from_model);
	/* And the inverse fusion->model conversion */
	next_dev->model_from_fusion = next_dev->fusion_from_model;
	oposef_inverse(&next_dev->model_from_fusion);

	next_dev->debug_metadata = ohmd_pw_debug_stream_new (device_name, "Rift Device");

	uint64_t now = ohmd_monotonic_get(ctx->ohmd_ctx);
	rift_tracked_device_send_debug_printf (next_dev, now, "{ \"type\": \"device\", "
		 "\"device-id\": %d,"
		"\"imu-calibration\": { \"accel-offset\": [ %f, %f, %f ], "
		"\"accel-matrix\": [ %f, %f, %f, %f, %f, %f, %f, %f, %f ], "
		"\"gyro_offset\": [ %f, %f, %f ], "
		"\"gyro-matrix\": [ %f, %f, %f, %f, %f, %f, %f, %f, %f ] } },", device_id,
		imu_calib->accel_offset.x, imu_calib->accel_offset.y, imu_calib->accel_offset.z,
		imu_calib->accel_matrix[0], imu_calib->accel_matrix[1], imu_calib->accel_matrix[2],
		imu_calib->accel_matrix[3], imu_calib->accel_matrix[4], imu_calib->accel_matrix[5],
		imu_calib->accel_matrix[6], imu_calib->accel_matrix[7], imu_calib->accel_matrix[8],
		imu_calib->gyro_offset.x, imu_calib->gyro_offset.y, imu_calib->gyro_offset.z,
		imu_calib->gyro_matrix[0], imu_calib->gyro_matrix[1], imu_calib->gyro_matrix[2],
		imu_calib->gyro_matrix[3], imu_calib->gyro_matrix[4], imu_calib->gyro_matrix[5],
		imu_calib->gyro_matrix[6], imu_calib->gyro_matrix[7], imu_calib->gyro_matrix[8]);

	next_dev->base.leds = leds;
	next_dev->base.led_search = led_search_model_new (leds);
	ctx->n_devices++;
	ohmd_unlock_mutex (ctx->tracker_lock);

	/* Tell the sensors about the new device */
	for (i = 0; i < ctx->n_sensors; i++) {
		rift_sensor_ctx *sensor_ctx = ctx->sensors[i];
		if (!rift_sensor_add_device (sensor_ctx, (rift_tracked_device *) next_dev)) {
			LOGE("Failed to configure object tracking for device %d\n", device_id);
		}
	}

	printf("device %d online. Now tracking.\n", device_id);
	return (rift_tracked_device *) next_dev;
}

static unsigned int uvc_handle_events(void *arg)
{
	rift_tracker_ctx *tracker_ctx = arg;

	while (!tracker_ctx->usb_completed) {
		struct timeval timeout;

		timeout.tv_sec = 0;
		timeout.tv_usec = 100000;

		libusb_handle_events_timeout_completed(tracker_ctx->usb_ctx, &timeout, &tracker_ctx->usb_completed);
	}

	return 0;
}

rift_tracker_ctx *
rift_tracker_new (ohmd_context* ohmd_ctx,
		const uint8_t radio_id[5])
{
	rift_tracker_ctx *tracker_ctx = NULL;
	int ret, i;
	libusb_device **devs;

	tracker_ctx = ohmd_alloc(ohmd_ctx, sizeof (rift_tracker_ctx));
	tracker_ctx->ohmd_ctx = ohmd_ctx;
	tracker_ctx->tracker_lock = ohmd_create_mutex(ohmd_ctx);

	for (i = 0; i < RIFT_MAX_TRACKED_DEVICES; i++) {
		rift_tracked_device_priv *dev = tracker_ctx->devices + i;
		dev->index = i;
		dev->device_lock = ohmd_create_mutex(ohmd_ctx);
	}

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

		rift_sensor_device *sensor_device = rift_sensor_usb_new (ohmd_ctx, tracker_ctx->n_sensors, (char *) serial, tracker_ctx->usb_ctx, usb_devh, radio_id);

		if (sensor_device == NULL) {
			LOGW("Failed to open Rift Sensor USB device %s\n", serial);
			continue;
		}

		rift_sensor_ctx *sensor_ctx = rift_sensor_new (ohmd_ctx, tracker_ctx->n_sensors, (char *) serial, sensor_device, tracker_ctx);
		if (sensor_ctx == NULL) {
			LOGW("Failed to open Rift Sensor analyser for %s\n", serial);
			continue;
		}

		tracker_ctx->sensors[tracker_ctx->n_sensors] = sensor_ctx;
		tracker_ctx->n_sensors++;
		if (tracker_ctx->n_sensors == MAX_SENSORS) {
			LOGI("Found the maximum number of supported sensors: %d.\n", MAX_SENSORS);
			break;
		}
	}
	libusb_free_device_list(devs, 1);

	printf ("Opened %u Rift Sensor cameras\n", tracker_ctx->n_sensors);

	/* Loop over the sensors we found and start the video flowing */
	for (i = 0; i < tracker_ctx->n_sensors; i++) {
		rift_sensor_ctx *sensor = tracker_ctx->sensors[i];

		if (!rift_sensor_start (sensor)) {
			LOGW("Failed to start video stream for sensor %s\n", rift_sensor_serial_no (sensor));
		}
	}

	return tracker_ctx;

fail:
	if (tracker_ctx)
		rift_tracker_free (tracker_ctx);
	return NULL;
}

/* Called from the rift IMU / packet handling loop
 * when processing an IMU update from the HMD. If the
 * packet signalled a new camera exposure, we take
 * a snapshot of the predicted state of each device
 * into a lagged fusion slot */
void rift_tracker_on_new_exposure (rift_tracker_ctx *ctx, uint32_t hmd_ts, uint16_t exposure_count, uint32_t exposure_hmd_ts, uint8_t led_pattern_phase)
{
	rift_tracker_exposure_info *info;
	bool is_new_exposure = false;
	int i;

	ohmd_lock_mutex (ctx->tracker_lock);

	if (ctx->exposure_history_size > 0) {
		if (ctx->exposure_history[ctx->exposure_history_index].count != exposure_count) {
			is_new_exposure = true;

			ctx->exposure_history_index = (ctx->exposure_history_index + 1) % NUM_EXPOSURE_HISTORY;
			info = ctx->exposure_history + ctx->exposure_history_index;

			if (ctx->exposure_history_size < NUM_EXPOSURE_HISTORY)
				ctx->exposure_history_size++;
		}
	}
	else {
		info = ctx->exposure_history;
		ctx->exposure_history_index = 0;
		ctx->exposure_history_size = 1;
		is_new_exposure = true;
	}

	if (!is_new_exposure)
		goto done;

	if (info->led_pattern_phase != led_pattern_phase) {
		LOGD ("%f LED pattern phase changed to %d",
			(double) (ohmd_monotonic_get(ctx->ohmd_ctx)) / 1000000.0, led_pattern_phase);
		info->led_pattern_phase = led_pattern_phase;
	}

	uint64_t now = ohmd_monotonic_get(ctx->ohmd_ctx);

	info->local_ts = now;
	info->count = exposure_count;
	info->hmd_ts = exposure_hmd_ts;
	info->led_pattern_phase = led_pattern_phase;

	LOGD ("%f Have new exposure TS %u count %u LED pattern phase %d",
		(double) (now) / 1000000.0, exposure_count, exposure_hmd_ts, led_pattern_phase);

	if ((int32_t)(exposure_hmd_ts - hmd_ts) < -1500) {
		LOGW("Exposure timestamp %u was more than 1.5 IMU samples earlier than IMU ts %u by %u µS",
				exposure_hmd_ts, hmd_ts, hmd_ts - exposure_hmd_ts);
	}

	info->n_devices = ctx->n_devices;

	for (i = 0; i < ctx->n_devices; i++) {
		rift_tracked_device_priv *dev = ctx->devices + i;
		rift_tracked_device_exposure_info *dev_info = info->devices + i;

		ohmd_lock_mutex (dev->device_lock);
		rift_tracked_device_on_new_exposure(dev, dev_info);

		rift_tracked_device_send_imu_debug(dev);

		rift_tracked_device_send_debug_printf(dev, now,
				",\n{ \"type\": \"exposure\", \"local-ts\": %llu, "
				"\"hmd-ts\": %u, \"exposure-ts\": %u, \"count\": %u, \"device-ts\": %llu, "
				"\"delay-slot\": %d	}",
				(unsigned long long) now,
				hmd_ts, exposure_hmd_ts, exposure_count,
				(unsigned long long) dev_info->device_time_ns, dev_info->fusion_slot);
		ohmd_unlock_mutex (dev->device_lock);
	}
	/* Clear the info for non-existent devices */
	for (; i < RIFT_MAX_TRACKED_DEVICES; i++) {
		rift_tracked_device_exposure_info *dev_info = info->devices + i;
		dev_info->fusion_slot = -1;
	}

done:
	ohmd_unlock_mutex (ctx->tracker_lock);
}

/* Called from a sensor device when a video frame has been captured.
 * Iterate the exposure history and find the exposure that matches
 * the arrival time of the frame within +/- 10ms
 */
bool
rift_tracker_frame_captured (rift_tracker_ctx *ctx, uint64_t local_ts, uint64_t frame_start_local_ts, rift_tracker_exposure_info *out_info, const char *source)
{
	int i;
	ohmd_lock_mutex (ctx->tracker_lock);

	/* Find and populate the exposure info and return true if found */
	bool have_exposure_info = false;

	for (i = 0; i < ctx->exposure_history_size; i++) {
		rift_tracker_exposure_info *info = ctx->exposure_history + i;
		int64_t time_diff_ns = frame_start_local_ts - info->local_ts;
		if (time_diff_ns > -10000000 && time_diff_ns < 10000000) {
			have_exposure_info = true;
			*out_info = *info;
		}
	}

	if (!have_exposure_info)
		goto done;

	for (i = 0; i < ctx->n_devices; i++) {
		rift_tracked_device_priv *dev = ctx->devices + i;

		ohmd_lock_mutex (dev->device_lock);

		if (i < out_info->n_devices) {
			rift_tracked_device_exposure_info *dev_info = out_info->devices + i;

#if LOGLEVEL == 0
			LOGD("Frame capture - ts %llu, delay slot %d for dev %d",
				(unsigned long long) dev_info->device_time_ns, dev_info->fusion_slot, dev->base.id);
#endif
			dev_info->fusion_slot = rift_tracked_device_exposure_claim(dev, dev_info);
		}

		rift_tracked_device_send_imu_debug(dev);

		if (dev->debug_file != NULL) {
			fprintf(dev->debug_file, ",\n{ \"type\": \"frame-captured\", \"local-ts\": %llu, "
				"\"frame-start-local-ts\": %llu, \"source\": \"%s\" }",
				(unsigned long long) local_ts, (unsigned long long) frame_start_local_ts, source);
		}
		ohmd_unlock_mutex (dev->device_lock);
	}

done:
	ohmd_unlock_mutex (ctx->tracker_lock);

	return have_exposure_info;
}

void
rift_tracker_frame_release (rift_tracker_ctx *ctx, uint64_t local_ts, uint64_t frame_local_ts, rift_tracker_exposure_info *info, const char *source)
{
	int i;
	ohmd_lock_mutex (ctx->tracker_lock);
	for (i = 0; i < ctx->n_devices; i++) {
		rift_tracked_device_priv *dev = ctx->devices + i;

		ohmd_lock_mutex (dev->device_lock);

		/* This device might not have exposure info for this frame if it
		 * recently came online */
		if (info && i < info->n_devices) {
			rift_tracked_device_exposure_info *dev_info = info->devices + i;
			rift_tracked_device_exposure_release_locked(dev, dev_info);
		}

		rift_tracked_device_send_imu_debug(dev);

		if (dev->debug_file != NULL) {
			fprintf(dev->debug_file, ",\n{ \"type\": \"frame-release\", \"local-ts\": %llu, "
				"\"frame-local-ts\": %llu, \"source\": \"%s\" }",
				(unsigned long long) local_ts, (unsigned long long) frame_local_ts, source);
		}
		ohmd_unlock_mutex (dev->device_lock);
	}
	ohmd_unlock_mutex (ctx->tracker_lock);
}

void
rift_tracker_free (rift_tracker_ctx *tracker_ctx)
{
	int i;

	if (!tracker_ctx)
		return;

	for (i = 0; i < tracker_ctx->n_sensors; i++) {
		rift_sensor_ctx *sensor_ctx = tracker_ctx->sensors[i];
		rift_sensor_free (sensor_ctx);
	}

	for (i = 0; i < RIFT_MAX_TRACKED_DEVICES; i++) {
		rift_tracked_device_priv *dev = tracker_ctx->devices + i;
		if (dev->base.led_search)
			led_search_model_free (dev->base.led_search);
		if (dev->debug_metadata != NULL)
			ohmd_pw_debug_stream_free (dev->debug_metadata);

		rift_kalman_6dof_clear(&dev->ukf_fusion);
		ohmd_destroy_mutex (dev->device_lock);
	}

	/* Stop USB event thread */
	tracker_ctx->usb_completed = true;
	ohmd_destroy_thread (tracker_ctx->usb_thread);

	if (tracker_ctx->usb_ctx)
		libusb_exit (tracker_ctx->usb_ctx);

	ohmd_destroy_mutex (tracker_ctx->tracker_lock);
	free (tracker_ctx);
}

void rift_tracked_device_imu_update(rift_tracked_device *dev_base, uint64_t local_ts, uint32_t device_ts, float dt, const vec3f* ang_vel, const vec3f* accel, const vec3f* mag_field)
{
	rift_tracked_device_priv *dev = (rift_tracked_device_priv *) (dev_base);
	rift_tracked_device_imu_observation *obs;

	ohmd_lock_mutex (dev->device_lock);

	/* Handle device_ts wrap by extending to 64-bit and working in nanoseconds */
	if (dev->device_time_ns == 0) {
		dev->device_time_ns = device_ts * 1000;
	} else {
		uint64_t dt_ns = ((uint32_t)(device_ts - dev->last_device_ts)) * 1000;
		dev->device_time_ns += dt_ns;
	}
	dev->last_device_ts = device_ts;

	rift_kalman_6dof_imu_update (&dev->ukf_fusion, dev->device_time_ns, ang_vel, accel, mag_field);

	obs = dev->pending_imu_observations + dev->num_pending_imu_observations;
	obs->local_ts = local_ts;
	obs->device_ts = dev->device_time_ns;
	obs->dt = dt;
	obs->ang_vel = *ang_vel;
	obs->accel = *accel;
	obs->mag = *mag_field;

	dev->num_pending_imu_observations++;

	if (dev->num_pending_imu_observations == RIFT_MAX_PENDING_IMU_OBSERVATIONS) {
		/* No camera observations for a while - send our observations from here instead */
		rift_tracked_device_send_imu_debug(dev);
	}

	ohmd_unlock_mutex (dev->device_lock);
}

void rift_tracked_device_get_view_pose(rift_tracked_device *dev_base, posef *pose, vec3f *vel, vec3f *accel, vec3f *ang_vel)
{
	rift_tracked_device_priv *dev = (rift_tracked_device_priv *) (dev_base);

	ohmd_lock_mutex (dev->device_lock);

	if (dev->device_time_ns > dev->last_reported_pose) {
		posef device_pose;
		posef imu_global_pose;
		vec3f imu_ang_vel = { 0, };
		vec3f imu_vel = { 0, }, imu_accel = { 0, };

		rift_kalman_6dof_get_pose_at(&dev->ukf_fusion, dev->device_time_ns, &imu_global_pose, &imu_vel, &imu_accel, &imu_ang_vel, NULL, NULL);

		/* Take our fusion / IMU global pose back to device pose by
		 * computing the IMU->device pose and applying the
		 * IMU->world pose to get device->world pose */
		oposef_apply(&dev->device_from_fusion, &imu_global_pose, &device_pose);

		dev->reported_pose.orient = device_pose.orient;
		if (dev->device_time_ns - dev->last_observed_pose_ts >= (POSE_LOST_THRESHOLD * 1000000UL)) {
		        /* Don't let the device move unless there's a recent observation of actual position */
		        device_pose.pos = dev->reported_pose.pos;
		        imu_vel.x = imu_vel.y = imu_vel.z = 0.0;
		        imu_accel.x = imu_accel.y = imu_accel.z = 0.0;
		}

		exp_filter_pose_run(&dev->pose_output_filter, dev->device_time_ns, &device_pose, &dev->reported_pose);
		dev->last_reported_pose = dev->device_time_ns;

		/* Angular Velocity and acceleration need rotating into the device space.
		 * Linear velocity should also acquire a component from angular velocity */
		oquatf_get_rotated(&dev->device_from_fusion.orient, &imu_ang_vel, &dev->reported_ang_vel);
		oquatf_get_rotated(&dev->device_from_fusion.orient, &imu_accel, &dev->reported_lin_accel);

		/* Linear velocity generated by the angular velocity at the IMU offset
		 * is the cross product of the (rotated) position and the angular
		 * velocity */
		vec3f rotated_imu_pos, extra_lin_vel;
		oquatf_get_rotated(&dev->device_from_fusion.orient, &dev->device_from_fusion.pos, &rotated_imu_pos);
		ovec3f_cross(&dev->reported_ang_vel, &rotated_imu_pos, &extra_lin_vel);

		oquatf_get_rotated(&dev->device_from_fusion.orient, &imu_vel, &dev->reported_lin_vel);
		ovec3f_add(&dev->reported_lin_vel, &dev->reported_lin_vel, &extra_lin_vel);
	}

	if (pose)
		*pose = dev->reported_pose;
	if (ang_vel)
		*ang_vel = dev->reported_ang_vel;
	if (accel)
		*accel = dev->reported_lin_accel;
	if (vel)
		*vel = dev->reported_lin_vel;

	ohmd_unlock_mutex (dev->device_lock);
}

static rift_tracker_pose_delay_slot *get_matching_delay_slot(rift_tracked_device_priv *dev, rift_tracked_device_exposure_info *dev_info);

/* Retrieve the latest model pose estimate from a delay slot into the exposure info.
 * Because we can receive pose updates and new IMU data between frame capture and
 * when we go to do a visual search, and those can improve the estimate of the
 * pose estimate we had when the exposure happened */
bool rift_tracked_device_get_latest_exposure_info_pose (rift_tracked_device *dev_base, rift_tracked_device_exposure_info *dev_info)
{
	rift_tracked_device_priv *dev = (rift_tracked_device_priv *) (dev_base);
	rift_tracker_pose_delay_slot *slot = NULL;
	bool res = false;

	if (dev_info->fusion_slot == -1)
		return false;

	ohmd_lock_mutex (dev->device_lock);

	slot = get_matching_delay_slot(dev, dev_info);
	if (slot != NULL) {
		posef imu_global_pose;
		vec3f global_pos_error, global_rot_error;

		rift_kalman_6dof_get_delay_slot_pose_at(&dev->ukf_fusion, dev_info->device_time_ns, slot->slot_id, &imu_global_pose,
						NULL, NULL, NULL, &global_pos_error, &global_rot_error);

		oposef_apply(&dev->model_from_fusion, &imu_global_pose, &dev_info->capture_pose);

		int i;
		for (i = 0; i < 3; i++) {
			if (global_rot_error.arr[i] < MIN_ROT_ERROR)
				global_rot_error.arr[i] = MIN_ROT_ERROR;
			if (global_pos_error.arr[i] < MIN_POS_ERROR)
				global_pos_error.arr[i] = MIN_POS_ERROR;
		}

		oquatf_get_rotated_abs(&dev->model_from_fusion.orient, &global_pos_error, &dev_info->pos_error);
		oquatf_get_rotated_abs(&dev->model_from_fusion.orient, &global_rot_error, &dev_info->rot_error);
		res = true;
	}
	else {
		/* If we failed to get the pose, it means the delay slot was overridden,
		 * so clear it in the device info */
		dev_info->fusion_slot = -1;
	}

	ohmd_unlock_mutex (dev->device_lock);

	return res;
}

bool rift_tracked_device_model_pose_update(rift_tracked_device *dev_base, uint64_t local_ts, uint64_t frame_start_local_ts, rift_tracker_exposure_info *exposure_info,
    rift_pose_metrics *score, posef *model_pose, const char *source)
{
	rift_tracked_device_priv *dev = (rift_tracked_device_priv *) (dev_base);
	uint64_t frame_device_time_ns = 0;
	rift_tracker_pose_delay_slot *slot = NULL;
	int frame_fusion_slot = -1;
	bool update_position = false;
	bool update_orientation = false;
	posef imu_pose;

	ohmd_lock_mutex (dev->device_lock);

	/* Apply the fusion->model pose on top of the passed model->global pose,
	 * to get the global IMU pose */
	oposef_apply(&dev->fusion_from_model, model_pose, &imu_pose);

	rift_tracked_device_send_imu_debug(dev);

	if (dev->index < exposure_info->n_devices) {
		/* This device existed when the exposure was taken and therefore has info */
		rift_tracked_device_exposure_info *dev_info = exposure_info->devices + dev->index;
		frame_device_time_ns = dev_info->device_time_ns;

		slot = get_matching_delay_slot(dev, dev_info);
		if (slot != NULL) {
			quatf orient_diff;
			vec3f pos_error, rot_error;

			ovec3f_subtract(&model_pose->pos, &dev_info->capture_pose.pos, &pos_error);

			oquatf_diff(&model_pose->orient, &dev_info->capture_pose.orient, &orient_diff);
			oquatf_normalize_me(&orient_diff);
			oquatf_to_rotation(&orient_diff, &rot_error);

			LOGD ("Got pose update for delay slot %d for dev %d, ts %llu (delay %f) orient %f %f %f %f diff %f %f %f pos %f %f %f diff %f %f %f from %s\n",
				slot->slot_id, dev->base.id,
				(unsigned long long) frame_device_time_ns, (double) (dev->device_time_ns - frame_device_time_ns) / 1000000000.0,
				model_pose->orient.x, model_pose->orient.y, model_pose->orient.z, model_pose->orient.w,
				rot_error.x, rot_error.y, rot_error.z,
				model_pose->pos.x, model_pose->pos.y, model_pose->pos.z,
				pos_error.x, pos_error.y, pos_error.z,
				source);

			/* If this observation was based on a prior, but position didn't match and we already received a newer observation,
			 * ignore it. */
			if (dev_info->had_pose_lock && !POSE_HAS_FLAGS(score, RIFT_POSE_MATCH_POSITION) && dev->last_observed_pose_ts > frame_device_time_ns) {
				update_position = false;
				LOGI("Ignoring position observation with error %f %f %f (prior stddev was %f %f %f)\n",
					pos_error.x, pos_error.y, pos_error.z,
					dev_info->pos_error.x, dev_info->pos_error.y, dev_info->pos_error.z);
			}
			else {
				update_position = true;
			}

			/* If we have a strong match, update both position and orientation */
			if (POSE_HAS_FLAGS(score, RIFT_POSE_MATCH_ORIENT)) {
				update_orientation = true;
				if ((dev->device_time_ns - dev->last_observed_orient_ts) > (POSE_LOST_ORIENT_THRESHOLD * 1000000UL)) {
					LOGI("Matched orientation after %f sec", (dev->device_time_ns - dev->last_observed_pose_ts) / 1000000000.0);
				}
				/* Only update the time if we're actually going to apply this matched orientation below */
				if (update_position)
					dev->last_observed_orient_ts = dev->device_time_ns;
			}
			else if ((dev->device_time_ns - dev->last_observed_orient_ts) > (POSE_LOST_ORIENT_THRESHOLD * 1000000UL)) {
				LOGI("Forcing orientation observation");
				update_orientation = true;
				/* Don't update the orientation match time here - only do that on an actual match */
			}
			else {
				/* FIXME: If roll and pitch are acceptable (the gravity vector matched), but yaw is out of spec, we could perhaps do a
				 * yaw-only update for this device and see if that brings it into matching orientation */
			}

			if (update_position) {
				if (update_orientation) {
					rift_kalman_6dof_pose_update(&dev->ukf_fusion, dev->device_time_ns, &imu_pose, slot->slot_id);
				} else {
					rift_kalman_6dof_position_update(&dev->ukf_fusion, dev->device_time_ns, &imu_pose.pos, slot->slot_id);
				}

				dev->last_observed_pose_ts = dev->device_time_ns;
				dev->last_observed_pose = imu_pose;
			}

			frame_fusion_slot = slot->slot_id;

			if (slot->n_pose_reports < MAX_SENSORS) {
				rift_tracker_pose_report *report = slot->pose_reports + slot->n_pose_reports;

				report->report_used = update_position;
				report->pose = imu_pose;
				report->score = *score;

				if (update_position)
					slot->n_used_reports++;
				slot->n_pose_reports++;
			}
		}
	}

	rift_tracked_device_send_debug_printf(dev, local_ts, ",\n{ \"type\": \"pose\", \"local-ts\": %llu, "
			"\"device-ts\": %u, \"frame-start-local-ts\": %llu, "
			"\"frame-local-ts\": %llu, \"frame-hmd-ts\": %u, "
			"\"frame-exposure-count\": %u, \"frame-device-ts\": %llu, \"frame-fusion-slot\": %d, "
			"\"source\": \"%s\", "
			"\"pos\" : [ %f, %f, %f ], "
			"\"orient\" : [ %f, %f, %f, %f ] }",
			(unsigned long long) local_ts, dev->last_device_ts,
			(unsigned long long) frame_start_local_ts,
			(unsigned long long) exposure_info->local_ts, exposure_info->hmd_ts,
			exposure_info->count,
			(unsigned long long) frame_device_time_ns, frame_fusion_slot,
			source,
			model_pose->pos.x, model_pose->pos.y, model_pose->pos.z,
			model_pose->orient.x, model_pose->orient.y, model_pose->orient.z, model_pose->orient.w);
	ohmd_unlock_mutex (dev->device_lock);

	return update_position || update_orientation;
}

/* Called with the device lock held */
void rift_tracked_device_get_model_pose_locked(rift_tracked_device_priv *dev, uint32_t device_ts, posef *pose, vec3f *pos_error, vec3f *rot_error)
{
	posef imu_global_pose, model_pose;
	vec3f global_pos_error, global_rot_error;

	rift_kalman_6dof_get_pose_at(&dev->ukf_fusion, dev->device_time_ns, &imu_global_pose, NULL, NULL, NULL, &global_pos_error, &global_rot_error);

	/* Apply the pose conversion from IMU->model */
	oposef_apply(&dev->model_from_fusion, &imu_global_pose, &model_pose);

	if (pos_error) {
		int i;

		for (i = 0; i < 3; i++) {
			if (global_pos_error.arr[i] < MIN_POS_ERROR)
				global_pos_error.arr[i] = MIN_POS_ERROR;
		}

		oquatf_get_rotated_abs(&dev->model_from_fusion.orient, &global_pos_error, pos_error);
	}
	if (rot_error) {
		int i;

		for (i = 0; i < 3; i++) {
			if (global_rot_error.arr[i] < MIN_ROT_ERROR)
				global_rot_error.arr[i] = MIN_ROT_ERROR;
		}

		oquatf_get_rotated_abs(&dev->model_from_fusion.orient, &global_rot_error, rot_error);
	}

	dev->model_pose.orient = model_pose.orient;
	if (dev->device_time_ns - dev->last_observed_pose_ts < (POSE_LOST_THRESHOLD * 1000000UL)) {
		/* Don't let the device move unless there's a recent observation of actual position */
		dev->model_pose.pos = model_pose.pos;
	}
	*pose = dev->model_pose;

	LOGD ("Reporting pose for dev %d, orient %f %f %f %f pos %f %f %f",
		dev->base.id,
		pose->orient.x, pose->orient.y, pose->orient.z, pose->orient.w,
		pose->pos.x, pose->pos.y, pose->pos.z);
}

/* Called with the device lock held */
static void
rift_tracked_device_send_imu_debug(rift_tracked_device_priv *dev)
{
	int i;

	if (dev->num_pending_imu_observations == 0)
		return;

	if (dev->debug_metadata && ohmd_pw_debug_stream_connected(dev->debug_metadata)) {
		char debug_str[1024];

		for (i = 0; i < dev->num_pending_imu_observations; i++) {
			rift_tracked_device_imu_observation *obs = dev->pending_imu_observations + i;

			snprintf (debug_str, 1024, ",\n{ \"type\": \"imu\", \"local-ts\": %llu, "
				 "\"device-ts\": %llu, \"dt\": %f, "
				 "\"ang_vel\": [ %f, %f, %f ], \"accel\": [ %f, %f, %f ], "
				 "\"mag\": [ %f, %f, %f ] }",
				(unsigned long long) obs->local_ts,
				(unsigned long long) obs->device_ts,
				obs->dt,
				obs->ang_vel.x, obs->ang_vel.y, obs->ang_vel.z,
				obs->accel.x, obs->accel.y, obs->accel.z,
				obs->mag.x, obs->mag.y, obs->mag.z);

			debug_str[1023] = '\0';

			ohmd_pw_debug_stream_push (dev->debug_metadata, obs->local_ts, debug_str);
		}
	}

	dev->num_pending_imu_observations = 0;
}

static void
rift_tracked_device_send_debug_printf(rift_tracked_device_priv *dev, uint64_t local_ts, const char *fmt, ...)
{
	if (dev->debug_metadata && ohmd_pw_debug_stream_connected(dev->debug_metadata)) {
		char debug_str[1024];
		va_list args;

		/* Send any pending IMU debug first */
		rift_tracked_device_send_imu_debug(dev);

		/* Print output string and send */
		va_start(args, fmt);
		vsnprintf(debug_str, 1024, fmt, args);
		va_end(args);

		debug_str[1023] = '\0';

		ohmd_pw_debug_stream_push (dev->debug_metadata, local_ts, debug_str);
	}
}

static rift_tracker_pose_delay_slot *
find_free_delay_slot(rift_tracked_device_priv *dev)
{
	/* Pose observation delay slots */
	for (int i = 0; i < NUM_POSE_DELAY_SLOTS; i++) {
		int slot_no = dev->delay_slot_index;
		rift_tracker_pose_delay_slot *slot = dev->delay_slots + slot_no;

		/* Cycle through the free delay slots */
		dev->delay_slot_index = (slot_no+1) % NUM_POSE_DELAY_SLOTS;

		if (slot->use_count == 0)
			return slot;
	}

	/* Failed to find a free slot */
	return NULL;
}

static rift_tracker_pose_delay_slot *
reclaim_delay_slot(rift_tracked_device_priv *dev)
{
	/* Pose observation delay slots */
	for (int i = 0; i < NUM_POSE_DELAY_SLOTS; i++) {
		rift_tracker_pose_delay_slot *slot = dev->delay_slots + i;

		/* If a slot already received a pose observation, use that one */
		/* FIXME: Check that the poses were integrated, and integrate them as-needed if not */
		if (slot->valid && slot->n_used_reports > 0)
			return slot;
	}

	/* Failed to find a free slot */
	return NULL;
}


static rift_tracker_pose_delay_slot *
get_matching_delay_slot(rift_tracked_device_priv *dev, rift_tracked_device_exposure_info *dev_info)
{
	rift_tracker_pose_delay_slot *slot = NULL;
	int slot_no = dev_info->fusion_slot;

	if (slot_no >= 0 && slot_no < NUM_POSE_DELAY_SLOTS) {
		slot = dev->delay_slots + slot_no;
	}

	if (slot && slot->valid && slot->device_time_ns == dev_info->device_time_ns)
		return slot;

	return NULL;
}

/* Called with the device lock held. Allocate a delay slot and populate the device exposure info */
static void
rift_tracked_device_on_new_exposure(rift_tracked_device_priv *dev, rift_tracked_device_exposure_info *dev_info) {
	rift_tracker_pose_delay_slot *slot = find_free_delay_slot(dev);

	dev_info->device_time_ns = dev->device_time_ns;

	if (slot == NULL) {
		/* We might reclaim a busy delay slot if some frame search is being slow and we already got an observation from another camera */
		slot = reclaim_delay_slot(dev);
		if (slot) {
			LOGI ("Reclaimed delay slot %d for dev %d, ts %llu (delay %f)", slot->slot_id, dev->base.id, (unsigned long long) dev->device_time_ns,
				(double) (dev->device_time_ns - slot->device_time_ns) / 1000000000.0);
		}
	}

	if (dev->device_time_ns - dev->last_observed_pose_ts < (POSE_LOST_THRESHOLD * 1000000UL))
		dev_info->had_pose_lock = true;
	else
		dev_info->had_pose_lock = false;

	rift_tracked_device_get_model_pose_locked(dev, dev->device_time_ns, &dev_info->capture_pose, &dev_info->pos_error, &dev_info->rot_error);

	if (slot) {
		slot->device_time_ns = dev_info->device_time_ns;
		slot->valid = true;
		slot->use_count = 0;
		slot->n_pose_reports = 0;
		slot->n_used_reports = 0;

		LOGD ("Assigning free delay slot %d for dev %d, ts %llu", slot->slot_id, dev->base.id, (unsigned long long) dev->device_time_ns);
		dev_info->fusion_slot = slot->slot_id;

		/* Tell the kalman filter to prepare the delay slot */
		rift_kalman_6dof_prepare_delay_slot(&dev->ukf_fusion, dev_info->device_time_ns, slot->slot_id);

		/* Clear the last no-free-delay-slot tracking to avoid logging noise */
		dev->last_no_free_delay_slot = 0;
	}
	else {
		if (dev->last_no_free_delay_slot != 0 && (dev->device_time_ns - dev->last_no_free_delay_slot > (NO_FREE_DELAY_SLOT_THRESHOLD * 1000000UL))) {
			LOGW("No free delay slot for dev %d @ ts %llu (for %ums now)", dev->base.id,
				  (unsigned long long) dev->device_time_ns, NO_FREE_DELAY_SLOT_THRESHOLD);
		}
		dev->last_no_free_delay_slot = dev->device_time_ns;
		dev_info->fusion_slot = -1;
	}
}

static int
rift_tracked_device_exposure_claim(rift_tracked_device_priv *dev, rift_tracked_device_exposure_info *dev_info)
{
	rift_tracker_pose_delay_slot *slot = get_matching_delay_slot(dev, dev_info);

	/* There is a delay slot for this frame, claim it */
	if (slot) {
		slot->use_count++;

		LOGD ("Claimed delay slot %d for dev %d, ts %llu. use_count now %d",
			slot->slot_id, dev->base.id, (unsigned long long) dev_info->device_time_ns, slot->use_count);

		return slot->slot_id;
	}
	else {
		/* The slot was not allocated (we missed the exposure event), or it
		 * was overridden by a later exposure because there's not enough slots */
		if (dev_info->fusion_slot != -1) {
#if LOGLEVEL == 0
			rift_tracker_pose_delay_slot *slot = dev->delay_slots + dev_info->fusion_slot;

			LOGD ("Lost delay slot %d for dev %d, ts %llu (slot valid %d ts %llu)",
				dev_info->fusion_slot, dev->base.id, (unsigned long long) dev_info->device_time_ns,
				slot->valid, (unsigned long long) slot->device_time_ns);
#endif
		}
	}

	return -1;
}

static void
rift_tracked_device_exposure_release_locked(rift_tracked_device_priv *dev, rift_tracked_device_exposure_info *dev_info)
{
	rift_tracker_pose_delay_slot *slot = get_matching_delay_slot(dev, dev_info);

	/* There is a delay slot for this frame, release it */
	if (slot) {
		if (slot->use_count > 0) {
			slot->use_count--;
			LOGD ("Released delay slot %d for dev %d, ts %llu. use_count now %d",
				dev_info->fusion_slot, dev->base.id, (unsigned long long) dev_info->device_time_ns,
				slot->use_count);
		}
	}
}

void rift_tracked_device_frame_release(rift_tracked_device *dev_base, rift_tracker_exposure_info *exposure_info)
{
	rift_tracked_device_priv *dev = (rift_tracked_device_priv *) (dev_base);

	ohmd_lock_mutex (dev->device_lock);
	if (dev->index < exposure_info->n_devices) {
		/* This device existed when the exposure was taken and therefore has info */
		rift_tracked_device_exposure_info *dev_info = exposure_info->devices + dev->index;
		rift_tracked_device_exposure_release_locked(dev, dev_info);
	}
	ohmd_unlock_mutex (dev->device_lock);
}
