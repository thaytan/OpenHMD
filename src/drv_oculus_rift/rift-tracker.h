// Copyright 2020, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Oculus Rift driver - positional tracking interface */

#include "rift.h"
#include "rift-kalman-6dof.h"
#include "correspondence_search.h"
#include "ohmd-pipewire.h"

#ifndef __RIFT_TRACKER_H__
#define __RIFT_TRACKER_H__

typedef struct rift_tracker_ctx_s rift_tracker_ctx;

#define RIFT_MAX_TRACKED_DEVICES 3

#define RIFT_MAX_PENDING_IMU_OBSERVATIONS 1000

typedef struct rift_tracked_device_imu_observation rift_tracked_device_imu_observation;
typedef struct rift_tracker_exposure_info rift_tracker_exposure_info;

struct rift_tracker_exposure_info {
	uint64_t local_ts;
	uint32_t hmd_ts;

	uint16_t count;
	uint8_t led_pattern_phase;
};

struct rift_tracked_device_imu_observation {
	uint64_t local_ts;
	uint32_t device_ts;
	float dt;

	vec3f ang_vel;
	vec3f accel;
	vec3f mag;

	quatf simple_orient;
};

struct rift_tracked_device_s
{
	int id;
	rift_leds *leds;
	led_search_model_t *led_search;

	ohmd_mutex *device_lock;
	fusion simple_fusion;

	/* 6DOF Kalman Filter */
	rift_kalman_6dof_filter ukf_fusion;

	/* Transform from the fusion pose (which tracks the IMU, oriented to the screens/view)
	 * to the model the camera will see, which is offset and rotated 180 degrees */
	posef fusion_to_model;

	/* The model (HMD/controller) pose -> world transform */
	posef pose;

	uint32_t last_device_ts;
  uint64_t device_time_ns;

	int num_pending_imu_observations;
	rift_tracked_device_imu_observation pending_imu_observations[RIFT_MAX_PENDING_IMU_OBSERVATIONS];

	ohmd_pw_debug_stream *debug_metadata;
};

rift_tracker_ctx *rift_tracker_new (ohmd_context* ohmd_ctx,
		const uint8_t radio_id[5]);

rift_tracked_device *rift_tracker_add_device (rift_tracker_ctx *ctx, int device_id, posef *imu_pose, rift_leds *leds);
void rift_tracker_update_exposure (rift_tracker_ctx *ctx, uint32_t hmd_ts, uint16_t exposure_count, uint32_t exposure_hmd_ts, uint8_t led_pattern_phase);
bool rift_tracker_get_exposure_info (rift_tracker_ctx *ctx, rift_tracker_exposure_info *info);
uint8_t rift_tracker_get_device_list(rift_tracker_ctx *tracker_ctx, rift_tracked_device **dev_list);

void rift_tracker_free (rift_tracker_ctx *ctx);

void rift_tracked_device_imu_update(rift_tracked_device *dev, uint64_t local_ts, uint32_t device_ts, float dt, const vec3f* ang_vel, const vec3f* accel, const vec3f* mag_field);
void rift_tracked_device_get_view_pose(rift_tracked_device *dev, posef *pose);

void rift_tracked_device_model_pose_update(rift_tracked_device *dev, uint64_t local_ts, rift_tracker_exposure_info *exposure_info, posef *pose);
void rift_tracked_device_get_model_pose(rift_tracked_device *dev, double ts, posef *pose, float *gravity_error_rad);

#endif
