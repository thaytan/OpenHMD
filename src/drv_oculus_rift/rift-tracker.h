// Copyright 2020, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Oculus Rift driver - positional tracking interface */

#include "rift.h"
#include "rift-kalman-6dof.h"
#include "rift-tracker-common.h"
#include "rift-sensor-pose-helper.h"

#ifndef __RIFT_TRACKER_H__
#define __RIFT_TRACKER_H__

typedef struct rift_tracker_ctx_s rift_tracker_ctx;

rift_tracker_ctx *rift_tracker_new (ohmd_context* ohmd_ctx, const uint8_t radio_id[5]);

rift_tracked_device *rift_tracker_add_device (rift_tracker_ctx *ctx, int device_id, posef *imu_pose, posef *model_pose, rift_leds *leds);
void rift_tracker_on_new_exposure (rift_tracker_ctx *ctx, uint32_t hmd_ts, uint16_t exposure_count, uint32_t exposure_hmd_ts, uint8_t led_pattern_phase);
bool rift_tracker_get_exposure_info (rift_tracker_ctx *ctx, rift_tracker_exposure_info *info);
uint8_t rift_tracker_get_device_list(rift_tracker_ctx *tracker_ctx, rift_tracked_device **dev_list);

void rift_tracker_frame_start (rift_tracker_ctx *ctx, uint64_t local_ts, const char *source, rift_tracker_exposure_info *info);
void rift_tracker_frame_changed_exposure(rift_tracker_ctx *ctx, rift_tracker_exposure_info *old_info, rift_tracker_exposure_info *new_info);
void rift_tracker_frame_captured (rift_tracker_ctx *ctx, uint64_t local_ts, uint64_t frame_start_local_ts, rift_tracker_exposure_info *info, const char *source);
void rift_tracker_frame_release (rift_tracker_ctx *ctx, uint64_t local_ts, uint64_t frame_local_ts, rift_tracker_exposure_info *info, const char *source);

void rift_tracker_free (rift_tracker_ctx *ctx);

void rift_tracked_device_imu_update(rift_tracked_device *dev, uint64_t local_ts, uint32_t device_ts, float dt, const vec3f* ang_vel, const vec3f* accel, const vec3f* mag_field);
void rift_tracked_device_get_view_pose(rift_tracked_device *dev, posef *pose, vec3f *vel, vec3f *accel, vec3f *ang_vel);

bool rift_tracked_device_get_latest_exposure_info_pose (rift_tracked_device *dev, rift_tracked_device_exposure_info *dev_info);

bool rift_tracked_device_model_pose_update(rift_tracked_device *dev_base, uint64_t local_ts, uint64_t frame_start_local_ts, rift_tracker_exposure_info *exposure_info, rift_pose_metrics *score, posef *pose, const char *source);
void rift_tracked_device_get_model_pose(rift_tracked_device *dev_base, double ts, posef *pose, vec3f *pos_error, vec3f *rot_error);
void rift_tracked_device_frame_release (rift_tracked_device *dev_base, rift_tracker_exposure_info *info);

#endif
