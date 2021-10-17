// Copyright 2020, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Common definitions for tracker and sensor code */

#include "rift.h"
#include "rift-sensor-maths.h"
#include "led_search.h"

#ifndef __RIFT_TRACKER_COMMON_H__
#define __RIFT_TRACKER_COMMON_H__

#define RIFT_MAX_TRACKED_DEVICES 3

/* Side-on angle at which LEDs become occluded */
#define RIFT_LED_ANGLE 75

typedef struct rift_tracker_exposure_info rift_tracker_exposure_info;
typedef struct rift_tracked_device_exposure_info rift_tracked_device_exposure_info;
typedef struct rift_tracked_device_imu_calibration rift_tracked_device_imu_calibration;

struct rift_tracked_device_exposure_info {
	uint64_t device_time_ns; /* Device time this sensor exposure was captured */
	int fusion_slot; /* Fusion slot assigned to the exposure, or -1 */

	/* TRUE if we had a recent pose prior for this device at exposure time */
	bool had_pose_lock;

	/* World pose and error std dev at exposure time */
	posef capture_pose;
	vec3f pos_error;
	vec3f rot_error;
};

struct rift_tracker_exposure_info {
	uint64_t local_ts;
	uint32_t hmd_ts;

	uint16_t count;
	uint8_t led_pattern_phase;

	/* Per device info */
	uint8_t n_devices;
	rift_tracked_device_exposure_info devices[RIFT_MAX_TRACKED_DEVICES];
};

struct rift_tracked_device_s
{
	int id;

	rift_leds *leds;

	led_search_model_t *led_search;
};

struct rift_tracked_device_imu_calibration {
	vec3f accel_offset;
	float accel_matrix[9];

	vec3f gyro_offset;
	float gyro_matrix[9];
};


#endif
