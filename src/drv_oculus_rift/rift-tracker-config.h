// Copyright 2021, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Oculus Rift driver - tracking config store */

#include "rift-tracker-common.h"
#include "rift-sensor.h"

#ifndef __RIFT_TRACKER_CONFIG_H__
#define __RIFT_TRACKER_CONFIG_H__

typedef struct rift_tracker_config_s rift_tracker_config;
typedef struct rift_tracker_sensor_config_s rift_tracker_sensor_config;

struct rift_tracker_sensor_config_s {
	char serial_no[RIFT_SENSOR_SERIAL_LEN+1];
	posef pose;
};

struct rift_tracker_config_s {
	bool modified;

	/* Room configuration */
	vec3f room_center_offset;

	int n_sensors;
	rift_tracker_sensor_config sensors[RIFT_MAX_SENSORS];
};

void rift_tracker_config_init(rift_tracker_config *config);
void rift_tracker_config_load(ohmd_context *ctx, rift_tracker_config *config);
void rift_tracker_config_save(ohmd_context *ctx, rift_tracker_config *config);

void rift_tracker_config_get_center_offset(rift_tracker_config *config, vec3f *room_center_offset);

void rift_tracker_config_set_sensor_pose(rift_tracker_config *config, const char *serial_no, posef *pose);
bool rift_tracker_config_get_sensor_pose(rift_tracker_config *config, const char *serial_no, posef *pose);

#endif

