// Copyright 2013, Fredrik Hultin.
// Copyright 2013, Jakob Bornecrantz.
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Sensor Fusion */


#ifndef FUSION_H
#define FUSION_H

#include "omath.h"

#define FF_USE_GRAVITY 1

typedef struct {
	int state;

	quatf orient;   // orientation
	vec3f accel;    // acceleration
	vec3f ang_vel;  // angular velocity
	vec3f mag;      // magnetometer
	vec3f raw_mag;  // raw magnetometer values

	int iterations;
	double time;

	int flags;

	// filter queues for magnetometer, accelerometers and angular velocity
	filter_queue mag_fq, accel_fq, ang_vel_fq;

	// gravity correction
	int device_level_count;
	float grav_error_angle;
	vec3f grav_error_axis;
	float grav_gain; // amount of correction

	// world coordinates velocity and position tracking
	vec3f world_vel;
	vec3f world_position;
	float accel_mean;

	double last_output_time;
	double last_tracker_obs_time;
	vec3f last_tracker_position;
	double last_gravity_vector_time;
} fusion;

void ofusion_init(fusion* me);
void ofusion_update(fusion* me, float dt, const vec3f* ang_vel, const vec3f* accel, const vec3f* mag_field);
void ofusion_update_at (fusion* me, double time, const vec3f* ang_vel, const vec3f* accel, const vec3f* mag_field);
void ofusion_tracker_update(fusion* me, double time, const vec3f* pos, const quatf *orient);

#endif
