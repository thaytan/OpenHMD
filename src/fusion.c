// Copyright 2013, Fredrik Hultin.
// Copyright 2013, Jakob Bornecrantz.
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Sensor Fusion Implementation */


#include <string.h>
#include "openhmdi.h"

#define GRAVITY_CONSTANT (9.82f)
#define GRAVITY_TOLERANCE (0.4f) // Hack for rift for now

void ofusion_init(fusion* me)
{
	memset(me, 0, sizeof(fusion));
	me->orient.w = 1.0f;

	ofq_init(&me->mag_fq, 20);
	ofq_init(&me->accel_fq, 20);
	ofq_init(&me->ang_vel_fq, 20);

	me->time = 0.0;
	me->flags = FF_USE_GRAVITY;
	me->grav_gain = 0.05f;
	me->accel_mean = GRAVITY_CONSTANT;

	/* Assume we start at rest at world 0,0,0 */
	ovec3f_set(&me->world_position, 0, 0, 0);
	ovec3f_set(&me->world_vel, 0, 0, 0);

	me->last_output_time = 0.0;
	me->last_tracker_obs_time = 0.0;
	me->last_gravity_vector_time = 0.0;
}

static void ofusion_update_dt(fusion* me, float dt, const vec3f* ang_vel, const vec3f* accel, const vec3f* mag)
{
	double now = ohmd_get_tick();
	vec3f world_accel;

	oquatf_get_rotated(&me->orient, accel, &world_accel);
	me->iterations += 1;

	me->ang_vel = *ang_vel;
	ofq_add(&me->ang_vel_fq, ang_vel);

	me->accel = *accel;
	ofq_add(&me->accel_fq, &world_accel);

	if (mag) {
		me->raw_mag = *mag;
		me->mag = *mag;
		ofq_add(&me->mag_fq, mag);
	}

	// Accumulate our rolling average of gravity
	int iters = OHMD_CLAMP (me->iterations, 1000, 10000);
	me->accel_mean = (iters*me->accel_mean + ovec3f_get_length(accel)) / (iters+1);

	float ang_vel_length = ovec3f_get_length(ang_vel);
	if(ang_vel_length > 0.0001f){
		vec3f rot_axis =
			{{ ang_vel->x / ang_vel_length, ang_vel->y / ang_vel_length, ang_vel->z / ang_vel_length }};

		float rot_angle = ang_vel_length * dt;

		quatf delta_orient;
		oquatf_init_axis(&delta_orient, &rot_axis, rot_angle);

		oquatf_mult_me(&me->orient, &delta_orient);
	}

	// gravity correction
	if(me->flags & FF_USE_GRAVITY){
		const float gravity_tolerance = GRAVITY_TOLERANCE, ang_vel_tolerance = .1f;
		const float min_tilt_error = 0.05f, max_tilt_error = 0.01f;

		// if the device is within tolerance levels, count this as the device is level and add to the counter
		// otherwise reset the counter and start over

		me->device_level_count =
			fabsf(ovec3f_get_length(accel) - me->accel_mean) < gravity_tolerance * 2.0f && ang_vel_length < ang_vel_tolerance
			? me->device_level_count + 1 : 0;

		// device has been level for long enough, grab mean from the accelerometer filter queue (last n values)
		// and use for correction

		if(me->device_level_count > 50){
			me->device_level_count = 0;

			vec3f accel_mean;
			ofq_get_mean(&me->accel_fq, &accel_mean);
			if (ovec3f_get_length(&accel_mean) - GRAVITY_CONSTANT < gravity_tolerance)
			{
				// Calculate a cross product between what the device
				// thinks is up and what gravity indicates is down.
				// The values are optimized of what we would get out
				// from the cross product.
				vec3f tilt = {{accel_mean.z, 0, -accel_mean.x}};

				ovec3f_normalize_me(&tilt);
				ovec3f_normalize_me(&accel_mean);

				vec3f up = {{0, 1.0f, 0}};
				float tilt_angle = ovec3f_get_angle(&up, &accel_mean);

				if(tilt_angle > max_tilt_error){
					me->grav_error_angle = tilt_angle;
					me->grav_error_axis = tilt;
				}
			}
			me->last_gravity_vector_time = now;
		}

		// perform gravity tilt correction
		if(me->grav_error_angle > min_tilt_error){
			float use_angle;
			// if less than 2000 iterations have passed, set the up axis to the correction value outright
			if(me->iterations < 2000){
				use_angle = -me->grav_error_angle;
				me->grav_error_angle = 0;
			}

			// otherwise try to correct
			else {
				use_angle = -me->grav_gain * me->grav_error_angle * 0.005f * (5.0f * ang_vel_length + 1.0f);
				me->grav_error_angle += use_angle;
			}

			// perform the correction
			quatf corr_quat, old_orient;
			oquatf_init_axis(&corr_quat, &me->grav_error_axis, use_angle);
			old_orient = me->orient;

			oquatf_mult(&corr_quat, &old_orient, &me->orient);
		}
	}

	// mitigate drift due to floating point
	// inprecision with quat multiplication.
	oquatf_normalize_me(&me->orient);

	/* Skip position interpolation at first until we've had
	 * time to measure gravity, otherwise the local acceleration
	 * information can't be trusted anyway */
	if (now - me->last_gravity_vector_time < 1.0) {
		// Take the new corrected orientation and re-convert the accel to world accel
		// then subtract expected gravity acceleration, and double integrate to get
		// new position / velocity
		vec3f gravity_vec = {{0, me->accel_mean, 0}};
		vec3f delta_vel, delta_position, tmp;
		oquatf_get_rotated(&me->orient, accel, &world_accel);

#if 0
		printf ("dt %f accel %f,%f,%f (mag %f) world %f,%f,%f (mag %f)\n", dt,
				accel->x, accel->y, accel->z, ovec3f_get_length(accel),
				world_accel.x, world_accel.y, world_accel.z, ovec3f_get_length(&world_accel));
#endif

		/* Subtract gravity vector to get local acceleration */
		vec3f local_accel;
		ovec3f_subtract (&world_accel, &gravity_vec, &local_accel);

		// new position = old_velocity*dt + 0.5 * accel*dt*dt
		ovec3f_multiply_scalar (&me->world_vel, dt, &delta_position);
		ovec3f_multiply_scalar (&local_accel, 0.5*dt*dt, &tmp);
		ovec3f_add (&delta_position, &tmp, &delta_position);

		/* If we've seen a tracker observation in the last half second,
		 * then interpolate position. Stop after 0.5 seconds, before
		 * we drift too far. */
		/* TODO; use filtering that refines the sensor bias information
		 * that to do more accurate position interpolation,
		 * Use state variance to know if our estimate of acceleration/
		 * position are well known enough to do the interpolation */
		if (now - me->last_tracker_obs_time < 0.5)
			ovec3f_add (&me->world_position, &delta_position, &me->world_position);

		// new velocity = old_velocity + accel*dt
		ovec3f_multiply_scalar (&local_accel, dt, &delta_vel);
		ovec3f_add (&me->world_vel, &delta_vel, &me->world_vel);
		// clamp to 20m/s (9.8 for Y)
		me->world_vel.x = OHMD_CLAMP (me->world_vel.x, -20.0, 20.0);
		me->world_vel.y = OHMD_CLAMP (me->world_vel.y, -GRAVITY_CONSTANT, 20);
		me->world_vel.z = OHMD_CLAMP (me->world_vel.z, -20, 20);

		if (ovec3f_get_length (&me->world_position) > 50.0)
				ovec3f_multiply_scalar (&me->world_vel, -1.0, &me->world_vel);
	}
}

void ofusion_update(fusion* me, float dt, const vec3f* ang_vel, const vec3f* accel, const vec3f* mag)
{
	me->time += dt;
	ofusion_update_dt (me, dt, ang_vel, accel, mag);
}

void ofusion_update_at (fusion* me, double time, const vec3f* ang_vel, const vec3f* accel, const vec3f* mag)
{
	float dt = time - me->time;
	ofusion_update_dt (me, dt, ang_vel, accel, mag);
}

void ofusion_tracker_update(fusion* me, double time, const vec3f* pos, const quatf *orient)
{
	double now = time;
	double dt = now - me->last_tracker_obs_time;

	// TODO: Use Kalman filtering
	// For now, directly update the time, pose and position for now, and calculate
	// some max velocity based on error from the current position
	// TODO: Add variance information to the data, about how certain the pose is,
	// in each dimension, based on tracker-specific information such as
	// observation angle and distance.
	//
	// For the pose, only use it to update the yaw, and take the gravity
	// vector from the IMU
	if (me->last_tracker_obs_time != 0 && dt > 0 && dt < 0.3) {
		vec3f dist;
		ovec3f_subtract(pos, &me->last_tracker_position, &dist);
		ovec3f_multiply_scalar(&dist, 1.0/dt, &me->world_vel);
		LOGD("Calculated velocity time %f (%f,%f,%f)/%f = (%f,%f,%f) m/s",
				time, dist.x, dist.y, dist.z, dt,
				me->world_vel.x, me->world_vel.y, me->world_vel.z);
	}	else {
		ovec3f_set(&me->world_vel, 0, 0, 0);
	}

	me->last_tracker_position = me->world_position = *pos;

	// Calculate a correction quat between what the fusion
	// thinks is forward and what the tracker thinks is
	// forward
	vec3f forward = {{0.0, 0.0, 1.0 }};
	vec3f up = {{0.0, 1.0, 0.0 }};
	vec3f cur_yaw, new_yaw;

	oquatf_get_rotated(&me->orient, &forward, &cur_yaw);
	oquatf_get_rotated(orient, &forward, &new_yaw);

	/* We're only interested in the XZ plane rotation */
	cur_yaw.y = 0.0; new_yaw.y = 0.0;

	float yaw_angle_error = ovec3f_get_angle(&cur_yaw, &new_yaw);

	/* Check if the angle needs reversing. This is a simplified cross product
	 * to check the direction, based on knowing the normal is the Y axis */
	if (cur_yaw.z*new_yaw.x - cur_yaw.x*new_yaw.z < 0.0)
		yaw_angle_error = -yaw_angle_error;

	LOGD("fusion prior orient %f %f %f %f forward %f %f %f yaw error %f deg",
		me->orient.x, me->orient.y, me->orient.z, me->orient.w,
		cur_yaw.x, cur_yaw.y, cur_yaw.z, RAD_TO_DEG(yaw_angle_error));
	LOGD("fusion new orient %f %f %f %f forward %f %f %f",
		orient->x, orient->y, orient->z, orient->w,
		new_yaw.x, new_yaw.y, new_yaw.z);

	/* Apply a small correction to yaw, to push it toward the forward direction */
	float correction_angle = 0.05 * yaw_angle_error;

	quatf corr_quat, old_orient;
	oquatf_init_axis(&corr_quat, &up, correction_angle);
	old_orient = me->orient;
	oquatf_mult(&corr_quat, &old_orient, &me->orient);

	me->last_tracker_obs_time = now;
}

