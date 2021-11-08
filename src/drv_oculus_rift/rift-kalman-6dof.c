// Copyright 2020, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 *
 * 6DOF Unscented Kalman Filter for positional tracking
 *
 */

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "rift-kalman-6dof.h"

/* 0 = constant acceleration
 * 1 = constant velocity
 * 2 = hybrid constant acccel / constant velocity
 */
#define MOTION_MODEL 2

/* threshold for hybrid motion model switching */
#define HYBRID_MOTION_THRESHOLD (3 / 1000.0)

/* IMU biases noise levels */
#define IMU_GYRO_BIAS_NOISE 1e-17 /* gyro bias (rad/s)^2 */
#define IMU_GYRO_BIAS_NOISE_INITIAL 1e-3 /* gyro bias (rad/s)^2 */
#define IMU_ACCEL_BIAS_NOISE 1e-16 /* accelerometer bias (m/s^2)^2 */
#define IMU_ACCEL_BIAS_NOISE_INITIAL 0.25 /* accelerometer bias (m/s^2)^2 */

/* Acceleration and Velocity damping factor (1.0 = undamped) */
#define ACCEL_DAMP 0.9
#define VEL_DAMP 0.999

#define GRAVITY_MAG 9.80665

/* Maximum sensible acceleration */
const double MAX_ACCEL = 16.0 * GRAVITY_MAG;

typedef struct imu_filter_state imu_filter_state;

/* The state vector is bigger than the covariance
 * matrices, because it includes the quaternion, but
 * the covariance is parameterised with an exponential
 * map */
const int BASE_STATE_SIZE = 19;
const int BASE_COV_SIZE = 18;

/* A lagged slot consists of orientation + position */
const int DELAY_SLOT_STATE_SIZE = 7;
const int DELAY_SLOT_COV_SIZE = 6;

#define STATE_ORIENTATION 0
#define STATE_POSITION 4
#define STATE_VELOCITY 7
#define STATE_ACCEL 10
#define STATE_ACCEL_BIAS 13
#define STATE_GYRO_BIAS 16

/* Indices into covariance / noise matrices for the state */
#define COV_ORIENTATION 0
#define COV_POSITION 3
#define COV_VELOCITY 6
#define COV_ACCEL 9
#define COV_ACCEL_BIAS 12
#define COV_GYRO_BIAS 15
/* Indices into a delay slot for orientation and posiion */
#define DELAY_SLOT_STATE_ORIENTATION 0
#define DELAY_SLOT_STATE_POSITION 4

#define DELAY_SLOT_COV_ORIENTATION 0
#define DELAY_SLOT_COV_POSITION 3

/* Indices into the IMU measurement vector */
#define IMU_MEAS_ACCEL 0

/* Indices into the pose measurement vector */
#define POSE_MEAS_POSITION 0
#define POSE_MEAS_ORIENTATION 3

#define NS_TO_SEC(t) ((double)(t) / 1000000000.0)

static bool process_func(const ukf_base *ukf, const double dt, const matrix2d *X_prior, matrix2d *X)
{
	/*
	 *	quatd orientation (quaternion) (0:3)
	 *
	 *	vec3d position (4:6)
	 *	vec3d velocity; (7:9)
	 *	vec3d accel; (10:12)
	 *
	 *	vec3d angular_velocity (13:15)
	 *
	 *	vec3d accel-bias; (16:18)
	 *	vec3d gyro-bias (19:21)
	 */
	rift_kalman_6dof_filter *filter_state = (rift_kalman_6dof_filter *)(ukf);

	/* Most of the state stays constant - constant acceleration, constant angular velocity, biases -
	 * so copy the whole state to start, then adjust the predicted part */
	matrix2d_copy(X, X_prior);

	/* Compute orientation update correctly using a delta quat from ang_vel */
	quatd orient = {{ MATRIX2D_Y(X_prior, STATE_ORIENTATION),
	                  MATRIX2D_Y(X_prior, STATE_ORIENTATION+1),
	                  MATRIX2D_Y(X_prior, STATE_ORIENTATION+2),
	                  MATRIX2D_Y(X_prior, STATE_ORIENTATION+3) }};

	vec3d imu_ang_vel;
	vec3d ang_vel_bias;
	vec3d accel_bias;

	ang_vel_bias.x = MATRIX2D_Y(X, STATE_GYRO_BIAS);
	ang_vel_bias.y = MATRIX2D_Y(X, STATE_GYRO_BIAS+1);
	ang_vel_bias.z = MATRIX2D_Y(X, STATE_GYRO_BIAS+2);

	ang_vel_bias.x = OHMD_CLAMP(ang_vel_bias.x, -0.3, 0.3);
	ang_vel_bias.y = OHMD_CLAMP(ang_vel_bias.y, -0.3, 0.3);
	ang_vel_bias.z = OHMD_CLAMP(ang_vel_bias.z, -0.3, 0.3);

	accel_bias.x = MATRIX2D_Y(X, STATE_ACCEL_BIAS);
	accel_bias.y = MATRIX2D_Y(X, STATE_ACCEL_BIAS+1);
	accel_bias.z = MATRIX2D_Y(X, STATE_ACCEL_BIAS+2);

	accel_bias.x = OHMD_CLAMP(accel_bias.x, -1.0, 1.0);
	accel_bias.y = OHMD_CLAMP(accel_bias.y, -1.0, 1.0);
	accel_bias.z = OHMD_CLAMP(accel_bias.z, -1.0, 1.0);

	/* Subtract estimated IMU bias from the ang vel */
	ovec3d_subtract (&filter_state->ang_vel, &ang_vel_bias, &imu_ang_vel);

	double ang_vel_length = ovec3d_get_length (&imu_ang_vel);
	double rot_angle = ang_vel_length * dt;

	/* Update the orientation from angular velocity */
	if (rot_angle != 0) {
		vec3d rot_axis = {{ imu_ang_vel.x / ang_vel_length, imu_ang_vel.y / ang_vel_length, imu_ang_vel.z / ang_vel_length }};
		quatd delta_orient;

		oquatd_init_axis(&delta_orient, &rot_axis, rot_angle);
		oquatd_mult_me(&orient, &delta_orient);
	}

	MATRIX2D_Y(X, STATE_ORIENTATION) = orient.x;
	MATRIX2D_Y(X, STATE_ORIENTATION+1) = orient.y;
	MATRIX2D_Y(X, STATE_ORIENTATION+2) = orient.z;
	MATRIX2D_Y(X, STATE_ORIENTATION+3) = orient.w;

	vec3d global_accel = {{ MATRIX2D_Y(X_prior, STATE_ACCEL),
	                        MATRIX2D_Y(X_prior, STATE_ACCEL+1),
	                        MATRIX2D_Y(X_prior, STATE_ACCEL+2) }};

	vec3d global_vel = {{ MATRIX2D_Y(X_prior, STATE_VELOCITY),
	                        MATRIX2D_Y(X_prior, STATE_VELOCITY+1),
	                        MATRIX2D_Y(X_prior, STATE_VELOCITY+2) }};

	/* Clamp accel to the measurable +/- 16g to restrict variance */
	global_accel.x = OHMD_CLAMP(global_accel.x, -MAX_ACCEL, MAX_ACCEL);
	global_accel.y = OHMD_CLAMP(global_accel.y, -MAX_ACCEL, MAX_ACCEL);
	global_accel.z = OHMD_CLAMP(global_accel.z, -MAX_ACCEL, MAX_ACCEL);

	/* Exponentially damped acceleration */
	global_accel.x *= ACCEL_DAMP;
	global_accel.y *= ACCEL_DAMP;
	global_accel.z *= ACCEL_DAMP;

	global_vel.x *= VEL_DAMP;
	global_vel.y *= VEL_DAMP;
	global_vel.z *= VEL_DAMP;

#if MOTION_MODEL == 0
	/* Constant acceleration model */

	/* Position */
	if (dt >= 0) {
		MATRIX2D_Y(X, STATE_POSITION)   += dt * MATRIX2D_Y(X_prior, STATE_VELOCITY)   + 0.5 * dt * dt * global_accel.x;
		MATRIX2D_Y(X, STATE_POSITION+1) += dt * MATRIX2D_Y(X_prior, STATE_VELOCITY+1) + 0.5 * dt * dt * global_accel.y;
		MATRIX2D_Y(X, STATE_POSITION+2) += dt * MATRIX2D_Y(X_prior, STATE_VELOCITY+2) + 0.5 * dt * dt * global_accel.z;
	} else {
		/* If predicting backward, make sure to decelerate */
		MATRIX2D_Y(X, STATE_POSITION)   += dt * MATRIX2D_Y(X_prior, STATE_VELOCITY)   - 0.5 * dt * dt * global_accel.x;
		MATRIX2D_Y(X, STATE_POSITION+1) += dt * MATRIX2D_Y(X_prior, STATE_VELOCITY+1) - 0.5 * dt * dt * global_accel.y;
		MATRIX2D_Y(X, STATE_POSITION+2) += dt * MATRIX2D_Y(X_prior, STATE_VELOCITY+2) - 0.5 * dt * dt * global_accel.z;
	}

	/* Velocity */
	global_vel.x += dt * global_accel.x;
	global_vel.y += dt * global_accel.y;
	global_vel.z += dt * global_accel.z;
#elif MOTION_MODEL == 1

	MATRIX2D_Y(X, STATE_POSITION)   += dt * MATRIX2D_Y(X_prior, STATE_VELOCITY);
	MATRIX2D_Y(X, STATE_POSITION+1) += dt * MATRIX2D_Y(X_prior, STATE_VELOCITY+1);
	MATRIX2D_Y(X, STATE_POSITION+2) += dt * MATRIX2D_Y(X_prior, STATE_VELOCITY+2);
#elif MOTION_MODEL == 2
	/* Position - hybrid model using constant-V for IMU data gaps */
	/* If dt is < the threshold, use constant accel, otherwise switch to constant velocity,
	 * because we are missing some IMU updates and acceleration vals are definitely wrong */
	if (dt < HYBRID_MOTION_THRESHOLD) {
		/* Constant acceleration model */
		if (dt >= 0) {
			MATRIX2D_Y(X, STATE_POSITION)   += dt * MATRIX2D_Y(X_prior, STATE_VELOCITY)   + 0.5 * dt * dt * global_accel.x;
			MATRIX2D_Y(X, STATE_POSITION+1) += dt * MATRIX2D_Y(X_prior, STATE_VELOCITY+1) + 0.5 * dt * dt * global_accel.y;
			MATRIX2D_Y(X, STATE_POSITION+2) += dt * MATRIX2D_Y(X_prior, STATE_VELOCITY+2) + 0.5 * dt * dt * global_accel.z;
		} else {
			/* If predicting backward, make sure to decelerate */
			MATRIX2D_Y(X, STATE_POSITION)   += dt * MATRIX2D_Y(X_prior, STATE_VELOCITY)   - 0.5 * dt * dt * global_accel.x;
			MATRIX2D_Y(X, STATE_POSITION+1) += dt * MATRIX2D_Y(X_prior, STATE_VELOCITY+1) - 0.5 * dt * dt * global_accel.y;
			MATRIX2D_Y(X, STATE_POSITION+2) += dt * MATRIX2D_Y(X_prior, STATE_VELOCITY+2) - 0.5 * dt * dt * global_accel.z;
		}

		/* Velocity */
		global_vel.x += dt * global_accel.x;
		global_vel.y += dt * global_accel.y;
		global_vel.z += dt * global_accel.z;

	} else {
		/* Constant Velocity mode */
		MATRIX2D_Y(X, STATE_POSITION)   += dt * MATRIX2D_Y(X_prior, STATE_VELOCITY);
		MATRIX2D_Y(X, STATE_POSITION+1) += dt * MATRIX2D_Y(X_prior, STATE_VELOCITY+1);
		MATRIX2D_Y(X, STATE_POSITION+2) += dt * MATRIX2D_Y(X_prior, STATE_VELOCITY+2);
	}
#else
#error "Invalid motion model. MOTION_MODEL must be 0 or 1"
#endif

	/* Clamped accel */
	MATRIX2D_Y(X, STATE_ACCEL)   = global_accel.x;
	MATRIX2D_Y(X, STATE_ACCEL+1) = global_accel.y;
	MATRIX2D_Y(X, STATE_ACCEL+2) = global_accel.z;

	/* Update velocity */
	MATRIX2D_Y(X, STATE_VELOCITY)   = global_vel.x;
	MATRIX2D_Y(X, STATE_VELOCITY+1) = global_vel.y;
	MATRIX2D_Y(X, STATE_VELOCITY+2) = global_vel.z;

	/* Clamped Biases */
	MATRIX2D_Y(X, STATE_GYRO_BIAS)   = ang_vel_bias.x;
	MATRIX2D_Y(X, STATE_GYRO_BIAS+1) = ang_vel_bias.y;
	MATRIX2D_Y(X, STATE_GYRO_BIAS+2) = ang_vel_bias.z;

	MATRIX2D_Y(X, STATE_ACCEL_BIAS)   = accel_bias.x;
	MATRIX2D_Y(X, STATE_ACCEL_BIAS+1) = accel_bias.y;
	MATRIX2D_Y(X, STATE_ACCEL_BIAS+2) = accel_bias.z;

	if (filter_state->reset_slot != -1) {
		/* One of the lagged slot states needs resetting - assign the current orientation and position */
		int out_index = BASE_STATE_SIZE + (DELAY_SLOT_STATE_SIZE * filter_state->reset_slot);
		int in_index = STATE_ORIENTATION;

		for (int i = 0; i < DELAY_SLOT_STATE_SIZE; i++) {
			MATRIX2D_Y(X, out_index) = MATRIX2D_Y(X, in_index);
			out_index++; in_index++;
		}
	}

	return true;
}

static bool calc_quat_mean(const matrix2d *sigmas, int quat_index, const matrix2d *weights, matrix2d *mean)
{
	/* Compute barycentric mean of quaternion component at 'quat_index' iteratively */
	vec3d error_s;

	/* Start with the prior mean as the initial quat */
	quatd mean_orient = {{ MATRIX2D_XY(sigmas, quat_index, 0),
	                       MATRIX2D_XY(sigmas, quat_index+1, 0),
	                       MATRIX2D_XY(sigmas, quat_index+2, 0),
	                       MATRIX2D_XY(sigmas, quat_index+3, 0) }};
	quatd delta_q;
	int s, iters = 0;

	do {
		error_s.x = error_s.y = error_s.z = 0.0;
		double w_M = 1.0 / sigmas->cols;

		for (s = 1; s < sigmas->cols; s++) {
			quatd cur_q = {{ MATRIX2D_XY(sigmas, quat_index, s),
			                 MATRIX2D_XY(sigmas, quat_index+1, s),
			                 MATRIX2D_XY(sigmas, quat_index+2, s),
			                 MATRIX2D_XY(sigmas, quat_index+3, s) }};
			vec3d delta_rot;

			oquatd_diff(&mean_orient, &cur_q, &delta_q);
			oquatd_normalize_me(&delta_q);
			oquatd_to_rotation(&delta_q, &delta_rot);

			error_s.x += w_M * delta_rot.x;
			error_s.y += w_M * delta_rot.y;
			error_s.z += w_M * delta_rot.z;
		}

		/* Correct the mean using the error */
		oquatd_from_rotation(&delta_q, &error_s);

		quatd tmp_q = mean_orient;
		oquatd_mult(&tmp_q, &delta_q, &mean_orient);

		if (iters++ > 100) {
			printf("Quaternion mean failed\n");
			return false; /* Not converging */
		}
	} while (ovec3d_get_length(&error_s) > (1e-4));

	MATRIX2D_Y(mean, quat_index)   = mean_orient.x;
	MATRIX2D_Y(mean, quat_index+1) = mean_orient.y;
	MATRIX2D_Y(mean, quat_index+2) = mean_orient.z;
	MATRIX2D_Y(mean, quat_index+3) = mean_orient.w;

	return true;
}

/* Callback function that takes a set of sigma points (N_sigmaxN_state) and weights (1xN_sigma) and computes the mean (1xN_state) */
static bool state_mean_func(const unscented_transform *ut, const matrix2d *sigmas, const matrix2d *weights, matrix2d *mean)
{
	rift_kalman_6dof_filter *filter_state = (rift_kalman_6dof_filter *)(ut);
	int i;

	/* Compute euclidean barycentric mean, then fix up the quaternion component */
	/* FIXME: Skip the quaternion part of this multiplication to save some cycles */
	if (matrix2d_multiply (mean, sigmas, weights) != MATRIX_RESULT_OK) {
		return false;
	}

	if (!calc_quat_mean(sigmas, STATE_ORIENTATION, weights, mean))
		return false;

	/* Delay slots */
	for (i = 0; i < filter_state->num_delay_slots; i++) {
		int slot_index = BASE_STATE_SIZE + (DELAY_SLOT_STATE_SIZE * i);

		if (filter_state->slot_inuse[i]) {
			/* Quaternion */
			if (!calc_quat_mean(sigmas, slot_index + DELAY_SLOT_STATE_ORIENTATION, weights, mean))
				return false;
		}
		else {
			MATRIX2D_Y(mean, slot_index + DELAY_SLOT_STATE_ORIENTATION) = 0.0;
			MATRIX2D_Y(mean, slot_index + DELAY_SLOT_STATE_ORIENTATION+1) = 0.0;
			MATRIX2D_Y(mean, slot_index + DELAY_SLOT_STATE_ORIENTATION+2) = 0.0;
			MATRIX2D_Y(mean, slot_index + DELAY_SLOT_STATE_ORIENTATION+3) = 1.0;
		}
	}
	return true;
}

static void calc_quat_residual(const matrix2d *X, const matrix2d *Y, int state_index, matrix2d *residual, int cov_index)
{
	quatd X_q, Y_q, delta_q;
	vec3d delta_rot;

	X_q.x = MATRIX2D_Y(X, state_index);
	X_q.y = MATRIX2D_Y(X, state_index+1);
	X_q.z = MATRIX2D_Y(X, state_index+2);
	X_q.w = MATRIX2D_Y(X, state_index+3);

	Y_q.x = MATRIX2D_Y(Y, state_index);
	Y_q.y = MATRIX2D_Y(Y, state_index+1);
	Y_q.z = MATRIX2D_Y(Y, state_index+2);
	Y_q.w = MATRIX2D_Y(Y, state_index+3);

	oquatd_diff(&Y_q, &X_q, &delta_q);
	oquatd_normalize_me(&delta_q);
	oquatd_to_rotation(&delta_q, &delta_rot);

	MATRIX2D_Y(residual, cov_index+0) = delta_rot.x;
	MATRIX2D_Y(residual, cov_index+1) = delta_rot.y;
	MATRIX2D_Y(residual, cov_index+2) = delta_rot.z;
}

static bool state_residual_func(const unscented_transform *ut, const matrix2d *X, const matrix2d *Y, matrix2d *residual)
{
	rift_kalman_6dof_filter *filter_state = (rift_kalman_6dof_filter *)(ut);
	int i, j;

	/* Quaternion component */
	calc_quat_residual(X, Y, STATE_ORIENTATION, residual, COV_ORIENTATION);

	/* Euclidean portion */
	for (i = STATE_POSITION, j = COV_POSITION; i < BASE_STATE_SIZE; i++, j++)
		MATRIX2D_Y(residual, j) = MATRIX2D_Y(X, i) - MATRIX2D_Y(Y, i);

	/* Delay slots */
	for (i = 0; i < filter_state->num_delay_slots; i++) {
		int slot_index = BASE_STATE_SIZE + (DELAY_SLOT_STATE_SIZE * i);
		int cov_index = BASE_COV_SIZE + (DELAY_SLOT_COV_SIZE * i);

		if (filter_state->slot_inuse[i]) {
			/* Quaternion */
			calc_quat_residual(X, Y, slot_index + DELAY_SLOT_STATE_ORIENTATION, residual, cov_index + DELAY_SLOT_COV_ORIENTATION);

			/* Position */
			for (j = 0; j < 3; j++) {
				MATRIX2D_Y(residual, cov_index+DELAY_SLOT_COV_POSITION+j) =
					MATRIX2D_Y(X, slot_index+DELAY_SLOT_STATE_POSITION+j) - MATRIX2D_Y(Y, slot_index+DELAY_SLOT_STATE_POSITION+j);
			}
		}
		else {
			for (j = 0; j < DELAY_SLOT_COV_SIZE; j++)
				MATRIX2D_Y(residual, cov_index + j) = 0.0;
		}
	}

	return true;
}

static void calc_quat_sum(const matrix2d *Y, int state_index, const matrix2d *addend, int cov_index, matrix2d *X)
{
	quatd out_q, delta_q;
	vec3d delta_rot;

	/* Quaternion component */
	out_q.x = MATRIX2D_Y(Y, state_index);
	out_q.y = MATRIX2D_Y(Y, state_index+1);
	out_q.z = MATRIX2D_Y(Y, state_index+2);
	out_q.w = MATRIX2D_Y(Y, state_index+3);

	delta_rot.x = MATRIX2D_Y(addend, cov_index);
	delta_rot.y = MATRIX2D_Y(addend, cov_index+1);
	delta_rot.z = MATRIX2D_Y(addend, cov_index+2);

	oquatd_from_rotation(&delta_q, &delta_rot);
	oquatd_mult_me(&out_q, &delta_q);

	MATRIX2D_Y(X, state_index) = out_q.x;
	MATRIX2D_Y(X, state_index+1) = out_q.y;
	MATRIX2D_Y(X, state_index+2) = out_q.z;
	MATRIX2D_Y(X, state_index+3) = out_q.w;
}

static bool state_sum_func(const unscented_transform *ut, const matrix2d *Y, const matrix2d *addend, matrix2d *X)
{
	rift_kalman_6dof_filter *filter_state = (rift_kalman_6dof_filter *)(ut);
	int i, j;

	/* Quaternion component */
	calc_quat_sum(Y, STATE_ORIENTATION, addend, COV_ORIENTATION, X);

	/* Euclidean portion */
	for (i = STATE_POSITION, j = COV_POSITION; i < BASE_STATE_SIZE; i++, j++)
		MATRIX2D_Y(X, i) = MATRIX2D_Y(Y, i) + MATRIX2D_Y(addend, j);

	/* Delay slots */
	for (i = 0; i < filter_state->num_delay_slots; i++) {
		int slot_index = BASE_STATE_SIZE + (DELAY_SLOT_STATE_SIZE * i);
		int cov_index = BASE_COV_SIZE + (DELAY_SLOT_COV_SIZE * i);

		if (filter_state->slot_inuse[i]) {
			/* Quaternion */
			calc_quat_sum(Y, slot_index + DELAY_SLOT_STATE_ORIENTATION, addend, cov_index + DELAY_SLOT_COV_ORIENTATION, X);

			/* Position */
			for (j = 0; j < 3; j++) {
				MATRIX2D_Y(X, slot_index+DELAY_SLOT_STATE_POSITION+j) =
					MATRIX2D_Y(Y, slot_index+DELAY_SLOT_STATE_POSITION+j) + MATRIX2D_Y(addend, cov_index+DELAY_SLOT_COV_POSITION+j);
			}
		}
		else {
			for (j = 0; j < DELAY_SLOT_STATE_SIZE; j++)
				MATRIX2D_Y(X, slot_index + j) = 0.0;
			MATRIX2D_Y(X, slot_index + DELAY_SLOT_STATE_ORIENTATION + 3) = 1.0;
		}
	}

	return true;
}

static bool imu_measurement_func(const ukf_base *ukf, const ukf_measurement *m, const matrix2d *X, matrix2d *z)
{
	/* Measure accel and gyro, plus biases, plus gravity vector from orientation */
	/* Accel + bias + gravity vector */
	quatd orient = {{ MATRIX2D_Y(X, STATE_ORIENTATION),
	                  MATRIX2D_Y(X, STATE_ORIENTATION+1),
	                  MATRIX2D_Y(X, STATE_ORIENTATION+2),
	                  MATRIX2D_Y(X, STATE_ORIENTATION+3) }};

	vec3d global_accel = {{ MATRIX2D_Y(X, STATE_ACCEL),
	                        MATRIX2D_Y(X, STATE_ACCEL+1) + GRAVITY_MAG,
	                        MATRIX2D_Y(X, STATE_ACCEL+2) }};
	vec3d local_accel;

	/* Move global accel into the IMU body frame, and add bias */
	oquatd_inverse(&orient);
	oquatd_get_rotated(&orient, &global_accel, &local_accel);

	MATRIX2D_Y(z, IMU_MEAS_ACCEL)   = local_accel.x + MATRIX2D_Y(X, STATE_ACCEL_BIAS);
	MATRIX2D_Y(z, IMU_MEAS_ACCEL+1) = local_accel.y + MATRIX2D_Y(X, STATE_ACCEL_BIAS+1);
	MATRIX2D_Y(z, IMU_MEAS_ACCEL+2) = local_accel.z + MATRIX2D_Y(X, STATE_ACCEL_BIAS+2);

	// print_col_vec("IMU measurement prediction (z_bar)", z);

	return true;
}

static bool pose_measurement_func(const ukf_base *ukf, const ukf_measurement *m, const matrix2d *x, matrix2d *z)
{
	rift_kalman_6dof_filter *filter_state = (rift_kalman_6dof_filter *)(ukf);
	int state_position_index = STATE_POSITION;
	int state_orientation_index = STATE_ORIENTATION;

	if (filter_state->pose_slot != -1) {
		/* A delay slot was set, get the measurement from it */
		int slot_index = BASE_STATE_SIZE + (DELAY_SLOT_STATE_SIZE * filter_state->pose_slot);
		state_position_index = slot_index + DELAY_SLOT_STATE_POSITION;
		state_orientation_index = slot_index + DELAY_SLOT_STATE_ORIENTATION;
	}
	/* Measure position and orientation */
	MATRIX2D_Y(z, POSE_MEAS_POSITION)   = MATRIX2D_Y(x, state_position_index);
	MATRIX2D_Y(z, POSE_MEAS_POSITION+1) = MATRIX2D_Y(x, state_position_index+1);
	MATRIX2D_Y(z, POSE_MEAS_POSITION+2) = MATRIX2D_Y(x, state_position_index+2);

	MATRIX2D_Y(z, POSE_MEAS_ORIENTATION)   = MATRIX2D_Y(x, state_orientation_index);
	MATRIX2D_Y(z, POSE_MEAS_ORIENTATION+1) = MATRIX2D_Y(x, state_orientation_index+1);
	MATRIX2D_Y(z, POSE_MEAS_ORIENTATION+2) = MATRIX2D_Y(x, state_orientation_index+2);
	MATRIX2D_Y(z, POSE_MEAS_ORIENTATION+3) = MATRIX2D_Y(x, state_orientation_index+3);

	// print_col_vec("Pose measurement prediction (z_bar)", -1, z);

	return true;
}

static bool position_measurement_func(const ukf_base *ukf, const ukf_measurement *m, const matrix2d *x, matrix2d *z)
{
	rift_kalman_6dof_filter *filter_state = (rift_kalman_6dof_filter *)(ukf);
	int state_position_index = STATE_POSITION;

	if (filter_state->pose_slot != -1) {
		/* A delay slot was set, get the measurement from it */
		int slot_index = BASE_STATE_SIZE + (DELAY_SLOT_STATE_SIZE * filter_state->pose_slot);
		state_position_index = slot_index + DELAY_SLOT_STATE_POSITION;
	}
	/* Measure position */
	MATRIX2D_Y(z, POSE_MEAS_POSITION)   = MATRIX2D_Y(x, state_position_index);
	MATRIX2D_Y(z, POSE_MEAS_POSITION+1) = MATRIX2D_Y(x, state_position_index+1);
	MATRIX2D_Y(z, POSE_MEAS_POSITION+2) = MATRIX2D_Y(x, state_position_index+2);

	return true;
}

/* Callback function that takes a set of sigma points (N_sigmaxN_measurement and weights (1xN_sigma) and computes the
 * mean (1xN_measurement) */
static bool pose_mean_func(const unscented_transform *ut, const matrix2d *sigmas, const matrix2d *weights, matrix2d *mean)
{
	/* Compute euclidean barycentric mean, then fix up the quaternion component */
	/* FIXME: Skip the quaternion part of this multiplication to save some cycles */
	if (matrix2d_multiply (mean, sigmas, weights) != MATRIX_RESULT_OK) {
		return false;
	}

	if (!calc_quat_mean(sigmas, POSE_MEAS_ORIENTATION, weights, mean))
		return false;

	return true;
}

static bool pose_residual_func(const unscented_transform *ut, const matrix2d *X, const matrix2d *Y, matrix2d *residual)
{
	int i;

	/* Euclidean portion */
	for (i = POSE_MEAS_POSITION; i < POSE_MEAS_ORIENTATION; i++)
		MATRIX2D_Y(residual, i) = MATRIX2D_Y(X, i) - MATRIX2D_Y(Y, i);

	quatd X_q, Y_q, delta_q;
	vec3d delta_rot;

	/* Quaternion component */
	X_q.x = MATRIX2D_Y(X, POSE_MEAS_ORIENTATION);
	X_q.y = MATRIX2D_Y(X, POSE_MEAS_ORIENTATION+1);
	X_q.z = MATRIX2D_Y(X, POSE_MEAS_ORIENTATION+2);
	X_q.w = MATRIX2D_Y(X, POSE_MEAS_ORIENTATION+3);

	Y_q.x = MATRIX2D_Y(Y, POSE_MEAS_ORIENTATION);
	Y_q.y = MATRIX2D_Y(Y, POSE_MEAS_ORIENTATION+1);
	Y_q.z = MATRIX2D_Y(Y, POSE_MEAS_ORIENTATION+2);
	Y_q.w = MATRIX2D_Y(Y, POSE_MEAS_ORIENTATION+3);

	oquatd_diff(&Y_q, &X_q, &delta_q);
	oquatd_normalize_me(&delta_q);
	oquatd_to_rotation(&delta_q, &delta_rot);

	MATRIX2D_Y(residual, 3) = delta_rot.x;
	MATRIX2D_Y(residual, 4) = delta_rot.y;
	MATRIX2D_Y(residual, 5) = delta_rot.z;

	return true;
}

/* The addend here is a 6 value vector, 3 position, 3 orientation,
 * to be added to the 7-value measurement vector non-linearly */
static bool pose_sum_func(const unscented_transform *ut, const matrix2d *Y, const matrix2d *addend, matrix2d *X)
{
	int i;

	/* Euclidean portion */
	for (i = POSE_MEAS_POSITION; i < POSE_MEAS_ORIENTATION; i++)
		MATRIX2D_Y(X, i) = MATRIX2D_Y(Y, i) + MATRIX2D_Y(addend, i);

	quatd out_q, delta_q;
	vec3d delta_rot;

	/* Quaternion component */
	out_q.x = MATRIX2D_Y(Y, POSE_MEAS_ORIENTATION);
	out_q.y = MATRIX2D_Y(Y, POSE_MEAS_ORIENTATION+1);
	out_q.z = MATRIX2D_Y(Y, POSE_MEAS_ORIENTATION+2);
	out_q.w = MATRIX2D_Y(Y, POSE_MEAS_ORIENTATION+3);

	delta_rot.x = MATRIX2D_Y(addend, 3);
	delta_rot.y = MATRIX2D_Y(addend, 4);
	delta_rot.z = MATRIX2D_Y(addend, 5);

	oquatd_from_rotation(&delta_q, &delta_rot);
	oquatd_mult_me(&out_q, &delta_q);

	MATRIX2D_Y(X, POSE_MEAS_ORIENTATION) = out_q.x;
	MATRIX2D_Y(X, POSE_MEAS_ORIENTATION+1) = out_q.y;
	MATRIX2D_Y(X, POSE_MEAS_ORIENTATION+2) = out_q.z;
	MATRIX2D_Y(X, POSE_MEAS_ORIENTATION+3) = out_q.w;

	return true;
}

void rift_kalman_6dof_init(rift_kalman_6dof_filter *state, posef *init_pose, int num_delay_slots)
{
	int i, d;

	assert(num_delay_slots <= MAX_DELAY_SLOTS);

	state->first_update = true;
	state->num_delay_slots = num_delay_slots;

	const int STATE_SIZE = BASE_STATE_SIZE + (num_delay_slots * DELAY_SLOT_STATE_SIZE);
	const int COV_SIZE = BASE_COV_SIZE + (num_delay_slots * DELAY_SLOT_COV_SIZE);

	/* FIXME: These process noise values are pretty randomly chosen */
	state->Q_noise = matrix2d_alloc0 (COV_SIZE, COV_SIZE);
	for (i = COV_ORIENTATION; i < COV_ORIENTATION + 3; i++)
		MATRIX2D_XY(state->Q_noise, i, i) = 1e-5;

	for (i = COV_POSITION; i < COV_POSITION + 3; i++)
		MATRIX2D_XY(state->Q_noise, i, i) = 1e-4;
	for (i = COV_VELOCITY; i < COV_VELOCITY + 3; i++)
		MATRIX2D_XY(state->Q_noise, i, i) = 1e-3;

	/* Accelerometer and Gyro estimates can change sharply -
	 * even "gentle" motion leads to +/- 5g in a millisecond,
	 * and gyro can easily change 10dps in a millisecond, but that's
	 * not the common case. Typical variance for small motions is more
	 * like 0.2 (m/s^2)^2. The value here is an experimentally determined
	 * mid-ground */
	for (i = COV_ACCEL; i < COV_ACCEL + 3; i++)
		MATRIX2D_XY(state->Q_noise, i, i) = 160.0;

	/* Gyro and accel bias have very small variance, since we
	 * want them to change slowly */
	for (i = COV_ACCEL_BIAS; i < COV_ACCEL_BIAS + 3; i++)
		MATRIX2D_XY(state->Q_noise, i, i) = IMU_ACCEL_BIAS_NOISE;

	for (i = COV_GYRO_BIAS; i < COV_GYRO_BIAS + 3; i++)
		MATRIX2D_XY(state->Q_noise, i, i) = IMU_GYRO_BIAS_NOISE;

	/* Delay slots - copy the noise from the main slot */
	for (d = 0; d < num_delay_slots; d++) {
		int cov_index = BASE_COV_SIZE + (DELAY_SLOT_COV_SIZE * d);
		for (i = 0; i < 6; i++)
			MATRIX2D_XY(state->Q_noise, cov_index + i, cov_index + i) = MATRIX2D_XY(state->Q_noise, i, i);
	}

	/* Takes ownership of Q_noise */
	ukf_base_init(&state->ukf, STATE_SIZE, COV_SIZE, state->Q_noise, process_func, state_mean_func, state_residual_func, state_sum_func);

	/* Initialise the state with the init_pose */
	for (i = 0; i < 3; i++)
		MATRIX2D_Y(state->ukf.x_prior, STATE_POSITION + i) = init_pose->pos.arr[i];
	for (i = 0; i < 4; i++)
		MATRIX2D_Y(state->ukf.x_prior, STATE_ORIENTATION + i) = init_pose->orient.arr[i];

	for (i = 0; i < num_delay_slots; i++) {
		int slot_index = BASE_STATE_SIZE + (DELAY_SLOT_STATE_SIZE * i);
		MATRIX2D_Y(state->ukf.x_prior, slot_index + DELAY_SLOT_STATE_ORIENTATION + 3) = 1.0;
		state->slot_inuse[i] = false;
	}

	/* Initialise the prior covariance / uncertainty - particularly around the biases,
	 * where we assume they are close to 0 somewhere */
	for (i = COV_ACCEL_BIAS; i < COV_ACCEL_BIAS + 3; i++)
		MATRIX2D_XY(state->ukf.P_prior, i, i) = IMU_ACCEL_BIAS_NOISE_INITIAL;

	for (i = COV_GYRO_BIAS; i < COV_GYRO_BIAS + 3; i++)
		MATRIX2D_XY(state->ukf.P_prior, i, i) = IMU_GYRO_BIAS_NOISE_INITIAL;

	/* m1 is for IMU measurement - accel */
	ukf_measurement_init(&state->m1, 3, 3, &state->ukf, imu_measurement_func, NULL, NULL, NULL);

	/* FIXME: Set R matrix to something based on IMU noise */
	for (int i = 0; i < 3; i++)
	 MATRIX2D_XY(state->m1.R, i, i) = 1e-6;

	/* m2 is for pose measurements - position and orientation. We trust the position more than
	 * the orientation. */
	ukf_measurement_init(&state->m2, 7, 6, &state->ukf, pose_measurement_func, pose_mean_func, pose_residual_func, pose_sum_func);
	for (int i = 0; i < 3; i++)
		MATRIX2D_XY(state->m2.R, i, i) = 0.01 * 0.01; /* 1cm error std dev */

	MATRIX2D_XY(state->m2.R, 3, 3) = (DEG_TO_RAD(90) * DEG_TO_RAD(90)); /* 90 degrees std dev (don't trust observations much for X/Z, 20 degrees for yaw) */
	MATRIX2D_XY(state->m2.R, 4, 4) = (DEG_TO_RAD(20) * DEG_TO_RAD(20)); /* Y */
	MATRIX2D_XY(state->m2.R, 5, 5) = (DEG_TO_RAD(90) * DEG_TO_RAD(90)); /* Z */

	/* m_position is for position-only measurements - no orientation. */
	ukf_measurement_init(&state->m_position, 3, 3, &state->ukf, position_measurement_func, NULL, NULL, NULL);
	for (int i = 0; i < 3; i++)
		MATRIX2D_XY(state->m_position.R, i, i) = 0.01 * 0.01; /* 1cm error std dev */

	state->reset_slot = -1;
	state->pose_slot = -1;

	ovec3d_set(&state->ang_vel, 0.0, 0.0, 0.0);
}

void rift_kalman_6dof_clear(rift_kalman_6dof_filter *state)
{
	ukf_measurement_clear(&state->m2);
	ukf_measurement_clear(&state->m1);
	ukf_base_clear(&state->ukf);
}

static void
rift_kalman_6dof_update(rift_kalman_6dof_filter *state, uint64_t time, ukf_measurement *m)
{
	/* Calculate dt */
	int64_t dt = 0;
	if (time != 0) {
		if (state->first_update) {
			dt = 0;
			state->first_update = false;
		}
		else {
			dt = (int64_t)(time - state->current_ts);
		}
		state->current_ts = time;
	}

	if (!ukf_base_predict(&state->ukf, NS_TO_SEC(dt))) {
			LOGE ("Failed to compute UKF prediction at time %llu (dt %f)", (unsigned long long) state->current_ts, NS_TO_SEC(dt));
			return;
	}

	if (m) {
		if (!ukf_base_update(&state->ukf, m)) {
			LOGE ("Failed to perform %s UKF update at time %llu (dt %f)",
						m == &state->m1 ? "IMU" : "Pose", (unsigned long long) state->current_ts, NS_TO_SEC(dt));
			return;
		}
	}
	else {
		/* Pseudo-measurement - just commit the predicted state */
		if (!ukf_base_commit(&state->ukf)) {
			LOGE ("Failed to commit UKF prediction at time %llu (dt %f)",
					(unsigned long long) state->current_ts, NS_TO_SEC(dt));
			return;
		}
	}

	// print_col_vec ("UKF Mean after update", state->current_ts, state->ukf.x_prior);
}

void rift_kalman_6dof_prepare_delay_slot(rift_kalman_6dof_filter *state, uint64_t time, int delay_slot)
{
	/* Predict state forward to the timestamp, and set the
	 * assigned lagged slot state */
	state->slot_inuse[delay_slot] = true;
	state->reset_slot = delay_slot;
	rift_kalman_6dof_update(state, time, NULL);
	state->reset_slot = -1;
}

void rift_kalman_6dof_release_delay_slot(rift_kalman_6dof_filter *state, int delay_slot)
{
	state->slot_inuse[delay_slot] = false;
}

void rift_kalman_6dof_imu_update (rift_kalman_6dof_filter *state, uint64_t time, const vec3f* ang_vel, const vec3f* accel, const vec3f* mag_field)
{
	ukf_measurement *m;

	/* Put angular velocity into the input vector to update the orientation */
	state->ang_vel.x = ang_vel->x;
	state->ang_vel.y = ang_vel->y;
	state->ang_vel.z = ang_vel->z;

	/* and acceleration in the measurement vector to correct the orientation by gravity */
	/* FIXME: Use mag if set? */
	m = &state->m1;
	MATRIX2D_Y(m->z, IMU_MEAS_ACCEL+0) = accel->x;
	MATRIX2D_Y(m->z, IMU_MEAS_ACCEL+1) = accel->y;
	MATRIX2D_Y(m->z, IMU_MEAS_ACCEL+2) = accel->z;

	rift_kalman_6dof_update(state, time, m);
}

void rift_kalman_6dof_pose_update(rift_kalman_6dof_filter *state, uint64_t time, posef *pose, int delay_slot)
{
	ukf_measurement *m;

	/* Use lagged state vector entries to correct for delay */
	state->pose_slot = delay_slot;

	/* If doing a delayed update, then the slot must be in use (
	 * or else it contains empty data */
	if (delay_slot != -1) {
		assert(state->slot_inuse[delay_slot]);
		/* HACK: Ignore the time when doing a delay slot update,
		 * since we don't want time to go backward. The delay slot is
		 * already tracking the delay */
		time = 0;
	}

	m = &state->m2;
	MATRIX2D_Y(m->z, POSE_MEAS_POSITION+0) = pose->pos.x;
	MATRIX2D_Y(m->z, POSE_MEAS_POSITION+1) = pose->pos.y;
	MATRIX2D_Y(m->z, POSE_MEAS_POSITION+2) = pose->pos.z;

	MATRIX2D_Y(m->z, POSE_MEAS_ORIENTATION+0) = pose->orient.x;
	MATRIX2D_Y(m->z, POSE_MEAS_ORIENTATION+1) = pose->orient.y;
	MATRIX2D_Y(m->z, POSE_MEAS_ORIENTATION+2) = pose->orient.z;
	MATRIX2D_Y(m->z, POSE_MEAS_ORIENTATION+3) = pose->orient.w;

	rift_kalman_6dof_update(state, time, m);
}

void rift_kalman_6dof_position_update(rift_kalman_6dof_filter *state, uint64_t time, vec3f *pos, int delay_slot)
{
	ukf_measurement *m;

	/* Use lagged state vector entries to correct for delay */
	state->pose_slot = delay_slot;

	/* If doing a delayed update, then the slot must be in use (
	 * or else it contains empty data */
	if (delay_slot != -1)
		assert(state->slot_inuse[delay_slot]);

	m = &state->m_position;
	MATRIX2D_Y(m->z, POSE_MEAS_POSITION+0) = pos->x;
	MATRIX2D_Y(m->z, POSE_MEAS_POSITION+1) = pos->y;
	MATRIX2D_Y(m->z, POSE_MEAS_POSITION+2) = pos->z;

	rift_kalman_6dof_update(state, time, m);
}

/* Get the pose info from a delay slot, or the main state
 * if delay_slot = -1 */
void rift_kalman_6dof_get_delay_slot_pose_at(rift_kalman_6dof_filter *state, uint64_t time, int delay_slot, posef *pose,
  vec3f *vel, vec3f *accel, vec3f *ang_vel, vec3f *pos_error, vec3f *rot_error)
{
	matrix2d *x = state->ukf.x_prior;
	matrix2d *P = state->ukf.P_prior; /* Covariance */

	int state_position_index = STATE_POSITION;
	int state_orientation_index = STATE_ORIENTATION;
	int cov_position_index = COV_POSITION;
	int cov_orientation_index = COV_ORIENTATION;

	if (delay_slot != -1) {
	  assert(state->slot_inuse[delay_slot]);
	  assert(delay_slot < MAX_DELAY_SLOTS);

		/* We want the position / orientation info from a delay slot */
		int slot_index = BASE_STATE_SIZE + (DELAY_SLOT_STATE_SIZE * delay_slot);
		state_position_index = slot_index + DELAY_SLOT_STATE_POSITION;
		state_orientation_index = slot_index + DELAY_SLOT_STATE_ORIENTATION;

		slot_index = BASE_COV_SIZE + (DELAY_SLOT_COV_SIZE * delay_slot);
		cov_position_index = slot_index + DELAY_SLOT_COV_POSITION;
		cov_orientation_index = slot_index + DELAY_SLOT_COV_ORIENTATION;
	}

	/* FIXME: Do prediction using the time? */
	pose->pos.x = MATRIX2D_Y(x, state_position_index);
	pose->pos.y = MATRIX2D_Y(x, state_position_index+1);
	pose->pos.z = MATRIX2D_Y(x, state_position_index+2);

	pose->orient.x = MATRIX2D_Y(x, state_orientation_index);
	pose->orient.y = MATRIX2D_Y(x, state_orientation_index+1);
	pose->orient.z = MATRIX2D_Y(x, state_orientation_index+2);
	pose->orient.w = MATRIX2D_Y(x, state_orientation_index+3);

	/* Velocity, accel and ang_vel aren't tracked in the delay slots,
	 * so always return the current state */
	if (vel) {
		vel->x = MATRIX2D_Y(x, STATE_VELOCITY);
		vel->y = MATRIX2D_Y(x, STATE_VELOCITY+1);
		vel->z = MATRIX2D_Y(x, STATE_VELOCITY+2);
	}

	if (accel) {
		accel->x = MATRIX2D_Y(x, STATE_ACCEL);
		accel->y = MATRIX2D_Y(x, STATE_ACCEL+1);
		accel->z = MATRIX2D_Y(x, STATE_ACCEL+2);
	}

	if (ang_vel) {
	  ang_vel->x = state->ang_vel.x;
	  ang_vel->y = state->ang_vel.y;
	  ang_vel->z = state->ang_vel.z;
	}

	if (pos_error) {
		pos_error->x = sqrtf(MATRIX2D_XY(P, cov_position_index, cov_position_index));
		pos_error->y = sqrtf(MATRIX2D_XY(P, cov_position_index+1, cov_position_index+1));
		pos_error->z = sqrtf(MATRIX2D_XY(P, cov_position_index+2, cov_position_index+2));
	}

	if (rot_error) {
		rot_error->x = sqrtf(MATRIX2D_XY(P, cov_orientation_index, cov_orientation_index));
		rot_error->y = sqrtf(MATRIX2D_XY(P, cov_orientation_index+1, cov_orientation_index+1));
		rot_error->z = sqrtf(MATRIX2D_XY(P, cov_orientation_index+2, cov_orientation_index+2));
	}
}

void rift_kalman_6dof_get_pose_at(rift_kalman_6dof_filter *state, uint64_t time, posef *pose, vec3f *vel, vec3f *accel,
  vec3f *ang_vel, vec3f *pos_error, vec3f *rot_error)
{
	rift_kalman_6dof_get_delay_slot_pose_at(state, time, -1, pose, vel, accel, ang_vel, pos_error, rot_error);
}
