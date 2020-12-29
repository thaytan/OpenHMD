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

#include "rift-kalman-6dof.h"

/* 0 = constant acceleration
 * 1 = constant velocity
 */
#define MOTION_MODEL 0

/* IMU biases noise levels */
#define IMU_GYRO_BIAS_NOISE 1e-7 /* gyro bias (rad/s)^2 */
#define IMU_GYRO_BIAS_NOISE_INITIAL 1e-3 /* gyro bias (rad/s)^2 */
#define IMU_ACCEL_BIAS_NOISE 1e-6 /* accelerometer bias (m/s^2)^2 */
#define IMU_ACCEL_BIAS_NOISE_INITIAL 0.25 /* accelerometer bias (m/s^2)^2 */

const double GRAVITY_MAG = 9.80665;

/* The state vector is bigger than the covariance
 * matrices, because it includes the quaternion, but
 * the covariance is parameterised with an exponential
 * map */
const int STATE_SIZE = 19;
const int COV_SIZE = 18;

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
	quatd orient = {{ X_prior->mat1D[STATE_ORIENTATION],
										X_prior->mat1D[STATE_ORIENTATION+1],
										X_prior->mat1D[STATE_ORIENTATION+2],
										X_prior->mat1D[STATE_ORIENTATION+3]
								 }};

	vec3d imu_ang_vel;
	vec3d ang_vel_bias;

	ang_vel_bias.x = X->mat1D[STATE_GYRO_BIAS];
	ang_vel_bias.y = X->mat1D[STATE_GYRO_BIAS+1];
	ang_vel_bias.z = X->mat1D[STATE_GYRO_BIAS+2];

	/* Subtract estimated IMU bias from the ang vel */
	ovec3d_subtract (&filter_state->ang_vel, &ang_vel_bias, &imu_ang_vel);

	float ang_vel_length = ovec3d_get_length (&imu_ang_vel);
	float rot_angle = ang_vel_length * dt;

	/* Update the orientation from angular velocity */
	if (rot_angle != 0) {
		vec3d rot_axis = {{ imu_ang_vel.x / ang_vel_length, imu_ang_vel.y / ang_vel_length, imu_ang_vel.z / ang_vel_length }};
		quatd delta_orient;

		oquatd_init_axis(&delta_orient, &rot_axis, rot_angle);
		oquatd_mult_me(&orient, &delta_orient);
	}

	X->mat1D[STATE_ORIENTATION] = orient.x;
	X->mat1D[STATE_ORIENTATION+1] = orient.y;
	X->mat1D[STATE_ORIENTATION+2] = orient.z;
	X->mat1D[STATE_ORIENTATION+3] = orient.w;

#if MOTION_MODEL == 0
	/* Constant acceleration model */
	vec3d global_accel = {{ X_prior->mat1D[STATE_ACCEL],
													X_prior->mat1D[STATE_ACCEL+1],
													X_prior->mat1D[STATE_ACCEL+2] }};

	/* Position */
	if (dt >= 0) {
		X->mat1D[STATE_POSITION]	 += dt * X_prior->mat1D[STATE_VELOCITY]	 + 0.5 * dt * dt * global_accel.x;
		X->mat1D[STATE_POSITION+1] += dt * X_prior->mat1D[STATE_VELOCITY+1] + 0.5 * dt * dt * global_accel.y;
		X->mat1D[STATE_POSITION+2] += dt * X_prior->mat1D[STATE_VELOCITY+2] + 0.5 * dt * dt * global_accel.z;
	} else {
		/* If predicting backward, make sure to decelerate */
		X->mat1D[STATE_POSITION]	 += dt * X_prior->mat1D[STATE_VELOCITY]	 - 0.5 * dt * dt * global_accel.x;
		X->mat1D[STATE_POSITION+1] += dt * X_prior->mat1D[STATE_VELOCITY+1] - 0.5 * dt * dt * global_accel.y;
		X->mat1D[STATE_POSITION+2] += dt * X_prior->mat1D[STATE_VELOCITY+2] - 0.5 * dt * dt * global_accel.z;
	}

	/* Velocity */
	X->mat1D[STATE_VELOCITY]	 += dt * global_accel.x;
	X->mat1D[STATE_VELOCITY+1] += dt * global_accel.y;
	X->mat1D[STATE_VELOCITY+2] += dt * global_accel.z;

#elif MOTION_MODEL == 1
	/* Position - constant velocity model */
	X->mat1D[STATE_POSITION]	 += dt * X_prior->mat1D[STATE_VELOCITY];
	X->mat1D[STATE_POSITION+1] += dt * X_prior->mat1D[STATE_VELOCITY+1];
	X->mat1D[STATE_POSITION+2] += dt * X_prior->mat1D[STATE_VELOCITY+2];
#else
#error "Invalid motion model. MOTION_MODEL must be 0 or 1"
#endif

	return true;
}

static bool calc_quat_mean(const matrix2d *sigmas, int quat_index, const matrix2d *weights, matrix2d *mean)
{
	/* Compute barycentric mean of quaternion component at 'quat_index' iteratively */
	vec3d error_s;

	/* Start with the prior mean as the initial quat */
	quatd mean_orient = {{ sigmas->mat[0][quat_index], sigmas->mat[0][quat_index+1],
										sigmas->mat[0][quat_index+2], sigmas->mat[0][quat_index+3] }};
	quatd delta_q;
	int s, iters = 0;

	do {
		error_s.x = error_s.y = error_s.z = 0.0;

		double w_M = 1.0 / sigmas->n;

		for (s = 0; s < sigmas->n; s++) {
			quatd cur_q = {{ sigmas->mat[s][quat_index], sigmas->mat[s][quat_index+1],
											 sigmas->mat[s][quat_index+2], sigmas->mat[s][quat_index+3] }};
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

	mean->mat1D[quat_index]	 = mean_orient.x;
	mean->mat1D[quat_index+1] = mean_orient.y;
	mean->mat1D[quat_index+2] = mean_orient.z;
	mean->mat1D[quat_index+3] = mean_orient.w;

	return true;
}

/* Callback function that takes a set of sigma points (N_sigmaxN_state) and weights (1xN_sigma) and computes the mean (1xN_state) */
static bool state_mean_func(const unscented_transform *ut, const matrix2d *sigmas, const matrix2d *weights, matrix2d *mean)
{
	/* Compute euclidean barycentric mean, then fix up the quaternion component */
	/* FIXME: Skip the quaternion part of this multiplication to save some cycles */
	if (matrix2d_multiply (mean, sigmas, weights) != MATRIX_RESULT_OK) {
		return false;
	}

	if (!calc_quat_mean(sigmas, STATE_ORIENTATION, weights, mean))
		return false;

	return true;
}

static bool state_residual_func(const unscented_transform *ut, const matrix2d *X, const matrix2d *Y, matrix2d *residual)
{
	int i;
	quatd X_q, Y_q, delta_q;
	vec3d delta_rot;

	/* Quaternion component */
	X_q.x = X->mat1D[STATE_ORIENTATION];
	X_q.y = X->mat1D[STATE_ORIENTATION+1];
	X_q.z = X->mat1D[STATE_ORIENTATION+2];
	X_q.w = X->mat1D[STATE_ORIENTATION+3];

	Y_q.x = Y->mat1D[STATE_ORIENTATION];
	Y_q.y = Y->mat1D[STATE_ORIENTATION+1];
	Y_q.z = Y->mat1D[STATE_ORIENTATION+2];
	Y_q.w = Y->mat1D[STATE_ORIENTATION+3];

	oquatd_diff(&Y_q, &X_q, &delta_q);
	oquatd_normalize_me(&delta_q);
	oquatd_to_rotation(&delta_q, &delta_rot);

	residual->mat1D[0] = delta_rot.x;
	residual->mat1D[1] = delta_rot.y;
	residual->mat1D[2] = delta_rot.z;

	/* Euclidean portion */
	for (i = STATE_POSITION; i < STATE_SIZE; i++)
		residual->mat1D[i-1] = X->mat1D[i] - Y->mat1D[i];

	return true;
}

static bool state_sum_func(const unscented_transform *ut, const matrix2d *Y, const matrix2d *addend, matrix2d *X)
{
	int i;
	quatd out_q, delta_q;
	vec3d delta_rot;

	/* Quaternion component */
	out_q.x = Y->mat1D[STATE_ORIENTATION];
	out_q.y = Y->mat1D[STATE_ORIENTATION+1];
	out_q.z = Y->mat1D[STATE_ORIENTATION+2];
	out_q.w = Y->mat1D[STATE_ORIENTATION+3];

	delta_rot.x = addend->mat1D[0];
	delta_rot.y = addend->mat1D[1];
	delta_rot.z = addend->mat1D[2];

	oquatd_from_rotation(&delta_q, &delta_rot);
	oquatd_mult_me(&out_q, &delta_q);

	X->mat1D[STATE_ORIENTATION] = out_q.x;
	X->mat1D[STATE_ORIENTATION+1] = out_q.y;
	X->mat1D[STATE_ORIENTATION+2] = out_q.z;
	X->mat1D[STATE_ORIENTATION+3] = out_q.w;

	/* Euclidean portion */
	for (i = STATE_POSITION; i < STATE_SIZE; i++)
		X->mat1D[i] = Y->mat1D[i] + addend->mat1D[i-1];

	return true;
}

static bool imu_measurement_func(const ukf_base *ukf, const ukf_measurement *m, const matrix2d *X, matrix2d *z)
{
	/* Measure accel and gyro, plus biases, plus gravity vector from orientation */
	/* Accel + bias + gravity vector */
	quatd orient = {{ X->mat1D[STATE_ORIENTATION],
										X->mat1D[STATE_ORIENTATION+1],
										X->mat1D[STATE_ORIENTATION+2],
										X->mat1D[STATE_ORIENTATION+3] }};

	vec3d global_accel = {{ X->mat1D[STATE_ACCEL],
													X->mat1D[STATE_ACCEL+1] + GRAVITY_MAG,
													X->mat1D[STATE_ACCEL+2] }};
	vec3d local_accel;

	/* Move global accel into the IMU body frame, and add bias */
	oquatd_inverse(&orient);
	oquatd_get_rotated(&orient, &global_accel, &local_accel);

	z->mat1D[IMU_MEAS_ACCEL]	 = local_accel.x + X->mat1D[STATE_ACCEL_BIAS];
	z->mat1D[IMU_MEAS_ACCEL+1] = local_accel.y + X->mat1D[STATE_ACCEL_BIAS+1];
	z->mat1D[IMU_MEAS_ACCEL+2] = local_accel.z + X->mat1D[STATE_ACCEL_BIAS+2];

	return true;
}

bool pose_measurement_func(const ukf_base *ukf, const ukf_measurement *m, const matrix2d *x, matrix2d *z)
{
	/* Measure position and orientation */
	z->mat1D[POSE_MEAS_POSITION]	 = x->mat1D[STATE_POSITION];
	z->mat1D[POSE_MEAS_POSITION+1] = x->mat1D[STATE_POSITION+1];
	z->mat1D[POSE_MEAS_POSITION+2] = x->mat1D[STATE_POSITION+2];

	z->mat1D[POSE_MEAS_ORIENTATION]	 = x->mat1D[STATE_ORIENTATION];
	z->mat1D[POSE_MEAS_ORIENTATION+1] = x->mat1D[STATE_ORIENTATION+1];
	z->mat1D[POSE_MEAS_ORIENTATION+2] = x->mat1D[STATE_ORIENTATION+2];
	z->mat1D[POSE_MEAS_ORIENTATION+3] = x->mat1D[STATE_ORIENTATION+3];

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
		residual->mat1D[i] = X->mat1D[i] - Y->mat1D[i];

	quatd X_q, Y_q, delta_q;
	vec3d delta_rot;

	/* Quaternion component */
	X_q.x = X->mat1D[POSE_MEAS_ORIENTATION];
	X_q.y = X->mat1D[POSE_MEAS_ORIENTATION+1];
	X_q.z = X->mat1D[POSE_MEAS_ORIENTATION+2];
	X_q.w = X->mat1D[POSE_MEAS_ORIENTATION+3];

	Y_q.x = Y->mat1D[POSE_MEAS_ORIENTATION];
	Y_q.y = Y->mat1D[POSE_MEAS_ORIENTATION+1];
	Y_q.z = Y->mat1D[POSE_MEAS_ORIENTATION+2];
	Y_q.w = Y->mat1D[POSE_MEAS_ORIENTATION+3];

	oquatd_diff(&Y_q, &X_q, &delta_q);
	oquatd_normalize_me(&delta_q);
	oquatd_to_rotation(&delta_q, &delta_rot);

	residual->mat1D[3] = delta_rot.x;
	residual->mat1D[4] = delta_rot.y;
	residual->mat1D[5] = delta_rot.z;

	return true;
}

/* The addend here is a 6 value vector, 3 position, 3 orientation,
 * to be added to the 7-value measurement vector non-linearly */
static bool pose_sum_func(const unscented_transform *ut, const matrix2d *Y, const matrix2d *addend, matrix2d *X)
{
	int i;

	/* Euclidean portion */
	for (i = POSE_MEAS_POSITION; i < POSE_MEAS_ORIENTATION; i++)
		X->mat1D[i] = Y->mat1D[i] + addend->mat1D[i];

	quatd out_q, delta_q;
	vec3d delta_rot;

	/* Quaternion component */
	out_q.x = Y->mat1D[POSE_MEAS_ORIENTATION];
	out_q.y = Y->mat1D[POSE_MEAS_ORIENTATION+1];
	out_q.z = Y->mat1D[POSE_MEAS_ORIENTATION+2];
	out_q.w = Y->mat1D[POSE_MEAS_ORIENTATION+3];

	delta_rot.x = addend->mat1D[3];
	delta_rot.y = addend->mat1D[4];
	delta_rot.z = addend->mat1D[5];

	oquatd_from_rotation(&delta_q, &delta_rot);
	oquatd_mult_me(&out_q, &delta_q);

	X->mat1D[POSE_MEAS_ORIENTATION] = out_q.x;
	X->mat1D[POSE_MEAS_ORIENTATION+1] = out_q.y;
	X->mat1D[POSE_MEAS_ORIENTATION+2] = out_q.z;
	X->mat1D[POSE_MEAS_ORIENTATION+3] = out_q.w;

	return true;
}

void rift_kalman_6dof_init(rift_kalman_6dof_filter *state)
{
	int i;

	state->first_update = true;

	/* FIXME: These process noise values are pretty randomly chosen */
	state->Q_noise = matrix2d_alloc0 (COV_SIZE, COV_SIZE);
	for (i = COV_ORIENTATION; i < COV_ORIENTATION + 3; i++)
		state->Q_noise->mat[i][i] = 1e-5;

	for (i = COV_POSITION; i < COV_POSITION + 3; i++)
		state->Q_noise->mat[i][i] = 1e-4;
	for (i = COV_VELOCITY; i < COV_VELOCITY + 3; i++)
		state->Q_noise->mat[i][i] = 2e-4;

	/* Accelerometer and Gyro estimates can change sharply -
	 * even "gentle" motion leads to +/- 5g in a millisecond,
	 * and gyro can easily change 10dps in a millisecond */
	for (i = COV_ACCEL; i < COV_ACCEL + 3; i++)
		state->Q_noise->mat[i][i] = sqrt(3) * 10.0 * 10.0;

	/* Gyro and accel bias have very small variance, since we
	 * want them to change slowly */
	for (i = COV_ACCEL_BIAS; i < COV_ACCEL_BIAS + 3; i++)
		state->Q_noise->mat[i][i] = IMU_ACCEL_BIAS_NOISE;

	for (i = COV_GYRO_BIAS; i < COV_GYRO_BIAS + 3; i++)
		state->Q_noise->mat[i][i] = IMU_GYRO_BIAS_NOISE;

	/* Takes ownership of Q_noise */
	ukf_base_init(&state->ukf, STATE_SIZE, COV_SIZE, state->Q_noise, process_func, state_mean_func, state_residual_func, state_sum_func);

	/* Init unit quaternion in the state */
	state->ukf.x_prior->mat1D[STATE_ORIENTATION + 3] = 1.0;

	/* Initialise the prior covariance / uncertainty - particularly around the biases,
	 * where we assume they are close to 0 somewhere */
	for (i = COV_ACCEL_BIAS; i < COV_ACCEL_BIAS + 3; i++)
		state->ukf.P_prior->mat[i][i] = IMU_ACCEL_BIAS_NOISE_INITIAL;

	for (i = COV_GYRO_BIAS; i < COV_GYRO_BIAS + 3; i++)
		state->ukf.P_prior->mat[i][i] = IMU_GYRO_BIAS_NOISE_INITIAL;

	/* m1 is for IMU measurement - accel */
	ukf_measurement_init(&state->m1, 3, 3, &state->ukf, imu_measurement_func, NULL, NULL, NULL);

	/* FIXME: Set R matrix to something based on IMU noise */
	for (int i = 0; i < 3; i++)
		state->m1.R->mat[i][i] = 1e-6;

	/* m2 is for pose measurements - position and orientation. We trust the position more than
	 * the orientation. */
	ukf_measurement_init(&state->m2, 7, 6, &state->ukf, pose_measurement_func, pose_mean_func, pose_residual_func, pose_sum_func);
	for (int i = 0; i < 3; i++)
		state->m2.R->mat[i][i] = 2.5e-6;
	for (int i = 3; i < 6; i++)
		state->m2.R->mat[i][i] = 0.25;
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
	int64_t dt;
	if (state->first_update) {
		dt = 0;
		state->first_update = false;
	}
	else {
		dt = (int64_t)(time - state->current_ts);
	}
	state->current_ts = time;

	if (!ukf_base_predict(&state->ukf, NS_TO_SEC(dt))) {
			LOGE ("Failed to compute UKF prediction at time %llu (dt %f)", (unsigned long long) state->current_ts, NS_TO_SEC(dt));
			return;
	}

	if (!ukf_base_update(&state->ukf, m)) {
		LOGE ("Failed to perform %s UKF update at time %llu (dt %f)",
					m == &state->m1 ? "IMU" : "Pose", (unsigned long long) state->current_ts, NS_TO_SEC(dt));
		return;
	}

	// print_col_vec ("UKF Mean after update", state->current_ts, state->ukf.x_prior);
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
	m->z->mat1D[IMU_MEAS_ACCEL+0] = accel->x;
	m->z->mat1D[IMU_MEAS_ACCEL+1] = accel->y;
	m->z->mat1D[IMU_MEAS_ACCEL+2] = accel->z;

	rift_kalman_6dof_update(state, time, m);
}

void rift_kalman_6dof_position_update(rift_kalman_6dof_filter *state, uint64_t time, posef *pose)
{
	ukf_measurement *m;

	/* FIXME: Use lagged state vector entries to correct for delay */
	m = &state->m2;
	m->z->mat1D[POSE_MEAS_POSITION+0] = pose->pos.x;
	m->z->mat1D[POSE_MEAS_POSITION+1] = pose->pos.y;
	m->z->mat1D[POSE_MEAS_POSITION+2] = pose->pos.z;

	m->z->mat1D[POSE_MEAS_ORIENTATION+0] = pose->orient.x;
	m->z->mat1D[POSE_MEAS_ORIENTATION+1] = pose->orient.y;
	m->z->mat1D[POSE_MEAS_ORIENTATION+2] = pose->orient.z;
	m->z->mat1D[POSE_MEAS_ORIENTATION+3] = pose->orient.w;

	rift_kalman_6dof_update(state, time, m);
}

void rift_kalman_6dof_get_pose_at(rift_kalman_6dof_filter *state, uint64_t time, posef *pose)
{
	matrix2d *x = state->ukf.x_prior;

	/* FIXME: Do prediction using the time */
	pose->pos.x = x->mat1D[STATE_POSITION];
	pose->pos.y = x->mat1D[STATE_POSITION+1];
	pose->pos.z = x->mat1D[STATE_POSITION+2];

	pose->orient.x = x->mat1D[STATE_ORIENTATION];
	pose->orient.y = x->mat1D[STATE_ORIENTATION+1];
	pose->orient.z = x->mat1D[STATE_ORIENTATION+2];
	pose->orient.w = x->mat1D[STATE_ORIENTATION+3];
}

