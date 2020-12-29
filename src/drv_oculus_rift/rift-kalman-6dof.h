// Copyright 2020, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 *
 * 6DOF Unscented Kalman Filter for positional tracking
 *
 */
#ifndef RIFT_KALMAN_6DOF_H
#define RIFT_KALMAN_6DOF_H

#include "rift.h"
#include "../ukf.h"

typedef struct rift_kalman_6dof_filter rift_kalman_6dof_filter;

struct rift_kalman_6dof_filter {
  /* 22 element state vector:
   * Inertial frame:
   *  quatd orientation (quaternion) (0:3)
   *
   *  vec3d position (4:6)
   *  vec3d velocity; (7:9)
   *  vec3d accel; (10:12)
   *
   * Body frame:
   *  vec3d angular_velocity (13:15)
   *
   *  vec3d accel-bias; (16:18)
   *  vec3d gyro-bias (19:21)
   */
  /* ukf_base needs to be the first element in the struct */
  ukf_base ukf;

  /* Current time tracking */
  bool first_update;
  uint64_t current_ts;

  /* Control vector (gyro reading) */
  vec3d ang_vel;

  /* Process noise */
  matrix2d *Q_noise;

  /* Measurment 1: IMU accel (gyro is in the input vector)
   *   vec3f accel
   */
  ukf_measurement m1;

  /* Measurment 2: Pose
   *  vec3f position
   *  quatd orientation
   */
  ukf_measurement m2;
};

void rift_kalman_6dof_init(rift_kalman_6dof_filter *state);
void rift_kalman_6dof_clear(rift_kalman_6dof_filter *state);

void rift_kalman_6dof_imu_update (rift_kalman_6dof_filter *state, uint64_t time, const vec3f* ang_vel, const vec3f* accel, const vec3f* mag_field);
void rift_kalman_6dof_position_update(rift_kalman_6dof_filter *state, uint64_t time, posef *pose);

void rift_kalman_6dof_get_pose_at(rift_kalman_6dof_filter *state, uint64_t time, posef *pose);

#endif
