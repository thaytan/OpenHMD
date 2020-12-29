// Copyright 2020, Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
#ifndef __UNSCENTED_H__
#define __UNSCENTED_H__

#include "matrices.h"

typedef struct unscented_transform unscented_transform;

/* Callback function that takes a set of sigma points (N_sigmaxN_state) and weights (1xN_sigma) and computes the mean (1xN_state) */
typedef bool (* unscented_mean_fn)(const unscented_transform *ut, const matrix2d *sigmas, const matrix2d *weights, matrix2d *mean);

/* Callback function to compute the state vector (1xstate_N) with the addition of the addend (1xN_cov) */
typedef bool (* unscented_sum_fn)(const unscented_transform *ut, const matrix2d *Y, const matrix2d *addend, matrix2d *X);

/* Callback function to compute the residual between two state vectors (1xL) */
typedef bool (* unscented_residual_fn)(const unscented_transform *ut, const matrix2d *X, const matrix2d *Y, matrix2d *residual);

struct unscented_transform {
  int N_state; /* Dimension of the state vector */
  int N_cov; /* Dimension of the covariance matrices */
  int N_sigma; /* Number of sigma points (2*N_cov+1) */

  /* Scaling factor for sigma points, derived from alpha, beta, kappa */
  double lambda;

  /* Callback functions */
  unscented_mean_fn mean_fn;
  unscented_sum_fn sum_fn;
  unscented_residual_fn residual_fn;

  /* single column of Nx1 weights matrices, where N
   * = the number of sigma points */
  matrix2d *w_means;
  matrix2d *w_cov;

  /* workspace for compute_sigma_points(), LxL */
  matrix2d *P_root;
  /* workspace for compute_ukf_transform() variance computation, N_covx1 */
  matrix2d *P_var;
  /* workspace for residual computation with a residual_fn, N_statex1 */
  matrix2d *X_tmp;
};

void ut_init_van_der_merwe(unscented_transform *ut, uint16_t N_state, uint16_t N_cov, double alpha, double beta, double kappa);
void ut_init_julier(unscented_transform *ut, uint16_t N_state, uint16_t N_cov, double kappa);
void ut_init_matching(unscented_transform *ut, uint16_t N_state, uint16_t N_cov, const unscented_transform *ut_src);
void ut_clear(unscented_transform *ut);

bool ut_compute_sigma_points(const unscented_transform *ut, matrix2d *sigmas, const matrix2d *mean, const matrix2d *cov);
bool ut_compute_transform (const unscented_transform *ut, const matrix2d *sigmas, matrix2d *mean, matrix2d *cov, matrix2d *noise);
bool ut_compute_crossvariance(unscented_transform *utX, unscented_transform *utZ, const matrix2d *sigmasX, const matrix2d *meanX, const matrix2d *sigmasZ, const matrix2d *meanZ, matrix2d *Pxz);

#endif
