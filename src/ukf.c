// Copyright 2020, Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
#include <stdio.h>

#include "ukf.h"

static void print_mat(const char *label, const matrix2d *mat)
{
#if 0
    int i, j;

    if (label)
        printf ("%s: %u rows, %u cols [\n", label, mat->rows, mat->cols);

    for (i = 0; i < mat->rows; i++) {
        printf ("  [");
        for (j = 0; j < mat->cols-1; j++) {
            printf ("%8.4f, ", MATRIX2D_XY(mat, i, j));
        }
        printf ("%10.4f ],\n", MATRIX2D_XY(mat, i, j));
    }
    printf("]\n");
#endif
}

void ukf_base_init(ukf_base *u, int N_state, int N_cov, matrix2d *Q, ukf_process_fn process_fn,
  unscented_mean_fn mean_fn, unscented_residual_fn residual_fn, unscented_sum_fn sum_fn)
{
  u->N_state = N_state;
  u->N_cov = N_cov;

  ut_init_van_der_merwe(&u->ut_X, u->N_state, u->N_cov, 0.1, 2.0, 1.0);

  u->ut_X.mean_fn = mean_fn;
  u->ut_X.residual_fn = residual_fn;
  u->ut_X.sum_fn = sum_fn;

  u->x_prior= matrix2d_alloc0(u->N_state, 1);
  u->P_prior= matrix2d_alloc_identity(u->N_cov);

  u->x = matrix2d_alloc(u->N_state, 1);
  u->P = matrix2d_alloc(u->N_cov, u->N_cov);

  u->Q = Q;

  u->num_sigmas = u->ut_X.N_sigma;
  u->process_fn = process_fn;

  u->sigmas = matrix2d_alloc (u->N_state, u->num_sigmas);

  u->X_tmp = matrix2d_alloc(u->N_state, 1);
  u->X_tmp_prior = matrix2d_alloc(u->N_state, 1);

  u->P_tmp = matrix2d_alloc(u->N_cov, u->N_cov);
}

void ukf_base_clear(ukf_base *u)
{
  matrix2d_free(u->x_prior);
  matrix2d_free(u->P_prior);

  matrix2d_free(u->x);
  matrix2d_free(u->P);
  if (u->Q)
    matrix2d_free(u->Q);

  matrix2d_free(u->sigmas);

  matrix2d_free(u->X_tmp);
  matrix2d_free(u->X_tmp_prior);

  matrix2d_free(u->P_tmp);

  ut_clear(&u->ut_X);
}

bool ukf_base_predict(ukf_base *u, double dt)
{
  return ukf_base_predict_with_process(u, dt, u->process_fn);
}

bool ukf_base_predict_with_process(ukf_base *u, double dt, ukf_process_fn process_fn)
{

  /* Add the additive process noise before prediction, so it is
   * already incorporated in the prediction, and we don't need
   * to re-draw points after */
  if (u->Q) {
    if (matrix2d_add_in_place (u->P_prior, u->Q) != MATRIX_RESULT_OK) {
      return false;
    }
  }

  /* Generate the sigma points around the current state prior */
  if (!ut_compute_sigma_points(&u->ut_X, u->sigmas, u->x_prior, u->P_prior)) {
      printf ("Prediction UKF weight computation failed\n");
      return false;
  }

  /* Loop over each sigma point state vector and transform
   * it through the process function */
  for (int i = 0; i < u->num_sigmas; i++) {
    matrix2d X_tmp;

    if (matrix2d_column_ref(u->sigmas, i, &X_tmp) != MATRIX_RESULT_OK)
      return false;

    /* FIXME:_only pass a single matrix to the process func and make it
     * its responsibility to not modify inputs it needs */
    if (!process_fn(u, dt, &X_tmp, &X_tmp))
      return false;
  }

  if (!ut_compute_transform (&u->ut_X, u->sigmas, u->x, u->P, NULL)) {
      printf ("Failed to compute UKF transformed mean and covariance\n");
      return false;
  }

  /* Fix-up numerical errors in the computed covariance */
  if (matrix2d_condition_symmetry(u->P, 1e-26) != MATRIX_RESULT_OK)
    return false;

  return true;
}

bool
ukf_base_commit(ukf_base *u)
{
  if (matrix2d_copy(u->x_prior, u->x) != MATRIX_RESULT_OK)
    return false;
  if (matrix2d_copy(u->P_prior, u->P) != MATRIX_RESULT_OK)
    return false;

  return true;
}

bool
ukf_base_update(ukf_base *u, ukf_measurement *m)
{
  int i;

  /* Loop over each sigma point state vector and transform
   * it through the measurement function and into the measurement
   * sigmas matrix */
  for (i = 0; i < u->num_sigmas; i++) {
    matrix2d X_tmp;
    matrix2d Z_est;

    if (matrix2d_column_ref(u->sigmas, i, &X_tmp) != MATRIX_RESULT_OK)
      return false;

    if (matrix2d_column_ref(m->sigmas, i, &Z_est) != MATRIX_RESULT_OK)
      return false;

    if (!m->measurement_fn(u, m, &X_tmp, &Z_est))
      return false;
  }

  if (!ut_compute_transform (&m->ut_Z, m->sigmas, m->Z_est, m->Pz, m->R)) {
      printf ("Failed to compute UKF transformed mean and covariance of measurement\n");
      return false;
  }

  print_mat("Measurement sigmas", m->sigmas);
  //print_mat("Measurement weights", m->ut_Z.w_means);

  print_mat("Measurement estimated Z", m->Z_est);
  print_mat("Measurement covariance", m->Pz);

  /* Compute cross-variance Pxz of the state and the measurement */
  if (!ut_compute_crossvariance(&u->ut_X, &m->ut_Z, u->sigmas, u->x, m->sigmas, m->Z_est, m->Pxz))
    return false;

  print_mat("Cross-variance Pxz", m->Pxz);

  /* Compute Kalman gain K = Pxz * Pz^-1 */
  /* The matrix invert function destroys the input, so make a copy */
  if (matrix2d_copy(m->Pz_tmp1, m->Pz) != MATRIX_RESULT_OK)
    return false;
  if (matrix2d_invert(m->Pz_tmp2, m->Pz_tmp1) != MATRIX_RESULT_OK)
    return false;
  if (matrix2d_multiply(m->K, m->Pxz, m->Pz_tmp2) != MATRIX_RESULT_OK)
    return false;

  /* Compute innovation y */
  if (m->ut_Z.residual_fn) {
    if (!m->ut_Z.residual_fn(&m->ut_Z, m->z, m->Z_est, m->y))
      return false;
  }
  else {
    if (matrix2d_subtract(m->y, m->z, m->Z_est) != MATRIX_RESULT_OK)
      return false;
  }

  print_mat("Kalman gain", m->K);
  print_mat("Innovation", m->y);

  /* scale innovation by the Kalman gain */
  if (matrix2d_multiply(u->ut_X.P_var, m->K, m->y) != MATRIX_RESULT_OK)
    return false;

  print_mat("State adjustment matrix", u->ut_X.P_var);

  /* Update the state by the adjustment matrix to be the new
   * x_prior */

  /* Use non-linear state update function if provided */
  if (u->ut_X.sum_fn) {
    if (!u->ut_X.sum_fn(&u->ut_X, u->x, u->ut_X.P_var, u->x_prior))
      return false;
  }
  else {
    if (matrix2d_add(u->x_prior, u->x, u->ut_X.P_var) != MATRIX_RESULT_OK)
      return false;
  }

  /* update P_prior = P_prior - K Py K^T */
  if (matrix2d_multiplyXYXt(u->P_tmp, m->K, m->Pz) != MATRIX_RESULT_OK)
    return false;
  if (matrix2d_subtract(u->P_prior, u->P, u->P_tmp) != MATRIX_RESULT_OK)
    return false;
  if (matrix2d_condition_symmetry(u->P_prior, 1e-26) != MATRIX_RESULT_OK)
    return false;

  print_mat("Covariance adjustment matrix", u->P_tmp);

  return true;
}

void ukf_measurement_init(ukf_measurement *m, int N_measurement, int N_cov, const ukf_base *u, ukf_measurement_fn measurement_fn,
  unscented_mean_fn mean_fn, unscented_residual_fn residual_fn, unscented_sum_fn sum_fn)
{
  m->N_measurement = N_measurement;
  m->N_cov = N_cov;

  /* Initialise UT for our measurement vector dimension */
  ut_init_matching(&m->ut_Z, N_measurement, N_cov, &u->ut_X);
  m->ut_Z.mean_fn = mean_fn;
  m->ut_Z.residual_fn = residual_fn;
  m->ut_Z.sum_fn = sum_fn;
  
  m->num_sigmas = m->ut_Z.N_sigma;
  m->measurement_fn = measurement_fn;

  m->z = matrix2d_alloc0(N_measurement, 1);
  m->R = matrix2d_alloc0(N_cov, N_cov);

  m->y = matrix2d_alloc(N_cov, 1);

  /* Kalman gain from the last update */
  m->K = matrix2d_alloc(u->N_cov, N_cov);

  m->sigmas = matrix2d_alloc (N_measurement, m->num_sigmas);
  m->Z_est = matrix2d_alloc(N_measurement, 1);
  m->Pz = matrix2d_alloc(N_cov, N_cov);

  m->Pxz = matrix2d_alloc(u->N_cov, N_cov);
  m->Pz_tmp1 = matrix2d_alloc(N_cov, N_cov);
  m->Pz_tmp2 = matrix2d_alloc(N_cov, N_cov);
}

void ukf_measurement_clear(ukf_measurement *m)
{
  matrix2d_free(m->z);
  matrix2d_free(m->R);

  matrix2d_free(m->y);
  matrix2d_free(m->K);

  matrix2d_free(m->Pxz);

  matrix2d_free(m->Pz);

  matrix2d_free(m->Pz_tmp1);
  matrix2d_free(m->Pz_tmp2);
  matrix2d_free(m->Z_est);
  matrix2d_free(m->sigmas);

  ut_clear(&m->ut_Z);
}
