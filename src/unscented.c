// Copyright 2020, Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
#include <stddef.h>
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>

#include "unscented.h"

static void print_mat(const char *label, const matrix2d *mat)
{
#if 1
    int i, j;

    if (label)
        printf ("%s: %u rows, %u cols [\n", label, mat->rows, mat->cols);

    for (i = 0; i < mat->rows; i++) {
        printf ("  [");
        for (j = 0; j < mat->cols-1; j++) {
            printf ("%8.4g, ", MATRIX2D_XY(mat, i, j));
        }
        printf ("%10.4g ],\n", MATRIX2D_XY(mat, i, j));
    }
    printf("]\n");
#endif
}

static bool compute_one_residual(const unscented_transform *ut, const matrix2d *sigmas, const int i, const matrix2d *mean, matrix2d *dest);

/*
 * From the van der Merwe paper
 * Sigma-Point Kalman Filters for Nonlinear Estimation and Sensor-Fusion
 * DOI: 10.2514/6.2004-5120
 *
 * N_state is the dimension of the state vector.
 * N_cov is the dimension of the covariance matrices - which can be different
 * to the state vector if sum/residual functions are used to do custom operations.
 *
 * alpha, beta, kappa are tuning parameters controlling the distribution
 * of the sigma points.
 */
void
ut_init_van_der_merwe(unscented_transform *ut, uint16_t N_state, uint16_t N_cov, double alpha, double beta, double kappa)
{
  double L_d = N_cov;
  int i;

  ut->N_state = (int) N_state;
  ut->N_cov = (int) N_cov;

  ut->lambda = alpha*alpha * (L_d + kappa) - L_d;

  ut->mean_fn = NULL;
  ut->residual_fn = NULL;
  ut->sum_fn = NULL;

  int N_sigma = ut->N_sigma = 2*ut->N_cov+1;

  double w_C = 0.5 / (L_d + ut->lambda);

  ut->w_means = matrix2d_alloc (N_sigma, 1);
  ut->w_cov = matrix2d_alloc (N_sigma, 1);

  /* Working space for the cholesky decomp and UKF transform */
  ut->P_root = matrix2d_alloc (ut->N_cov, ut->N_cov);
  ut->P_var = matrix2d_alloc (ut->N_cov, 1);
  ut->X_tmp = matrix2d_alloc (ut->N_state, 1);

  MATRIX2D_Y(ut->w_means, 0) = ut->lambda / (L_d + ut->lambda);
  MATRIX2D_Y(ut->w_cov, 0) = ut->lambda / (L_d + ut->lambda) + (1.0 - alpha*alpha + beta);

  for (i = 1; i < N_sigma; i++)
    MATRIX2D_Y(ut->w_means, i) = MATRIX2D_Y(ut->w_cov, i) = w_C;
}

/*
 * From the Julier & Uhlmann original paper
 * A new extension of the Kalman filter to nonlinear systems
 * DOI:10.1117/12.280797
 *
 * N_state is the dimension of the state vector.
 * N_cov is the dimension of the covariance matrices - which can be different
 * to the state vector if sum/residual functions are used to do custom operations.
 *
 * kappa is a weighting parameter controlling the emphasis on the original mean vs
 * sigma points.
 */
void
ut_init_julier(unscented_transform *ut, uint16_t N_state, uint16_t N_cov, double kappa)
{
  double L_d = N_cov;
  int i;

  ut->N_state = (int) N_state;
  ut->N_cov = (int) N_cov;

  ut->lambda = kappa;

  ut->mean_fn = NULL;
  ut->residual_fn = NULL;
  ut->sum_fn = NULL;

  int N_sigma = ut->N_sigma = 2*ut->N_cov+1;

  double w_C = 0.5 / (L_d + ut->lambda);

  ut->w_means = matrix2d_alloc (N_sigma, 1);
  ut->w_cov = matrix2d_alloc (N_sigma, 1);

  /* Working space for the cholesky decomp and UKF transform */
  ut->P_root = matrix2d_alloc (ut->N_cov, ut->N_cov);
  ut->P_var = matrix2d_alloc (ut->N_cov, 1);
  ut->X_tmp = matrix2d_alloc (ut->N_state, 1);

  MATRIX2D_Y(ut->w_cov, 0) = MATRIX2D_Y(ut->w_means, 0) = ut->lambda;
  for (i = 1; i < N_sigma; i++)
    MATRIX2D_Y(ut->w_means, i) = MATRIX2D_Y(ut->w_cov, i) = w_C;
}

/* Set up a UT from an existing one, with the same sigma points distribution but
 * a different state/measurement vector dimension L */
void
ut_init_matching(unscented_transform *ut, uint16_t N_state, uint16_t N_cov, const unscented_transform *ut_src)
{
  ut->N_state = (int) N_state;
  ut->N_cov = (int) N_cov;
  ut->N_sigma = ut_src->N_sigma;
  ut->lambda = ut_src->lambda;

  ut->mean_fn = NULL;
  ut->residual_fn = NULL;
  ut->sum_fn = NULL;

  ut->w_means = matrix2d_dup (ut_src->w_means);
  ut->w_cov = matrix2d_dup (ut_src->w_cov);

  /* Working space for the cholesky decomp and UKF transform */
  ut->P_root = matrix2d_alloc (ut->N_cov, ut->N_cov);
  ut->P_var = matrix2d_alloc (ut->N_cov, 1);
  ut->X_tmp = matrix2d_alloc (ut->N_state, 1);
}

void
ut_clear(unscented_transform *ut)
{
  matrix2d_free (ut->w_means);
  matrix2d_free (ut->w_cov);
  matrix2d_free (ut->P_root);
  matrix2d_free (ut->P_var);
  matrix2d_free (ut->X_tmp);
}

/* Calculate the array of sigma-points (sigmas) around the given
 * mean and covariance using the weights matrix
 *
 * Input:
 *   mean: 1xN_state means matrix
 *    cov: N_cov x N_cov covariance matrix
 * Output:
 * sigmas: N_sigma x N_cov sigma points (N_sigma columns of N_cov rows)
 */
bool
ut_compute_sigma_points(const unscented_transform *ut, matrix2d *sigmas, const matrix2d *mean, const matrix2d *cov)
{
  double L_d = ut->N_cov;

  /* Means is a single column of L values */
  if (mean->rows != ut->N_state || mean->cols != 1)
    return false;

  /* Covariance matrix must be square */
  if (cov->rows != ut->N_cov || cov->cols != ut->N_cov)
    return false;

  /* We need space for 2*n + 1 sigma point columns, each N_state rows */
  if (sigmas->rows != ut->N_state || sigmas->cols != ut->N_sigma)
    return false;

  if (matrix2d_multiply_scalar (ut->P_root, cov, ut->lambda + L_d) != MATRIX_RESULT_OK)
    return false;

  if (matrix2d_cholesky_in_place (ut->P_root) != MATRIX_RESULT_OK) {
		print_mat("covariance", cov);
    print_mat("Cholesky decomposition failed on P_root", ut->P_root);
		abort();
    return false;
  }

  /* External sum_fn - extract each column of the P_root and
   * generate the sigma point using it */
  if (ut->sum_fn != NULL) {
    /* zero-th entry is the unmodified state vector */
    for (int i = 0; i < ut->N_state; i++) {
      MATRIX2D_XY(sigmas, i, 0) = MATRIX2D_Y(mean, i);
    }

    for (int i = 0; i < ut->N_cov; i++) {
      /* Implicit transpose of the P_root matrix here, because the
       * cholesky calc generates the lower-triangular matrix. */
      /* Generate X = X_prior + P_var sigma points */
      for (int j = 0; j < ut->N_cov; j++) {
        MATRIX2D_Y(ut->P_var, j) = MATRIX2D_XY(ut->P_root, j, i);
      }
      if (!ut->sum_fn(ut, mean, ut->P_var, ut->X_tmp))
        return false;
      if (matrix2d_replace_column(sigmas, i+1, ut->X_tmp) != MATRIX_RESULT_OK)
        return false;

      /* Repeat for X = X_prior - P_var */
      for (int j = 0; j < ut->N_cov; j++) {
        MATRIX2D_Y(ut->P_var, j) = -MATRIX2D_XY(ut->P_root, j, i);
      }
      if (!ut->sum_fn(ut, mean, ut->P_var, ut->X_tmp))
        return false;
      if (matrix2d_replace_column(sigmas, ut->N_cov+i+1, ut->X_tmp) != MATRIX_RESULT_OK)
        return false;
    }
  } else {
    for (int i = 0; i < ut->N_cov; i++) {
      MATRIX2D_XY(sigmas, i, 0) = MATRIX2D_Y(mean, i);

      for (int j = 0; j < ut->N_cov; j++) {
        /* Implicit transpose of the P_root matrix here, because the
         * cholesky calc generates the lower-triangular matrix. */
        MATRIX2D_XY(sigmas, i, j+1) = MATRIX2D_Y(mean, i) + MATRIX2D_XY(ut->P_root, i, j);
        MATRIX2D_XY(sigmas, i, ut->N_cov+j+1) = MATRIX2D_Y(mean, i) - MATRIX2D_XY(ut->P_root, i, j);
      }
    }
  }

  return true;
}

/* Compute the new mean and covariance of transformed
 * sigma points, weighted by the weights array stored
 * in the unscented_transform struct.
 *
 * This is the 2nd equations in step 3 (Time-update
 * equations) of the UKF section of the paper,
 *
 * Used for the predict step to compute the predicted
 * means and covariance, or in the update step
 * to compute the measurement update means and covariances.
 *
 * If a noise matrix is provided, it is added to the final
 * covariance matrix.
 *
 * Input:
 * sigmas: N_sigma x N_cov sigma points (N_sigma columns of N_cov rows)
 * noise: N_cov x N_cov noise matrix to add to the covariance (optional)
 *
 * Output:
 *
 * mean: 1xN_state state vector
 * cov: N_cov x N_cov covariance matrix
 *
 */
bool
ut_compute_transform (const unscented_transform *ut, const matrix2d *sigmas, matrix2d *mean, matrix2d *cov, matrix2d *noise)
{
  if (sigmas->rows != mean->rows || sigmas->cols != ut->N_sigma)
    return false;

  if (cov->rows != ut->N_cov || cov->cols != ut->N_cov)
    return false;

  if (noise) {
    if (cov->rows != noise->rows || cov->cols != noise->cols)
      return false;
  }

  /* Mean is the sum of the sigmas (LxN) * weights (1xN) */
  if (ut->mean_fn == NULL) {
    if (matrix2d_multiply(mean, sigmas, ut->w_means) != MATRIX_RESULT_OK) {
      return false;
    }
  } else {
    /* Call the callback for computing non-linear means */
    if (!ut->mean_fn(ut, sigmas, ut->w_means, mean))
      return false;
  }

  /* Compute NxN covariance matrix */
  int i, j, k;

  /* Clear the output covariance matrix.
   * FIXME: This could be skipped with an if check
   * in the loop below to set instead of accumulate
   * on the first loop. Not sure if that'll be any
   * quicker for small matrices */
  for (i = 0; i < ut->N_cov * ut->N_cov; i++) {
    MATRIX2D_Y(cov, i) = 0.0;
  }

  for (k = 0; k < ut->N_sigma; k++) {
    /* Compute residual for this sigma point */
    if (!compute_one_residual(ut, sigmas, k, mean, ut->P_var))
      return false;

    /* Add the weighted version to the covariance matrix using
     * outer product */
    double w_C = MATRIX2D_Y(ut->w_cov, k);

    /* Optimised by looping over the cov array directly instead
     * of via pointers, compute the lower half triangle */
    for (i = 0; i < ut->N_cov; i++) {
      double *cov_ptr = &MATRIX2D_XY(cov, 0, i);
      double P_var_i = w_C * MATRIX2D_Y(ut->P_var, i);

      for (j = i; j < ut->N_cov; j++)
        cov_ptr[j] += MATRIX2D_Y(ut->P_var, j) * P_var_i;
    }
  }

  for (i = 0; i < ut->N_cov; i++) {
    for (j = i; j < ut->N_cov; j++) {
      /* Copy to the upper half triangle */
      MATRIX2D_XY(cov, i, j) = MATRIX2D_XY(cov, j, i);
    }
  }

  if (noise) {
    if (matrix2d_add_in_place (cov, noise) != MATRIX_RESULT_OK) {
      return false;
    }
  }

  return true;
}

static bool compute_one_residual(const unscented_transform *ut, const matrix2d *sigmas, const int i, const matrix2d *mean, matrix2d *dest) {
  int j;

  assert(i >= 0 && i < ut->N_sigma);

  if (ut->residual_fn == NULL) {
    for (j = 0; j < ut->N_cov; j++) {
      MATRIX2D_Y(dest, j) = MATRIX2D_XY(sigmas, j, i) - MATRIX2D_Y(mean, j);
    }
  } else {
    /* Use external residual func ptr, to compute residuals with
     * methods other than direct subtraction */
    matrix2d X_tmp;

    if (matrix2d_column_ref(sigmas, i, &X_tmp) != MATRIX_RESULT_OK)
      return false;

    if (!ut->residual_fn(ut, &X_tmp, mean, dest))
      return false;
  }
  return true;
}

bool ut_compute_crossvariance(unscented_transform *utX, unscented_transform *utZ, const matrix2d *sigmasX, const matrix2d *meanX, const matrix2d *sigmasZ, const matrix2d *meanZ, matrix2d *Pxz)
{
  int i, j, k;
  double *Pxz_ptr = &MATRIX2D_Y(Pxz, 0);
  double *P_var_X, *P_var_Z;

  /* Clear the Pxz matrix */
  for (i = 0; i < utX->N_cov * utZ->N_cov; i++)
    *Pxz_ptr++ = 0.0;

  for (k = 0; k < utX->N_sigma; k++) {
    /* Compute residuals for this sigma point */
    if (!compute_one_residual(utX, sigmasX, k, meanX, utX->P_var))
      return false;
    if (!compute_one_residual(utZ, sigmasZ, k, meanZ, utZ->P_var))
      return false;

    /* Add the weighted version to the cross-variance matrix using
     * outer product */
    double w_C = MATRIX2D_Y(utX->w_cov, k);

    /* Optimise by looping over the Pxz and input arrays directly instead
     * of via pointers */
    Pxz_ptr = &MATRIX2D_Y(Pxz, 0);
    P_var_X = &MATRIX2D_Y(utX->P_var, 0);
    P_var_Z = &MATRIX2D_Y(utZ->P_var, 0);

    for (j = 0; j < utZ->N_cov; j++) {
      for (i = 0; i < utX->N_cov; i++) {
        *Pxz_ptr++ += w_C * P_var_X[i] * P_var_Z[j];
      }
    }
  }

  return true;
}
