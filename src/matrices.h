// Copyright 2020, Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
#ifndef __MATRICES_H__
#define __MATRICES_H__

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct {
  uint16_t m, n; /* rows, cols */

  /* Array of n column pointers. Data stored in column-major order */
  double **mat;
  double *mat1D;
} matrix2d;

typedef enum {
  MATRIX_RESULT_OK = 0,
  MATRIX_RESULT_INVALID = -1, /* Input parameter was invalid */
  MATRIX_RESULT_FAILED = -2, /* The operation failed due to input domain or numerical error */
} matrix_result;

/* Init a matrix of M rows x N columns */
matrix2d *matrix2d_alloc (uint16_t m, uint16_t n);
matrix2d *matrix2d_alloc0 (uint16_t m, uint16_t n);
matrix2d *matrix2d_alloc_init (uint16_t m, uint16_t n, const double **init_vals);
matrix2d *matrix2d_alloc_identity (uint16_t m);
matrix2d *matrix2d_dup (const matrix2d *mat);
void matrix2d_free (matrix2d *mat);

matrix_result matrix2d_copy (matrix2d *dest, const matrix2d *src);
matrix_result matrix2d_extract_row (matrix2d *dest, const matrix2d *src, const uint16_t m);
matrix_result matrix2d_replace_row (matrix2d *dest, const uint16_t m, const matrix2d *src);

matrix_result matrix2d_extract_column (matrix2d *dest, const matrix2d *src, const uint16_t m);
matrix_result matrix2d_replace_column (matrix2d *dest, const uint16_t m, const matrix2d *src);

matrix_result matrix2d_transpose (matrix2d *dest, const matrix2d *src);
matrix_result matrix2d_transpose_in_place (matrix2d *dest);

matrix_result matrix2d_add (matrix2d *dest, const matrix2d *src1, const matrix2d *src2);
matrix_result matrix2d_add_in_place (matrix2d *dest, const matrix2d *src);

matrix_result matrix2d_subtract (matrix2d *dest, const matrix2d *src1, const matrix2d *src2);
matrix_result matrix2d_subtract_in_place (matrix2d *dest, const matrix2d *src);

matrix_result matrix2d_multiply_scalar (matrix2d *dest, const matrix2d *src, double s);
matrix_result matrix2d_multiply (matrix2d *dest, const matrix2d *src1, const matrix2d *src2);

matrix_result matrix2d_multiplyXYXt (matrix2d *dest, const matrix2d *X, const matrix2d *Y);

matrix_result matrix2d_cholesky (matrix2d *dest, const matrix2d *src);
matrix_result matrix2d_cholesky_in_place (matrix2d *mat);

matrix_result matrix2d_condition_symmetry (matrix2d *mat, double epsilon);

/* Note: modifies the source */
matrix_result matrix2d_invert (matrix2d *dest, matrix2d *src);

bool matrix2d_equal (const matrix2d *src1, const matrix2d *src2, double epsilon);
#endif
