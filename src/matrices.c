// Copyright 2020, Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
#include <stdlib.h>
#include <string.h>

#include "matrices.h"

#if MATRIX_CHECKS
#define MATRIX_CHECK(s,r) \
    if (!(s)) \
      return (r);
#else
#define MATRIX_CHECK(s,r)
#endif

matrix2d *
matrix2d_alloc (uint16_t rows, uint16_t cols)
{
    uint8_t *mem;
    matrix2d *mat;
    size_t mem_size;

    MATRIX_CHECK(rows > 0 && cols > 0, NULL);

    mem_size = sizeof(matrix2d);
    mem_size += sizeof(double)*rows*cols;
    mem = malloc (mem_size);

    mat = (matrix2d *)(mem);
    mem += sizeof (matrix2d);

    mat->mem = (double *)(mem);

    mat->cols = cols;
    mat->stride = mat->rows = rows;

    return mat;
}

matrix2d *
matrix2d_alloc0 (uint16_t rows, uint16_t cols)
{
    matrix2d *mat = matrix2d_alloc (rows, cols);
    memset (mat->mem, 0, sizeof (double) * rows * cols);

    return mat;
}

matrix2d *
matrix2d_alloc_init (uint16_t rows, uint16_t cols, const double **init_vals)
{
    matrix2d *mat = matrix2d_alloc (rows, cols);
    memcpy (mat->mem, init_vals, sizeof (double) * rows * cols);

    return mat;
}

matrix2d *
matrix2d_alloc_identity (uint16_t rows)
{
    matrix2d *mat = matrix2d_alloc0(rows, rows);
    double *ptr = mat->mem;
		int i;

    for (i = 0; i < rows; i++) {
      ptr[0] = 1.0;
      ptr += mat->stride + 1;
    }
    return mat;
}

matrix2d *
matrix2d_dup (const matrix2d *src)
{
    matrix2d *dest = matrix2d_alloc (src->rows, src->cols);

		if (dest->stride == src->stride) {
			memcpy (dest->mem, src->mem, sizeof (double) * src->stride * src->cols);
		}
		else {
			double *in_ptr = src->mem;
			double *out_ptr = dest->mem;
			int i;

			/* Copy column by column because the stride is changing
			* (due to dup'ing a sub-matrix */
			for (i = 0; i < src->cols; i++) {
				memcpy(out_ptr, in_ptr, sizeof(double) * src->rows);
				out_ptr += dest->stride;
				in_ptr += src->stride;
			}
		}
    return dest;
}

void matrix2d_free (matrix2d *mat)
{
    free (mat);
}

matrix_result
matrix2d_copy (matrix2d *dest, const matrix2d *src)
{
    MATRIX_CHECK(dest->rows == src->rows && dest->cols == src->cols, MATRIX_RESULT_INVALID);

		if (dest->stride == src->stride) {
			memcpy (dest->mem, src->mem, sizeof (double) * src->stride * src->cols);
		}
		else {
			double *in_ptr = src->mem;
			double *out_ptr = dest->mem;
			int i;

			/* Copy column by column because the src and dest have different stride */
			for (i = 0; i < src->cols; i++) {
				memcpy(out_ptr, in_ptr, sizeof(double) * src->rows);
				out_ptr += dest->stride;
				in_ptr += src->stride;
			}
		}

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_submatrix_ref(const matrix2d *src, uint16_t x, uint16_t y, uint16_t rows, uint16_t cols, matrix2d *dest)
{
    MATRIX_CHECK(x < src->rows && x + rows < src->rows, MATRIX_RESULT_INVALID);
    MATRIX_CHECK(y < src->cols && y + cols < src->cols, MATRIX_RESULT_INVALID);

    dest->stride = src->stride;
    dest->rows = rows;
    dest->cols = cols;

    dest->mem = &MATRIX2D_XY(src, x, y);

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_column_ref(const matrix2d *src, uint16_t col, matrix2d *dest)
{
    MATRIX_CHECK(col < src->cols, MATRIX_RESULT_INVALID);

    dest->stride = src->stride;
    dest->rows = src->rows;
    dest->cols = 1;

    dest->mem = &MATRIX2D_XY(src, 0, col);

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_extract_row (matrix2d *dest, const matrix2d *src, const uint16_t row)
{
    int i;

    MATRIX_CHECK(dest->rows == 1 && dest->cols == src->cols && row < src->rows, MATRIX_RESULT_INVALID);

		double *in_ptr = &MATRIX2D_XY(src, row, 0);
		double *out_ptr = dest->mem;

    for (i = 0; i < src->cols; i++) { 
			 *out_ptr++ = *in_ptr;
			 in_ptr += src->stride;
		}

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_replace_row (matrix2d *dest, const uint16_t row, const matrix2d *src)
{
    int i;

    MATRIX_CHECK(src->rows == 1 && dest->cols == src->cols && row < dest->rows, MATRIX_RESULT_INVALID);

		double *in_ptr = src->mem;
		double *out_ptr = &MATRIX2D_XY(dest, row, 0);

    for (i = 0; i < src->cols; i++) { 
			 *out_ptr++ = *in_ptr;
			 in_ptr += dest->stride;
		}

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_extract_column (matrix2d *dest, const matrix2d *src, const uint16_t col)
{
    int i;

    MATRIX_CHECK(dest->cols == 1 && dest->rows == src->rows && col < src->cols, MATRIX_RESULT_INVALID);

		double *src_ptr = &MATRIX2D_XY(src, 0, col);
		double *out_ptr = dest->mem;

    for (i = 0; i < src->rows; i++)
      out_ptr[i] = src_ptr[i];

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_replace_column (matrix2d *dest, const uint16_t col, const matrix2d *src)
{
    int i;

    MATRIX_CHECK(src->cols == 1 && dest->rows == src->rows && col < dest->cols, MATRIX_RESULT_INVALID);

		double *in_ptr = src->mem;
		double *out_ptr = &MATRIX2D_XY(dest, 0, col);

    for (i = 0; i < src->rows; i++)
			out_ptr[i] = in_ptr[i];

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_transpose (matrix2d *dest, const matrix2d *src)
{
    int i, j;

    MATRIX_CHECK(dest->rows == src->cols && dest->cols == src->rows, MATRIX_RESULT_INVALID);

    for (i = 0; i < dest->rows; i++) {
				double *dest_row = &MATRIX2D_XY(dest, i, 0);
				double *src_col = &MATRIX2D_XY(src, 0, i);

        for (j = 0; j < dest->cols; j++) {
						*dest_row = *src_col++;
						dest_row += dest->stride;
        }
    }
    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_transpose_in_place (matrix2d *dest)
{
    int i, j;

    /* Must be square */
    MATRIX_CHECK(dest->cols == dest->rows, MATRIX_RESULT_INVALID);

    for (i = 0; i < dest->rows; i++) {
        for (j = i; j < dest->cols; j++) {
            double tmp = MATRIX2D_XY(dest, j, i);
            MATRIX2D_XY(dest, j, i) = MATRIX2D_XY(dest, i, j);
            MATRIX2D_XY(dest, i, j) = tmp;
        }
    }
    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_add (matrix2d *dest, const matrix2d *src1, const matrix2d *src2)
{
    int i, j;

    MATRIX_CHECK(dest->rows == src1->rows && src1->rows == src2->rows &&
        dest->cols == src1->cols && src1->cols == src2->cols, MATRIX_RESULT_INVALID);

		double *dest_ptr = dest->mem;
		double *src1_ptr = src1->mem;
		double *src2_ptr = src2->mem;

    for (j = 0; j < dest->cols; j++) {
        for (i = 0; i < dest->rows; i++) {
						dest_ptr[i] = src1_ptr[i] + src2_ptr[i];
        }
				dest_ptr += dest->stride;
				src1_ptr += src1->stride;
				src2_ptr += src2->stride;
    }

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_add_in_place (matrix2d *dest, const matrix2d *src)
{
    int i, j;

    MATRIX_CHECK(dest->rows == src->rows && dest->cols && src->cols, MATRIX_RESULT_INVALID);

		double *dest_ptr = dest->mem;
		double *src_ptr = src->mem;

    for (j = 0; j < dest->cols; j++) {
      for (i = 0; i < dest->rows; i++) {
        dest_ptr[i] += src_ptr[i];
      }
			dest_ptr += dest->stride;
			src_ptr += src->stride;
    }

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_subtract (matrix2d *dest, const matrix2d *src1, const matrix2d *src2)
{
    int i, j;

    MATRIX_CHECK(dest->rows == src1->rows && src1->rows == src2->rows &&
        dest->cols == src1->cols && src1->cols && src2->cols, MATRIX_RESULT_INVALID);

		double *dest_ptr = dest->mem;
		double *src1_ptr = src1->mem;
		double *src2_ptr = src2->mem;

    for (j = 0; j < dest->cols; j++) {
        for (i = 0; i < dest->rows; i++) {
						dest_ptr[i] = src1_ptr[i] - src2_ptr[i];
        }
				dest_ptr += dest->stride;
				src1_ptr += src1->stride;
				src2_ptr += src2->stride;
    }

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_subtract_in_place (matrix2d *dest, const matrix2d *src)
{
    int i, j;

    MATRIX_CHECK(dest->cols == src->rows && dest->cols == src->cols, MATRIX_RESULT_INVALID);

		double *dest_ptr = dest->mem;
		double *src_ptr = src->mem;

    for (j = 0; j < dest->cols; j++) {
      for (i = 0; i < dest->rows; i++) {
        dest_ptr[i] -= src_ptr[i];
      }
			dest_ptr += dest->stride;
			src_ptr += src->stride;
    }

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_multiply (matrix2d *dest, const matrix2d *src1, const matrix2d *src2)
{
    int i, j, k;

    /* Number of columns in src1 must equal number of rows in src2,
     * and dest must be src1->rows x src2->cols */
    MATRIX_CHECK(src1->cols == src2->rows && dest->rows == src1->rows && dest->cols == src2->cols,
        MATRIX_RESULT_INVALID);

    for (j = 0; j < dest->cols; j++) {
        for (i = 0; i < dest->rows; i++) {
            double s = 0.0;
            for (k = 0; k < src1->cols; k++)
                s += MATRIX2D_XY(src1, i, k) * MATRIX2D_XY(src2, k, j);
            MATRIX2D_XY(dest, i, j) = s;
        }
    }

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_multiply_scalar (matrix2d *dest, const matrix2d *src, double s)
{
    int i, j;

    /* Dest and src must be the same dimensions */
    MATRIX_CHECK(dest->rows == src->rows && dest->cols == src->cols, MATRIX_RESULT_INVALID);

    for (j = 0; j < dest->cols; j++) {
        for (i = 0; i < dest->rows; i++) {
            MATRIX2D_XY(dest, i, j) = s * MATRIX2D_XY(src, i, j);
        }
    }

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_cholesky (matrix2d *dest, const matrix2d *src)
{
    int i, j, k;

    MATRIX_CHECK(dest->rows == dest->cols && src->rows == src->cols && dest->rows == src->rows, MATRIX_RESULT_INVALID);

    for (i = 0; i < src->rows; i++) {
        for (j = i; j < src->cols; j++) {
            double s = MATRIX2D_XY(src, i, j);
            for (k = 0; k < i; k++)
                s -= MATRIX2D_XY(dest, i, k) * MATRIX2D_XY(dest, j, k);
            if (j == i) {
                if (s > 0)
										MATRIX2D_XY(dest, i, j) = sqrt(s);
                else
                    return MATRIX_RESULT_FAILED; /* Input was not positive-definite */
            }
            else {
								MATRIX2D_XY(dest, j, i) = s / MATRIX2D_XY(dest, i, i);
						}
        }
    }

    /* Clear the upper-right triangle */
    for (i = 0; i < src->rows; i++) {
        for (j = 0; j < i; j++) {
						MATRIX2D_XY(dest, j, i) = 0.0;
        }
    }

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_cholesky_in_place (matrix2d *mat)
{
    /* The Cholesky Decomposition can actually be done in place */
    return matrix2d_cholesky (mat, mat);
}

bool matrix2d_equal (const matrix2d *src1, const matrix2d *src2, const double epsilon)
{
    int i, j;

    MATRIX_CHECK(src1->rows == src2->rows && src1->cols == src2->cols, false);

    for (j = 0; j < src1->cols; j++) {
        for (i = 0; i < src1->rows; i++) {
            double delta = MATRIX2D_XY(src1, i, j) - MATRIX2D_XY(src2, i, j);
            if (delta < -epsilon || delta > epsilon)
                return false;
        }
    }

    return true;
}

matrix_result matrix2d_invert (matrix2d *dest, matrix2d *src)
{
  int i, j, k;
  double s;

  MATRIX_CHECK(dest->rows == dest->cols && dest->rows == src->rows && dest->cols == src->cols, MATRIX_RESULT_INVALID);

  /* Initialise output to the identity matrix */
  for (j = 0; j < dest->cols; j++) {
    for (i = 0; i < dest->rows; i++) {
      if (i == j)
        MATRIX2D_XY(dest, i, j) = 1.0;
      else
        MATRIX2D_XY(dest, i, j) = 0.0;
    }
  }

  /* Gauss-Jordan elimination */
  for (k = 0; k < dest->rows; k++) {
    s = MATRIX2D_XY(src, k, k);
    if (s == 0)
      return MATRIX_RESULT_FAILED; /* Inversion failed */

    for (j = 0; j < dest->cols; j++) {
      MATRIX2D_XY(src,k,j) /= s;
      MATRIX2D_XY(dest,k,j) /= s;
    }

    for (i = 0; i < dest->rows; i++) {
      s = MATRIX2D_XY(src,i,k);
      for (j = 0; j < dest->cols && i != k; j++) {
        MATRIX2D_XY(src,i,j)  -= s * MATRIX2D_XY(src,k,j);
        MATRIX2D_XY(dest,i,j) -= s * MATRIX2D_XY(dest,k,j);
      }
    }
  }

  return MATRIX_RESULT_OK;
}

/* Compute X * Y * Xt */
matrix_result matrix2d_multiplyXYXt (matrix2d *dest, const matrix2d *X, const matrix2d *Y)
{
  matrix_result ret;
  matrix2d *tmp; /* FIXME: Avoid allocation - pass a tmp space? */
  int i, j, k;

  /* Dest and X must be square matrices, with dimensions that match correspondingly
   * to Y */
  MATRIX_CHECK(Y->cols == Y->rows && dest->rows == dest->cols &&
       dest->cols == X->rows && X->cols == Y->rows, MATRIX_RESULT_INVALID);

  tmp = matrix2d_alloc(X->rows, X->cols);

  ret = matrix2d_multiply(tmp, X, Y);
  if (ret != MATRIX_RESULT_OK)
    goto out;

  /* Multiply tmp by the transpose of X */
  for (j = 0; j < dest->cols; j++) {
    for (i = 0; i < dest->rows; i++) {
      double s = 0.0;
      for (k = 0; k < tmp->cols; k++)
        s += MATRIX2D_XY(tmp,i,k) * MATRIX2D_XY(X,j,k);
      MATRIX2D_XY(dest,i,j) = s;
    }
  }

out:
  matrix2d_free(tmp);
  return ret;
}

/* Do some fixup on a symmetric matrix to average out numeric
 * instability, and add an epsilon to the diagonal to help make it
 * more well-conditioned */
matrix_result matrix2d_condition_symmetry (matrix2d *mat, double epsilon)
{
    int i, j;
    double *m1, *m2;

    /* Must be square */
    MATRIX_CHECK(mat->cols == mat->rows, MATRIX_RESULT_INVALID);

    for (i = 0; i < mat->rows; i++) {
        MATRIX2D_XY(mat,i,i) += epsilon;

        for (j = i+1; j < mat->cols; j++) {
            double tmp;
            m1 = &MATRIX2D_XY(mat,j,i);
            m2 = &MATRIX2D_XY(mat,i,j);
            tmp = (0.5 * *m1 + 0.5 * *m2);

            *m1 = *m2 = tmp;
        }
    }

    return MATRIX_RESULT_OK;
}
