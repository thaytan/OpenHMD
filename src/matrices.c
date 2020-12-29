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
matrix2d_alloc (uint16_t m, uint16_t n)
{
    uint8_t *mem;
    matrix2d *mat;
    size_t mem_size;
    int i;

    MATRIX_CHECK(m > 0 && n > 0, NULL);

    mem_size = sizeof(matrix2d) + sizeof (double *) * n;
    mem_size += sizeof(double)*m*n;
    mem = malloc (mem_size);

    mat = (matrix2d *)(mem);
    mem += sizeof (matrix2d);

    mat->mat = (double **)(mem);
    mem += sizeof (double *) * n;

    mat->mat1D = (double *) mem;

    for (i = 0; i < n; i++) {
      mat->mat[i] = (double *)(mem);
      mem += sizeof (double) * m;
    }

    mat->m = m;
    mat->n = n;

    return mat;
}

matrix2d *
matrix2d_alloc0 (uint16_t m, uint16_t n)
{
    matrix2d *mat = matrix2d_alloc (m, n);
    memset (mat->mat1D, 0, sizeof (double) * m * n);

    return mat;
}

matrix2d *
matrix2d_alloc_init (uint16_t m, uint16_t n, const double **init_vals)
{
    matrix2d *mat = matrix2d_alloc (m, n);
    memcpy (mat->mat1D, init_vals, sizeof (double) * m * n);

    return mat;
}

matrix2d *
matrix2d_alloc_identity (uint16_t m)
{
    matrix2d *mat = matrix2d_alloc (m, m);
    int i;

    memset (mat->mat[0], 0, sizeof (double)*m*m);
    for (i = 0; i < m; i++)
        mat->mat[i][i] = 1.0;
    return mat;
}

matrix2d *
matrix2d_dup (const matrix2d *src)
{
    matrix2d *dest = matrix2d_alloc (src->m, src->n);
    memcpy (dest->mat1D, src->mat1D, sizeof (double) * src->m * src->n);

    return dest;
}

void matrix2d_free (matrix2d *mat)
{
    free (mat);
}

matrix_result
matrix2d_copy (matrix2d *dest, const matrix2d *src)
{
    MATRIX_CHECK(dest->m == src->m && dest->n == src->n, MATRIX_RESULT_INVALID);

    memcpy (dest->mat1D, src->mat1D, sizeof (double) * src->m * src->n);

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_extract_row (matrix2d *dest, const matrix2d *src, const uint16_t m)
{
    int i;

    MATRIX_CHECK(dest->m == 1 && dest->n == src->n && m < src->m, MATRIX_RESULT_INVALID);

    for (i = 0; i < src->n; i++)
       dest->mat[i][0] = src->mat[i][m];

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_replace_row (matrix2d *dest, const uint16_t m, const matrix2d *src)
{
    int i;

    MATRIX_CHECK(src->m == 1 && dest->n == src->n && m < dest->m, MATRIX_RESULT_INVALID);

    for (i = 0; i < src->n; i++)
       dest->mat[i][m] = src->mat[i][0];

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_extract_column (matrix2d *dest, const matrix2d *src, const uint16_t n)
{
    int i;
    double *col, *d;

    MATRIX_CHECK(dest->n == 1 && dest->m == src->m && n < src->n, MATRIX_RESULT_INVALID);

    d = dest->mat1D;
    col = src->mat[n];
    for (i = 0; i < src->m; i++)
      *d++ = *col++;

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_replace_column (matrix2d *dest, const uint16_t n, const matrix2d *src)
{
    int i;

    MATRIX_CHECK(src->n == 1 && dest->m == src->m && n < dest->n, MATRIX_RESULT_INVALID);

    for (i = 0; i < src->m; i++)
       dest->mat[n][i] = src->mat1D[i];

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_transpose (matrix2d *dest, const matrix2d *src)
{
    int i, j;

    MATRIX_CHECK(dest->m == src->n && dest->n == src->m, MATRIX_RESULT_INVALID);

    for (i = 0; i < dest->m; i++) {
        for (j = 0; j < dest->n; j++) {
            dest->mat[j][i] = src->mat[i][j];
        }
    }
    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_transpose_in_place (matrix2d *dest)
{
    int i, j;

    /* Must be square */
    MATRIX_CHECK(dest->n == dest->m, MATRIX_RESULT_INVALID);

    for (i = 0; i < dest->m; i++) {
        for (j = i; j < dest->n; j++) {
            double tmp = dest->mat[j][i];
            dest->mat[j][i] = dest->mat[i][j];
            dest->mat[i][j] = tmp;
        }
    }
    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_add (matrix2d *dest, const matrix2d *src1, const matrix2d *src2)
{
    int i, j;

    MATRIX_CHECK(dest->m == src1->m && src1->m == src2->m &&
        dest->n == src1->n && src1->n == src2->n, MATRIX_RESULT_INVALID);

    for (j = 0; j < dest->n; j++) {
        for (i = 0; i < dest->m; i++) {
            dest->mat[j][i] = src1->mat[j][i] + src2->mat[j][i];
        }
    }

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_add_in_place (matrix2d *dest, const matrix2d *src)
{
    int i, j;

    MATRIX_CHECK(dest->m == src->m && dest->n && src->n, MATRIX_RESULT_INVALID);

    for (j = 0; j < dest->n; j++) {
      for (i = 0; i < dest->m; i++) {
            dest->mat[j][i] += src->mat[j][i];
        }
    }

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_subtract (matrix2d *dest, const matrix2d *src1, const matrix2d *src2)
{
    int i;
    double *d, *s1, *s2;

    MATRIX_CHECK(dest->m == src1->m && src1->m == src2->m &&
        dest->n == src1->n && src1->n && src2->n, MATRIX_RESULT_INVALID);

    d = dest->mat1D;
    s1 = src1->mat1D;
    s2 = src2->mat1D;

    for (i = 0; i < dest->m * dest->n; i++)
      *d++ = *s1++ - *s2++;

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_subtract_in_place (matrix2d *dest, const matrix2d *src)
{
    int i;
    double *d, *s;

    MATRIX_CHECK(dest->n == src->m && dest->n == src->n, MATRIX_RESULT_INVALID);

    d = dest->mat1D;
    s = src->mat1D;

    for (i = 0; i < dest->m * dest->n; i++)
      *d++ -= *s++;

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_multiply (matrix2d *dest, const matrix2d *src1, const matrix2d *src2)
{
    int i, j, k;

    /* Number of columns in src1 must equal number of rows in src2,
     * and dest must be src1->m x src2->n */
    MATRIX_CHECK(src1->n == src2->m && dest->m == src1->m && dest->n == src2->n,
        MATRIX_RESULT_INVALID);

    for (j = 0; j < dest->n; j++) {
        for (i = 0; i < dest->m; i++) {
            double s = 0.0;
            for (k = 0; k < src1->n; k++)
                s += src1->mat[k][i] * src2->mat[j][k];
            dest->mat[j][i] = s;
        }
    }

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_multiply_scalar (matrix2d *dest, const matrix2d *src, double s)
{
    int i, j;

    /* Dest and src must be the same dimensions */
    MATRIX_CHECK(dest->m == src->m && dest->n == src->n, MATRIX_RESULT_INVALID);

    for (j = 0; j < dest->n; j++) {
        for (i = 0; i < dest->m; i++) {
            dest->mat[j][i] = s * src->mat[j][i];
        }
    }

    return MATRIX_RESULT_OK;
}

matrix_result
matrix2d_cholesky (matrix2d *dest, const matrix2d *src)
{
    int i, j, k;

    MATRIX_CHECK(dest->m == dest->n && src->m == src->n && dest->m == src->m, MATRIX_RESULT_INVALID);

    for (i = 0; i < src->m; i++) {
        for (j = i; j < src->n; j++) {
            double s = src->mat[j][i];
            for (k = 0; k < i; k++)
                s -= dest->mat[k][i]*dest->mat[k][j];
            if (j == i) {
                if (s > 0)
                    dest->mat[j][i] = sqrt(s);
                else
                    return MATRIX_RESULT_FAILED; /* Input was not positive-definite */
            }
            else
                dest->mat[i][j] = s / dest->mat[i][i];
        }
    }

    /* Clear the upper-right triangle */
    for (i = 0; i < src->m; i++) {
        for (j = 0; j < i; j++) {
            dest->mat[i][j] = 0.0;
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

    MATRIX_CHECK(src1->m == src2->m && src1->n == src2->n, false);

    for (j = 0; j < src1->n; j++) {
        for (i = 0; i < src1->m; i++) {
            double delta = src1->mat[j][i] - src2->mat[j][i];
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

  MATRIX_CHECK(dest->m == dest->n && dest->m == src->m && dest->n == src->n, MATRIX_RESULT_INVALID);

  /* Initialise output to the identity matrix */
  for (j = 0; j < dest->n; j++) {
    for (i = 0; i < dest->m; i++) {
      if (i == j)
        dest->mat[j][i] = 1.0;
      else
        dest->mat[j][i] = 0.0;
    }
  }

  /* Gauss-Jordan elimination */
  for (k = 0; k < dest->m; k++) {
    s = src->mat[k][k];
    if (s == 0)
      return MATRIX_RESULT_FAILED; /* Inversion failed */

    for (j = 0; j < dest->n; j++) {
      src->mat[j][k] /= s;
      dest->mat[j][k] /= s;
    }

    for (i = 0; i < dest->m; i++) {
      s = src->mat[k][i];
      for (j = 0; j < dest->n && i != k; j++) {
        src->mat[j][i]  -= s * src->mat[j][k];
        dest->mat[j][i] -= s * dest->mat[j][k];
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
  MATRIX_CHECK(Y->n == Y->m && dest->m == dest->n &&
       dest->n == X->m && X->n == Y->m, MATRIX_RESULT_INVALID);

  tmp = matrix2d_alloc(X->m, X->n);

  ret = matrix2d_multiply(tmp, X, Y);
  if (ret != MATRIX_RESULT_OK)
    goto out;

  /* Multiply tmp by the transpose of X */
  for (j = 0; j < dest->n; j++) {
    for (i = 0; i < dest->m; i++) {
      double s = 0.0;
      for (k = 0; k < tmp->n; k++)
        s += tmp->mat[k][i] * X->mat[k][j];
      dest->mat[j][i] = s;
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
    MATRIX_CHECK(mat->n == mat->m, MATRIX_RESULT_INVALID);

    for (i = 0; i < mat->m; i++) {
        mat->mat[i][i] += epsilon;

        for (j = i+1; j < mat->n; j++) {
            double tmp;
            m1 = &mat->mat[j][i];
            m2 = &mat->mat[i][j];
            tmp = (0.5 * *m1 + 0.5 * *m2);

            *m1 = *m2 = tmp;
        }
    }

    return MATRIX_RESULT_OK;
}
