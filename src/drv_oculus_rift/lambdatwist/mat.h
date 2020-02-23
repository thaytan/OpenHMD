/*
 * Matrix inversion and multiplication
 * Copyright 2019 Philipp Zabel
 * SPDX-License-Identifier: (GPL-2.0-or-later or BSL-1.0)
 */
#ifndef MAT_H
#define MAT_H

/* invert a row-major 3x3 matrix */
static inline void mat3_invert(double *a)
{
        const double a00 = a[0];
        const double a01 = a[1];
        const double a02 = a[2];
        const double a10 = a[3];
        const double a11 = a[4];
        const double a12 = a[5];
        const double a20 = a[6];
        const double a21 = a[7];
        const double a22 = a[8];

        const double M00 = a11 * a22 - a12 * a21;
        const double M01 = a02 * a21 - a01 * a22;
        const double M02 = a01 * a12 - a02 * a11;
        const double M10 = a12 * a20 - a10 * a22;
        const double M11 = a00 * a22 - a02 * a20;
        const double M12 = a02 * a10 - a00 * a12;
        const double M20 = a10 * a21 - a11 * a20;
        const double M21 = a01 * a20 - a00 * a21;
        const double M22 = a00 * a11 - a01 * a10;

        const double idet = 1.0 / (a00 * M00 + a01 * M10 + a02 * M20);

        a[0] = M00 * idet;
        a[1] = M01 * idet;
        a[2] = M02 * idet;
        a[3] = M10 * idet;
        a[4] = M11 * idet;
        a[5] = M12 * idet;
        a[6] = M20 * idet;
        a[7] = M21 * idet;
        a[8] = M22 * idet;
}

/* multiply two row-major 3x3 matrices */
static inline void mat3_mul(double *ret, const double *a, const double *b)
{
	ret[0] = a[0] * b[0] + a[1] * b[3] + a[2] * b[6];
	ret[1] = a[0] * b[1] + a[1] * b[4] + a[2] * b[7];
	ret[2] = a[0] * b[2] + a[1] * b[5] + a[2] * b[8];
	ret[3] = a[3] * b[0] + a[4] * b[3] + a[5] * b[6];
	ret[4] = a[3] * b[1] + a[4] * b[4] + a[5] * b[7];
	ret[5] = a[3] * b[2] + a[4] * b[5] + a[5] * b[8];
	ret[6] = a[6] * b[0] + a[7] * b[3] + a[8] * b[6];
	ret[7] = a[6] * b[1] + a[7] * b[4] + a[8] * b[7];
	ret[8] = a[6] * b[2] + a[7] * b[5] + a[8] * b[8];
}

/* multiply a 3x3 row-major matrix and a column 3-vector */
static inline void mat3vec3_mul(double *ret, const double *m, const double *v)
{
	ret[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
	ret[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
	ret[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
}

#endif /* MAT_H */
