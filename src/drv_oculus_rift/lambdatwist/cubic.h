/*
 * Lambda Twist P3P
 * Copyright 2020 Jan Schmidt <jan@centricular.com>
 * SPDX-License-Identifier: (GPL-2.0-or-later or BSL-1.0)
 */
#ifndef __CUBIC_H__
#define __CUBIC_H__

#include <math.h>

/* Tuning variable for maximum loops and stopping
 * criteria */
#ifndef CUBIC_MAX_ITERS
#define CUBIC_MAX_ITERS 100
#endif
#ifndef CUBIC_ROOT_EPSILON
#define CUBIC_ROOT_EPSILON 1e-10
#endif

/*
 * Given a cubic equation with coefficients b,c,d:
 *   f(x) = x^3 + bx^2 + cx + d
 *
 * return one root of the equation, preferably the 
 * one with the highest derivative. There is always
 * at least 1 real root for such a cubic.
 *
 * We do that by choosing a starting point and using
 * the Newton-Raphson method to find the root.
 *
 * The derivative of the cubic equation is:
 *   f'(x) = 3x^2 + 2bx + c
 *
 * Using the quadratic formula on the derivative, we can find
 * the critical points:
 * x = (-2*b ± sqrt(4*b^2 - 4*3*c) / 2*3
 * x = (-b ± sqrt(b^2 - 3*c) / 3.0
 *
 * From the sqrt part, if b^2 < 3*c, there are no real critical
 * points. If b^2 == 3*c, there is 1 critical point @ -b/3.0.
 * In both these cases, the cubic is strictly monotonic, with only 1 root.
 *
 * Othwise, there are up to 3 roots, and we want the one
 * with the larger derivative, so we choose a starting point
 * as explained below.
 * 
 * It's also worth remembering that given
 * roots x1, x2, x3 then x1 + x2 + x3 = -b,
 * which is why it is good to start at x = -b/3.0 for the
 * single-root case.
 *
 * cubic b=0.000144 c=-2.460081 d=1.747825 -> r0 = 0.961102 f(r0) = 0.271356 (FAIL)
 *
 * t1 = 2.0736e-08
 * t2 = -7.380243
 */
static inline double reduced_cubic_real_root(double b, double c, double d) {
		double t1 = b*b;
		double t2 = 3.0*c;
		double t3 = -b / 3.0;
		double r0;

    if (t1 <= t2) {
				r0 = t3;
		}
		else {
        /* We need to decide which root to iterate toward,
         * by starting to the left of the left critical point,
         * the right of the right critical point, or in between
         * the 2.
         *
         * We know the cubic is increasing, because coefficient a = 1,
         * so the left critical point (cl) is a maxima, and the right (cr) is a minima.
         *
         * If f(cl) > 0, go left from there, else go right from cr.
         */
        double t4 = sqrt(t1 - t2) / 3.0;
        double cl = t3 - t4;
		    double fx = cl * (cl * (cl + b) + c) + d;
        if (fx > 0)
		      r0 = cl - t4/4.0;
        else {
          double cr = t3 + t4;
		      r0 = cr + t4/4.0;
        }
		}

		/* Newton-Raphson - r0 = r0 - f(r0)/f'(r0) */
    int i = CUBIC_MAX_ITERS;
		do {
		  double fx = r0 * (r0 * (r0 + b) + c) + d;
			double dx = fx / (r0*(3*r0 + 2*b) + c);

      /* If we converged, exit or if the delta gets really small, break out too,
       * because we're not moving any closer */
			if (fabs (fx) < CUBIC_ROOT_EPSILON || fabs(dx) < CUBIC_ROOT_EPSILON) {
				return r0;
      }

			r0 -= dx;
    } while (i-- > 0);

		return r0;
}

#endif /* __CUBIC_H__*/
