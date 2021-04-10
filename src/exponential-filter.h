/* Copyright 2021, Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 *
 * Implementation of a One Euro exponential filter
 */

#ifndef EXPONENTIAL_FILTER_H__
#define EXPONENTIAL_FILTER_H__

#include <stdint.h>
#include <stdbool.h>
#include "omath.h"

typedef struct exp_filter3d exp_filter3d;
typedef struct exp_filter_pose exp_filter_pose;

struct exp_filter3d {
		/* Minimum frequency cutoff for filter and derivative respectively - default = 25.0 and 10.0 */
		double fc_min, fc_min_d;
	  /* Beta value for "responsiveness" of filter - default = 0.01 */
		double beta;

		/* true if we have already processed a history sample */
		bool have_prev_y;
	
		/* Timestamp of previous sample (nanoseconds) and the sample */
		uint64_t prev_ts;
		vec3f prev_y;
		vec3f prev_dy;
};

struct exp_filter_pose {
		exp_filter3d pos_filter;
		exp_filter3d orient_filter;
};

void exp_filter3d_init(exp_filter3d *f);
void exp_filter3d_set_params(exp_filter3d *f, double fc_min, double beta, double fc_min_d);
void exp_filter3d_run(exp_filter3d *f, uint64_t ts, const vec3f *in_y, vec3f *out_y);

void exp_filter_pose_init(exp_filter_pose *f);
void exp_filter_pose_set_params(exp_filter_pose *f, double fc_min, double beta, double fc_min_d);
void exp_filter_pose_run(exp_filter_pose *f, uint64_t ts, const posef *in_pose, posef *out_pose);

#endif
