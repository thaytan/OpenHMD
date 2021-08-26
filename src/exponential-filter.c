/* Copyright 2021, Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 *
 * Implementation of a One Euro exponential filter
 */

#include <math.h>

#include "exponential-filter.h"

#define DEFAULT_FCMIN 30.0
#define DEFAULT_FCMIN_D 25.0
#define DEFAULT_BETA 0.6

static double calc_smoothing_alpha(double Fc, double dt) {
	/* Calculate alpha = (1 / (1 + tau/dt)) where tau = 1.0 / (2 * pi * Fc),
	 * this is a straight rearrangement with fewer divisions */
	double r = 2.0 * M_PI * Fc * dt;
	return r / (r + 1.0);
}

static double exp_smooth(double alpha, const float y, const float prev_y)
{
	return alpha * y + (1.0 - alpha) * prev_y;
}

void exp_filter3d_init(exp_filter3d *f)
{
	f->fc_min = DEFAULT_FCMIN;
	f->beta = DEFAULT_BETA;
	f->fc_min_d = DEFAULT_FCMIN_D;

	f->have_prev_y = false;
}

void exp_filter3d_set_params(exp_filter3d *f, double fc_min, double beta, double fc_min_d)
{
	f->fc_min = fc_min;
	f->beta = beta;
	f->fc_min_d = fc_min_d;
}

void exp_filter3d_run(exp_filter3d *f, uint64_t ts, const vec3f *in_y, vec3f *out_y)
{
	vec3f dy;
	double dt, alpha_d;
	int i;

	if (!f->have_prev_y) {
		/* First sample - no filtering yet */
		ovec3f_set(&f->prev_dy, 0.0, 0.0, 0.0);
		f->prev_ts = ts;
		f->prev_y = *in_y;
		f->have_prev_y = true;

		*out_y = *in_y;
		return;
	}

	dt = (double)(ts - f->prev_ts) / 1000000000.0;
	f->prev_ts = ts;

	ovec3f_subtract(in_y, &f->prev_y, &dy);
	alpha_d = calc_smoothing_alpha(f->fc_min_d, dt);

	/* Smooth the dy values and use them to calculate the frequency cutoff for the main filter */
	for (i = 0; i < 3; i++) {
		double abs_dy, alpha, fc_cutoff;

		f->prev_dy.arr[i] = exp_smooth(alpha_d, dy.arr[i], f->prev_dy.arr[i]);
		abs_dy = fabs(f->prev_dy.arr[i]);

		fc_cutoff = f->fc_min + f->beta * abs_dy;
		alpha = calc_smoothing_alpha(fc_cutoff, dt);

		out_y->arr[i] = f->prev_y.arr[i] = exp_smooth(alpha, in_y->arr[i], f->prev_y.arr[i]);
	}
}

void exp_filter_pose_init(exp_filter_pose *f)
{
	exp_filter3d_init(&f->pos_filter);
	exp_filter3d_init(&f->orient_filter);
}

void exp_filter_pose_set_params(exp_filter_pose *f, double fc_min, double beta, double fc_min_d)
{
	exp_filter3d_set_params(&f->pos_filter, fc_min, beta, fc_min_d);
	exp_filter3d_set_params(&f->orient_filter, fc_min, beta, fc_min_d);
}

static void
adjust_exp_map_proximity(vec3f *prev_map, vec3f *cur_map)
{
	vec3f delta;

	ovec3f_subtract(prev_map, cur_map, &delta);
	float delta_mag = ovec3f_get_length(&delta);

	if (delta_mag < -M_PI || delta_mag > M_PI) {
		float prev_map_mag = ovec3f_get_length(prev_map);
		int n_pi_region = ceil(delta_mag / (2*M_PI));

		ovec3f_multiply_scalar(prev_map, 1 - (n_pi_region * 2 * M_PI) / prev_map_mag, prev_map);
	}
}

void exp_filter_pose_run(exp_filter_pose *f, uint64_t ts, const posef *in_pose, posef *out_pose)
{
	vec3f in_exp_map, out_exp_map;

	/* Filtering on poses is simple for the position. For the orientation, filter on the exponential map and transform back on the way out */
	exp_filter3d_run(&f->pos_filter, ts, &in_pose->pos, &out_pose->pos);

	oquatf_to_rotation(&in_pose->orient, &in_exp_map);
	/* Avoid singularity wrap-around glitches by adjusting
	 * the old filter observation to keep it within +/- pi of the new */
	if (f->orient_filter.have_prev_y) {
		adjust_exp_map_proximity(&f->orient_filter.prev_y, &in_exp_map);
	}
	exp_filter3d_run(&f->orient_filter, ts, &in_exp_map, &out_exp_map);
	oquatf_from_rotation(&out_pose->orient, &out_exp_map);
}
