#define _GNU_SOURCE // For qsort_r
#include <stdlib.h>
#include <stdio.h>

#include "led_search.h"

/* Set to 0 to use 3D euclidean distance to sort neighbours,
 * 1 to use orthographic projected 2D distance if the anchor LED is forward-facing */
#define PROJECTED_DISTANCE 1
#define DUMP_FULL_DEBUG 0

#if DUMP_FULL_DEBUG
#define DEBUG(s,...) printf(s,__VA_ARGS__)
#else
#define DEBUG(s,...)
#endif

static double calc_led_dist(led_search_candidate_t *c, const rift_led *l, vec3f *out_led_pos)
{
  vec3f led_pos;

#if PROJECTED_DISTANCE
  vec3f tmp;

  ovec3f_add(&l->pos, &c->pose.pos, &tmp);
  oquatf_get_rotated(&c->pose.orient, &tmp, &led_pos);

  if (out_led_pos)
    *out_led_pos = led_pos;

  return POW2(led_pos.x) + POW2(led_pos.y);
#else
  ovec3f_subtract(&l->pos, &c->led->pos, &led_pos);

  if (out_led_pos)
    *out_led_pos = led_pos;

  return ovec3f_get_length(&led_pos);
#endif
}

static int compare_leds (const void *elem1, const void *elem2, void *arg)
{
  led_search_candidate_t *c = arg;
  const rift_led *l1 = * (const rift_led **) elem1;
  const rift_led *l2 = * (const rift_led **) elem2; 
  double dist1, dist2;

  dist1 = calc_led_dist(c, l1, NULL);
  dist2 = calc_led_dist(c, l2, NULL);

  if (dist1 > dist2) return  1;
  if (dist1 < dist2) return -1;

  return 0;
}

led_search_candidate_t *led_search_candidate_new (rift_led *led, rift_leds *led_model)
{
  led_search_candidate_t *c = calloc(1, sizeof (led_search_candidate_t));
  const vec3f fwd = {{ 0.0, 0.0, 1.0 }};
  int i;

  c->led = led;
  c->neighbours = calloc (led_model->num_points-1, sizeof (rift_led *));
  c->num_neighbours = 0; 

  /* Calculate the pose that places this LED forward-facing at 0,0,0
   * and then calculate the distance for all visible LEDs when (orthographic)
   * projected in that pose */
  c->pose.pos = led->pos;
  ovec3f_inverse(&c->pose.pos);
  oquatf_from_vectors(&c->pose.orient, &led->dir, &fwd);

  for (i = 0; i < led_model->num_points; i++) {
      rift_led *cur = led_model->points + i;

      if (cur == led)
        continue; // Don't put the current LED in its own neighbour list
      
      if (ovec3f_get_dot(&led->dir, &cur->dir) <= 0)
        continue; // Normals are more than 90 degrees apart - these are mutually exclusive LEDs

      c->neighbours[c->num_neighbours++] = cur;
  }
  // FIXME: qsort_r is a GNU extension.
  if (c->num_neighbours > 1)
    qsort_r (c->neighbours, c->num_neighbours, sizeof(rift_led *), compare_leds, c);

#if DUMP_FULL_DEBUG
  DEBUG ("Have %u neighbours for LED %u (%f,%f,%f) dir (%f,%f,%f):\n", c->num_neighbours, c->led->led_id,
      c->led->pos.x, c->led->pos.y, c->led->pos.z,
      c->led->dir.x, c->led->dir.y, c->led->dir.z);
  for (i = 0; i < c->num_neighbours; i++) {
      rift_led *cur = c->neighbours[i];
      vec3f led_pos;
      double sq_distance = calc_led_dist(c, cur, &led_pos);

      DEBUG ("  LED id %2u @ %10.7f %10.7f %10.7f dir %10.7f %10.7f %10.7f -> %10.7f %10.7f dist %10.7f\n", cur->led_id,
              cur->pos.x, cur->pos.y, cur->pos.z,
              cur->dir.x, cur->dir.y, cur->dir.z,
              led_pos.x, led_pos.y, sq_distance);
  }
#endif

  return c;
}

void led_search_candidate_free (led_search_candidate_t *candidate)
{
  free (candidate->neighbours);
  free (candidate);
}

led_search_model_t *led_search_model_new (rift_leds *led_model)
{
    led_search_model_t *m = calloc(1, sizeof (led_search_model_t));
    int i;

    m->leds = led_model;

    m->points = calloc(led_model->num_points, sizeof (led_search_candidate_t *));
    m->num_points = led_model->num_points;

    for (i = 0; i < led_model->num_points; i++) {
        m->points[i] = led_search_candidate_new (led_model->points + i, led_model);
    }

    return m;
}

void led_search_model_free (led_search_model_t *model)
{
    int i;

    for (i = 0; i < model->num_points; i++) {
        led_search_candidate_free (model->points[i]);
    }
    free (model->points);
    free (model);
}
