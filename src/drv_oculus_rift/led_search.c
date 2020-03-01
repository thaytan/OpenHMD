#define _GNU_SOURCE // For qsort_r
#include <stdlib.h>
#include <stdio.h>

#include "led_search.h"

static int compare_leds (const void *elem1, const void *elem2, void *arg)
{
  led_search_candidate_t *c = arg;
  const rift_led *l1 = * (const rift_led **) elem1;
  const rift_led *l2 = * (const rift_led **) elem2; 
  double dist1, dist2;
  vec3f tmp;

  ovec3f_subtract(&l1->pos, &c->led->pos, &tmp);
  dist1 = ovec3f_get_length(&tmp);

  ovec3f_subtract(&l2->pos, &c->led->pos, &tmp);
  dist2 = ovec3f_get_length(&tmp);
 
  if (dist1 > dist2) return  1;
  if (dist1 < dist2) return -1;

  return 0;
}

led_search_candidate_t *led_search_candidate_new (rift_led *led, rift_leds *led_model)
{
  led_search_candidate_t *c = calloc(1, sizeof (led_search_candidate_t));
  int i;

  c->led = led;
  c->neighbours = calloc (led_model->num_points-1, sizeof (rift_led *));
  c->num_neighbours = 0; 

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

#if 0
  printf ("Have %u neighbours for LED (%f,%f,%f) dir (%f,%f,%f):\n", c->num_neighbours,
      c->led->pos.x, c->led->pos.y, c->led->pos.z,
      c->led->dir.x, c->led->dir.y, c->led->dir.z);
  for (i = 0; i < c->num_neighbours; i++) {
      rift_led *cur = c->neighbours[i];
      vec3f delta;

      ovec3f_subtract (&cur->pos, &c->led->pos, &delta);
      printf ("  LED %u @ (%f, %f, %f) dir (%f, %f, %f) dist %f\n", i,
              cur->pos.x, cur->pos.y, cur->pos.z,
              cur->dir.x, cur->dir.y, cur->dir.z,
              ovec3f_get_length (&delta));
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
    free (model);
}
