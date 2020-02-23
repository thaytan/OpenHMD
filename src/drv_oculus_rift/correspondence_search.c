#define _GNU_SOURCE // For qsort_r
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include <math.h>

#include "lambdatwist/lambdatwist_p3p.h"
#include "correspondence_search.h"
#include "rift-sensor-opencv.h"
#include "rift-sensor-blobwatch.h"

#define MAX_SEARCH_DEPTH 7

#define abs(x) ((x) >= 0 ? (x) : -(x))

correspondence_search_t *
correspondence_search_new(dmat3 *camera_matrix, double *dist_coeffs)
{
    correspondence_search_t *cs = calloc(1, sizeof (correspondence_search_t));
    cs->camera_matrix = camera_matrix;
    cs->dist_coeffs = dist_coeffs;
    return cs;
}

#define DUMP_SCENE 0

#define MIN(a,b) ((a) < (b) ? a : b)
#define MAX(a,b) ((a) > (b) ? a : b)

void
correspondence_search_set_blobs (correspondence_search_t *cs, struct blob *blobs, int num_blobs)
{
    int i;
    vec3f undistorted_points[MAX_BLOBS_PER_FRAME];

    assert (num_blobs <= MAX_BLOBS_PER_FRAME);

    if (cs->points != NULL)
        free (cs->points);

    cs->points = calloc(num_blobs, sizeof (cs_image_point_t));
    cs->num_points = num_blobs;

    /* Undistort points so we can project / match properly */
    undistort_points (blobs, num_blobs, undistorted_points, cs->camera_matrix->m, cs->dist_coeffs);

    //printf ("Building blobs search array\n");
    for (i = 0; i < num_blobs; i++) {
        cs_image_point_t *p = cs->points + i;
        struct blob *b = blobs + i;

        p->point_homog[0] = undistorted_points[i].x;
        p->point_homog[1] = undistorted_points[i].y;
        p->point_homog[2] = 1;

        p->size[0] = b->width / cs->camera_matrix->m[0];
        p->size[1] = b->height / cs->camera_matrix->m[4];
        p->max_size = MAX (p->size[0], p->size[1]);

        p->blob = b;

#if 0
        printf ("Blob %u = (%u,%u) -> (%f, %f) %f x %f (LED id %d)\n", i,
                b->x, b->y,
                p->point_homog[0], p->point_homog[1],
                p->size[0], p->size[1], b->led_id);
#endif
    }
}

bool
correspondence_search_set_model (correspondence_search_t *cs, int id, led_search_model_t *model)
{
    if (cs->num_models == CS_MAX_MODELS)
        return false;

    cs->models[cs->num_models].id = id;
    cs->models[cs->num_models].model = model;
    cs->num_models++;

    return true;
}

void
correspondence_search_free (correspondence_search_t *cs)
{
    int m;
    for (m = 0; m < cs->num_models; m++)
        led_search_model_free (cs->models[m].model);
    if (cs->points)
      free (cs->points);
    free (cs);
}

static int compare_blobs_distance (const void *elem1, const void *elem2, void *arg)
{
  const cs_image_point_t *b1 = * (const cs_image_point_t **) elem1;
  const cs_image_point_t *b2 = * (const cs_image_point_t **) elem2;
  cs_image_point_t *anchor = arg;
  double dist1, dist2;

  dist1 = (b1->blob->y - anchor->blob->y) * (b1->blob->y - anchor->blob->y) +
          (b1->blob->x - anchor->blob->x) * (b1->blob->x - anchor->blob->x);
  dist2 = (b2->blob->y - anchor->blob->y) * (b2->blob->y - anchor->blob->y) +
          (b2->blob->x - anchor->blob->x) * (b2->blob->x - anchor->blob->x);

  if (dist1 > dist2) return  1;
  if (dist1 < dist2) return -1;

  return 0;
}

static cs_image_point_t *
blobs_find_blob_at(correspondence_search_t *cs, double x, double y, double match_factor, double *sq_error)
{
    int i;
    for (i = 0; i < cs->num_points; i++) {
      cs_image_point_t *p = cs->points + i;
      double dx = abs(x - p->point_homog[0]);
      double dy = abs(y - p->point_homog[1]);

      if (match_factor*dx <= p->size[0] && match_factor*dy <= p->size[1]) {
        *sq_error = dx*dx + dy*dy;
        return p;
      }
    }
    return NULL;
}

static void
dump_pose (correspondence_search_t *cs, led_search_model_t *model, quatf *orient, vec3f *trans, cs_model_info_t *mi)
{
    int i;
	  rift_leds *leds = model->leds;

    printf ("pose_points[%i] = [\n", mi->id);
    for (i = 0; i < leds->num_points; i++) {
      vec3f pos, dir;

      /* Project HMD LED into the image (no distortion) */
      oquatf_get_rotated(orient, &leds->points[i].pos, &pos);
      ovec3f_add (&pos, trans, &pos);
      if (pos.z < 0)
        continue; // Can't be behind the camera

      ovec3f_multiply_scalar (&pos, 1.0/pos.z, &pos);

      double x = pos.x;
      double y = pos.y;
      ovec3f_normalize_me(&pos);

      oquatf_get_rotated(orient, &leds->points[i].dir, &dir);
      ovec3f_normalize_me(&dir);

      double facing_dot;
      facing_dot = ovec3f_get_dot (&pos, &dir);

      if (facing_dot < 0) {
          printf ("  (%f,%f),\n", x, y);
      }
    }
    printf ("]\n");
}

/* FIXME: Given 1 visible LED, we could just iterate the visible neighbours
 * from the model that we already sorted and checked and save time */
bool
correspondence_search_project_pose (correspondence_search_t *cs, led_search_model_t *model,
        quatf *orient, vec3f *trans, cs_model_info_t *mi)
{
  /* Given a pose, project each 3D LED point back to 2D and assign correspondences to blobs */
  /* If enough match, print out the pose */
	rift_leds *leds = model->leds;
  int i;
  double projection_sqerror;
  int matched_visible_blobs, prev_matched_blobs = 0;
  int visible_leds;
  double *matched_blobs[MAX_BLOBS_PER_FRAME];
  rift_led *matched_leds[MAX_BLOBS_PER_FRAME];

  /* Incrememnt stats */
  cs->num_pose_checks++;

again:
  matched_visible_blobs = 0;
  visible_leds = 0;
  projection_sqerror = 0.0;

  /* Check how many LEDs have matching blobs in this pose,
   * if there's enough we have a good match */

  for (i = 0; i < leds->num_points; i++) {
    rift_led *led = &leds->points[i];
    vec3f pos, dir;

    /* Project HMD LED into the image (no distortion) */
    oquatf_get_rotated(orient, &led->pos, &pos);
    ovec3f_add (&pos, trans, &pos);

    if (pos.z < 0)
      continue; // Can't be behind the camera

    ovec3f_multiply_scalar (&pos, 1.0/pos.z, &pos);
    oquatf_get_rotated(orient, &led->dir, &dir);
    ovec3f_normalize_me(&dir);

    /* Perspective projection */
    double x = pos.x;
    double y = pos.y;

    ovec3f_normalize_me(&pos);

    double facing_dot;
    facing_dot = ovec3f_get_dot (&pos, &dir);

    /* check it's camera facing */
    if (facing_dot < 0) {
      double sq_error;

      cs_image_point_t *p = blobs_find_blob_at(cs, x, y, 1.25, &sq_error);
      visible_leds++;

      if (p != NULL) {
        //printf ("matched (%f, %f) dot %f,\n", x, y, facing_dot);
        matched_blobs[matched_visible_blobs] = p->point_homog;
        matched_leds[matched_visible_blobs] = led;

        matched_visible_blobs++;
        projection_sqerror += sq_error;
      }
#if 0
      else
        printf ("no match (%f, %f) dot %f,\n", x, y, facing_dot);
#endif
    }
  }

  if (visible_leds > 4 && matched_visible_blobs > 4) {
    if (visible_leds < 2 * matched_visible_blobs) {

      /* Refine the pose to best match all points we found */
      if (matched_visible_blobs > prev_matched_blobs) {
          refine_pose(matched_blobs, matched_leds, matched_visible_blobs, orient, trans, &projection_sqerror);
          prev_matched_blobs = matched_visible_blobs;
          goto again;
      }

      if (mi) {
        bool is_new_best = false;

        if (mi->best_matched < 5 && mi->best_matched < matched_visible_blobs)
            is_new_best = true; /* If < 5 blobs matched, prefer more matches */
        else if (mi->best_sqerror > projection_sqerror)
            is_new_best = true; /* else, prefer closer reprojection */

        if (is_new_best) {

           printf ("model %d new best pose candidate orient %f %f %f %f pos %f %f %f has %u visible LEDs, error %f\n",
                   mi->id, orient->x, orient->y, orient->z, orient->w,
                         trans->x, trans->y, trans->z, visible_leds, projection_sqerror);
           printf("model %d matched %u blobs of %u\n", mi->id, matched_visible_blobs, visible_leds);

            mi->best_matched = matched_visible_blobs;
            mi->best_visible = visible_leds;
            mi->best_orient = *orient;
            mi->best_trans = *trans;
            mi->best_sqerror = projection_sqerror;
            dump_pose (cs, model, orient, trans, mi);
            return true;
        }
      } else {
        printf ("pose candidate orient %f %f %f %f pos %f %f %f has %u visible LEDs, error %f\n",
                orient->x, orient->y, orient->z, orient->w,
                trans->x, trans->y, trans->z, visible_leds, projection_sqerror);
        printf("matched %u blobs of %u\n", matched_visible_blobs, visible_leds);
      }
  
      printf ("Found good pose match - %u LEDs matched %u visible ones\n",
          matched_visible_blobs, visible_leds);
      return true;
    }
    else {
      printf ("Failed pose match - only %u LEDs matched %u visible ones\n",
          matched_visible_blobs, visible_leds);
    }
  }
  return false;
}

static void
oquatf_from_rotation_matrix (quatf* me, double R[9])
{
  float trace = R[0] + R[3 + 1] + R[6+2];
  if (trace > 0) {
    float s = 0.5f / sqrtf(trace+ 1.0f);
    me->w = -0.25f / s;
    me->x = -( R[6+1] - R[3+2] ) * s;
    me->y = -( R[2] - R[6] ) * s;
    me->z = -( R[3] - R[1] ) * s;
  } else {
    if ( R[0] > R[4] && R[0] > R[8] ) {
      float s = 2.0f * sqrtf( 1.0f + R[0] - R[3+1] - R[6+2]);
      me->w = -(R[6+1] - R[3+2] ) / s;
      me->x = -0.25f * s;
      me->y = -(R[1] + R[3] ) / s;
      me->z = -(R[2] + R[6] ) / s;
    } else if (R[3+1] > R[6+2]) {
      float s = 2.0f * sqrtf( 1.0f + R[3+1] - R[0] - R[6+2]);
      me->w = -(R[2] - R[6] ) / s;
      me->x = -(R[1] + R[3] ) / s;
      me->y = -0.25f * s;
      me->z = -(R[3+2] + R[6+1] ) / s;
    } else {
      float s = 2.0f * sqrtf( 1.0f + R[6+2] - R[0] - R[3+1] );
      me->w = -(R[3]   - R[1] ) / s;
      me->x = -(R[2]   + R[6] ) / s;
      me->y = -(R[3+2] + R[6+1] ) / s;
      me->z = -0.25f * s;
    }
  }
}

static void
check_led_against_model_subset (correspondence_search_t *cs, cs_model_info_t *mi,
     cs_image_point_t **blobs, rift_led *model_leds[4])
{
  led_search_model_t *model = mi->model;
  double x[3][3];
  vec3f *xcheck;
  int i;
  double *y1, *y2, *y3;
  double Rs[4][9], Ts[4][3];
  vec3f checkblob, blob0;

  y1 = blobs[0]->point_homog;
  y2 = blobs[1]->point_homog;
  y3 = blobs[2]->point_homog;

  blob0.x = blobs[0]->point_homog[0];
  blob0.y = blobs[0]->point_homog[1];
  blob0.z = blobs[0]->point_homog[2];

  /* 4th point we'll check against */
  checkblob.x = blobs[3]->point_homog[0];
  checkblob.y = blobs[3]->point_homog[1];
  checkblob.z = blobs[3]->point_homog[2];

  cs->num_trials++;

  for (i = 0; i < 3; i++) {
    x[i][0] = model_leds[i]->pos.x;
    x[i][1] = model_leds[i]->pos.y;
    x[i][2] = model_leds[i]->pos.z;
  }
  xcheck = &model_leds[3]->pos;

  /* FIXME: It would be better if this spat out quaternions,
   * then we wouldn't need to convert below */
  /* NOTE: Modifies y1/y2/y3 */
  int valid = lambdatwist_p3p (y1, y2, y3, x[0], x[1], x[2],
          Rs, Ts);

  if (valid) {
    for (i = 0; i < valid; i++) {
      quatf orient;
      vec3f trans;
      vec3f checkpos, checkdir;
      float l;

      /* Construct quat and trans for this pose */
      oquatf_from_rotation_matrix (&orient, Rs[i]);

      trans.x = Ts[i][0];
      trans.y = Ts[i][1];
      trans.z = Ts[i][2];

      if (trans.z < 0.05) {
          /* The object is unlikely to be < 5cm (50000µm) from the camera */
          continue;
      }

      /* The quaternion produced should already be normalised, but sometimes it's not! */
#if 1
      oquatf_normalize_me (&orient);
#else
      l = oquatf_get_length (&orient);
      assert (l > 0.9999 && l < 1.0001);
#endif

      /* This pose must yield a projection of the anchor
       * point, or something is really wrong */
      oquatf_get_rotated (&orient, &model_leds[0]->pos, &checkpos);
      ovec3f_add (&checkpos, &trans, &checkpos);

      /* And should be camera facing in this pose */
      oquatf_get_rotated (&orient, &model_leds[0]->dir, &checkdir);
      ovec3f_normalize_me(&checkpos);

      double facing_dot = ovec3f_get_dot (&checkpos, &checkdir);
      if (facing_dot > 0) {
        // Anchor LED not facing the camera -> invalid pose
        continue;
      }

      ovec3f_multiply_scalar (&checkpos, 1.0/checkpos.z, &checkpos);
      ovec3f_subtract (&checkpos, &blob0, &checkpos);
      l = ovec3f_get_length (&checkpos);
      if ((l >= 0.0005)) {
	      printf ("pose candidate orient %f %f %f %f pos %f %f %f\n",
            orient.x, orient.y, orient.z, orient.w,
            trans.x, trans.y, trans.z);
        printf ("Anchor LED %f %f %f projected to %f %f %f\n",
            blob0.x, blob0.y, blob0.z,
            checkpos.x, checkpos.y, checkpos.z);
        continue; /* FIXME: Figure out why this happened */
      }
      assert (l < 0.0005);

      /* check against the 4th point to check the proposed P3P solution */
      oquatf_get_rotated (&orient, xcheck, &checkpos);
      ovec3f_add (&checkpos, &trans, &checkpos);
      ovec3f_multiply_scalar (&checkpos, 1.0/checkpos.z, &checkpos);

      /* Subtract projected point from reference point to check error */
      vec3f tmp;
      ovec3f_subtract (&checkpos, &checkblob, &tmp);
      float distance = ovec3f_get_length (&tmp);

      /* Check that the 4th point projected to within its blob */
      if (distance <= blobs[3]->max_size) {
#if 0
        /* Convert back to pixels for the debug output */
        ovec3f_multiply_scalar (&checkpos, cs->camera_matrix->m[0], &checkpos);

  	    printf ("model %u pose candidate orient %f %f %f %f pos %f %f %f\n",
            mi->id, orient.x, orient.y, orient.z, orient.w,
            trans.x, trans.y, trans.z);
        printf ("4th point @ %f %f offset %f pixels\n",
            checkpos.x, checkpos.y, distance * cs->camera_matrix->m[0]);
#endif
        if (correspondence_search_project_pose (cs, model, &orient, &trans, mi)) {
          printf ("  P4P points %f,%f,%f -> %f %f\n"
                  "         %f,%f,%f -> %f %f\n"
                  "         %f,%f,%f -> %f %f\n"
                  "         %f,%f,%f -> %f %f\n",
                x[0][0], x[0][1], x[0][2],
                y1[0], y1[1],
                x[1][0], x[1][1], x[1][2],
                y2[0], y2[1],
                x[2][0], x[2][1], x[2][2],
                y3[0], y3[1],
                xcheck->x, xcheck->y, xcheck->z,
                checkblob.x,
                checkblob.y);
        }
      }
    }
  }
}

static void
check_leds_against_model (correspondence_search_t *cs, cs_model_info_t *mi, cs_image_point_t **blobs)
{
  led_search_model_t *model = mi->model;
  int l;
  rift_led *model_leds[4];

  for (l = 0; l < model->num_points; l++) {
    led_search_candidate_t *c = model->points[l];

    if (c->num_neighbours < 3)
      continue;

    model_leds[0] = c->led;
    model_leds[1] = c->neighbours[0];
    model_leds[2] = c->neighbours[1];
    model_leds[3] = c->neighbours[2];

    check_led_against_model_subset (cs, mi, blobs, model_leds);

    /* If we have at least 4 neighbour LEDs, try some more combos, in case some LEDs
     * are occluded */
    if (c->num_neighbours < 4)
      continue;

    model_leds[1] = c->neighbours[0];
    model_leds[2] = c->neighbours[1];
    model_leds[3] = c->neighbours[3];
    check_led_against_model_subset (cs, mi, blobs, model_leds);

    model_leds[1] = c->neighbours[0];
    model_leds[2] = c->neighbours[2];
    model_leds[3] = c->neighbours[3];
    check_led_against_model_subset (cs, mi, blobs, model_leds);

    model_leds[1] = c->neighbours[1];
    model_leds[2] = c->neighbours[2];
    model_leds[3] = c->neighbours[3];
    check_led_against_model_subset (cs, mi, blobs, model_leds);

#if 0
    /* Testing more LED neighbours slows
     * things down a lot, without any significant improvement
     * in results */
    if (c->num_neighbours < 5)
      continue;

    model_leds[1] = c->neighbours[0];
    model_leds[2] = c->neighbours[1];
    model_leds[3] = c->neighbours[4];

    check_led_against_model_subset (cs, mi, blobs, model_leds);
    model_leds[1] = c->neighbours[0];
    model_leds[2] = c->neighbours[2];
    model_leds[3] = c->neighbours[4];
    check_led_against_model_subset (cs, mi, blobs, model_leds);

    model_leds[1] = c->neighbours[0];
    model_leds[2] = c->neighbours[3];
    model_leds[3] = c->neighbours[4];
    check_led_against_model_subset (cs, mi, blobs, model_leds);

    model_leds[1] = c->neighbours[1];
    model_leds[2] = c->neighbours[2];
    model_leds[3] = c->neighbours[4];
    check_led_against_model_subset (cs, mi, blobs, model_leds);

    model_leds[1] = c->neighbours[1];
    model_leds[2] = c->neighbours[3];
    model_leds[3] = c->neighbours[4];
    check_led_against_model_subset (cs, mi, blobs, model_leds);

    model_leds[1] = c->neighbours[2];
    model_leds[2] = c->neighbours[3];
    model_leds[3] = c->neighbours[4];
    check_led_against_model_subset (cs, mi, blobs, model_leds);
#endif
  }
}

static void
check_led_match (correspondence_search_t *cs, cs_image_point_t **blobs)
{
    int m;

    for (m = 0; m < cs->num_models; m++) {
        cs_model_info_t *mi = cs->models + m;
        check_leds_against_model (cs, mi, blobs);
    }
}

/* Select k entries from the n provided in candidate_list into
 * output_list, then call check_led_match() with the result_list */
static void
select_k_from_n (correspondence_search_t *cs,
    cs_image_point_t **result_list, cs_image_point_t **output_list,
    cs_image_point_t **candidate_list, int k, int n)
{
    if (k == 1) {
        cs_image_point_t *swap_list[4];

        output_list[0] = candidate_list[0];
        check_led_match (cs, result_list);

        /* Check the other orientation of blob 2/3, without
         * affecting result_list that needs to stay intact
         * for other recursive calls */
        swap_list[0] = result_list[0];
        swap_list[1] = result_list[2];
        swap_list[2] = result_list[1];
        swap_list[3] = result_list[3];
        check_led_match (cs, swap_list);

        return;
    }

    /*
     * 2 branches here:
     *   take the first entry, then select k-1 from n-1
     *   don't take the first entry, select k from n-1 if n > k
     */
    assert (k > 1);
    assert (n > 1);
    output_list[0] = candidate_list[0];
    select_k_from_n (cs, result_list, output_list+1, candidate_list+1, k-1, n-1);

    if (n > k)
      select_k_from_n (cs, result_list, output_list, candidate_list+1, k, n-1);
}

/* Generate quadruples of neighbouring blobs and pass to check_led_match().
 * We want to
 *   a) preserve the first blob in the list each iteration
 *   b) Select combinations of 3 from the next 'max_search_depth' points.
 *   c) Try 2 permutations, [0,1,2,3] and [0,2,1,3]. In each case index 0 is
 *   the anchor and index 3 is a disambiguation check on the P3P result
 *
 * We can work in a tmp array so as not to destroy the master list.
 *
 * The caller guarantees that the blob_list has at least max_search_depth+1
 * entries.
 */
static void
generate_match_candidates (correspondence_search_t *cs, cs_image_point_t **blob_list, int max_search_depth)
{
  cs_image_point_t *work_list[MAX_SEARCH_DEPTH+1];

  work_list[0] = blob_list[0];
  select_k_from_n (cs, work_list, work_list+1, blob_list+1, 3, max_search_depth);
}

int
correspondence_search_find_pose (correspondence_search_t *cs)
{
    int max_search_depth = MIN (MAX_SEARCH_DEPTH, cs->num_points-1);
    int b;
    int m;
    int found_poses = 0;

    for (m = 0; m < cs->num_models; m++) {
        cs_model_info_t *mi = &cs->models[m];
        mi->best_matched = mi->best_visible = 0;
    }

    if (cs->num_points < 4)
        return 0; /* Not enough blobs to bother */

    printf ("Max search depth %u\n", max_search_depth);
    cs->num_trials = cs->num_pose_checks = 0;

    /* Prepare a list of pointers to blobs that we can sort
     * by distance each iteration below */
    cs_image_point_t *blob_list[MAX_BLOBS_PER_FRAME];
    for (b = 0; b < cs->num_points; b++)
      blob_list[b] = cs->points + b;

    /* Start our correspondence search */
    for (b = 0; b < cs->num_points; b++) {
    //b = 3; {
        cs_image_point_t *anchor = cs->points + b;

        /* Sort the blobs by proximity to anchor blob */
        qsort_r (blob_list, cs->num_points, sizeof (cs_image_point_t *), compare_blobs_distance, anchor);

#if 0
        {
            printf ("Search list:\n");
            for (int i = 0; i < cs->num_points; i++) {
                cs_image_point_t *p1 = blob_list[i];
                double dist = (p1->blob->y - anchor->blob->y) * (p1->blob->y - anchor->blob->y) +
                              (p1->blob->x - anchor->blob->x) * (p1->blob->x - anchor->blob->x);
                printf ("  LED ID %u (%f,%f) @ %u,%u. Dist %f\n", p1->blob->led_id, p1->point_homog[0], p1->point_homog[1],
                    p1->blob->x, p1->blob->y, sqrt(dist));
            }
        }
#endif

        /* Test combinations and permutations of the top 'max_search_depth' blobs in this set */
        generate_match_candidates (cs, blob_list, max_search_depth);
    }

    printf ("Ran %u trials, and %u full pose checks\n", cs->num_trials, cs->num_pose_checks);

#if DUMP_SCENE
    printf ("ref_points += [\n");
    for (b = 0; b < cs->num_points; b++) {
        cs_image_point_t *p = cs->points + b;
        printf ("  (%f, %f),\n", p->point_homog[0], p->point_homog[1]);
    }
    printf ("]\n");
#endif

    for (m = 0; m < cs->num_models; m++) {
        cs_model_info_t *mi = &cs->models[m];
        printf ("# Best match for model %d was %d points out of %d with error %f (%d pixels)\n", m, mi->best_matched, mi->best_visible, mi->best_sqerror, (int) round(mi->best_sqerror * cs->camera_matrix->m[0] * cs->camera_matrix->m[4]));
        printf ("# pose orient %f %f %f %f pos %f %f %f\n",
                      mi->best_orient.x, mi->best_orient.y, mi->best_orient.z, mi->best_orient.w,
                      mi->best_trans.x, mi->best_trans.y, mi->best_trans.z);
#if DUMP_SCENE
        dump_pose (cs, mi->model, &mi->best_orient, &mi->best_trans, mi);
#endif
        if (mi->best_visible > 4 && mi->best_matched > 4) {
            if (mi->best_visible < 2 * mi->best_matched)
                found_poses++;
        }
    }

    return found_poses;
}

bool
correspondence_search_have_pose (correspondence_search_t *cs, int model_id, quatf *orient, vec3f *trans)
{
  int i;

  for (i = 0; i < cs->num_models; i++) {
    cs_model_info_t *mi = &cs->models[i];
    if (mi->id != model_id)
        continue;

    if (mi->best_visible > 4 && mi->best_matched > 4) {
      if (mi->best_visible < 2 * mi->best_matched) {
          *orient = mi->best_orient;
          *trans = mi->best_trans;
          return true;
      }
    }
  }
  return false;
}
