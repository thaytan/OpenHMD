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

#define MAX_SEARCH_DEPTH 6

//#define printf(s,...)
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

static int compare_blobs_distance (const void *elem1, const void *elem2, void *arg);

void
correspondence_search_set_blobs (correspondence_search_t *cs, struct blob *blobs, int num_blobs)
{
    int i;
    vec3f undistorted_points[MAX_BLOBS_PER_FRAME];
    cs_image_point_t *blob_list[MAX_BLOBS_PER_FRAME];

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
        blob_list[i] = p;

#if 0
        printf ("Blob %u = (%u,%u) -> (%f, %f) %f x %f (LED id %d)\n", i,
                b->x, b->y,
                p->point_homog[0], p->point_homog[1],
                p->size[0], p->size[1], b->led_id);
#endif
    }

    /* Now the blob_list is populated,
     * loop over the blob list and for each, prepare a list of neighbours sorted by distance */
    for (i = 0; i < cs->num_points; i++) {
        cs_image_point_t *anchor = cs->points + i;

        /* Sort the blobs by proximity to anchor blob */
        qsort_r (blob_list, cs->num_points, sizeof (cs_image_point_t *), compare_blobs_distance, anchor);
        memcpy (cs->blob_neighbours[i], blob_list, cs->num_points * sizeof(cs_image_point_t *));
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
  if (trans->z < 0.05 || trans->z > 15) /* Invalid position, out of range */
    return false;

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
    if (visible_leds <= 2 * matched_visible_blobs) {

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

            mi->best_matched = matched_visible_blobs;
            mi->best_visible = visible_leds;
            mi->best_orient = *orient;
            mi->best_trans = *trans;
            mi->best_sqerror = projection_sqerror;

#if 0
            printf ("model %d new best pose candidate orient %f %f %f %f pos %f %f %f has %u visible LEDs, error %f\n",
                   mi->id, orient->x, orient->y, orient->z, orient->w,
                         trans->x, trans->y, trans->z, visible_leds, projection_sqerror);
            printf("model %d matched %u blobs of %u\n", mi->id, matched_visible_blobs, visible_leds);
            dump_pose (cs, model, orient, trans, mi);
#endif
            return true;
        }
      } else {
#if 0
        printf ("pose candidate orient %f %f %f %f pos %f %f %f has %u visible LEDs, error %f\n",
                orient->x, orient->y, orient->z, orient->w,
                trans->x, trans->y, trans->z, visible_leds, projection_sqerror);
        printf("matched %u blobs of %u\n", matched_visible_blobs, visible_leds);
#endif
      }

#if 0
      printf ("Found good pose match for device %u - %u LEDs matched %u visible ones\n",
          mi->id, matched_visible_blobs, visible_leds);
#endif
      return true;
    }
#if 0
    else {
      printf ("Failed pose match - only %u LEDs matched %u visible ones\n",
          matched_visible_blobs, visible_leds);
    }
#endif
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

      if (trans.z < 0.05 || trans.z > 15) {
          /* The object is unlikely to be < 5cm (50000µm) or > 15m from the camera */
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
#if 0
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
#endif
        }
      }
    }
  }
}

/* Select k entries from the n provided in candidate_list into
 * output_list, then call check_led_match() with the result_list */
static void
select_k_blobs_from_n (correspondence_search_t *cs, cs_model_info_t *mi, rift_led **model_leds,
    cs_image_point_t **result_list, cs_image_point_t **output_list,
    cs_image_point_t **candidate_list, int k, int n)
{
    if (k == 1) {
        output_list[0] = candidate_list[0];
        check_led_against_model_subset (cs, mi, result_list, model_leds);
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
    select_k_blobs_from_n (cs, mi, model_leds, result_list, output_list+1, candidate_list+1, k-1, n-1);

    if (n > k)
      select_k_blobs_from_n (cs, mi, model_leds, result_list, output_list, candidate_list+1, k, n-1);
}

/* Generate quadruples of neighbouring blobs and pass to check_led_match().
 * We want to
 *   a) preserve the first blob in the list each iteration
 *   b) Select combinations of 3 from the next 'max_search_depth' points.
 *
 * Permutation checking is done in the model LED outer loop, so this inner selection
 * only needs to try combos.
 *
 * We can work in a tmp array so as not to destroy the master list.
 *
 * The caller guarantees that the blob_list has at least max_search_depth+1
 * entries.
 */
static void
check_leds_against_anchor (correspondence_search_t *cs, cs_model_info_t *mi,
    rift_led **model_leds, cs_image_point_t *anchor)
{
  cs_image_point_t *work_list[MAX_BLOB_SEARCH_DEPTH+1];
  int max_blob_search_depth = MIN(anchor->num_neighbours, MAX_BLOB_SEARCH_DEPTH);

  if (max_blob_search_depth < 3)
      return; // Not enough blobs to compare against

  work_list[0] = anchor;
  select_k_blobs_from_n (cs, mi, model_leds, work_list, work_list+1, anchor->neighbours, 3, max_blob_search_depth);
}

/* Called with 4 model_leds that need checking against blob combos */
static void
check_led_match (correspondence_search_t *cs, cs_model_info_t *mi, rift_led **model_leds)
{
    int b;

    for (b = 0; b < cs->num_points; b++) {
        cs_image_point_t *anchor = cs->points + b;
        check_leds_against_anchor (cs, mi, model_leds, anchor);
    }
}

/* Select k rift_led entries from the n provided in candidate_list into
 * output_list, then call check_led_match() with the result_list */
static void
select_k_leds_from_n (correspondence_search_t *cs,
    cs_model_info_t *mi,
    rift_led **result_list, rift_led **output_list,
    rift_led **candidate_list, int k, int n)
{
    if (k == 1) {
        rift_led *swap_list[4];

        output_list[0] = candidate_list[0];
        check_led_match (cs, mi, result_list);

        /* Check the other orientation of blob 2/3, without
         * affecting result_list that needs to stay intact
         * for other recursive calls */
        swap_list[0] = result_list[0];
        swap_list[1] = result_list[2];
        swap_list[2] = result_list[1];
        swap_list[3] = result_list[3];

        check_led_match (cs, mi, swap_list);

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
    select_k_leds_from_n (cs, mi, result_list, output_list+1, candidate_list+1, k-1, n-1);

    if (n > k)
      select_k_leds_from_n (cs, mi, result_list, output_list, candidate_list+1, k, n-1);
}

/* Generate quadruples of neighbouring leds and pass to check_led_match().
 * We want to
 *   a) preserve the first led in the list each iteration
 *   b) Select combinations of 3 from the 'max_search_depth' neighbouring LEDs.
 *   c) Try 2 permutations, [0,1,2,3] and [0,2,1,3]. In each case index 0 is
 *   the anchor and index 3 is a disambiguation check on the P3P result
 *
 * We can work in a tmp array so as not to destroy the master list.
 *
 * The caller guarantees that the LED neighbours list has at least max_search_depth
 * entries.
 */
static void
generate_led_match_candidates (correspondence_search_t *cs, cs_model_info_t *mi,
    led_search_candidate_t *c, int max_search_depth)
{
  rift_led *work_list[MAX_SEARCH_DEPTH+1];

  work_list[0] = c->led;
  select_k_leds_from_n (cs, mi, work_list, work_list+1, c->neighbours, 3, max_search_depth);
}

int
correspondence_search_find_pose (correspondence_search_t *cs)
{
    int b;
    int m, l;
    int found_poses = 0;

    /* Clear the pose information for each model */
    for (m = 0; m < cs->num_models; m++) {
        cs_model_info_t *mi = &cs->models[m];
        mi->best_matched = mi->best_visible = 0;
    }
    cs->num_trials = cs->num_pose_checks = 0;

    if (cs->num_points < 4)
        return 0; /* Not enough blobs to bother */

    /* Loop over each model and try to find correspondences */
    for (m = 0; m < cs->num_models; m++) {
        cs_model_info_t *mi = &cs->models[m];
        led_search_model_t *model = mi->model;
        struct blob matched_blobs[MAX_BLOBS_PER_FRAME];
        int already_known_blobs = 0;

        /* First, see if we have enough correspondences to directly get the pose */
        for (b = 0; b < cs->num_points; b++) {
            int led_id = cs->points[b].blob->led_id;

            if (led_id != LED_INVALID_ID && LED_OBJECT_ID(led_id) == m) {
              matched_blobs[already_known_blobs] = *cs->points[b].blob;
              already_known_blobs++;
            }
        }

       if (already_known_blobs > 3) {
         int num_matched_leds = 0, inliers = 0;

         printf ("We have %u correspondences already for model %u\n", already_known_blobs, m);
         /* FIXME: We should do this check for available correspondences and reproject before even giving these
          * blobs to the correspondence_search, and then set this model to be skipped */
         if (estimate_initial_pose(matched_blobs, already_known_blobs,
             model->leds->points, model->leds->num_points, cs->camera_matrix, cs->dist_coeffs,
             &mi->best_orient, &mi->best_trans, &num_matched_leds, &inliers, true)) {
             printf ("%u existing correspondences for model %u matched %u\n", already_known_blobs, m, inliers);
            if (correspondence_search_project_pose (cs, model, &mi->best_orient, &mi->best_trans, mi))
            {
               printf ("# pose orient %f %f %f %f pos %f %f %f\n",
                     mi->best_orient.x, mi->best_orient.y, mi->best_orient.z, mi->best_orient.w,
                     mi->best_trans.x, mi->best_trans.y, mi->best_trans.z);
               continue;
             }
             printf ("Pose from %u correspondences for model %u did not yield good pose\n", already_known_blobs, m);
         }
         else {
           printf ("Could not get pose from %u correspondences for model %u\n", already_known_blobs, m);
         }
       }

        /* filter the list of blobs to unknown, or belonging to this model */
        for (b = 0; b < cs->num_points; b++) {
            cs_image_point_t **all_neighbours = cs->blob_neighbours[b];
            cs_image_point_t *anchor = cs->points + b;
            int out_index = 0, in_index;

            for (in_index = 0; in_index < cs->num_points && out_index < MAX_BLOB_SEARCH_DEPTH; in_index++) {
                int led_id = all_neighbours[in_index]->blob->led_id;
                if (led_id == LED_INVALID_ID || LED_OBJECT_ID(led_id) == m)
                  anchor->neighbours[out_index++] = all_neighbours[in_index];
            }
            anchor->num_neighbours = out_index;

#if 0
            printf ("Model %d, blob %d Search list:\n", m, b);
            for (int i = 0; i < anchor->num_neighbours; i++) {
                cs_image_point_t *p1 = anchor->neighbours[i];
                double dist = (p1->blob->y - anchor->blob->y) * (p1->blob->y - anchor->blob->y) +
                              (p1->blob->x - anchor->blob->x) * (p1->blob->x - anchor->blob->x);
                printf ("  LED ID %u (%f,%f) @ %u,%u. Dist %f\n", p1->blob->led_id, p1->point_homog[0], p1->point_homog[1],
                    p1->blob->x, p1->blob->y, sqrt(dist));
            }
#endif
        }

        /* Start correspondence search for this model */
        /* At this point, each image point has a list of the nearest neighbours filtered for this model */
        for (l = 0; l < model->num_points; l++) {
            led_search_candidate_t *c = model->points[l];
            int max_search_depth = MIN (MAX_SEARCH_DEPTH, c->num_neighbours);

            if (max_search_depth < 3)
                continue; // Not enough LEDs to compare against
            generate_led_match_candidates (cs, mi, c, max_search_depth);
        }
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
