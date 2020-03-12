/*
 * Blob detection
 * Copyright 2014-2015 Philipp Zabel
 * SPDX-License-Identifier:	LGPL-2.0+ or BSL-1.0
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "rift-sensor-blobwatch.h"
#include "rift-sensor-flicker.h"

struct leds;

#include <stdio.h>

#define NUM_FRAMES_HISTORY	2
#define MAX_EXTENTS_PER_LINE	30

#define abs(x) ((x) >= 0 ? (x) : -(x))
#define min(x, y) ((x) < (y) ? (x) : (y))
#define max(x, y) ((x) > (y) ? (x) : (y))

struct extent {
	uint16_t start;
	uint16_t end;
	/* inherited parameters */
	uint16_t top;
	uint16_t left;
	uint16_t right;
	uint32_t area;
};

struct extent_line {
	struct extent extents[MAX_EXTENTS_PER_LINE];
	uint16_t num;
	uint16_t padding[3];
};


/*
 * Blob detector internal state
 */
struct blobwatch {
	uint8_t threshold;
	int width;
	int height;
	int last_observation;
	struct blobservation history[NUM_FRAMES_HISTORY];
	bool debug;
	bool flicker_enable;
};

void blobwatch_set_flicker(struct blobwatch *bw, bool enable)
{
	bw->flicker_enable = enable;
}

/*
 * Allocates and initializes blobwatch structure.
 *
 * Returns the newly allocated blobwatch structure.
 */
struct blobwatch *blobwatch_new(uint8_t threshold, int width, int height)
{
	struct blobwatch *bw = malloc(sizeof(*bw));

	if (!bw)
		return NULL;

	memset(bw, 0, sizeof(*bw));
	bw->threshold = threshold;
	bw->width = width;
	bw->height = height;
	bw->last_observation = -1;
	bw->debug = true;
	bw->flicker_enable = false;

	return bw;
}

void blobwatch_free (struct blobwatch *bw)
{
	free (bw);
}

/*
 * Stores blob information collected in the last extent e into the blob
 * array b at the given index.
 */
static inline void store_blob(struct extent *e, int index, int y, struct blob *b)
{
	b += index;
	b->x = (e->left + e->right) / 2.0;
	b->y = (e->top + y) / 2.0;
	b->vx = 0;
	b->vy = 0;
	b->width = e->right - e->left + 1;
	b->height = y - e->top + 1;
	b->area = e->area;
	b->age = 0;
	b->track_index = -1;
	b->pattern = 0;
	b->pattern_age = 0;
	b->pattern_prev_phase = -1;
	memset (b->pattern_bits, 0, sizeof (b->pattern_bits));
	b->led_id = -1;
}

/*
 * Collects contiguous ranges of pixels with values larger than a threshold of
 * THRESHOLD in a given scanline and stores them in extents. Processing stops after
 * num_extents.
 * Extents are marked with the same index as overlapping extents of the previous
 * scanline, and properties of the formed blobs are accumulated.
 *
 * Returns the number of extents found.
 */
static void process_scanline(uint8_t *line, struct blobwatch *bw, int y,
			    struct extent_line *el, struct extent_line *prev_el,
			    struct blobservation *ob)
{
	struct extent *le_end = prev_el->extents;
	struct extent *le = prev_el->extents;
	struct extent *extent = el->extents;
	struct blob *blobs = ob->blobs;
	int num_extents = MAX_EXTENTS_PER_LINE;
	int num_blobs = MAX_BLOBS_PER_FRAME;
	float center;
	int x, e = 0;

	if (prev_el)
		le_end += prev_el->num;

	for (x = 0; x < bw->width; x++) {
		int start, end;
		bool is_new_extent = true;

		/* Loop until pixel value exceeds threshold */
		if (line[x] <= bw->threshold)
			continue;

		start = x++;

		/* Loop until pixel value falls below threshold */
		while (x < bw->width && line[x] > bw->threshold)
			x++;

		end = x - 1;

		center = (start + end) / 2.0;

		extent->start = start;
		extent->end = end;
		extent->area = x - start;

		if (prev_el && ob->num_blobs < num_blobs) {
			/*
			 * Previous extents without significant overlap are the
			 * bottom of finished blobs. Store them into an array.
			 */
			while (le < le_end && le->end < center &&
			       ob->num_blobs < num_blobs) {
				/* Don't store 1x1 blobs */
				if (le->top != y || le->left != le->right)
					store_blob(le, ob->num_blobs++, y, blobs);

				le++;
			}

			/*
			 * A previous extent with significant overlap is
			 * considered to be part of the same blob.
			 */
			if (le < le_end &&
			    le->start <= center && le->end >= center) {
				extent->top = le->top;
				extent->left = min(extent->start, le->left);
				extent->right = max(extent->end, le->right);
				extent->area += le->area;
				is_new_extent = false;
				le++;
			}
		}

		/*
		 * If this extent is not part of a previous blob, increment the
		 * blob index.
		 */
		if (is_new_extent) {
			extent->top = y;
			extent->left = extent->start;
			extent->right = extent->end;
		}

		if (++e == num_extents)
			break;
		extent++;
	}

	if (prev_el) {
		/*
		 * If there are no more extents on this line, all remaining
		 * extents in the previous line are finished blobs. Store them.
		 */
		while (le < le_end && ob->num_blobs < num_blobs) {
			/* Don't store 1x1 blobs */
			if (le->top != y || le->left != le->right)
				store_blob(le, ob->num_blobs++, y, blobs);
			le++;
		}
	}

	el->num = e;

	if (y == bw->height - 1) {
		/* All extents of the last line are finished blobs, too. */
		for (extent = el->extents; extent < el->extents + el->num;
		     extent++) {
			if (ob->num_blobs >= num_blobs)
				break;

			/* Don't store 1x1 blobs */
			if (le->top != y || le->left != le->right)
				store_blob(extent, ob->num_blobs++, y, blobs);
		}
	}
}

/*
 * Processes extents from all scanlines in a frame and stores the
 * resulting blobs in ob->blobs.
 */
static void process_frame(uint8_t *lines, struct blobwatch *bw, struct blobservation *ob)
{
	struct extent_line el1;
	struct extent_line el2;
	int y = 2;

	ob->num_blobs = 0;

	lines += bw->width * y;
	process_scanline(lines, bw, y++, &el1, NULL, ob);

	for (; y < bw->height; y++) {
		lines += bw->width;
		process_scanline(lines, bw, y, y&1? &el2 : &el1, y&1? &el1 : &el2, ob);
	}
}

/*
 * Finds the first free tracking slot.
 */
static int find_free_track(uint8_t *tracked)
{
	int i;

	for (i = 0; i < MAX_BLOBS_PER_FRAME; i++) {
		if (tracked[i] == 0)
			return i;
	}

	return -1;
}

void copy_matching_blob(struct blob *to, struct blob* from) {
	to->vx = to->x - from->x;
	to->vy = to->y - from->y;
	memcpy (to->pattern_bits, from->pattern_bits, sizeof (from->pattern_bits));
	to->pattern = from->pattern;
	to->pattern_age = from->pattern_age;
	to->pattern_prev_phase = from->pattern_prev_phase;
	to->led_id = from->led_id;
	to->age = from->age + 1;
}

/*
 * Detects blobs in the current frame and compares them with the observation
 * history.
 */
void blobwatch_process(struct blobwatch *bw, uint8_t *frame,
		       int width, int height, uint8_t led_pattern_phase,
		       rift_led *leds, uint8_t num_leds,
		       struct blobservation **output)
{
	int last = bw->last_observation;
	int current = (last + 1) % NUM_FRAMES_HISTORY;
	struct blobservation *ob = &bw->history[current];
	struct blobservation *last_ob = &bw->history[last];
	int closest_ob[MAX_BLOBS_PER_FRAME]; // index of last_ob that is closest to each ob
	int closest_last_ob[MAX_BLOBS_PER_FRAME]; // index of ob that is closest to each last_ob
	int closest_last_ob_distsq[MAX_BLOBS_PER_FRAME]; // distsq of ob that is closest to each last_ob
	int i, j;

	bw->width = width;
	bw->height = height;

	process_frame(frame, bw, ob);

	/* If there is no previous observation, our work is done here */
	if (bw->last_observation == -1) {
		bw->last_observation = current;
		if (output)
			*output = ob;
		return;
	}

	/* Clear closest_* */
	for (i = 0; i < MAX_BLOBS_PER_FRAME; i++) {
		closest_ob[i] = -1;
		closest_last_ob[i] = -1;
		closest_last_ob_distsq[i] = 1000000;
	}

	int scan_again = 1;
	int scan_times = 0;
	while (scan_again) {
		scan_again = 0;

		/* Try to match each blob with the closest blob from the previous frame. */
	for (i = 0; i < ob->num_blobs; i++) {
			if (closest_ob[i] != -1)
				continue; // already has a match
			
		struct blob *b2 = &ob->blobs[i];
			int closest_j = -1;
			int closest_distsq = -1;

		for (j = 0; j < last_ob->num_blobs; j++) {
			struct blob *b1 = &last_ob->blobs[j];
				int x, y, dx, dy, distsq;

			/* Estimate b1's next position */
			x = b1->x + b1->vx;
			y = b1->y + b1->vy;

			/* Absolute distance */
			dx = abs(x - b2->x);
			dy = abs(y - b2->y);
				distsq = dx * dx + dy * dy;

				if (closest_distsq < 0 || distsq < closest_distsq) {
					if (closest_last_ob[j] != -1 && closest_last_ob_distsq[j] <= distsq) {
						// some blob already claimed this one as closest
						// don't usurp if previous one is closer
						continue;
					}
					closest_j = j;
					closest_distsq = distsq;
				}
			}

			closest_ob[i] = closest_j;

			// didn't find any matching blobs
			if (closest_j < 0)
				continue;

			if (closest_last_ob[closest_j] != -1) {
				// we are usurping some other blob because we are closer
				closest_ob[closest_last_ob[closest_j]] = -1;
				scan_again++;
			}

			closest_last_ob[closest_j] = i;
			closest_last_ob_distsq[closest_j] = closest_distsq;
		}

		if (scan_times++ > 100)
			printf("scan_times: %d\n", scan_times);

	}

	/* Copy blobs that found a closest match */
	for (i = 0; i < ob->num_blobs; i++) {
		if (closest_ob[i] < 0)
			continue; // no match
		
		struct blob *b2 = &ob->blobs[i];
		struct blob *b1 = &last_ob->blobs[closest_ob[i]];

			if (b1->track_index >= 0 &&
			    ob->tracked[b1->track_index] == 0) {
				/* Only overwrite tracks that are not already set */
				b2->track_index = b1->track_index;
				ob->tracked[b2->track_index] = i + 1;
			}
		copy_matching_blob(b2, b1);
		}
	// printf("done matching\n");

	/*
	 * Clear the tracking array where blobs have gone missing.
	 */
	for (i = 0; i < MAX_BLOBS_PER_FRAME; i++) {
		int t = ob->tracked[i];
		if (t > 0 && ob->blobs[t-1].track_index != i)
			ob->tracked[i] = 0;
	}

	/*
	 * Associate newly tracked blobs with a free space in the
	 * tracking array.
	 */
	for (i = 0; i < ob->num_blobs; i++) {
		struct blob *b2 = &ob->blobs[i];

		if (b2->age > 0 && b2->track_index < 0)
			b2->track_index = find_free_track(ob->tracked);
		if (b2->track_index >= 0)
			ob->tracked[b2->track_index] = i + 1;
	}

	/* Check blob <-> tracked array links for consistency */
	for (i = 0; i < ob->num_blobs; i++) {
		struct blob *b = &ob->blobs[i];

		if (b->track_index >= 0 &&
		    ob->tracked[b->track_index] != i + 1) {
			printf("Inconsistency! %d != %d\n",
			       ob->tracked[b->track_index], i + 1);
		}
	}

	if (bw->flicker_enable) {
		/* Identify blobs by their blinking pattern */
		rift_sensor_flicker_process(ob->blobs, ob->num_blobs, led_pattern_phase,
				leds, num_leds);
	}

	/* Return observed blobs */
	if (output)
		*output = ob;

	bw->last_observation = current;
}

struct blob *blobwatch_find_blob_at(struct blobwatch *bw, int x, int y)
{
	int last = bw->last_observation;
	struct blobservation *ob;
  int i;

	if (last == -1) {
      /* No blobs to match against yet */
      return NULL;
  }

	ob = &bw->history[last];
	for (i = 0; i < ob->num_blobs; i++) {
		struct blob *b = ob->blobs + i;
		int dx = abs(x - b->x);
		int dy = abs(y - b->y);

    /* Check if the target is outside the bounding box */
		if (2 * dx > b->width ||
		    2 * dy > b->height)
			continue;
    return b;
  }

  return NULL;
}
