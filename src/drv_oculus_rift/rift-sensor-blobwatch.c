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

#define THRESHOLD 0x9f

#define NUM_FRAMES_HISTORY	2
#define MAX_EXTENTS_PER_LINE	11

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
	uint8_t index;
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
	int width;
	int height;
	int last_observation;
	struct blobservation history[NUM_FRAMES_HISTORY];
	struct extent_line *el;
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
struct blobwatch *blobwatch_new(int width, int height)
{
	struct blobwatch *bw = malloc(sizeof(*bw));

	if (!bw)
		return NULL;

	memset(bw, 0, sizeof(*bw));
	bw->width = width;
	bw->height = height;
	bw->last_observation = -1;
	bw->debug = true;
	bw->flicker_enable = true;
	bw->el = calloc(height, sizeof(*bw->el));

	return bw;
}

/*
 * Stores blob information collected in the last extent e into the blob
 * array b at index e->index.
 */
static inline void store_blob(struct extent *e, int y, struct blob *b)
{
	b += e->index;
	b->x = (e->left + e->right) / 2;
	b->y = (e->top + y) / 2;
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
 * 0x9f in a given scanline and stores them in extents. Processing stops after
 * num_extents.
 * Extents are marked with the same index as overlapping extents of the previous
 * scanline, and properties of the formed blobs are accumulated.
 *
 * Returns the number of extents found.
 */
static int process_scanline(uint8_t *line, int width, int height, int y,
			    struct extent_line *el, struct extent_line *prev_el,
			    int index, struct blobservation *ob)
{
	struct extent *le_end = prev_el->extents;
	struct extent *le = prev_el->extents;
	struct extent *extent = el->extents;
	struct blob *blobs = ob->blobs;
	int num_extents = MAX_EXTENTS_PER_LINE;
	int num_blobs = MAX_BLOBS_PER_FRAME;
	int center;
	int x, e = 0;

	if (prev_el)
		le_end += prev_el->num;

	for (x = 0; x < width; x++) {
		int start, end;

		/* Loop until pixel value exceeds threshold */
		if (line[x] <= THRESHOLD)
			continue;

		start = x++;

		/* Loop until pixel value falls below threshold */
		while (x < width && line[x] > THRESHOLD)
			x++;

		end = x - 1;
		/* Filter out single pixel and two-pixel extents */
		if (end < start + 2)
			continue;

		center = (start + end) / 2;

		extent->start = start;
		extent->end = end;
		extent->index = index;
		extent->area = x - start;

		if (prev_el && index < num_blobs) {
			/*
			 * Previous extents without significant overlap are the
			 * bottom of finished blobs. Store them into an array.
			 */
			while (le < le_end && le->end < center &&
			       le->index < num_blobs)
				store_blob(le++, y, blobs);

			/*
			 * A previous extent with significant overlap is
			 * considered to be part of the same blob.
			 */
			if (le < le_end &&
			    le->start <= center && le->end > center) {
				extent->top = le->top;
				extent->left = min(extent->start, le->left);
				extent->right = max(extent->end, le->right);
				extent->area += le->area;
				extent->index = le->index;
				le++;
			}
		}

		/*
		 * If this extent is not part of a previous blob, increment the
		 * blob index.
		 */
		if (extent->index == index) {
			extent->top = y;
			extent->left = extent->start;
			extent->right = extent->end;
			index++;
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
		while (le < le_end && le->index < num_blobs)
			store_blob(le++, y, blobs);
	}

	el->num = e;

	if (y == height - 1) {
		/* All extents of the last line are finished blobs, too. */
		for (extent = el->extents; extent < el->extents + el->num;
		     extent++) {
			if (extent->index < num_blobs)
				store_blob(extent, y, blobs);
		}
	}

	return index;
}

/*
 * Collects extents from all scanlines in a frame and stores them in
 * the extent_line array el.
 */
static void process_frame(uint8_t *lines, int width, int height,
			  struct extent_line *el, struct blobservation *ob)
{
	struct extent_line *last_el;
	int index = 0;
	int y;

	ob->num_blobs = 0;

	index = process_scanline(lines, width, height, 0, el, NULL, 0, ob);

	for (y = 1; y < height; y++) {
		last_el = el++;
		lines += width;
		index = process_scanline(lines, width, height, y, el, last_el,
					 index, ob);
	}

	ob->num_blobs = min(MAX_BLOBS_PER_FRAME, index);
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
	struct extent_line *el = bw->el;
	int i, j;

	process_frame(frame, width, height, el, ob);

	/* If there is no previous observation, our work is done here */
	if (bw->last_observation == -1) {
		bw->last_observation = current;
		if (output)
			*output = NULL;
		return;
	}

	/* Otherwise track blobs over time */
	memset(ob->tracked, 0, sizeof(uint8_t) * MAX_BLOBS_PER_FRAME);

	/*
	 * Associate blobs found at a previous blobs' estimated next
	 * positions with their predecessors.
	 */
	for (i = 0; i < ob->num_blobs; i++) {
		struct blob *b2 = &ob->blobs[i];

		/* Filter out tall and wide (<= 1:2, >= 2:1) blobs */
		if (2 * b2->width <= b2->height ||
		    b2->width >= 2 * b2->height)
			continue;

		for (j = 0; j < last_ob->num_blobs; j++) {
			struct blob *b1 = &last_ob->blobs[j];
			int x, y, dx, dy;

			/* Estimate b1's next position */
			x = b1->x + b1->vx;
			y = b1->y + b1->vy;

			/* Absolute distance */
			dx = abs(x - b2->x);
			dy = abs(y - b2->y);

			/*
			 * Check if b1's estimated next position falls
			 * into b2's bounding box.
			 */
			if (2 * dx > b2->width ||
			    2 * dy > b2->height)
				continue;

			b2->age = b1->age + 1;
			if (b1->track_index >= 0 &&
			    ob->tracked[b1->track_index] == 0) {
				/* Only overwrite tracks that are not already set */
				b2->track_index = b1->track_index;
				ob->tracked[b2->track_index] = i + 1;
				memcpy (b2->pattern_bits, b1->pattern_bits, sizeof (b1->pattern_bits));
				b2->pattern = b1->pattern;
				b2->pattern_age = b1->pattern_age;
				b2->pattern_prev_phase = b1->pattern_prev_phase;
				b2->led_id = b1->led_id;
			}
			b2->vx = b2->x - b1->x;
			b2->vy = b2->y - b1->y;
			b2->last_area = b1->area;
			break;
		}
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
