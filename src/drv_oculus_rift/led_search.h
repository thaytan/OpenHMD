#ifndef __LED_SEARCH_H__
#define __LED_SEARCH_H__

#include "rift.h"

typedef struct led_search_candidate_s led_search_candidate_t;
typedef struct led_search_model_s led_search_model_t;

struct led_search_candidate_s {
    rift_led *led;

    /* Transform to rotate the anchor LED to face forward @ 0,0,0 */
    posef pose;

    /* List of possible neighbours for this LED, sorted by distance */
    uint8_t num_neighbours;
    rift_led **neighbours;
};

led_search_candidate_t *led_search_candidate_new (rift_led *led, rift_leds *led_model);
void led_search_candidate_free (led_search_candidate_t *candidate);

struct led_search_model_s {
    uint8_t num_points;
    led_search_candidate_t **points;

    rift_leds *leds;
};

led_search_model_t *led_search_model_new (rift_leds *led_model);
void led_search_model_free (led_search_model_t *model);
#endif
