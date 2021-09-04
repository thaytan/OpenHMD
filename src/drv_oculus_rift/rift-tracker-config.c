/*
 * Rift position tracking - room configuration
 * Copyright 2021 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 */

#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "../ext_deps/nxjson.h"

#include "rift-tracker-config.h"

#define CONFIG_FILE_KEY "rift-room-config.json"
#define CONFIG_BLOCK_LEN 2048

void rift_tracker_config_init(rift_tracker_config *config)
{
	memset(config, 0, sizeof(rift_tracker_config));
}

static bool json_read_vec3(const nx_json *nxj, const char *key, vec3f *out)
{
	const nx_json *member = nx_json_get (nxj, key);
	int i;

	if (member->type != NX_JSON_ARRAY)
		return false;

	for (i = 0; i < 3; i++) {
		const nx_json *item = nx_json_item (member, i);
		if (item->type != NX_JSON_DOUBLE && item->type != NX_JSON_INTEGER)
			return false;
		out->arr[i] = item->dbl_value;
	}

	return true;
}

static bool json_read_quat(const nx_json *nxj, const char *key, quatf *out)
{
	const nx_json *member = nx_json_get (nxj, key);
	int i;

	if (member->type != NX_JSON_ARRAY)
		return false;

	for (i = 0; i < 4; i++) {
		const nx_json *item = nx_json_item (member, i);
		if (item->type != NX_JSON_DOUBLE && item->type != NX_JSON_INTEGER)
			return false;
		out->arr[i] = item->dbl_value;
	}

	return true;
}

void rift_tracker_config_load(ohmd_context *ctx, rift_tracker_config *config)
{
	const nx_json* nxj, *obj, *array;
	unsigned long length;
	char *json = NULL;
	int i;

	if (ohmd_get_config(ctx, CONFIG_FILE_KEY, &json, &length) != 0) {
		LOGI("Could not load room config.");
		return;
	}

	nxj = nx_json_parse (json, NULL);
	if (nxj == NULL)
		goto fail_parse;

	if (!json_read_vec3(nxj, "room-center-offset", &config->room_center_offset))
		goto fail_parse;

	array = nx_json_get(nxj, "sensors");
	if (array->type != NX_JSON_ARRAY)
		goto fail_parse;

	/* Sensors array */
	for (i = 0; i < RIFT_MAX_SENSORS; i++) {
		rift_tracker_sensor_config *sensor = config->sensors + i;
		const nx_json *serial;

		obj = nx_json_item(array, i);
		if (obj->type != NX_JSON_OBJECT)
			break;

		/* Serial number */
		serial = nx_json_get(obj, "serial");
		if (serial->type != NX_JSON_STRING)
			goto fail_parse;
		strncpy(sensor->serial_no, serial->text_value, RIFT_SENSOR_SERIAL_LEN);
		sensor->serial_no[RIFT_SENSOR_SERIAL_LEN] = '\0';

		/* Pose */
		if (!json_read_vec3(obj, "pos", &sensor->pose.pos))
			goto fail_parse;
		if (!json_read_quat(obj, "orient", &sensor->pose.orient))
			goto fail_parse;
	}
	config->n_sensors = i;

	config->modified = false;
	nx_json_free (nxj);
	free(json);
	return;

fail_parse:
	LOGW("Failed to parse room config data");
	nx_json_free (nxj);
	free(json);
}

static bool
rift_tracker_config_printf(char **dest_ptr, char *end, const char *fmt, ...)
{
	va_list args;
	char *dest = *dest_ptr;
	int would_write;

	va_start(args, fmt);
	would_write = vsnprintf(dest, end-dest, fmt, args);
	va_end(args);
	if (would_write < 0)
		return false;

	*dest_ptr += would_write;
	return true;
}

void rift_tracker_config_save(ohmd_context *ctx, rift_tracker_config *config)
{
	if (!config->modified)
		return; /* Don't save an unmodified configuration */

	char config_block[CONFIG_BLOCK_LEN+1];
	char *cur = config_block, *end = config_block + CONFIG_BLOCK_LEN;
	int i;

	if (!rift_tracker_config_printf(&cur, end, "{\n"))
		goto fail_serialise;

	if (!rift_tracker_config_printf(&cur, end, "  \"room-center-offset\": [ %f, %f, %f ],\n",
	    config->room_center_offset.x, config->room_center_offset.y, config->room_center_offset.z))
		goto fail_serialise;

	if (!rift_tracker_config_printf(&cur, end, "  \"sensors\": [\n"))
		goto fail_serialise;

	for (i = 0; i < config->n_sensors; i++) {
		rift_tracker_sensor_config *sensor = config->sensors + i;

		if (!rift_tracker_config_printf(&cur, end,
				"    {\n"
				"       \"serial\": \"%s\",\n"
				"       \"pos\": [ %f, %f, %f ],\n"
				"       \"orient\": [ %f, %f, %f, %f ]\n"
				"    }",
				sensor->serial_no,
				sensor->pose.pos.x, sensor->pose.pos.y, sensor->pose.pos.z,
				sensor->pose.orient.x, sensor->pose.orient.y, sensor->pose.orient.z, sensor->pose.orient.w))
			goto fail_serialise;

		/* Print trailing comma, or just end the line for the last entry */
		if (!rift_tracker_config_printf(&cur, end, "%s\n", (i+1 < config->n_sensors) ? "," : ""))
			goto fail_serialise;
	}

	if (!rift_tracker_config_printf(&cur, end, "  ]\n"))
		goto fail_serialise;

	if (!rift_tracker_config_printf(&cur, end, "}\n"))
		goto fail_serialise;

	if (ohmd_set_config(ctx, CONFIG_FILE_KEY, config_block, cur-config_block) != 0) {
		LOGI("Could not save room config.");
		return;
	}

	config->modified = false;
	return;

fail_serialise:
	LOGE("Failed to serialise room config data for output");
}

void
rift_tracker_config_set_sensor_pose(rift_tracker_config *config, const char *serial_no, posef *pose)
{
	int i;
	rift_tracker_sensor_config *sensor_cfg = NULL;

	assert(config->n_sensors <= RIFT_MAX_SENSORS);
	for (i = 0; i < config->n_sensors; i++) {
		rift_tracker_sensor_config *cur = config->sensors + i;
		if (strcmp(cur->serial_no, serial_no) == 0) {
			sensor_cfg = cur;
			break;
		}
	}

	if (sensor_cfg == NULL) {
		assert(config->n_sensors < RIFT_MAX_SENSORS);

		sensor_cfg = config->sensors + config->n_sensors;
		config->n_sensors++;

		strcpy(sensor_cfg->serial_no, serial_no);
	}

	sensor_cfg->pose = *pose;
	config->modified = true;
}
