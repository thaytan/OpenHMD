/*
 * Oculus Rift CV1 Radio
 * Copyright 2016 Philipp Zabel
 * Copyright 2019 Jan Schmidt
 * SPDX-License-Identifier:	BSL-1.0
 */
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include "rift-hmd-radio.h"
#include "../ext_deps/nxjson.h"

static int get_feature_report(hid_device *handle, rift_sensor_feature_cmd cmd, unsigned char* buf)
{
	memset(buf, 0, FEATURE_BUFFER_SIZE);
	buf[0] = (unsigned char)cmd;
	return hid_get_feature_report(handle, buf, FEATURE_BUFFER_SIZE);
}

static int send_feature_report(hid_device *handle, unsigned char* buf, int length)
{
	return hid_send_feature_report(handle, buf, length);
}

static int rift_hmd_radio_get_cmd_response(hid_device *handle, bool wait)
{
	unsigned char buffer[FEATURE_BUFFER_SIZE];
	int ret_size;

	do {
		ret_size = get_feature_report(handle, RIFT_CMD_RADIO_CONTROL, buffer);
		if (ret_size < 1) {
			LOGE("HMD radio command failed - response too small");
			return -EIO;
		}

		/* If this isn't a blocking wait, return EAGAIN */
		if (!wait && (buffer[3] & 0x80)) {
			return -EINPROGRESS;
		}
	} while (buffer[3] & 0x80);

	/* 0x08 means the device isn't responding */
	if (buffer[3] & 0x08)
		return -ETIMEDOUT;

	return 0;
}

static int rift_hmd_radio_send_cmd(hid_device *handle, uint8_t a, uint8_t b, uint8_t c)
{
	unsigned char buffer[FEATURE_BUFFER_SIZE];
	int cmd_size = encode_radio_control_cmd(buffer, a, b, c);
	int ret_size;

	ret_size = send_feature_report(handle, buffer, cmd_size);
	return ret_size;

}

static int rift_radio_read_flash(rift_hmd_radio_state *radio, uint8_t device_type,
				uint16_t offset, uint16_t length, uint8_t *flash_data)
{
	int ret;
	unsigned char buffer[FEATURE_BUFFER_SIZE];
	int cmd_size;

	if (!radio->command_result_pending) {
		LOGV("Reading FW calibration block for controller %d at offset %u", device_type, offset);
		cmd_size = encode_radio_data_read_cmd(buffer, offset, length);
		if ((ret = send_feature_report(radio->handle, buffer, cmd_size)) < 0)
			goto done;

		if ((ret = rift_hmd_radio_send_cmd (radio->handle, 0x03, RIFT_HMD_RADIO_READ_FLASH_CONTROL,
		    device_type)) < 0)
			goto done;
	}

	radio->command_result_pending = false;
	if ((ret = rift_hmd_radio_get_cmd_response(radio->handle, false)) < 0) {
		if (ret == -EINPROGRESS) {
			radio->command_result_pending = true;
		}
		goto done;
	}

	ret = get_feature_report(radio->handle, RIFT_CMD_RADIO_READ_DATA, buffer);
	if (ret < 0)
		goto done;

	memcpy (flash_data, buffer+7, length);

done:
	return ret;
}

static int rift_radio_write_complete(rift_hmd_radio_state *radio)
{
	int ret;

	radio->command_result_pending = false;
	if ((ret = rift_hmd_radio_get_cmd_response(radio->handle, false)) < 0) {
		if (ret == -EINPROGRESS) {
			radio->command_result_pending = true;
		}
	}

	return ret;
}

static int rift_radio_write(rift_hmd_radio_state *radio, uint8_t cmd_byte1, uint8_t cmd_byte2, uint8_t device_type,
	unsigned char *buf, int length)
{
	int ret;

	if (buf[0] != RIFT_CMD_RADIO_READ_DATA)
		return -EINVAL;

	if (!radio->command_result_pending) {
		ret = send_feature_report(radio->handle, buf, length);
		if (ret < 0)
			return ret;

		if ((ret = rift_hmd_radio_send_cmd(radio->handle, cmd_byte1, cmd_byte2, device_type)) < 0)
			goto done;
	}

	ret = rift_radio_write_complete(radio);
done:
	return ret;
}


static int rift_radio_read_calibration_hash(rift_hmd_radio_state *radio, uint8_t device_type,
					    uint8_t hash[16])
{
	if (radio->cur_read_cmd == RIFT_RADIO_CMD_NONE) {
		LOGV("Starting FW hash read for controller %d\n", device_type);
		radio->cur_read_cmd = RIFT_FW_READ_CMD_CALIBRATION_HASH;
	}
	return rift_radio_read_flash(radio, device_type, 0x1bf0, 16, hash);
}

static int rift_radio_read_calibration(rift_hmd_radio_state *radio, uint8_t device_type,
		char **json_out, uint16_t *length)
{
	int ret;

	/* First, read the flash header to get the block size */
	if (radio->cur_read_cmd == RIFT_RADIO_CMD_NONE ||
	    radio->cur_read_cmd == RIFT_FW_READ_CMD_CALIBRATION_HDR) {
		uint16_t json_length;
		uint8_t flash_data[20];

		if (radio->cur_read_cmd == RIFT_RADIO_CMD_NONE) {
			LOGV("Starting FW calibration hdr read for controller %d\n", device_type);
			radio->cur_read_cmd = RIFT_FW_READ_CMD_CALIBRATION_HDR;
		}

		ret = rift_radio_read_flash(radio, device_type, 0, 20, flash_data);
		if (ret < 0) {
			if (ret != -EINPROGRESS)
				radio->cur_read_cmd = RIFT_RADIO_CMD_NONE;
			return ret;
		}

		if (flash_data[0] != 1 || flash_data[1] != 0) {
			radio->cur_read_cmd = RIFT_RADIO_CMD_NONE;
			return -1; /* Invalid data */
		}
		json_length = (flash_data[3] << 8) | flash_data[2];

		if (radio->read_block_alloc < json_length + 1) {
			radio->read_block = realloc(radio->read_block, json_length + 1);
			radio->read_block_alloc = json_length + 1;
		}

		radio->read_data_offset = 20;
		radio->read_data_size = json_length;

		memcpy(radio->read_block, flash_data + 4, 16);

		LOGV("Starting FW calibration block read for controller %d\n", device_type);
		radio->cur_read_cmd = RIFT_FW_READ_CMD_CALIBRATION;
	}

	for (; radio->read_data_offset < radio->read_data_size + 4; radio->read_data_offset += 20) {
		uint16_t offset = radio->read_data_offset;
		uint16_t json_offset = offset - 4;

		ret = rift_radio_read_flash(radio, device_type, offset,
				OHMD_MIN (20, radio->read_data_size - json_offset),
				radio->read_block + json_offset);
		if (ret < 0) {
			if (ret != -EINPROGRESS)
				radio->cur_read_cmd = RIFT_RADIO_CMD_NONE;
			return ret;
		}

	}

	*json_out = (char *) radio->read_block;
	*length = radio->read_data_size;
	radio->cur_read_cmd = RIFT_RADIO_CMD_NONE;

	return 0;
}

static bool json_read_vec3(const nx_json *nxj, const char *key, vec3f *out)
{
	const nx_json *member = nx_json_get (nxj, key);

	if (member->type != NX_JSON_ARRAY)
		return false;

	out->x = nx_json_item (member, 0)->dbl_value;
	out->y = nx_json_item (member, 1)->dbl_value;
	out->z = nx_json_item (member, 2)->dbl_value;

	return true;
}

static int rift_touch_parse_calibration(char *json_in,
		rift_touch_calibration *c)
{
	const nx_json* nxj, *obj, *version, *array;
	int version_number = -1;
	unsigned int i;
	/* Operate on a copy of the json, so we can print it
	 * an error if invalid, as nx_json modifies the input string */
	char *json = malloc (strlen (json_in) + 1);
	strcpy (json, json_in);

	nxj = nx_json_parse (json, NULL);
	if (nxj == NULL)
		return -1;

	obj = nx_json_get(nxj, "TrackedObject");
	if (obj->type == NX_JSON_NULL)
		goto fail_version;

	version = nx_json_get (obj, "JsonVersion");
	if (version->type != NX_JSON_INTEGER)
		goto fail_version;
	version_number = version->int_value;
	if (version_number != 2)
		goto fail_version;

	if (!json_read_vec3 (obj, "ImuPosition", &c->imu_position))
		goto fail_imu;

	c->joy_x_range_min = nx_json_get (obj, "JoyXRangeMin")->int_value;
	c->joy_x_range_max = nx_json_get (obj, "JoyXRangeMax")->int_value;
	c->joy_x_dead_min = nx_json_get (obj, "JoyXDeadMin")->int_value;
	c->joy_x_dead_max = nx_json_get (obj, "JoyXDeadMax")->int_value;
	c->joy_y_range_min = nx_json_get (obj, "JoyYRangeMin")->int_value;
	c->joy_y_range_max = nx_json_get (obj, "JoyYRangeMax")->int_value;
	c->joy_y_dead_min = nx_json_get (obj, "JoyYDeadMin")->int_value;
	c->joy_y_dead_max = nx_json_get (obj, "JoyYDeadMax")->int_value;

	c->trigger_min_range = nx_json_get (obj, "TriggerMinRange")->int_value;
	c->trigger_mid_range = nx_json_get (obj, "TriggerMidRange")->int_value;
	c->trigger_max_range = nx_json_get (obj, "TriggerMaxRange")->int_value;

	/* The calibration matrices are 12 values, a 3x3 rotation
	 * matrix, then 4 factory calibrated offset values */
	array = nx_json_get (obj, "GyroCalibration");
	for (i = 0; i < 9; i++)
		c->gyro_matrix[i/3][i%3] = nx_json_item (array, i)->dbl_value;
	for (i = 0; i < 3; i++)
		c->gyro_offset.arr[i] = nx_json_item (array, i + 9)->dbl_value;

	array = nx_json_get (obj, "AccCalibration");
	for (i = 0; i < 9; i++)
		c->accel_matrix[i/3][i%3] = nx_json_item (array, i)->dbl_value;
	for (i = 0; i < 3; i++)
		c->accel_offset.arr[i] = nx_json_item (array, i + 9)->dbl_value;

	c->middle_min_range = nx_json_get (obj, "MiddleMinRange")->int_value;
	c->middle_mid_range = nx_json_get (obj, "MiddleMidRange")->int_value;
	c->middle_max_range = nx_json_get (obj, "MiddleMaxRange")->int_value;

	c->middle_flipped = nx_json_get (obj, "MiddleFlipped")->int_value;

	array = nx_json_get (obj, "CapSenseMin");
	for (i = 0; i < 8; i++)
		c->cap_sense_min[i] = nx_json_item (array, i)->int_value;

	array = nx_json_get (obj, "CapSenseTouch");
	for (i = 0; i < 8; i++)
		c->cap_sense_touch[i] = nx_json_item (array, i)->int_value;

	/* LED model points */
	const nx_json *led_model = nx_json_get (obj, "ModelPoints");
	if (led_model->type != NX_JSON_OBJECT) {
		goto fail_leds;
	}

	if (led_model->length > 255) {
		LOGE("Too many LEDs in the controller mode - %d > 255", led_model->length);
		goto fail_leds;
	}

	rift_leds_init (&c->leds, led_model->length);

	for (i = 0; i < c->leds.num_points; i++) {
		rift_led *led = &c->leds.points[i];
		const nx_json* point[6];
		char name[9];
		int j;

		snprintf(name, 9, "Point%d", i);
		array = nx_json_get (led_model, name);
		if (array->type != NX_JSON_ARRAY) {
			goto fail_leds;
		}

		for (j = 0; j < 6; j++)
			point[j] = nx_json_item (array, j);

		led->pos.x = point[0]->dbl_value;
		led->pos.y = point[1]->dbl_value;
		led->pos.z = point[2]->dbl_value;
		led->dir.x = point[3]->dbl_value;
		led->dir.y = point[4]->dbl_value;
		led->dir.z = point[5]->dbl_value;
		led->pattern = 0xff;
	}

	free (json);
	nx_json_free (nxj);
	return 0;
fail_version:
	LOGW ("Unrecognised Touch Controller JSON data version %d\n%s\n", version_number, json);
	goto fail;
fail_imu:
	LOGW ("Unrecognised Touch Controller JSON data. Invalid IMU data\n%s\n", json);
	goto fail;
fail_leds:
	LOGW ("Unrecognised Touch Controller JSON data. Invalid LED data\n%s\n", json);
	goto fail;
fail:
	free (json);
	nx_json_free (nxj);
	return -1;
}

static void
make_calibration_hash_key(rift_hmd_radio_state *radio, char *config_block_key)
{
	char hash_hex[64], *cur;
	int i;

	for (i = 0, cur = hash_hex; i < 16; i++) {
		sprintf(cur, "%02x", radio->calibration_hash[i]);
		cur += 2;
	}
	snprintf(config_block_key, 255, "rift-touch-config-%s.bin", hash_hex);
	config_block_key[255] = '0';
}

int rift_touch_get_calibration(ohmd_context* ctx, rift_hmd_radio_state *radio,
	int device_id, rift_touch_calibration *calibration)
{
	unsigned long length;
	char *json = NULL;
	int ret = -1;

	if (radio->cur_read_cmd != RIFT_RADIO_CMD_NONE && radio->device_in_progress != device_id)
		return -EBUSY; /* Another device is being read right now */

	if (radio->cur_read_cmd != RIFT_RADIO_CMD_NONE &&
	    radio->cur_read_cmd != RIFT_FW_READ_CMD_CALIBRATION_HASH &&
	    radio->cur_read_cmd != RIFT_FW_READ_CMD_CALIBRATION_HDR &&
	    radio->cur_read_cmd != RIFT_FW_READ_CMD_CALIBRATION)
		return -EBUSY; /* A different command is in progress */

	radio->device_in_progress = device_id;

	if (radio->cur_read_cmd == RIFT_RADIO_CMD_NONE ||
	    radio->cur_read_cmd == RIFT_FW_READ_CMD_CALIBRATION_HASH) {
		/* If the controller isn't on yet, we might fail to read the calibration data */
		ret = rift_radio_read_calibration_hash(radio, device_id, radio->calibration_hash);
		if (ret < 0) {
			if (ret != -EINPROGRESS) {
				LOGV ("Failed to read calibration hash from device %d", device_id);
			}
			return ret;
		}
		radio->cur_read_cmd = RIFT_RADIO_CMD_NONE;
	}

	/* Use the calibration hash to try and read the block from our
	 * config cache */
	bool had_config_cache = false;
	if (radio->cur_read_cmd == RIFT_RADIO_CMD_NONE) {
		char config_block_key[256];
		make_calibration_hash_key(radio, config_block_key);
		if (ohmd_get_config(ctx, config_block_key, &json, &length) == 0) {
			LOGD("Loaded cached touch controller calibration file %s", config_block_key);
			had_config_cache = true;
		}
	}

	if (radio->cur_read_cmd == RIFT_FW_READ_CMD_CALIBRATION_HASH || !had_config_cache) {
		uint16_t short_len;
		ret = rift_radio_read_calibration(radio, device_id, &json, &short_len);
		if (ret < 0)
			return ret;
		length = (unsigned long) short_len;
	}

	if (rift_touch_parse_calibration(json, calibration) < 0)
		return ret;

	if (had_config_cache) {
		free(json);
	} else {
		char config_block_key[256];
		make_calibration_hash_key(radio, config_block_key);

		/* Store the freshly read config block.
		 * We don't need to free on this branch - our json block is in the radio read store */
		LOGD("Storing touch controller calibration cache file %s", config_block_key);
		ret = ohmd_set_config(ctx, config_block_key, json, length);
		if (ret < 0)
			return ret;
	}

	char control_name[20];
	snprintf (control_name, 20, "Controller %u", device_id);
	rift_leds_dump (&calibration->leds, control_name);

	radio->device_in_progress = -1;
	radio->cur_read_cmd = RIFT_RADIO_CMD_NONE;

	return 0;
}

void
rift_touch_clear_calibration(rift_touch_calibration *calibration)
{
	rift_leds_clear (&calibration->leds);
}

void rift_hmd_radio_init(rift_hmd_radio_state *radio, hid_device *handle)
{
	radio->handle = handle;
}

void rift_hmd_radio_clear(rift_hmd_radio_state *radio)
{
	if (radio->read_block) {
		free(radio->read_block);
		radio->read_block = NULL;
		radio->read_block_alloc = 0;
	}
}

bool rift_hmd_radio_get_address(hid_device *handle, uint8_t radio_address[5])
{
	unsigned char buf[FEATURE_BUFFER_SIZE];
	int ret_size;

	ret_size = rift_hmd_radio_send_cmd (handle, 0x05, 0x03, 0x05);
	if (ret_size < 0)
		return false;

	if ((ret_size = rift_hmd_radio_get_cmd_response(handle, true)) < 0)
		return false;

	ret_size = get_feature_report(handle, RIFT_CMD_RADIO_READ_DATA, buf);
	if (ret_size < 0)
		return false;

	if (!decode_radio_address (radio_address, buf, ret_size)) {
		LOGE("Failed to decode received radio address");
		return false;
	}

	return true;
}

int rift_touch_send_haptics(rift_hmd_radio_state *radio, int device_id, bool low_freq, uint8_t amplitude)
{
	int ret = -1;

	if (radio->cur_read_cmd != RIFT_RADIO_CMD_NONE && radio->device_in_progress != device_id)
		return -EBUSY; /* Another device is being read right now */

	if (radio->cur_read_cmd != RIFT_RADIO_CMD_NONE &&
	    radio->cur_read_cmd != RIFT_RADIO_WRITE_CMD_HAPTICS)
		return -EBUSY; /* A different command is in progress */

	radio->device_in_progress = device_id;

	if (radio->cur_read_cmd == RIFT_RADIO_CMD_NONE) {
		/* Prepare command buffer then dispatch and await */
		unsigned char buf[31];
		int buf_size = 31;

		memset(buf, 0, sizeof(buf));
		if (amplitude != 0) {
			if (low_freq)
				buf[3] = 0xa0; /* 160Hz vibration */
			else
				buf[4] = 0xa0; /* 320Hz vibration */
			buf[7] = amplitude; /* Intensity byte */
		}

		buf[0] = RIFT_CMD_RADIO_READ_DATA;

		radio->cur_read_cmd = RIFT_RADIO_WRITE_CMD_HAPTICS;
		ret = rift_radio_write(radio, 0x02, 0x03, device_id, buf, buf_size);
	}
	else {
		/* Just waiting for command result */
		ret = rift_radio_write_complete(radio);
	}

	if (ret < 0) {
		return ret;
	}

	radio->device_in_progress = -1;
	radio->cur_read_cmd = RIFT_RADIO_CMD_NONE;

	return 0;
}

void rift_touch_cancel_in_progress(rift_hmd_radio_state *radio, int device_id)
{
	if (radio->cur_read_cmd != RIFT_RADIO_CMD_NONE && radio->device_in_progress != device_id)
		return; /* Another device is being read right now */

	radio->device_in_progress = -1;
	radio->cur_read_cmd = RIFT_RADIO_CMD_NONE;
}
