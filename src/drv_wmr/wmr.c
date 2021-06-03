// Copyright 2018, Philipp Zabel.
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Windows Mixed Reality Driver */


#define FEATURE_BUFFER_SIZE 497

#define TICK_LEN (1.0f / 10000000.0f) // 1000 Hz ticks

#define MICROSOFT_VID        0x045e
#define HOLOLENS_SENSORS_PID 0x0659


#define HP_VID           0x03f0
#define REVERB_PID       0x0c6a
#define REVERB_G2_PID    0x0580

#include <string.h>
#include <wchar.h>
#include <hidapi.h>
#include <assert.h>
#include <limits.h>
#include <stdint.h>
#include <stdbool.h>

#include "wmr.h"
#include "config_key.h"

#include "../ext_deps/nxjson.h"

typedef struct wmr_priv wmr_priv;

static bool init_reverb(wmr_priv *priv);
static void deinit_reverb(wmr_priv *priv);

enum wmr_headset_type {
	WMR_HEADSET_GENERIC,
	WMR_HEADSET_REVERB_G1,
	WMR_HEADSET_REVERB_G2,
	WMR_HEADSET_SAMSUNG_800ZAA
};

struct wmr_headset_info {
	enum wmr_headset_type hmd_type;

	/* String by which we recognise the device */
	const char *dev_id_str;
	/* Friendly ID string for debug */
	const char *debug_name;

	/* VID/PID of the auxilliary control device, if any (0x0,0x0 if none) */
	unsigned short vid;
	unsigned short pid;

	bool (*init_func)(wmr_priv *priv);
	void (*deinit_func)(wmr_priv *priv);
};

const struct wmr_headset_info headset_map[] = {
		{ WMR_HEADSET_GENERIC, NULL, "Unknown WMR HMD", 0x0, 0x0, NULL, NULL }, /* Catch-all for unknown headsets */
		{ WMR_HEADSET_REVERB_G1, "HP Reverb VR Headset VR1000-2xxx", "HP Reverb", HP_VID, REVERB_PID, init_reverb, deinit_reverb },
		{ WMR_HEADSET_REVERB_G2, "HP Reverb Virtual Reality Headset G2", "HP Reverb G2", HP_VID, REVERB_G2_PID, init_reverb, deinit_reverb },
		{ WMR_HEADSET_SAMSUNG_800ZAA, "Samsung Windows Mixed Reality 800ZAA", "Samsung Odyssey", 0, 0, NULL, NULL },
};
const int headset_map_n = sizeof(headset_map) / sizeof(headset_map[0]);

struct wmr_priv {
	ohmd_device base;
	const struct wmr_headset_info *hmd_info;

	hid_device* hmd_imu;

	hid_device* hmd_aux;

	fusion sensor_fusion;
	vec3f raw_accel, raw_gyro;
	uint32_t last_ticks;
	uint8_t last_seq;
	hololens_sensors_packet sensor;
};

static void vec3f_from_hololens_gyro(int16_t smp[3][32], int i, vec3f* out_vec)
{
	out_vec->x = (float)(smp[0][8*i+0] +
			     smp[0][8*i+1] +
			     smp[0][8*i+2] +
			     smp[0][8*i+3] +
			     smp[0][8*i+4] +
			     smp[0][8*i+5] +
			     smp[0][8*i+6] +
			     smp[0][8*i+7]) * 0.001f * -0.125f * -1.0f;
	out_vec->y = (float)(smp[1][8*i+0] +
			     smp[1][8*i+1] +
			     smp[1][8*i+2] +
			     smp[1][8*i+3] +
			     smp[1][8*i+4] +
			     smp[1][8*i+5] +
			     smp[1][8*i+6] +
			     smp[1][8*i+7]) * 0.001f * -0.125f;
	out_vec->z = (float)(smp[2][8*i+0] +
			     smp[2][8*i+1] +
			     smp[2][8*i+2] +
			     smp[2][8*i+3] +
			     smp[2][8*i+4] +
			     smp[2][8*i+5] +
			     smp[2][8*i+6] +
			     smp[2][8*i+7]) * 0.001f * -0.125f;
}

static void vec3f_from_hololens_accel(int32_t smp[3][4], int i, vec3f* out_vec)
{
	out_vec->x = (float)smp[0][i] * 0.001f * -1.0f;
	out_vec->y = (float)smp[1][i] * 0.001f * -1.0f;
	out_vec->z = (float)smp[2][i] * 0.001f * -1.0f;
}

static void handle_tracker_sensor_msg(wmr_priv* priv, unsigned char* buffer, int size)
{
	uint64_t last_sample_tick = priv->sensor.gyro_timestamp[3];

	if(!hololens_sensors_decode_packet(&priv->sensor, buffer, size)){
		LOGE("couldn't decode tracker sensor message");
	}

	hololens_sensors_packet* s = &priv->sensor;


	vec3f mag = {{0.0f, 0.0f, 0.0f}};

	for(int i = 0; i < 4; i++){
		uint64_t tick_delta = 1000;
		if(last_sample_tick > 0) //startup correction
			tick_delta = s->gyro_timestamp[i] - last_sample_tick;

		float dt = tick_delta * TICK_LEN;

		vec3f_from_hololens_gyro(s->gyro, i, &priv->raw_gyro);
		vec3f_from_hololens_accel(s->accel, i, &priv->raw_accel);

		ofusion_update(&priv->sensor_fusion, dt, &priv->raw_gyro, &priv->raw_accel, &mag);

		last_sample_tick = s->gyro_timestamp[i];
	}
}

static void update_device(ohmd_device* device)
{
	wmr_priv* priv = (wmr_priv*)device;

	unsigned char buffer[FEATURE_BUFFER_SIZE];

	while(true){
		int size = hid_read(priv->hmd_imu, buffer, FEATURE_BUFFER_SIZE);
		if(size < 0){
			LOGE("error reading from device");
			return;
		} else if(size == 0) {
			return; // No more messages, return.
		}

		// currently the only message type the hardware supports (I think)
		if(buffer[0] == HOLOLENS_IRQ_SENSORS){
			handle_tracker_sensor_msg(priv, buffer, size);
		}else if(buffer[0] != HOLOLENS_IRQ_DEBUG &&
			 buffer[0] != HOLOLENS_IRQ_UNKNOWN_23){
			LOGE("unknown message type: %u", buffer[0]);
		}
	}
}

static int getf(ohmd_device* device, ohmd_float_value type, float* out)
{
	wmr_priv* priv = (wmr_priv*)device;

	switch(type){
	case OHMD_ROTATION_QUAT:
		*(quatf*)out = priv->sensor_fusion.orient;
		break;

	case OHMD_POSITION_VECTOR:
		out[0] = out[1] = out[2] = 0;
		break;

	case OHMD_DISTORTION_K:
		// TODO this should be set to the equivalent of no distortion
		memset(out, 0, sizeof(float) * 6);
		break;

	default:
		ohmd_set_error(priv->base.ctx, "invalid type given to getf (%ud)", type);
		return -1;
		break;
	}

	return 0;
}

static void close_device(ohmd_device* device)
{
	wmr_priv* priv = (wmr_priv*)device;

	LOGD("closing Microsoft HoloLens Sensors device");

	hid_close(priv->hmd_imu);

	if (priv->hmd_aux) {
		if (priv->hmd_info->deinit_func) {
			priv->hmd_info->deinit_func(priv);
		}
		hid_close(priv->hmd_aux);
	}

	free(device);
}

static hid_device* open_device_idx(int manufacturer, int product, int iface, int iface_tot, int device_index)
{
	struct hid_device_info* devs = hid_enumerate(manufacturer, product);
	struct hid_device_info* cur_dev = devs;

	int idx = 0;
	int iface_cur = 0;
	hid_device* ret = NULL;

	while (cur_dev) {
		LOGI("%04x:%04x %s\n", manufacturer, product, cur_dev->path);

		if(idx == device_index && iface == iface_cur){
			ret = hid_open_path(cur_dev->path);
			LOGI("opening\n");
		}

		cur_dev = cur_dev->next;

		iface_cur++;

		if(iface_cur >= iface_tot){
			idx++;
			iface_cur = 0;
		}
	}

	hid_free_enumeration(devs);

	return ret;
}

static int config_command_sync(hid_device* hmd_imu, unsigned char type,
			       unsigned char* buf, int len)
{
	unsigned char cmd[64] = { 0x02, type };

	hid_write(hmd_imu, cmd, sizeof(cmd));
	do {
		int size = hid_read(hmd_imu, buf, len);
		if (size == -1)
			return -1;
		if (buf[0] == HOLOLENS_IRQ_CONTROL)
			return size;
	} while (buf[0] == HOLOLENS_IRQ_SENSORS ||
		 buf[0] == HOLOLENS_IRQ_DEBUG ||
		 buf[0] == HOLOLENS_IRQ_UNKNOWN_23);

	return -1;
}

int read_config_part(wmr_priv *priv, unsigned char type,
		     unsigned char *data, int len)
{
	unsigned char buf[33];
	int offset = 0;
	int size;
	int retries = 0;

	retries = 0;
	do {
		size = config_command_sync(priv->hmd_imu, 0x0b, buf, sizeof(buf));
		if (size != 33 || buf[0] != 0x02) {
			if (retries++ < 5) {
				ohmd_sleep(0.01);
				continue;
			}

			LOGE("Failed to issue command 0b: %02x %02x %02x\n",
			    buf[0], buf[1], buf[2]);
			return -1;
		}
		else {
			break;
		}
	} while (true);

	retries = 0;
	do {
		size = config_command_sync(priv->hmd_imu, type, buf, sizeof(buf));
		if (size != 33 || buf[0] != 0x02) {
			if (retries++ < 5) {
				ohmd_sleep(0.01);
				continue;
			}
			LOGE("Failed to issue command %02x: %02x %02x %02x\n", type,
			    buf[0], buf[1], buf[2]);
			return -1;
		}
		else {
			break;
		}
	} while (true);

	retries = 0;
	for (;;) {
		size = config_command_sync(priv->hmd_imu, 0x08, buf, sizeof(buf));
		if (size != 33 || (buf[1] != 0x01 && buf[1] != 0x02)) {
			if (retries++ < 10) {
				ohmd_sleep(0.01);
				continue;
			}
			LOGE("Failed to issue command 08: %02x %02x %02x, size %d\n",
			    buf[0], buf[1], buf[2], size);
			return -1;
		}
		if (buf[1] != 0x01)
			break;
		if (buf[2] > len || offset + buf[2] > len) {
			LOGE("Getting more information than requested\n");
			return -1;
		}
		memcpy(data + offset, buf + 3, buf[2]);
		offset += buf[2];
		retries = 0;
	}

	return offset;
}

void decrypt_config(unsigned char* config)
{
	wmr_config_header* hdr = (wmr_config_header*)config;
	for (int i = 0; i < hdr->json_size - sizeof(uint16_t); i++)
	{
		config[hdr->json_start + sizeof(uint16_t) + i] ^= wmr_config_key[i % sizeof(wmr_config_key)];
	}
}

unsigned char *read_config(wmr_priv *priv)
{
	unsigned char meta[84];
	unsigned char *data;
	int size, data_size;

	size = read_config_part(priv, 0x06, meta, sizeof(meta));

	if (size == -1)
		return NULL;

	/*
	 * No idea what the other 64 bytes of metadata are, but the first two
	 * seem to be little endian size of the data store.
	 */
	data_size = meta[0] | (meta[1] << 8);
	data = calloc(1, data_size);
	if (!data)
                return NULL;

	size = read_config_part(priv, 0x04, data, data_size);
	if (size == -1) {
		free(data);
		return NULL;
	}

	decrypt_config(data);

	LOGI("Read %d-byte config data\n", data_size);

	return data;
}


void process_nxjson_obj(const nx_json* node, const nx_json* (*list)[32], char* match)
{
	if (!node)
		return;

	if (node->key)
		if (strcmp(match,node->key) == 0) 
		{
			//LOGE("Found key %s\n", node->key);
			for (int i = 0; i < 32; i++)
			{
				if (!list[0][i]) {
					list[0][i] = node;
					break;
				}
			}
		}

	process_nxjson_obj(node->next, list, match);
	process_nxjson_obj(node->child, list, match);
}

void resetList(const nx_json* (*list)[32])
{
	memset(list, 0, sizeof(*list));
}

static void deinit_reverb(wmr_priv *priv) {
	hid_device *hid = priv->hmd_aux;

	/* Turn the screen off */
	unsigned char cmd_2[2] = { 0x04, 0x00 };
	hid_send_feature_report(hid, cmd_2, sizeof(cmd_2));
}

static bool init_reverb(wmr_priv *priv) {
	LOGI("Starting %s initialization sequence.", priv->hmd_info->debug_name);

	hid_device* hid = hid_open(priv->hmd_info->vid, priv->hmd_info->pid, NULL);
	if (hid == NULL) {
		LOGE("Failed to open the %s control device. Check USB permissions", priv->hmd_info->debug_name);
		return false;
	}
	priv->hmd_aux = hid;

	// sleep before we start seems to improve reliability
	// 300ms is what Windows seems to do, so cargo cult that.
	ohmd_sleep(0.3);

	unsigned char cmd[64] = { 0x50, 0x01, };
	for (int i = 0; i<4; i++) {
		hid_send_feature_report(hid, cmd, sizeof(cmd));
		unsigned char data[64] = { 0x50 };
		hid_get_feature_report(hid, data, sizeof(data));
		ohmd_sleep(0.01); // Sleep 10ms
	}
	unsigned char data[64] = { 0x09 };
	hid_get_feature_report(hid, data, sizeof(data));
	data[0] = 0x08;
	hid_get_feature_report(hid, data, sizeof(data));
	data[0] = 0x06;
	hid_get_feature_report(hid, data, sizeof(data));

	unsigned char cmd_2[2] = { 0x04, 0x01 };
	hid_send_feature_report(hid, cmd_2, sizeof(cmd_2));

	LOGI("Initialized %s", priv->hmd_info->debug_name);
	return true;
}

static ohmd_device* open_device(ohmd_driver* driver, ohmd_device_desc* desc)
{
	wmr_priv* priv = ohmd_alloc(driver->ctx, sizeof(wmr_priv));
	unsigned char *config;

	if(!priv)
		return NULL;

	priv->base.ctx = driver->ctx;

	/* Default to the GENERIC WMR headset entry: */
	priv->hmd_info = &headset_map[0];

	int idx = atoi(desc->path);

	// Open the HMD device
	priv->hmd_imu = open_device_idx(MICROSOFT_VID, HOLOLENS_SENSORS_PID, 0, 1, idx);

	if(!priv->hmd_imu)
		goto cleanup;

	//Bunch of temp variables to set to the display configs
	int resolution_h = 1440, resolution_v = 1440;

	/* Read and parse the config block */
	config = read_config(priv);
	if (config) {
		wmr_config_header* hdr = (wmr_config_header*)config;
		int i;

		LOGI("Model name: %.64s\n", hdr->name);

		/* Iterate the headset map for a match (skipping the 0th generic entry) */
		for (i = 1; i < headset_map_n; i++) {
			const struct wmr_headset_info *cur = &headset_map[i];
			if (cur->dev_id_str && strncmp(hdr->name, cur->dev_id_str, 64) == 0) {
				priv->hmd_info = cur;
				break;
			}
		}

		char *json_data = (char*)config + hdr->json_start + sizeof(uint16_t);
		const nx_json* json = nx_json_parse(json_data, 0);

		if (json->type != NX_JSON_NULL) 
		{
			//list to save found nodes with matching name
			const nx_json* returnlist[32] = {0};
			resetList(&returnlist); process_nxjson_obj(json, &returnlist, "DisplayHeight");
			LOGE("Found display height %lli\n", returnlist[0]->int_value); //taking the first element since it does not matter if you take display 0 or 1
			resolution_v = returnlist[0]->int_value;
			resetList(&returnlist); process_nxjson_obj(json, &returnlist, "DisplayWidth");
			LOGE("Found display width %lli\n", returnlist[0]->int_value); //taking the first element since it does not matter if you take display 0 or 1
			resolution_h = returnlist[0]->int_value;
			
			//Left in for debugging until we confirmed most variables working
			/*
		 	for (int i = 0; i < 32; i++)
		 	{
		 		if (returnlist[i] != 0)
		 		{
		 			if (returnlist[i]->type == NX_JSON_STRING)
		 				printf("Found %s\n", returnlist[i]->text_value);
		 			if (returnlist[i]->type == NX_JSON_INTEGER)
		 				printf("Found %lli\n", returnlist[i]->int_value);
		 			if (returnlist[i]->type == NX_JSON_DOUBLE)
		 				printf("Found %f\n", returnlist[i]->dbl_value);
		 			if (returnlist[i]->type == NX_JSON_ARRAY)
		 				printf("Found array, TODO\n");
		 		}
		 	}*/

		}
		else 
		{
			LOGE("Could not parse json\n");
		}

		//TODO: use new config data

		nx_json_free(json);

		free(config);
	}
	else {
		LOGE("Could not read config from the firmware\n");
	}

	if(hid_set_nonblocking(priv->hmd_imu, 1) == -1){
		ohmd_set_error(driver->ctx, "failed to set non-blocking on device");
		goto cleanup;
	}

	// turn the IMU on
	hid_write(priv->hmd_imu, hololens_sensors_imu_on, sizeof(hololens_sensors_imu_on));

	// Set default device properties
	ohmd_set_default_device_properties(&priv->base.properties);

	LOGI("Detected WMR headset: %s", priv->hmd_info->debug_name);
	if (priv->hmd_info->init_func && !priv->hmd_info->init_func(priv)) {
		ohmd_set_error(driver->ctx, "Failed to initialise %s headset", priv->hmd_info->debug_name);
		goto cleanup;
	}

	// Set device properties
	switch (priv->hmd_info->hmd_type) {
		case WMR_HEADSET_GENERIC:
			// Most Windows Mixed Reality Headsets have two 2.89" 1440x1440 LCDs
			priv->base.properties.hsize = 0.103812f;
			priv->base.properties.vsize = 0.051905f;
			priv->base.properties.hres = resolution_h;
			priv->base.properties.vres = resolution_v;
			priv->base.properties.lens_sep = 0.063f; /* FIXME */
			priv->base.properties.lens_vpos = 0.025953f; /* FIXME */
			priv->base.properties.fov = DEG_TO_RAD(95.0f);
			priv->base.properties.ratio = 1.0f;
			break;
		case WMR_HEADSET_REVERB_G1:
		case WMR_HEADSET_REVERB_G2:
			// Settings for Reverb G2 based on Windows Mixed Reality settings.
			priv->base.properties.hsize = 0.103812f;
			priv->base.properties.vsize = 0.051905f;
			priv->base.properties.hres = resolution_h;
			priv->base.properties.vres = resolution_v;
			priv->base.properties.lens_sep = 0.063f; /* FIXME */
			priv->base.properties.lens_vpos = 0.025953f; /* FIXME */
			priv->base.properties.fov = DEG_TO_RAD(114.0f); /* Got this value from HP's website */
			priv->base.properties.ratio = 1.0f;
			break;
		case WMR_HEADSET_SAMSUNG_800ZAA:
			// Samsung Odyssey has two 3.5" 1440x1600 OLED displays.
			priv->base.properties.hsize = 0.118942f;
			priv->base.properties.vsize = 0.066079f;
			priv->base.properties.hres = resolution_h;
			priv->base.properties.vres = resolution_v;
			priv->base.properties.lens_sep = 0.063f; /* FIXME */
			priv->base.properties.lens_vpos = 0.03304f; /* FIXME */
			priv->base.properties.fov = DEG_TO_RAD(110.0f);
			priv->base.properties.ratio = 0.9f;
			break;
	}

	// calculate projection eye projection matrices from the device properties
	ohmd_calc_default_proj_matrices(&priv->base.properties);

	// set up device callbacks
	priv->base.update = update_device;
	priv->base.close = close_device;
	priv->base.getf = getf;

	ofusion_init(&priv->sensor_fusion);

	return (ohmd_device*)priv;

cleanup:
	if(priv)
		free(priv);

	return NULL;
}

static void get_device_list(ohmd_driver* driver, ohmd_device_list* list)
{
	struct hid_device_info* devs = hid_enumerate(MICROSOFT_VID, HOLOLENS_SENSORS_PID);
	struct hid_device_info* cur_dev = devs;

	int idx = 0;
	while (cur_dev) {
		ohmd_device_desc* desc = &list->devices[list->num_devices++];

		strcpy(desc->driver, "OpenHMD Windows Mixed Reality Driver");
		strcpy(desc->vendor, "Microsoft");
		strcpy(desc->product, "HoloLens Sensors");

		desc->revision = 0;

		snprintf(desc->path, OHMD_STR_SIZE, "%d", idx);

		desc->driver_ptr = driver;

		desc->device_class = OHMD_DEVICE_CLASS_HMD;
		desc->device_flags = OHMD_DEVICE_FLAGS_ROTATIONAL_TRACKING;

		cur_dev = cur_dev->next;
		idx++;
	}

	hid_free_enumeration(devs);
}

static void destroy_driver(ohmd_driver* drv)
{
	LOGD("shutting down Windows Mixed Reality driver");
	free(drv);
}

ohmd_driver* ohmd_create_wmr_drv(ohmd_context* ctx)
{
	ohmd_driver* drv = ohmd_alloc(ctx, sizeof(ohmd_driver));

	if(!drv)
		return NULL;

	drv->get_device_list = get_device_list;
	drv->open_device = open_device;
	drv->destroy = destroy_driver;
	drv->ctx = ctx;

	return drv;
}
