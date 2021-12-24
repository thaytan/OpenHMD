#include <errno.h>
#include <libusb.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../../platform.h"
#include "esp770u.h"
#include "uvc.h"

#define ESP770U_EXTENSION_UNIT		4

#define ESP770U_SELECTOR_I2C		2
#define ESP770U_SELECTOR_REG		3
#define ESP770U_SELECTOR_COUNTER	10
#define ESP770U_SELECTOR_CONTROL	11
#define ESP770U_SELECTOR_DATA		12

#define XU_ENTITY       ESP770U_EXTENSION_UNIT
#define REG_SEL         ESP770U_SELECTOR_REG
#define CONTROL_SEL	    ESP770U_SELECTOR_CONTROL
#define DATA_SEL				ESP770U_SELECTOR_DATA

#define ESP_DEBUG 0

#define esp770u_read_reg(d,r,v) rift_sensor_esp770u_read_reg((d),0xf0,(r),(v))
#define esp770u_read_reg_f1(d,r,v) rift_sensor_esp770u_read_reg((d),0xf1,(r),(v))
#define esp770u_write_reg(d,r,v) rift_sensor_esp770u_write_reg((d),0xf0,(r),(v))
#define esp770u_write_reg_f1(d,r,v) rift_sensor_esp770u_write_reg((d),0xf1,(r),(v))

int rift_sensor_esp770u_read_reg(libusb_device_handle *dev, uint8_t reg_block, uint8_t reg, uint8_t *val)
{
	unsigned char buf[4] = { 0x82, reg_block, reg };
	int ret;

	ret = rift_sensor_uvc_set_cur(dev, 0, XU_ENTITY, REG_SEL, buf, sizeof buf);
	if (ret < 0)
		return ret;
	ret = rift_sensor_uvc_get_cur(dev, 0, XU_ENTITY, REG_SEL, buf, 3);
	if (ret < 0)
		return ret;
	if (buf[0] != 0x82 || buf[2] != 0x00)
		fprintf(stderr, "read_reg(0x%x,0x%x): %02x %02x %02x\n",
			reg_block, reg, buf[0], buf[1], buf[2]);
	*val = buf[1];
	return ret;
}

int rift_sensor_esp770u_write_reg(libusb_device_handle *dev, uint8_t reg_block, uint8_t reg, uint8_t val)
{
	unsigned char buf[4] = { 0x02, reg_block, reg, val };
	int ret;

	ret = rift_sensor_uvc_set_cur(dev, 0, XU_ENTITY, REG_SEL, buf, sizeof buf);
	if (ret < 0)
		return ret;
	ret = rift_sensor_uvc_get_cur(dev, 0, XU_ENTITY, REG_SEL, buf, sizeof buf);
	if (ret < 0)
		return ret;
	if (buf[0] != 0x02 || buf[1] != reg_block || buf[2] != reg || buf[3] != val)
		fprintf(stderr, "write_reg(0x%x,0x%x,0x%x): %02x %02x %02x %02x\n",
			reg_block, reg, val, buf[0], buf[1], buf[2], buf[3]);
	return ret;
}

static int set_get_verify_a0(libusb_device_handle *dev, uint8_t val,
			     uint8_t retval)
{
	uint8_t buf[4] = { 0xa0, val, 0x00, 0x00 };
	int ret;

	ret = rift_sensor_uvc_set_cur(dev, 0, XU_ENTITY, REG_SEL, buf, sizeof buf);
	if (ret < 0)
		return ret;
	memset(buf, 0, sizeof buf);
	ret = rift_sensor_uvc_get_cur(dev, 0, XU_ENTITY, REG_SEL, buf, sizeof buf);
	if (ret < 0)
		return ret;
	if (buf[0] != 0xa0 || buf[1] != retval || buf[2] != 0x00) {
		fprintf(stderr, "response, should be a0 %02x 00: %02x %02x %02x\n",
			retval, buf[0], buf[1], buf[2]);
		return -1;
	}

	return 0;
}

/*
 * Read self-incrementing counter.
 */
static int esp770u_get_counter(libusb_device_handle *devh, uint8_t *count)
{
	return rift_sensor_uvc_get_cur(devh, 0, ESP770U_EXTENSION_UNIT,
			   ESP770U_SELECTOR_COUNTER, count, 1);
}

/*
 * Write back self-incrementing counter.
 */
static int esp770u_set_counter(libusb_device_handle *devh, uint8_t count)
{
	return rift_sensor_uvc_set_cur(devh, 0, ESP770U_EXTENSION_UNIT,
			   ESP770U_SELECTOR_COUNTER, &count, 1);
}

/*
 * Reads a buffer from the flash storage.
 */
int rift_sensor_esp770u_flash_read(libusb_device_handle *devh, uint32_t addr,
		       uint8_t *data, uint16_t len)
{
	uint8_t control[16];
	uint8_t count;
	int ret;

	ret = esp770u_get_counter(devh, &count);
	if (ret < 0)
		return ret;

	memset(control, 0, sizeof control);
	control[0] = count;
	control[1] = 0x41;
	control[2] = 0x03;
	control[3] = 0x01;

	control[5] = (addr >> 16) & 0xff;
	control[6] = (addr >> 8) & 0xff;
	control[7] = addr & 0xff;

	control[8] = len >> 8;
	control[9] = len & 0xff;

	ret = rift_sensor_uvc_set_cur(devh, 0, ESP770U_EXTENSION_UNIT,
			  ESP770U_SELECTOR_CONTROL, control, sizeof control);
	if (ret < 0)
		return ret;

	memset(data, 0, len);
	ret = rift_sensor_uvc_get_cur(devh, 0, ESP770U_EXTENSION_UNIT,
			  ESP770U_SELECTOR_DATA, data, len);
	if (ret < 0)
		return ret;

	return esp770u_set_counter(devh, count);
}

static int esp770u_spi_set_control(libusb_device_handle *devh, uint8_t a,
				   size_t len)
{
	/* a is alternating, 0x81 or 0x41 */
	uint8_t control[16] = { 0x00, a, 0x80, 0x01, [9] = len };

	return rift_sensor_uvc_set_cur(devh, 0, ESP770U_EXTENSION_UNIT,
			   ESP770U_SELECTOR_CONTROL, control, sizeof control);
}

static int esp770u_spi_set_data(libusb_device_handle *devh, uint8_t *data,
				size_t len)
{
	return rift_sensor_uvc_set_cur(devh, 0, ESP770U_EXTENSION_UNIT,
			   ESP770U_SELECTOR_DATA, data, len);
}

static int esp770u_spi_get_data(libusb_device_handle *devh, uint8_t *data,
				size_t len)
{
	return rift_sensor_uvc_get_cur(devh, 0, ESP770U_EXTENSION_UNIT,
			   ESP770U_SELECTOR_DATA, data, len);
}

static int radio_write(libusb_device_handle *devhandle, const uint8_t *buf, size_t len)
{
	unsigned char data[127];
	int ret;
	int i;

	if (len > 126)
		return -EINVAL;

	memset(data, 0, sizeof data);
	for (i = 0; i < len; i++) {
		data[i] = buf[i];
		data[126] -= buf[i]; /* calculate checksum */
	}

	ret = esp770u_spi_set_control(devhandle, 0x81, sizeof data);
	if (ret < 0) return ret;

	/* send data */
	ret = esp770u_spi_set_data(devhandle, data, sizeof data);
	if (ret < 0) return ret;

	ret = esp770u_spi_set_control(devhandle, 0x41, sizeof data);
	if (ret < 0) return ret;

	/* expect all zeros */
	ret = esp770u_spi_get_data(devhandle, data, sizeof data);
	if (ret < 0) return ret;

	ret = esp770u_spi_set_control(devhandle, 0x81, sizeof data);
	if (ret < 0) return ret;

	/* clear */
	memset(data, 0, sizeof data);
	ret = esp770u_spi_set_data(devhandle, data, sizeof data);
	if (ret < 0) return ret;

	ret = esp770u_spi_set_control(devhandle, 0x41, sizeof data);
	if (ret < 0) return ret;

	ret = esp770u_spi_get_data(devhandle, data, sizeof data);
	if (ret < 0) return ret;

	/* Expect zeros */
	for (i = 2; i < 126; i++)
		if (data[i])
			break;
	if (data[0] != buf[0] || data[1] != buf[1]) {
		printf("eSP770U: Unexpected read (%02x %02x):\n", buf[0], buf[1]);
		for (i = 0; i < 127; i++)
			printf("%02x ", data[i]);
		printf("\n");
	}

	uint8_t chksum = 0;
	for (i = 0; i < 127; i++)
		chksum += data[i];
	if (chksum) {
		printf("eSP770U: Checksum mismatch: %02x\n", chksum);
		for (i = 0; i < 127; i++)
			printf("%02x ", data[i]);
		printf("\n");
	}

	return 0;
}

int rift_sensor_esp770u_setup_radio(libusb_device_handle *devhandle, const uint8_t radio_id[5])
{
	int ret;

	const uint8_t buf1[7] = { 0x40, 0x10, radio_id[0], radio_id[1],
				radio_id[2], radio_id[3], radio_id[4] };
	ret = radio_write(devhandle, buf1, sizeof buf1);
	if (ret < 0)
		return ret;

	const uint8_t buf2[10] = { 0x50, 0x11, 0xf4, 0x01, 0x00, 0x00,
					       0x67, 0xff, 0xff, 0xff };
	ret = radio_write(devhandle, buf2, sizeof buf2);
	if (ret < 0)
		return ret;

	const uint8_t buf3[2] = { 0x61, 0x12 };
	ret = radio_write(devhandle, buf3, sizeof buf3);
	if (ret < 0)
		return ret;

	const uint8_t buf4[2] = { 0x71, 0x85 };
	ret = radio_write(devhandle, buf4, sizeof buf4);
	if (ret < 0)
		return ret;

	const uint8_t buf5[2] = { 0x81, 0x86 };
	return radio_write(devhandle, buf5, sizeof buf5);
}

/* Initial register setup, only after camera plugin */
int rift_sensor_esp770u_init_regs(libusb_device_handle *devhandle)
{
	int ret;
	uint8_t val;

	ret = set_get_verify_a0(devhandle, 0x03, 0xb2);
	if (ret < 0) return ret;

	ret = esp770u_read_reg(devhandle, 0x5a, &val);
	if (ret < 0) return ret;
	if (val != 0x01 && val != 0x03) printf("unexpected 5a value: %02x\n", val);

	ret = esp770u_write_reg(devhandle, 0x5a, 0x01); /* &= 0x02? */
	if (ret < 0) return ret;
	ret = esp770u_read_reg(devhandle, 0x5a, &val);
	if (ret < 0) return ret;
	if (val != 0x01) printf("unexpected 5a value: %02x\n", val);

	ret = esp770u_read_reg(devhandle, 0x18, &val); /* |= 0x01 ? */
	if (ret < 0) return ret;
	ret = esp770u_write_reg(devhandle, 0x18, 0x0f);
	if (ret < 0) return ret;
	ret = esp770u_read_reg(devhandle, 0x17, &val);
	if (ret < 0) return ret;
	ret = esp770u_write_reg(devhandle, 0x17, 0xed); /* |= 0x01 ? */
	if (ret < 0) return ret;
	ret = esp770u_write_reg(devhandle, 0x17, 0xec); /* &= ~0x01 ? */
	if (ret < 0) return ret;
	ret = esp770u_write_reg(devhandle, 0x18, 0x0e); /* &= ~0x01 ? */
	if (ret < 0) return ret;
	ret = esp770u_read_reg(devhandle, 0x14, &val);
	if (ret < 0) return ret;

#if ESP_DEBUG
	for (int i = 0; i < 256; i += 1) {
		uint8_t rval;
		if (i % 16 == 0)
			printf("%02x: ", i);
		ret = esp770u_read_reg(devhandle, i, &rval);
		if (ret < 0) return ret;
		printf("%02x ", rval);
		if (i % 16 == 15)
			printf("\n");
	}

	printf("--------------------------------------\n");

	for (int i = 0; i < 256; i += 1) {
		uint8_t rval;
		if (i % 16 == 0)
			printf("%02x: ", i);
		ret = esp770u_read_reg_f1(devhandle, i, &rval);
		if (ret < 0) return ret;
		printf("%02x ", rval);
		if (i % 16 == 15)
			printf("\n");
	}
	printf("--------------------------------------\n");
#endif

	return ret;
}

/* Extra initialisation sent after UVC config when in USB2 / MJPEG mode */
int rift_sensor_esp770u_init_jpeg(libusb_device_handle *devhandle)
{
	int ret = 0;
	uint8_t val;

	/* 0xf11f = 0x04 */
	ret = esp770u_read_reg_f1(devhandle, 0x1f, &val);
	if (ret < 0) return ret;
	ret = esp770u_write_reg_f1(devhandle, 0x1f, 0x04);
	if (ret < 0) return ret;

	/* 0xf102 = 0x7f */
	ret = esp770u_read_reg_f1(devhandle, 0x02, &val);
	if (ret < 0) return ret;
	ret = esp770u_write_reg_f1(devhandle, 0x02, 0x7f);
	if (ret < 0) return ret;

	/* 0xf101 = 0xff */
	ret = esp770u_read_reg_f1(devhandle, 0x01, &val);
	if (ret < 0) return ret;
	ret = esp770u_write_reg_f1(devhandle, 0x01, 0xff);
	if (ret < 0) return ret;

	/* 0xf103 = 0xc0 */
	ret = esp770u_read_reg_f1(devhandle, 0x03, &val);
	if (ret < 0) return ret;
	ret = esp770u_write_reg_f1(devhandle, 0x03, 0xc0);
	if (ret < 0) return ret;

	/* 0xf101 = 0xff, 2 times */
	ret = esp770u_read_reg_f1(devhandle, 0x01, &val);
	if (ret < 0) return ret;
	ret = esp770u_write_reg_f1(devhandle, 0x01, 0xff);
	if (ret < 0) return ret;
	ret = esp770u_read_reg_f1(devhandle, 0x01, &val);
	if (ret < 0) return ret;
	ret = esp770u_write_reg_f1(devhandle, 0x01, 0xff);
	if (ret < 0) return ret;

	/* 0xf11e = 0x01 */
	ret = esp770u_read_reg_f1(devhandle, 0x1e, &val);
	if (ret < 0) return ret;
	ret = esp770u_write_reg_f1(devhandle, 0x1e, 0x01);
	if (ret < 0) return ret;

	/* 0xf103 = 0xc0 */
	ret = esp770u_read_reg_f1(devhandle, 0x03, &val);
	if (ret < 0) return ret;
	ret = esp770u_write_reg_f1(devhandle, 0x03, 0xc0);
	if (ret < 0) return ret;

	/* 0xf101 = 0xff, once */
	ret = esp770u_read_reg_f1(devhandle, 0x01, &val);
	if (ret < 0) return ret;
	ret = esp770u_write_reg_f1(devhandle, 0x01, 0xff);
	if (ret < 0) return ret;

	/* 0xf102 = 0x7f again */
	ret = esp770u_read_reg_f1(devhandle, 0x02, &val);
	if (ret < 0) return ret;
	ret = esp770u_write_reg_f1(devhandle, 0x02, 0x7f);
	if (ret < 0) return ret;

	/* 0xf101 = 0xff, 2 more times */
	ret = esp770u_read_reg_f1(devhandle, 0x01, &val);
	if (ret < 0) return ret;
	ret = esp770u_write_reg_f1(devhandle, 0x01, 0xff);
	if (ret < 0) return ret;
	return ret;

	/* 0xf102 = 0x7f, 2 more times */
	ret = esp770u_read_reg_f1(devhandle, 0x02, &val);
	if (ret < 0) return ret;
	ret = esp770u_write_reg_f1(devhandle, 0x02, 0x7f);
	if (ret < 0) return ret;
	ret = esp770u_read_reg_f1(devhandle, 0x02, &val);
	if (ret < 0) return ret;
	ret = esp770u_write_reg_f1(devhandle, 0x02, 0x7f);
	if (ret < 0) return ret;

	/* Extra 64-byte fw writes, not sent for now - doesn't seem to matter:
		Data Fragment: 1681030100012ed10040000000000000
		Data Fragment: 070707070707070707070707070707070707070707070707070707070707070707070707…
		Data Fragment: 16020100000000000000000000000000
		Data Fragment: 16
		Data Fragment: 1781030100012f160040000000000000
		Data Fragment: 010101010101010101010101010101010101010101010101010101010101010101010101…
		Data Fragment: 17020100000000000000000000000000
		Data Fragment: 17
	*/

	return ret;
}
