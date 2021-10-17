#include <errno.h>
#include <libusb.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "uvc.h"

#define ESP570_EXTENSION_UNIT		4

#define ESP570_SELECTOR_I2C		2
#define ESP570_SELECTOR_UNKNOWN_3	3
#define ESP570_SELECTOR_EEPROM		5

/*
 * Calls SET_CUR and then GET_CUR on a given selector of the eSP570 UVC
 * extension unit.
 */
static int uvc_xu_set_get_cur(libusb_device_handle *devh, int selector,
			      unsigned char *buf, uint8_t len)
{
	int ret;

	ret = rift_sensor_uvc_set_cur(devh, 0, ESP570_EXTENSION_UNIT, selector, buf, len);
	if (ret < 0)
		return ret;

	return rift_sensor_uvc_get_cur(devh, 0, ESP570_EXTENSION_UNIT, selector, buf, len);
}

/*
 * Reads a buffer from the Microchip 24AA128 EEPROM.
 */
int esp570_eeprom_read(libusb_device_handle *devh, uint16_t addr, uint8_t *data,
		       uint8_t len)
{
	unsigned char buf[59] = { 0x82, 0xa0, addr >> 8, addr & 0xff, len };
	int ret;

	if (len > 32)
		return -1;

	ret = uvc_xu_set_get_cur(devh, ESP570_SELECTOR_EEPROM, buf, sizeof buf);
	if (ret < 0)
		return ret;

	if (buf[0] != 0x82 || buf[1] != len) {
		printf("read_buf: error: 0x%02x 0x%02x\n", buf[0], buf[1]);
		return -1;
	}

	memcpy(data, buf + 2, len);
	return len;
}

/*
 * Performs a 16-bit read operation on the I2C bus.
 */
int esp570_i2c_read(libusb_device_handle *devh, uint8_t addr, uint8_t reg,
		    uint16_t *val)
{
	unsigned char buf[6] = { 0x84, addr, reg };
	int ret;

	ret = uvc_xu_set_get_cur(devh, ESP570_SELECTOR_I2C, buf, sizeof buf);
	if (ret < 0)
		return ret;

	if (buf[0] != 0x84 || buf[4] != 0x00 || buf[5] != 0x00) {
		printf("%s(%02x): %02x %02x %02x %02x %02x %02x\n", __func__,
		       reg, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
		return -1;
	}

	*val = (buf[1] << 8) | buf[2];

	return 0;
}

/*
 * Performs a 16-bit write operation on the I2C bus.
 */
int esp570_i2c_write(libusb_device_handle *devh, uint8_t addr, uint8_t reg,
		     uint16_t val)
{
	unsigned char buf[6] = { 0x04, addr, reg, val >> 8, val & 0xff };
	int ret;

	ret = uvc_xu_set_get_cur(devh, ESP570_SELECTOR_I2C, buf, sizeof buf);
	if (ret < 0)
		return ret;

	if (buf[0] != 0x04 || buf[1] != addr || buf[2] != reg || buf[5] != 0) {
		printf("%s(%02x, %04x): %02x %02x %02x %02x %02x %02x\n",
		       __func__, reg, val, buf[0], buf[1], buf[2], buf[3],
		       buf[4], buf[5]);
		return -1;
	}

	if (buf[3] != (val >> 8) || buf[4] != (val & 0xff)) {
		printf("%s(%02x, %04x): read back 0x%04x\n", __func__, reg,
		       val, (buf[3] << 8) | buf[4]);
	}

	return 0;
}

/*
 * Calls SET_CUR and GET_CUR on the extension unit's selector 3 with values
 * captured from the Oculus Windows drivers. I have no idea what these mean.
 */
int esp570_setup_unknown_3(libusb_device_handle *devh)
{
	unsigned char buf[3];
	int ret;

	buf[0] = 0x80;
	buf[1] = 0x14;
	buf[2] = 0x00;
	ret = uvc_xu_set_get_cur(devh, ESP570_SELECTOR_UNKNOWN_3, buf, 3);
	if (ret < 0)
		return ret;
	if (buf[0] != 0x80 || buf[1] != 0xdc || buf[2] != 0x00)
		printf("%s(80 14 00): %02x %02x %02x\n", __func__,
		       buf[0], buf[1], buf[2]);

	buf[0] = 0xa0;
	buf[1] = 0xf0;
	buf[2] = 0x00;
	ret = uvc_xu_set_get_cur(devh, ESP570_SELECTOR_UNKNOWN_3, buf, 3);
	if (ret < 0)
		return ret;
	if (buf[0] != 0xa0 || buf[1] != 0x98 || buf[2] != 0x00)
		printf("%s(80 14 00): %02x %02x %02x\n", __func__,
		       buf[0], buf[1], buf[2]);

	return 0;
}
