#include <libusb.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "uvc.h"

#define XU_ENTITY	4
#define SENSOR_REG_SEL	2

static int ar0134_read_reg(libusb_device_handle *dev, uint16_t reg, uint16_t *val)
{
	unsigned char buf[6] = { 0x86, 0x20, reg >> 8, reg & 0xff, 0x00, 0x00 };
	int ret;

	ret = rift_sensor_uvc_set_cur(dev, 0, XU_ENTITY, SENSOR_REG_SEL, buf, sizeof buf);
	if (ret < 0)
		return ret;
	ret = rift_sensor_uvc_get_cur(dev, 0, XU_ENTITY, SENSOR_REG_SEL, buf, 3);
	if (ret < 0)
		return ret;
	if (buf[0] != 0x86)
		fprintf(stderr, "%s(0x%x): %02x %02x %02x\n", __func__,
			reg, buf[0], buf[1], buf[2]);
	*val = (buf[2] << 8) | buf[1];
	return ret;
}

static int ar0134_write_reg(libusb_device_handle *dev, uint16_t reg, uint16_t val)
{
	unsigned char buf[64] = { 0x06, 0x20, reg >> 8, reg & 0xff, val >> 8, val & 0xff };
	int ret;

	ret = rift_sensor_uvc_set_cur(dev, 0, XU_ENTITY, SENSOR_REG_SEL, buf, 6);
	if (ret < 0)
		return ret;
	ret = rift_sensor_uvc_get_cur(dev, 0, XU_ENTITY, SENSOR_REG_SEL, buf, sizeof buf);
	if (ret < 0)
		return ret;
	if (buf[0] != 0x06 || buf[1] != 0x20 ||
	    buf[2] != (reg >> 8) || buf[3] != (reg & 0xff) ||
	    buf[4] != (val >> 8) || buf[5] != (val & 0xff))
		fprintf(stderr, "%s(0x%04x, 0x%04x): %02x %02x %02x %02x %02x %02x\n",
			__func__, reg, val, buf[0], buf[1], buf[2], buf[3],
			buf[4], buf[5]);
	return ret;
}

/* Sensor setup after stream start */
int rift_sensor_ar0134_init(libusb_device_handle *devh, bool usb2_mode)
{
	uint16_t val;
	int ret;

#if 0
	for (int i = 0x3000; i < 0x3100; i += 2) {
		if (i % 16 == 0)
			printf("%04x: ", i);
		ret = ar0134_read_reg(devh, i, &val);
		if (ret < 0) return ret;
		printf("%04x ", val);
		if (i % 16 == 14)
			printf("\n");
	}
#endif

	/* Read chip version and revision number registers */
	for (int i = 0; i < 10; i++) {
		ret = ar0134_read_reg(devh, 0x3000, &val);
		if (ret < 0 || val != 0x2406) {
			ohmd_sleep(0.005); /* 5ms sleep between retries */
			continue;
		}
		break;
	}
	if (ret < 0) return ret;
	if (val != 0x2406)
		fprintf(stderr, "This is not an AR0134 sensor: 0x%04x\n", val);

	ret = ar0134_read_reg(devh, 0x300e, &val);
	if (ret < 0) return ret;
	if (val != 0x1300)
		fprintf(stderr, "Unexpected revision number: 0x%04x\n", val);

	ret = ar0134_read_reg(devh, 0x30b0, &val);
	if (ret < 0) return ret;
	if (val != 0x0080)
		fprintf(stderr, "Expected monochrome mode instead of 0x%04x\n", val);

	/* Disable embedded register data and statistics. For now we can't use
	 * this anyway. */
	ret = ar0134_read_reg(devh, 0x3064, &val);
	if (ret < 0) return ret;
	if (val != 0x1882 && val != 0x1982)
		printf("Unexpected embedded data control: %04x\n", val);
	ret = ar0134_write_reg(devh, 0x3064, val & ~0x0180);
	if (ret < 0) return ret;

	/* Set data pedestal (black level) to zero */
	ret = ar0134_write_reg(devh, 0x301e, 0);
	if (ret < 0) return ret;

	/* Disable binning */
	ret = ar0134_write_reg(devh, 0x3032, 0x0000);
	if (ret < 0) return ret;

	/* Set all gain values to the default, in USB2 mode 0x0007 is used
 	 * instead. */
	if (usb2_mode)
		ret = ar0134_write_reg(devh, 0x305e, 0x0007);
	else
		ret = ar0134_write_reg(devh, 0x305e, 0x0020);
	if (ret < 0) return ret;

	/* This changes nothing, probably clear some already cleared bits. */
	ret = ar0134_read_reg(devh, 0x30b0, &val);
	if (ret < 0) return ret;
	val = 0x0080; /* val &= ~(?); */
	ret = ar0134_write_reg(devh, 0x30b0, val);
	if (ret < 0) return ret;

	/* Set vertical and horizontal capture ranges to maximum (0-959,
	 * 0-1279). */
	ret = ar0134_write_reg(devh, 0x3002, 0);
	if (ret < 0) return ret;
	ret = ar0134_write_reg(devh, 0x3006, 959);
	if (ret < 0) return ret;
	ret = ar0134_write_reg(devh, 0x3004, 0);
	if (ret < 0) return ret;
	ret = ar0134_write_reg(devh, 0x3008, 1279);
	if (ret < 0) return ret;

	/* Set minimum supported pixel clocks per line */
	ret = ar0134_write_reg(devh, 0x300c, 1388);
	if (ret < 0) return ret;

	/* Set a short line bit, needed for the 1388 pclk line length above. */
	ret = ar0134_read_reg(devh, 0x30b0, &val);
	if (ret < 0) return ret;
	val |= (1 << 10);
	ret = ar0134_write_reg(devh, 0x30b0, val);
	if (ret < 0) return ret;

	/* Set total number of lines, 37 lines vertical blanking */
	ret = ar0134_write_reg(devh, 0x300a, 997);
	if (ret < 0) return ret;

	/* Read number of pixel clocks per line, should be changed above. */
	ret = ar0134_read_reg(devh, 0x300c, &val);
	if (ret < 0) return ret;
	if (val != 1388)
		fprintf(stderr, "Too many pixel clocks per line: %u\n", val);

	/* Set coarse integration time (in multiples of lines) and
	 * fine integration time (in multiples of the pixel clock). */
	ret = ar0134_write_reg(devh, 0x3012, 26); /* 26 lines */
	if (ret < 0) return ret;
	ret = ar0134_write_reg(devh, 0x3014, 646);
	if (ret < 0) return ret;
	ret = ar0134_read_reg(devh, 0x3012, &val);
	if (ret < 0) return ret;
	if (val != 26)
		fprintf(stderr, "Failed to set coarse integration time: %u\n", val);
	ret = ar0134_read_reg(devh, 0x3014, &val);
	if (ret < 0) return ret;
	if (val != 646)
		fprintf(stderr, "Failed to set fine integration time: %u\n", val);

	/* Stop streaming, trigger external exposure from nRF51288 */
	ret = ar0134_read_reg(devh, 0x301a, &val);
	if (ret < 0) return ret;
	if (val != 0x10dc)
		fprintf(stderr, "Unexpected reset register value: 0x%04x\n", val);
	/* Force PLL always enabled */
	val |= 1 << 11;
	/* Enable trigger input pin */
	val |= 1 << 8;
	/* Disable streaming (switch to externally triggered mode) */
	val &= ~(1 << 2);
	ret = ar0134_write_reg(devh, 0x301a, val);
	if (ret < 0) return ret;

	/* Enable junction temperature sensor */
	ret = ar0134_read_reg(devh, 0x30b4, &val);
	if (ret < 0) return ret;
	/* Temperature sensor power */
	val |= 1 << 0;
	/* Start conversion */
	val |= 1 << 4;
	ret = ar0134_write_reg(devh, 0x30b4, val);
	if (ret < 0) return ret;

	/* Read 70 °C calibration point */
	uint16_t calib_70c;
	ret = ar0134_read_reg(devh, 0x30c6, &calib_70c);
	if (ret < 0) return ret;

	/* Read 50 °C calibration point */
	uint16_t calib_50c;
	ret = ar0134_read_reg(devh, 0x30c8, &calib_50c);
	if (ret < 0) return ret;

	/* Read temperature sensor */
	ret = ar0134_read_reg(devh, 0x30b2, &val);
	if (ret < 0) return ret;
	printf("Temperature: %.1f °C\n", 50.0 + 20.0 * (val - calib_50c) /
	       (calib_70c - calib_50c));

	return 0;
}
