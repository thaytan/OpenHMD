#include <errno.h>
#include <libusb.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp570.h"

#define MT9V034_CHIP_VERSION			0x00
#define MT9V034_WINDOW_HEIGHT			0x03
#define MT9V034_WINDOW_WIDTH			0x04
#define MT9V034_HORIZONTAL_BLANKING		0x05
#define MT9V034_VERTICAL_BLANKING		0x06
#define MT9V034_CHIP_CONTROL			0x07
#define MT9V034_COARSE_SHUTTER_WIDTH_TOTAL	0x0b
#define MT9V034_READ_MODE			0x0d
#define MT9V034_LED_OUT_CONTROL			0x1b
#define MT9V034_ANALOG_GAIN			0x35
#define MT9V034_BLACK_LEVEL_CALIB_CTRL		0x47
#define MT9V034_BLACK_LEVEL_CALIB_VALUE		0x48
#define MT9V034_AEC_AGC_ENABLE			0xaf
#define MT9V034_MAX_TOTAL_SHUTTER_WIDTH		0xbd
#define MT9V034_FINE_SHUTTER_WIDTH_TOTAL	0xd5

#define MT9V034_ANALOG_GAIN_MIN			16

#define MT9V034_CHIP_CONTROL_MASTER_MODE	(1 << 3)
#define MT9V034_CHIP_CONTROL_SNAPSHOT_MODE	(3 << 3)
#define MT9V034_CHIP_CONTROL_DOUT_ENABLE	(1 << 7)
#define MT9V034_CHIP_CONTROL_SEQUENTIAL		(1 << 8)

#define MT9V032_READ_MODE_RESERVED		0x300
#define MT9V034_READ_MODE_ROW_FLIP		(1 << 4)
#define MT9V034_READ_MODE_COLUMN_FLIP		(1 << 5)

#define MT9V034_BLACK_LEVEL_CALIB_OVERRIDE	(1 << 0)

#define MT9V034_LED_OUT_DISABLE			(1 << 0)
#define MT9V034_LED_OUT_INVERT			(1 << 1)

#define MT9V034_I2C_ADDR			0x4c

static inline int mt9v034_read_reg(libusb_device_handle *devh, uint8_t reg,
				   uint16_t *val)
{
        return esp570_i2c_read(devh, MT9V034_I2C_ADDR << 1, reg, val);
}

static inline int mt9v034_write_reg(libusb_device_handle *devh, uint8_t reg,
				    uint16_t val)
{
        return esp570_i2c_write(devh, MT9V034_I2C_ADDR << 1, reg, val);
}


/*
 * Sets up the MT9V034 sensor for synchronized exposure, with minimal gain
 * and raised black level calibration.
 */
int mt9v034_setup(libusb_device_handle *devh)
{
	uint16_t version, read_mode;

	mt9v034_read_reg(devh, MT9V034_CHIP_VERSION, &version);
	if (version != 0x1324)
		return -1;

	/* Enable horizontal and vertical flip */
	mt9v034_read_reg(devh, MT9V034_READ_MODE, &read_mode);
	mt9v034_write_reg(devh, MT9V034_READ_MODE, read_mode |
			  MT9V034_READ_MODE_ROW_FLIP |
			  MT9V034_READ_MODE_COLUMN_FLIP);
	return 0;
}

static int mt9v034_enable_sync(libusb_device_handle *devh)
{
	uint16_t chip_control;

	/* Disable AEC/AGC */
	mt9v034_write_reg(devh, MT9V034_AEC_AGC_ENABLE, 0);
	/* Raise black level with manual black level calibration override */
	mt9v034_write_reg(devh, MT9V034_BLACK_LEVEL_CALIB_CTRL,
			    MT9V034_BLACK_LEVEL_CALIB_OVERRIDE);
	mt9v034_write_reg(devh, MT9V034_BLACK_LEVEL_CALIB_VALUE, 0x81); /* -127 */
	/* Reduce analog gain to minimum */
	mt9v034_write_reg(devh, MT9V034_ANALOG_GAIN, MT9V034_ANALOG_GAIN_MIN);
	mt9v034_write_reg(devh, MT9V034_VERTICAL_BLANKING, 5);
	mt9v034_write_reg(devh, MT9V034_HORIZONTAL_BLANKING, 94);
	mt9v034_write_reg(devh, MT9V034_CHIP_CONTROL,
			  MT9V034_CHIP_CONTROL_MASTER_MODE |
			  MT9V034_CHIP_CONTROL_DOUT_ENABLE |
			  MT9V034_CHIP_CONTROL_SEQUENTIAL);

	/* Set integration time in number of rows + number of clock cycles */
	mt9v034_write_reg(devh, MT9V034_COARSE_SHUTTER_WIDTH_TOTAL, 11);
	mt9v034_write_reg(devh, MT9V034_FINE_SHUTTER_WIDTH_TOTAL, 111);
	/* Switch to snapshot mode, exposure controlled by Rift DK2 HMD */
	mt9v034_read_reg(devh, MT9V034_CHIP_CONTROL, &chip_control);
	if (chip_control != (MT9V034_CHIP_CONTROL_MASTER_MODE |
			     MT9V034_CHIP_CONTROL_DOUT_ENABLE |
			     MT9V034_CHIP_CONTROL_SEQUENTIAL)) {
		printf("MT9V034: Unexpected chip control value: 0x%04x\n",
		       chip_control);
	}
	mt9v034_write_reg(devh, MT9V034_CHIP_CONTROL,
			  MT9V034_CHIP_CONTROL_SNAPSHOT_MODE |
			  MT9V034_CHIP_CONTROL_DOUT_ENABLE |
			  MT9V034_CHIP_CONTROL_SEQUENTIAL);
	/* Enable LED? */
	mt9v034_write_reg(devh, MT9V034_LED_OUT_CONTROL,
			  MT9V034_LED_OUT_INVERT);
	return 0;
}

static int mt9v034_disable_sync(libusb_device_handle *devh)
{
	/* Disable LED? */
	mt9v034_write_reg(devh, MT9V034_LED_OUT_CONTROL, 0);

	/* Disable AEC/AGC */
	mt9v034_write_reg(devh, MT9V034_AEC_AGC_ENABLE, 0);
	/* Raise black level with manual black level calibration override */
	mt9v034_write_reg(devh, MT9V034_BLACK_LEVEL_CALIB_CTRL, 0x80);
	/* Set analog gain to default */
	mt9v034_write_reg(devh, MT9V034_ANALOG_GAIN, 0x20);
	mt9v034_write_reg(devh, MT9V034_VERTICAL_BLANKING, 57);
	mt9v034_write_reg(devh, MT9V034_HORIZONTAL_BLANKING, 94);
	mt9v034_write_reg(devh, MT9V034_CHIP_CONTROL,
			  MT9V034_CHIP_CONTROL_MASTER_MODE |
			  MT9V034_CHIP_CONTROL_DOUT_ENABLE |
			  MT9V034_CHIP_CONTROL_SEQUENTIAL);
	/* Set integration time in number of rows + number of clock cycles */
	mt9v034_write_reg(devh, MT9V034_COARSE_SHUTTER_WIDTH_TOTAL, 0xf0);
	mt9v034_write_reg(devh, MT9V034_FINE_SHUTTER_WIDTH_TOTAL, 0);

	return 0;
}

int mt9v034_set_sync(libusb_device_handle *devh, bool enabled)
{
	if (enabled)
		return mt9v034_enable_sync(devh);
	else
		return mt9v034_disable_sync(devh);
}
