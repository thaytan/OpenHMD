#ifndef __ESP570_H__
#define __ESP570_H__

#include <libusb.h>
#include <stdint.h>

int esp570_i2c_read(libusb_device_handle *devh, uint8_t addr, uint8_t reg,
		    uint16_t *val);

int esp570_i2c_write(libusb_device_handle *devh, uint8_t addr, uint8_t reg,
		     uint16_t val);

int esp570_eeprom_read(libusb_device_handle *devh, uint16_t addr, uint8_t *buf,
		       uint8_t len);

int esp570_setup_unknown_3(libusb_device_handle *devh);

#endif /* __ESP570_H__ */
