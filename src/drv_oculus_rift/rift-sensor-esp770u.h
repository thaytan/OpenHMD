#ifndef __RIFT_SENSOR_ESP770U_H__
#define __RIFT_SENSOR_ESP770U_H__

#include <libusb.h>
#include <stdint.h>

int rift_sensor_esp770u_read_reg(libusb_device_handle *dev, uint8_t reg, uint8_t *val);

int rift_sensor_esp770u_write_reg(libusb_device_handle *dev, uint8_t reg, uint8_t val);

int rift_sensor_esp770u_setup_radio(libusb_device_handle *devhandle, const uint8_t radio_id[5]);

int rift_sensor_esp770u_init_regs(libusb_device_handle *devhandle);

#endif /* __RIFT_SENSOR_ESP770U_H__ */
