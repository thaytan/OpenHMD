#ifndef __RIFT_SENSOR_ESP770U_H__
#define __RIFT_SENSOR_ESP770U_H__

#include <libusb.h>
#include <stdint.h>

int rift_sensor_esp770u_flash_read(libusb_device_handle *devh, uint32_t addr,
      uint8_t *data, uint16_t len);

int rift_sensor_esp770u_setup_radio(libusb_device_handle *devhandle, const uint8_t radio_id[5]);

int rift_sensor_esp770u_init_regs(libusb_device_handle *devhandle);
int rift_sensor_esp770u_init_jpeg(libusb_device_handle *devhandle);

#endif /* __RIFT_SENSOR_ESP770U_H__ */
