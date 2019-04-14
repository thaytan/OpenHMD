#ifndef __ESP770U_H__
#define __ESP770U_H__

#include <libusb.h>
#include <stdint.h>

int esp770u_read_reg(libusb_device_handle *dev, uint8_t reg, uint8_t *val);

int esp770u_write_reg(libusb_device_handle *dev, uint8_t reg, uint8_t val);

int esp770u_setup_radio(libusb_device_handle *devhandle, uint32_t radio_id);

int esp770u_init_regs(libusb_device_handle *devhandle);

#endif /* __ESP770U_H__ */
