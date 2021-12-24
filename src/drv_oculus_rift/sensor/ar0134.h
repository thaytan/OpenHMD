#ifndef __AR0134_H__
#define __AR0134_H__

#include <libusb.h>
#include <stdbool.h>

int rift_sensor_ar0134_init(libusb_device_handle *devh, bool usb2_mode);

#endif /* __AR0134_H__ */
