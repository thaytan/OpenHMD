#ifndef __MT9V034_H__
#define __MT9V034_H__

#include <libusb.h>
#include <stdbool.h>

int mt9v034_setup(libusb_device_handle *devh);
int mt9v034_set_sync(libusb_device_handle *devh, bool enabled);

#endif /* __MT9V034_H__ */
