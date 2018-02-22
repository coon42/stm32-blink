#ifndef __USB_SERIAL_H__
#define __USB_SERIAL_H__

#include <libopencm3/usb/usbd.h>

usbd_device* usb_serial_init();
const char*  usb_serial_rx();
void         usb_serial_tx(usbd_device*, const char*);


#endif

