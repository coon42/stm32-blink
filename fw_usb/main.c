/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "usb_serial.h"


int main(void)
{
	int i;
    const char* line;

    rcc_clock_setup_in_hse_8mhz_out_72mhz();
	usbd_device *usbd_dev = usb_serial_init();

	while (1) {
		usbd_poll(usbd_dev);

        line = usb_serial_rx();
        if (line) {
            usb_serial_tx(usbd_dev, "Received a line: ");
            usb_serial_tx(usbd_dev, line);
            usb_serial_tx(usbd_dev, "\r\n");
        }

        i++;
    }
}
