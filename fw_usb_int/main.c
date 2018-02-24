/*
 * Usb Serial Test
 *
 * (c) 2018 Matthias Hannig <matthias@hannig.cc>
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

    // Clock Setup
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    // Initialize USB
	usb_serial_init();

	while (1) {
        line = usb_serial_rx();
        if (line) {
            printf("Received a line: %s\r\n", line);
        }

        if( i % 10000  == 0 ) {
            printf("Fnord 42 :: %d\r\n", i);
        }

        i++;
    }
}
