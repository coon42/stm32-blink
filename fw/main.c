
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <math.h>

/*
 * Pulse the LED on GPIOC 13
 *
 * Board: Standard aliexpress stm32f103c8t6
 */

#define GPIO_LED_PORT GPIOC
#define GPIO_LED_PIN  GPIO13

#define __DELAY(X) { for(unsigned int _i = 0; _i<X; _i++) { __asm__("nop"); } }


void init()
{
    // Setup clock
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    // Enable periphials
    rcc_periph_clock_enable(RCC_GPIOC);

    // Setup GPIO
    gpio_set_mode(GPIO_LED_PORT,
                  GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL,
                  GPIO_LED_PIN);

}


int main()
{
    init();

    // Create some simple soft PWM
    double t = 0.0;
    double f_val = 0.0;

    uint16_t val = 0;
    uint16_t cnt = 0;

    while(42) {
        f_val = 0.5 + 0.5*cos(t*M_PI);
        val = (uint16_t)(f_val * 65535.0);

        for (cnt = 0; cnt < 65535; cnt++) {
            if (cnt < val) {
                gpio_clear(GPIOC, GPIO13);
            }
            else {
                gpio_set(GPIOC, GPIO13);
            }
        }

        t += 0.04;
    }

    return 0;
}


