
#include <math.h>

#include <libopencm3/stm32/rcc.h>

#include "ws2812.h"


/*
 * Board: Standard aliexpress stm32f103c8t6
 */

void init()
{
    // Setup clock
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    // Setup WS2812
    ws2812_init();
}

void render_rgb_test_pattern()
{
    for (unsigned int y = 0; y < WS2812_FB_HEIGHT; y++) {
        for (unsigned int x = 0; x < WS2812_FB_WIDTH; x+=3) {
            ws2812_putpixel(x,   y, 2, 0, 0);
            ws2812_putpixel(x+1, y, 0, 2, 0);
            ws2812_putpixel(x+2, y, 0, 0, 2);
        }
    }
}


void render_rainbow_test_pattern(float t)
{
    for (unsigned int w = 0; w < WS2812_FB_LENGTH; w++) {
        float u = (float)w / (float)WS2812_FB_LENGTH;

        uint8_t r = (0.5 + 0.5 * sin((u+t)*3.1315*2)) * 15;
        uint8_t g = (0.5 + 0.5 * sin(((u+t)+0.333)*3.1415*2)) * 15;
        uint8_t b = (0.5 + 0.5 * sin(((u+t)+0.666)*3.1415*2)) * 15;

        uint8_t x = w % WS2812_FB_WIDTH;
        uint8_t y = w / WS2812_FB_WIDTH;

        ws2812_putpixel(x, y, r, g, b);
    }
}


int main()
{
    init();

    float t = 0;

    // Nothing to do here
    while(42) {
        for(unsigned int _i = 0; _i < 80000; _i++) {
            __asm__("nop");
        }

        render_rainbow_test_pattern(t);
        // render_rgb_test_pattern();
        t += 0.02;
        ws2812_tx();
    }

    return 0;
}

