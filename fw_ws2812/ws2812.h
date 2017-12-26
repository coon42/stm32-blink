#ifndef WS2812_H
#define WS2812_H

/*
 * WS2812 library using DMA and TIM4 for
 * PWM generation.
 */

void ws2812_init();

void ws2812_putpixel(uint16_t x, uint16_t y,
                     uint8_t r, uint8_t g, uint8_t b);

void ws2812_tx();

#endif

