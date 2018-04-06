
#include <math.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>

#include "sh1106.h"


/*
 * Pulse the LED on GPIOC 13
 *
 * Board: Standard aliexpress stm32f103c8t6
 */

#define GPIO_LED_PORT GPIOC
#define GPIO_LED_PIN  GPIO13

volatile uint16_t pwm_val = 0;

void init_spi();

void init()
{
    // Setup clock
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    // Enable periphials
    rcc_periph_clock_enable(RCC_GPIOC);

    sh1106_init_clocks();

    // Setup GPIO
    gpio_set_mode(GPIO_LED_PORT,
                  GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL,
                  GPIO_LED_PIN);

    sh1106_init_gpio();
    sh1106_init_spi();
    sh1106_init_display();
}


void init_pwm()
{
    rcc_periph_clock_enable(RCC_TIM2);

    // Setup NVIC
    nvic_enable_irq(NVIC_TIM2_IRQ);
    nvic_set_priority(NVIC_TIM2_IRQ, 1);

    // Set mode: No Div, Edge aligned, Up
    timer_set_mode(TIM2,
                   TIM_CR1_CKD_CK_INT,
                   TIM_CR1_CMS_EDGE,
                   TIM_CR1_DIR_UP);

    // Interrupts:
    // Timer Compare Channel 1
    // Timer Update / Overflow
    TIM2_DIER |= TIM_DIER_UIE | TIM_DIER_CC1IE;

    // Configure prescaler
    TIM2_ARR = 65535; // Full range
    TIM2_PSC = 0; // f / (arr * 60 Hz)

    // Set compare value
    TIM2_CCR1 = 0;

    // Enable timer
    TIM2_CR1 |= TIM_CR1_CEN;
}


void tim2_isr()
{
    if(TIM2_SR & TIM_SR_UIF) {
        gpio_clear(GPIO_LED_PORT, GPIO_LED_PIN);
        TIM2_CCR1 = pwm_val;
        TIM2_SR &= ~TIM_SR_UIF; // Clear flag
    }

    if(TIM2_SR & TIM_SR_CC1IF) {
        // Compare flag triggered, pulse off
        gpio_set(GPIO_LED_PORT, GPIO_LED_PIN);
        TIM2_SR &= ~TIM_SR_CC1IF; // Clear flag
    }


}


int main()
{
    init();
    init_pwm();

    // Create some simple soft PWM
    double t = 0.0;
    double f_val = 0.0;

    uint16_t cnt = 0;

    while(42) {
        f_val = 0.5 + 0.5*cos(t*M_PI);
        pwm_val = 100 + (uint16_t)(f_val * 65335.0);

        t += 0.00009;
        cnt++;
    }

    return 0;
}

