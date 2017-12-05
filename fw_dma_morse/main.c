
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>

// 300 ms high
#define PULSE_LONG  39308
// 125 ms high
#define PULSE_SHORT 16378
// 0.5 s low
#define PULSE_PAUSE 0


/*
 * Pulse the LED on GPIOC 13:
 * Now with morse encoding
 *
 * Board: Standard aliexpress stm32f103c8t6
 * Update: now using TIM3
 */

#define GPIO_LED_PORT GPIOC
#define GPIO_LED_PIN  GPIO13

// .... . .-.. .-.. ---
const uint16_t msg_buf[] = {
    PULSE_SHORT, // H
    PULSE_SHORT,
    PULSE_SHORT,
    PULSE_SHORT,
    PULSE_PAUSE,

    PULSE_SHORT, // E
    PULSE_PAUSE,

    PULSE_SHORT, // L
    PULSE_LONG,
    PULSE_SHORT,
    PULSE_SHORT,
    PULSE_PAUSE,

    PULSE_SHORT, // L
    PULSE_LONG,
    PULSE_SHORT,
    PULSE_SHORT,
    PULSE_PAUSE,

    PULSE_LONG, // O
    PULSE_LONG,
    PULSE_LONG,
    PULSE_PAUSE
};

#define MSG_LEN 21


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

void init_dma()
{
    rcc_periph_clock_enable(RCC_DMA1);

    // Setup NVIC
    nvic_enable_irq(NVIC_DMA1_CHANNEL3_IRQ);

    // Setup DMA1 controller:
    // We are using channel 2 for transfer (TIM2_UP)
    // Transfer size is 16 bit
    dma_set_memory_size(DMA1, DMA_CHANNEL3, DMA_CCR_MSIZE_16BIT);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL3, DMA_CCR_PSIZE_16BIT);

    // Transfer mode: Mem to Periph
    dma_set_read_from_memory(DMA1, DMA_CHANNEL3);

    // Source and Destination
    dma_set_memory_address(DMA1, DMA_CHANNEL3,     (uint32_t)&msg_buf);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL3, (uint32_t)&TIM3_CCR1);

    // Length
    dma_set_number_of_data(DMA1, DMA_CHANNEL3, MSG_LEN);

    // Repeat and increment pointer with each request
    dma_enable_circular_mode(DMA1, DMA_CHANNEL3);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL3);

    // Enable interrupts:
    //  - Interrupt after transfer complete
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL3);

    // Enable channel
    dma_enable_channel(DMA1, DMA_CHANNEL3);
}


void dma1_channel3_isr()
{
    // Transfer complete interrupt flag
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL3, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL3, DMA_TCIF);
    }
}


void init_blink()
{
    rcc_periph_clock_enable(RCC_TIM3);

    // Setup NVIC
    nvic_enable_irq(NVIC_TIM3_IRQ);
    nvic_set_priority(NVIC_TIM3_IRQ, 1);

    // Set mode: No Div, Edge aligned, Up
    TIM3_CR1 |= TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE | TIM_CR1_DIR_UP;

    // DMA request on update event
    // TIM2_CR2 |= TIM_CR2_CCDS;

    // Interrupts:
    //  - Compare Channel 1 (CC1IE)
    //  - Update / Overflow Event (UIE)
    //  - Update DMA Request Enable (UDE)
    TIM3_DIER |= TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_UDE;

    // Configure prescaler
    TIM3_ARR = 65514; // Full range
    TIM3_PSC = 549; // 0.5s

    // Set compare value
    TIM3_CCR1 = 0;

    // Enable timer
    TIM3_CR1 |= TIM_CR1_CEN;
}


void tim3_isr()
{
    if(TIM3_SR & TIM_SR_UIF) {
        gpio_clear(GPIO_LED_PORT, GPIO_LED_PIN);
        TIM3_SR &= ~TIM_SR_UIF; // Clear flag
    }

    if(TIM3_SR & TIM_SR_CC1IF) {
        // Compare flag triggered, pulse off
        gpio_set(GPIO_LED_PORT, GPIO_LED_PIN);
        TIM3_SR &= ~TIM_SR_CC1IF; // Clear flag
    }
}


int main()
{
    init();
    init_dma();
    init_blink();

    // Nothing to do here
    while(42) {
    }

    return 0;
}

