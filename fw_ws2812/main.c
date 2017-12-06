
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>


// Timings
#define T_LO    26
#define T_HI    50
#define T_TOTAL 90

#define LED_COUNT 64

// Buffers
uint8_t ws2812_frame_buffer[LED_COUNT*3];
uint8_t ws2812_tx_buffer[LED_COUNT*3*8];
uint8_t ws2812_timings[] = {T_LO, T_HI};


/*
 * Board: Standard aliexpress stm32f103c8t6
 */

#define GPIO_LED_PORT GPIOC
#define GPIO_LED_PIN  GPIO13


void init()
{
    // Setup clock
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    // Enable periphials
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_AFIO);

    // Setup GPIO
    gpio_set_mode(GPIO_LED_PORT,
                  GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL,
                  GPIO_LED_PIN);

    gpio_set_mode(GPIOA,
                  GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  GPIO7);

    // gpio_primary_remap(0, AFIO_MAPR_TIM3_REMAP_PARTIAL_REMAP);

}

void init_dma()
{
    rcc_periph_clock_enable(RCC_DMA1);

    // Setup NVIC
    nvic_enable_irq(NVIC_DMA1_CHANNEL3_IRQ);

    // Setup DMA1 controller:
    // We are using channel 2 for transfer (TIM2_UP)
    // Transfer size is 16 bit
    dma_set_memory_size(DMA1, DMA_CHANNEL3, DMA_CCR_MSIZE_8BIT);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL3, DMA_CCR_PSIZE_16BIT);

    // Transfer mode: Mem to Periph
    dma_set_read_from_memory(DMA1, DMA_CHANNEL3);

    // Source and Destination
    dma_set_memory_address(DMA1, DMA_CHANNEL3, (uint32_t)&ws2812_tx_buffer);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL3, (uint32_t)&TIM3_CCR2);

    // Length
    dma_set_number_of_data(DMA1, DMA_CHANNEL3, LED_COUNT*3*8);

    // Repeat and increment pointer with each request
    dma_enable_circular_mode(DMA1, DMA_CHANNEL3);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL3);

    // Enable interrupts:
    //  - Interrupt after transfer complete
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL3);

    // Enable channel
    // dma_enable_channel(DMA1, DMA_CHANNEL2);
}


void dma1_channel3_isr()
{
    // Transfer complete interrupt flag
    if (DMA1_ISR & DMA_ISR_TCIF3) {
        DMA1_IFCR |= DMA_IFCR_CTCIF3;

        // Stop transmitting
        dma_disable_channel(DMA1, DMA_CHANNEL3);
        TIM3_CCR2 = 0;
        TIM3_CR1 &= ~TIM_CR1_CEN;

        gpio_set(GPIO_LED_PORT, GPIO_LED_PIN);
    }
}


void init_pwm()
{
    rcc_periph_clock_enable(RCC_TIM3);

    // Set mode: No Div, Edge aligned, Up
    TIM3_CR1 |= TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE | TIM_CR1_DIR_UP;

    // Interrupts:
    //  - Update DMA Request Enable (UDE)
    TIM3_DIER |= TIM_DIER_UDE;

    // PWM generation
    timer_disable_oc_output(TIM3, TIM_OC2);
    timer_disable_oc_clear(TIM3, TIM_OC2);
    // timer_enable_oc_preload(TIM3, TIM_OC2);
    timer_set_oc_slow_mode(TIM3, TIM_OC2);
    timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);
    timer_set_oc_polarity_high(TIM3, TIM_OC2);
    timer_set_oc_value(TIM3, TIM_OC2, 0);
    timer_enable_oc_output(TIM3, TIM_OC2);
    timer_enable_preload(TIM3);
	// TIM3_CCMR1 |= TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE;
	// TIM3_CCER  |= TIM_CCER_CC2P | TIM_CCER_CC2E;


    // Configure prescaler
    TIM3_ARR = T_TOTAL;
    TIM3_PSC = 0;

    // Set compare value
    TIM3_CCR2 = 0;

    // Disble timer for now
    TIM3_CR1 &= ~TIM_CR1_CEN;
}


void ws2812_init_buffers()
{
    for(unsigned int i = 0; i < LED_COUNT*3; i++) {
        ws2812_frame_buffer[i] = 0;
    }

    for(unsigned int i = 0; i < LED_COUNT*3*8; i++) {
        ws2812_tx_buffer[i] = 0;
    }
}


void ws2812_update_tx_buffer()
{
    for(unsigned int i = 0; i < LED_COUNT; i++) {
        uint8_t c;

        // G
        c = ws2812_frame_buffer[(i*3)+1];
        ws2812_tx_buffer[i*24]   = ws2812_timings[!!(c & 0b10000000)];
        ws2812_tx_buffer[i*24+1] = ws2812_timings[!!(c & 0b01000000)];
        ws2812_tx_buffer[i*24+2] = ws2812_timings[!!(c & 0b00100000)];
        ws2812_tx_buffer[i*24+3] = ws2812_timings[!!(c & 0b00010000)];
        ws2812_tx_buffer[i*24+4] = ws2812_timings[!!(c & 0b00001000)];
        ws2812_tx_buffer[i*24+5] = ws2812_timings[!!(c & 0b00000100)];
        ws2812_tx_buffer[i*24+6] = ws2812_timings[!!(c & 0b00000010)];
        ws2812_tx_buffer[i*24+7] = ws2812_timings[!!(c & 0b00000001)];

        // R
        c = ws2812_frame_buffer[i*3];
        ws2812_tx_buffer[i*24+8]  = ws2812_timings[!!(c & 0b10000000)];
        ws2812_tx_buffer[i*24+9]  = ws2812_timings[!!(c & 0b01000000)];
        ws2812_tx_buffer[i*24+10] = ws2812_timings[!!(c & 0b00100000)];
        ws2812_tx_buffer[i*24+11] = ws2812_timings[!!(c & 0b00010000)];
        ws2812_tx_buffer[i*24+12] = ws2812_timings[!!(c & 0b00001000)];
        ws2812_tx_buffer[i*24+13] = ws2812_timings[!!(c & 0b00000100)];
        ws2812_tx_buffer[i*24+14] = ws2812_timings[!!(c & 0b00000010)];
        ws2812_tx_buffer[i*24+15] = ws2812_timings[!!(c & 0b00000001)];

        // B
        c = ws2812_frame_buffer[(i*3)+2];
        ws2812_tx_buffer[i*24+16] = ws2812_timings[!!(c & 0b10000000)];
        ws2812_tx_buffer[i*24+17] = ws2812_timings[!!(c & 0b01000000)];
        ws2812_tx_buffer[i*24+18] = ws2812_timings[!!(c & 0b00100000)];
        ws2812_tx_buffer[i*24+19] = ws2812_timings[!!(c & 0b00010000)];
        ws2812_tx_buffer[i*24+20] = ws2812_timings[!!(c & 0b00001000)];
        ws2812_tx_buffer[i*24+21] = ws2812_timings[!!(c & 0b00000100)];
        ws2812_tx_buffer[i*24+22] = ws2812_timings[!!(c & 0b00000010)];
        ws2812_tx_buffer[i*24+23] = ws2812_timings[!!(c & 0b00000001)];
    }
}


void ws2812_tx()
{
    gpio_clear(GPIO_LED_PORT, GPIO_LED_PIN);

    ws2812_update_tx_buffer();

    // Start transmitting
    dma_enable_channel(DMA1, DMA_CHANNEL3);

    // Enable timer
    TIM3_CR1 |= TIM_CR1_CEN;
}



int main()
{
    init();
    init_dma();
    init_pwm();

    gpio_set(GPIO_LED_PORT, GPIO_LED_PIN);

    ws2812_init_buffers();

    for(unsigned i = 0; i < LED_COUNT; i++) {
        ws2812_frame_buffer[i*3] = 0;
        ws2812_frame_buffer[i*3+1] = 0;
        ws2812_frame_buffer[i*3+2] = 0;
    }




    // Nothing to do here
    while(42) {
        for(unsigned int i = 0; i < 8000000; i++) {
            __asm__("nop");
        }

        ws2812_tx();
    }

    return 0;
}

