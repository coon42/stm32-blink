/*
 * ADC test: Try to get as many samples as possible
 * using fast interleave mode and DMA
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>

#include "usb_serial.h"


#define MIC_RCC  RCC_GPIOA
#define MIC_PORT GPIOA
#define MIC_PIN  GPIO0

#define SAMPLE_BUF_LEN 1024
volatile uint16_t _adc_samples[SAMPLE_BUF_LEN];

void adc_gpio_init()
{
    // Enable GPIOA
    rcc_periph_clock_enable(MIC_RCC);

    // Configure pin as input
    gpio_set_mode(MIC_PORT,
                  GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_ANALOG,
                  MIC_PIN);
}

void adc_init()
{
    // Setup GPIO
    adc_gpio_init();

    // Configure ADC1 and 2
    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_ADC2);

    // ADC should not run during configuration
    adc_power_off(ADC1);
    adc_power_off(ADC2);

    // Configure ADCs
    adc_disable_scan_mode(ADC1);
    adc_disable_scan_mode(ADC2);

    adc_set_right_aligned(ADC1);
    adc_set_right_aligned(ADC2);

    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_1DOT5CYC);
    adc_set_sample_time_on_all_channels(ADC2, ADC_SMPR_SMP_1DOT5CYC);

    // Continuus conversion
    adc_disable_external_trigger_regular(ADC1);
    adc_disable_external_trigger_regular(ADC2);

    adc_set_continuous_conversion_mode(ADC1);
    adc_set_continuous_conversion_mode(ADC2);

    // Enable Fast Interleved Dual Mode
    adc_set_dual_mode(ADC_CR1_DUALMOD_FIM);

    // Set channels
    uint8_t channels[] = {0,};
    adc_set_regular_sequence(ADC1, 1, channels);
    adc_set_regular_sequence(ADC2, 1, channels);

    // Enable DMA
    adc_enable_dma(ADC1);

    // Power on
    adc_power_on(ADC1);
    adc_power_on(ADC2);

    // Wait a bit
    for (uint32_t i = 0; i < 800000; i++) {
        __asm__("nop");
    }

    // Calibrate
    adc_reset_calibration(ADC1);
    adc_reset_calibration(ADC2);
    adc_calibrate(ADC1);
    adc_calibrate(ADC2);

    // Start
    adc_start_conversion_direct(ADC1);
    adc_start_conversion_direct(ADC2);
}


void dma_init()
{
    // We use DMA2, Channel 1
    rcc_periph_clock_enable(RCC_DMA1);

    // Disable for configuration
    dma_disable_channel(DMA1, DMA_CHANNEL1);

    // Set source and dst address
    dma_set_memory_address(DMA1, DMA_CHANNEL1,     (uint32_t)&_adc_samples);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&ADC1_DR);

    // Setup DMA2 controller:
    // We transfer from peripheral ADC1 and ADC2, so we
    // transmit two half words at a time.
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_32BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_32BIT);

    // We read into mem
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);

    // As we read from ADC1 and 2, we need half of the samples
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, SAMPLE_BUF_LEN/2);

    // Increment addr
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);

    // Enable IRQ
    nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
}


void dma1_channel1_isr()
{
    // Check Transfer complete interrupt flag
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TCIF)) {
        dma_disable_channel(DMA1, DMA_CHANNEL1);

        // Process signal
        uint16_t max = 0;
        uint32_t avg = 0;

        for(uint16_t i = 0; i < SAMPLE_BUF_LEN; i++) {
            if (max < _adc_samples[i]) {
                max = _adc_samples[i];
            }
            avg += _adc_samples[i];
        }
        avg /= SAMPLE_BUF_LEN;

        printf("%d %d\r\n", max, max - avg);

        // Clear transfer complete.
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);

        // Enable Transfer
        dma_set_number_of_data(DMA1, DMA_CHANNEL1, SAMPLE_BUF_LEN/2);
        dma_enable_channel(DMA1, DMA_CHANNEL1);
    }
}


int main(void)
{
	int i = 0;
    // const char* line;

    // Clock Setup
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    // Initialize USB
	usb_serial_init();

    // Initialize DMA
    dma_init();

    // Initialize ADC
    adc_init();

    // Start fetching data
    printf("Starting ADC read\r\n");
    dma_enable_channel(DMA1, DMA_CHANNEL1);

	while (1) {
        /*
        if( i % 100000  == 0 ) {
            // Read ADC
            printf("Fnord 42 :: %d %d\r\n", _adc_samples[0], _adc_samples[1]);
        }
        */

        i++;
    }
}
