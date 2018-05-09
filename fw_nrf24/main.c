
#include <math.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>

#include "usb_serial.h"

/*
 * Pulse the LED on GPIOC 13
 *
 * Board: Standard aliexpress stm32f103c8t6
 */

#define GPIO_LED_PORT GPIOC
#define GPIO_LED_PIN  GPIO13

volatile uint16_t pwm_val = 0;

void init_spi();

static void initClocks() {
  rcc_periph_clock_enable(RCC_GPIOC);

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_SPI1);
  rcc_periph_clock_enable(RCC_AFIO);
}

static void initGpio() {
  // Setup CTRL
  // This is required to switch between a command and bitmap data
  gpio_set_mode(RCC_GPIOB,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO0 | GPIO1); // DC / RST


  // Setup SPI GPIO, See configuration in sh1106.h
  // for pin mapping.
  gpio_set_mode(RCC_SPI1,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO5 | GPIO7); // SCK, MOSI

  gpio_set_mode(RCC_SPI1,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_FLOAT,
                GPIO6); // MISO

  gpio_set_mode(RCC_SPI1,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO4); // NSS (SCK)
}

static void initSpi() {
  spi_reset(SPI1);

  // Initialize master:
  //   Baudrate: 72e6 / 64 = 1125000
  //   Clock Polarity: HI
  //   Clock Phase: falling edge
  //   Dataframe: 8bit
  //   Bit order: MSB first
  spi_init_master(SPI1,
                  SPI_CR1_BAUDRATE_FPCLK_DIV_256,
                  SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_2,
                  SPI_CR1_DFF_8BIT,
                  SPI_CR1_MSBFIRST);

  spi_enable_software_slave_management(SPI1);
  spi_set_nss_high(SPI1);

  spi_enable(SPI1);
}

void init()
{
    // Setup clock
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    // Enable periphials
    initClocks();

    // Setup GPIO
    gpio_set_mode(GPIO_LED_PORT,
                  GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL,
                  GPIO_LED_PIN);

    initGpio();
    initSpi();

    // TODO: init nrf24
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


static void usbSample() {
  int i;
  const char* line;

  rcc_clock_setup_in_hse_8mhz_out_72mhz();
  usbd_device *usbd_dev = usb_serial_init();

  while (1) {
    usbd_poll(usbd_dev);

    line = usb_serial_rx();
    if (line) {
      usb_serial_tx(usbd_dev, "Roflcopter: ");
      usb_serial_tx(usbd_dev, line);
      usb_serial_tx(usbd_dev, "\r\n");
    }

    i++;
  }
}

int main() {
  usbSample();

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

