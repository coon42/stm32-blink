
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>

#include <math.h>

/*
 * Pulse the LED on GPIOC 13
 *
 * Board: Standard aliexpress stm32f103c8t6
 */

#define GPIO_LED_PORT GPIOC
#define GPIO_LED_PIN  GPIO13

volatile uint16_t pwm_val = 0;

#define SH1106_CTRL GPIOB
#define SH1106_SPI  GPIOA

#define SH1106_NSS  GPIO4
#define SH1106_SCK  GPIO5
#define SH1106_MISO GPIO6
#define SH1106_MOSI GPIO7

#define SH1106_RST  GPIO1
#define SH1106_DC   GPIO0

#define SH1106_CMD  23
#define SH1106_DATA 42

void init_spi();

void init()
{
    // Setup clock
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    // Enable periphials
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_SPI1);

    // Setup GPIO
    gpio_set_mode(GPIO_LED_PORT,
                  GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL,
                  GPIO_LED_PIN);

    sh1106_init_gpio();
    sh1106_init_spi();
}

void sh1106_init_gpio()
{
    // Setup CTRL
    gpio_set_mode(SH1106_CTRL,
                 GPIO_MODE_OUTPUT_50_MHZ,
                 GPIO_CNF_OUTPUT_PUSHPULL,
                 SH1106_DC | SH1106_RST);


    // Setup SPI GPIO
    //    NSS:  PA4
    //    SCK:  PA5
    //    MISO: PA6
    //    MOSI: PA7
    gpio_set_mode(SH1106_SPI,
                  GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  SH1106_SCK | SH1106_MOSI);

    gpio_set_mode(SH1106_SPI,
                  GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL,
                  SH1106_NSS);


    gpio_set_mode(SH1106_SPI, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, SH1106_MISO);
}

void sh1106_init_spi()
{
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


/*
 * Oled display
 */
inline void sh1106_begin_tx()
{
    gpio_clear(GPIOB, SH1106_DC);
}

inline void sh1106_tx(uint8_t type, uint8_t b)
{
    if(type == SH1106_CMD) {
        gpio_set(GPIOB, SH1106_DC);
    } else {
        gpio_clear(GPIOB, SH1106_DC);
    }

    spi_send(SPI1, b);
}

inline void sh1106_end_tx()
{
    gpio_set(GPIOB, SH1106_DC);
}

void sh1106_set_col_start_addr()
{
    sh1106_tx(SH1106_CMD, 0x02);
    sh1106_tx(SH1106_CMD, 0x10);
}


void sh1106_init()
{
    // 4 wire spi
    gpio_set(SH1106_SPI, SH1106_NSS);
    gpio_clear(GPIOB, SH1106_DC);
    gpio_set(GPIOB, SH1106_RES);

    // Display initialization sequence
    sh1106_tx(SH1106_CMD, 0xAE); //--turn off oled panel
    sh1106_tx(SH1106_CMD, 0x02); //---set low column address
    sh1106_tx(SH1106_CMD, 0x10); //---set high column address
    sh1106_tx(SH1106_CMD, 0x40); //--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    sh1106_tx(SH1106_CMD, 0x81); //--set contrast control register
    sh1106_tx(SH1106_CMD, 0xA0); //--Set SEG/Column Mapping
    sh1106_tx(SH1106_CMD, 0xC0); //Set COM/Row Scan Direction
    sh1106_tx(SH1106_CMD, 0xA6); //--set normal display
    sh1106_tx(SH1106_CMD, 0xA8); //--set multiplex ratio(1 to 64)
    sh1106_tx(SH1106_CMD, 0x3F); //--1/64 duty
    sh1106_tx(SH1106_CMD, 0xD3); //-set display offset    Shift Mapping RAM Counter (0x00~0x3F)
    sh1106_tx(SH1106_CMD, 0x00); //-not offset
    sh1106_tx(SH1106_CMD, 0xd5); //--set display clock divide ratio/oscillator frequency
    sh1106_tx(SH1106_CMD, 0x80); //--set divide ratio, Set Clock as 100 Frames/Sec
    sh1106_tx(SH1106_CMD, 0xD9); //--set pre-charge period
    sh1106_tx(SH1106_CMD, 0xF1); //Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    sh1106_tx(SH1106_CMD, 0xDA); //--set com pins hardware configuration
    sh1106_tx(SH1106_CMD, 0x12);
    sh1106_tx(SH1106_CMD, 0xDB); //--set vcomh
    sh1106_tx(SH1106_CMD, 0x40); //Set VCOM Deselect Level
    sh1106_tx(SH1106_CMD, 0x20); //-Set Page Addressing Mode (0x00/0x01/0x02)
    sh1106_tx(SH1106_CMD, 0x02); //
    sh1106_tx(SH1106_CMD, 0xA4); // Disable Entire Display On (0xa4/0xa5)
    sh1106_tx(SH1106_CMD, 0xA6); // Disable Inverse Display On (0xa6/a7)
    sh1106_tx(SH1106_CMD, 0xAF); //--turn on oled panel
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
    }

    return 0;
}

