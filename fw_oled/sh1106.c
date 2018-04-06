
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>

#include "sh1106.h"

inline void sh1106_tx(uint8_t type, uint8_t b);
inline void sh1106_set_col_start_addr();

static uint8_t _display_buffer[128][8];

void sh1106_init_clocks()
{
    rcc_periph_clock_enable(SH1106_RCC_GPIO_SPI);
    rcc_periph_clock_enable(SH1106_RCC_GPIO_CTRL);
    rcc_periph_clock_enable(SH1106_RCC_SPI);
    rcc_periph_clock_enable(RCC_AFIO);
}

void sh1106_init_gpio()
{
    // Setup CTRL
    // This is required to switch between a command and bitmap data
    gpio_set_mode(SH1106_GPIO_CTRL,
                 GPIO_MODE_OUTPUT_50_MHZ,
                 GPIO_CNF_OUTPUT_PUSHPULL,
                 SH1106_DC | SH1106_RST);


    // Setup SPI GPIO, See configuration in sh1106.h 
    // for pin mapping.
    gpio_set_mode(SH1106_GPIO_SPI,
                  GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  SH1106_SCK | SH1106_MOSI);

    gpio_set_mode(SH1106_GPIO_SPI,
                  GPIO_MODE_INPUT,
                  GPIO_CNF_INPUT_FLOAT,
                  SH1106_MISO);

    gpio_set_mode(SH1106_GPIO_SPI,
                  GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL,
                  SH1106_NSS);

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
 * Transmit a byte using spi. Set the data / command bit
 * accordingly.
 */
inline void sh1106_tx(uint8_t type, uint8_t b)
{
    // We are using 4-wire SPI
    gpio_clear(SH1106_GPIO_SPI, SH1106_NSS); // !CS

    if(type == SH1106_CMD) {
        gpio_set(SH1106_GPIO_CTRL, SH1106_DC);
    } else {
        gpio_clear(SH1106_GPIO_CTRL, SH1106_DC);
    }

    spi_send(SH1106_SPI, b);

    gpio_set(SH1106_GPIO_SPI,  SH1106_NSS);
    gpio_set(SH1106_GPIO_CTRL, SH1106_DC);

}


/*
 * Display initialization
 */
void sh1106_init_display()
{
    // 4 wire spi
    gpio_set(SH1106_GPIO_SPI, SH1106_NSS); 
    gpio_clear(SH1106_RCC_GPIO_CTRL, SH1106_DC);
    gpio_set(SH1106_RCC_GPIO_CTRL, SH1106_RST);

                                 // Display initialization sequence
    sh1106_tx(SH1106_CMD, 0xAE); // --turn off oled panel
    sh1106_tx(SH1106_CMD, 0x02); // ---set low column address
    sh1106_tx(SH1106_CMD, 0x10); // ---set high column address
    sh1106_tx(SH1106_CMD, 0x40); // --set start line address
                                 //   Set Mapping RAM Display Start Line (0x00~0x3F)
    sh1106_tx(SH1106_CMD, 0x81); // --set contrast control register
    sh1106_tx(SH1106_CMD, 0xA0); // --Set SEG/Column Mapping

    sh1106_tx(SH1106_CMD, 0xC0); // Set COM/Row Scan Direction
    sh1106_tx(SH1106_CMD, 0xA6); // --set normal display
    sh1106_tx(SH1106_CMD, 0xA8); // --set multiplex ratio(1 to 64)
    sh1106_tx(SH1106_CMD, 0x3F); // --1/64 duty
    sh1106_tx(SH1106_CMD, 0xD3); // -set display offset

                                 // Shift Mapping RAM Counter (0x00~0x3F)
    sh1106_tx(SH1106_CMD, 0x00); // -not offset
    sh1106_tx(SH1106_CMD, 0xd5); // --set display clock divide ratio/oscillator
                                 //   frequency
    sh1106_tx(SH1106_CMD, 0x80); // --set divide ratio, Set Clock as 100 Frames/Sec
    sh1106_tx(SH1106_CMD, 0xD9); // --set pre-charge period

    sh1106_tx(SH1106_CMD, 0xF1); // Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    sh1106_tx(SH1106_CMD, 0xDA); // --set com pins hardware configuration
    sh1106_tx(SH1106_CMD, 0x12);
    sh1106_tx(SH1106_CMD, 0xDB); // --set vcomh

    sh1106_tx(SH1106_CMD, 0x40); // Set VCOM Deselect Level
    sh1106_tx(SH1106_CMD, 0x20); // -Set Page Addressing Mode (0x00/0x01/0x02)
    sh1106_tx(SH1106_CMD, 0x02); //
    sh1106_tx(SH1106_CMD, 0xA4); // Disable Entire Display On (0xa4/0xa5)
    sh1106_tx(SH1106_CMD, 0xA6); // Disable Inverse Display On (0xa6/a7)
    sh1106_tx(SH1106_CMD, 0xAF); // --turn on oled panel
}

inline void sh1106_set_col_start_addr()
{
    sh1106_tx(SH1106_CMD, 0x02);
    sh1106_tx(SH1106_CMD, 0x10);
}

/*
 * Tun display on
 */
void sh1106_display_on()
{
    sh1106_tx(SH1106_CMD, 0x8D);
    sh1106_tx(SH1106_CMD, 0x14);
    sh1106_tx(SH1106_CMD, 0xAF);
}

/*
 * Turn display off
 */
void sh1106_display_off()
{
    sh1106_tx(SH1106_CMD, 0x8D);
    sh1106_tx(SH1106_CMD, 0x10);
    sh1106_tx(SH1106_CMD, 0xAE);
}

/*
 * Update graphics ram
 */
void sh1106_update()
{
    for(uint8_t i = 0; i < 8; i++) {
        sh1106_tx(SH1106_CMD, 0x80 + i);
        sh1106_set_col_start_addr();

        for (uint8_t j = 0; j < 128; j++) {
            sh1106_tx(SH1106_DATA, _display_buffer[j][i]);
        };
    }
}

/*
 * Set a pixel in buffer
 */
void sh1106_putpixel(uint8_t x, uint8_t y, uint8_t c)
{
    if(x > (SH1106_WIDTH - 1) ||
       y > (SH1106_HEIGHT - 1)) {
        return; // Nothing to do here.
    }

    // As we are only having 1bit color depth, we store
    // our column pixel value as bit set in the selected row "page".
    // To make this a bit more fun, our row train is approaching today in
    // reverse order:
    //
    //   BMP[X][Y] = BIT_AT(BUF[X][7 - (Y / 8)], 7 - (Y % 8))
    //
    uint8_t yoffset = 7 - y / 8;
    uint8_t bitval = 1 << (7 - (y % 8));
    
    if (c) {
        _display_buffer[x][yoffset] |= bitval;
    } else {
        _display_buffer[x][yoffset] &= ~bitval;
    }
}

/*
 * Fill screen
 */
void sh1106_fill(uint8_t c)
{
    for(uint8_t x = 0; x < SH1106_WIDTH; x++) {
        for(uint8_t y = 0; y < SH1106_HEIGHT; y++) {
            sh1106_putpixel(x, y, c);
        }
    }

    sh1106_update();
}


/*
 * Draw a filled rect
 */
void sh1106_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t c)
{
    for(uint8_t i = 0; i < w; i++) {
        for(uint8_t j = 0; j < h; j++) {
            sh1106_putpixel(x + i, y + j, c);
        }
    }

    sh1106_update();
}

/*
 * Clear screen
 */
void sh1106_clear()
{
    for(uint8_t x = 0; x < SH1106_WIDTH; x++) {
        for(uint8_t y = 0; y < SH1106_HEIGHT / 8; y++) {
            _display_buffer[x][y] = 0x00;
        }
    }
}

