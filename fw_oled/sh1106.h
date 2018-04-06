#ifndef SH1106_H
#define SH1106_H

// -------- Configuration ---------
#define SH1106_WIDTH    128
#define SH1106_HEIGHT   64

#define SH1106_RCC_GPIO_SPI   RCC_GPIOA
#define SH1106_RCC_GPIO_CTRL  RCC_GPIOB
#define SH1106_RCC_SPI        RCC_SPI1

#define SH1106_GPIO_CTRL GPIOB
#define SH1106_GPIO_SPI  GPIOA

#define SH1106_SPI  SPI1

#define SH1106_NSS  GPIO4 // A04
#define SH1106_SCK  GPIO5 // A05
#define SH1106_MISO GPIO6 // A06 
#define SH1106_MOSI GPIO7 // A07

#define SH1106_RST  GPIO1 // B01
#define SH1106_DC   GPIO0 // B00

#define SH1106_CMD  23
#define SH1106_DATA 42


// ------- Functions -------

// Initialization
void sh1106_init_clocks();
void sh1106_init_gpio();
void sh1106_init_spi();
void sh1106_init_display();

// Control
void sh1106_display_on();
void sh1106_display_off();

// Drawing
void sh1106_putpixel(uint8_t x, uint8_t y, uint8_t c);

#endif

