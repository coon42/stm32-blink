
#include <math.h>
#include <stdio.h>
#include <limits.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>

#include "usb_serial.h"
#include "nrf24l01p.h"

#define GPIO_LED_PORT GPIOC
#define GPIO_LED_PIN  GPIO13

static usbd_device* _pUsbd_dev = NULL;

static void sendUartText(const char* pText) {
  usb_serial_tx(_pUsbd_dev, pText);
}

static void debugPrint(const char* pText) {
  sendUartText(pText);
}

static void debugPrintln(const char* pText) {
  sendUartText(pText);
  sendUartText("\n");
}

void debugPrintDec(int n) {
  char pNum[32];
  snprintf(pNum, sizeof(pNum), "%d", n);

  sendUartText(pNum);
}

void debugPrintHex(int n) {
  char pHex[32];
  snprintf(pHex, sizeof(pHex), "0x%02X", n);

  sendUartText(pHex);
  gpio_set_mode(GPIO_LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO_LED_PIN);
  gpio_set_mode(GPIO_LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO_LED_PIN);
  gpio_set_mode(GPIO_LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO_LED_PIN);
  gpio_set_mode(GPIO_LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO_LED_PIN);
  gpio_set_mode(GPIO_LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO_LED_PIN);
  gpio_set_mode(GPIO_LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO_LED_PIN);
}

void printFullConfig() {
  debugPrintln("NRF24 Configuration:");
  debugPrint("Mode: "); debugPrintln(nrf24_isListening() ? "Listening" : "Transmitting");
  debugPrint("RF Channel: "); debugPrintDec(nrf24_getRFChannel()); debugPrintln("");
  debugPrint("RF Speed: "); debugPrintln(nrf24_getDataRate() == SPEED_2M ? "2M" : nrf24_getDataRate() == SPEED_1M ? "1M" : "250K");
  debugPrint("CRC: "); debugPrintln(nrf24_crcIsEnabled() ? "Enabled" : "Disabled");
  debugPrint("Encoding scheme: "); debugPrintDec(nrf24_crcGetEncodingScheme()); debugPrintln("");
  debugPrint("Power Status: "); debugPrintln(nrf24_isPoweredOn() ? "On" : "Off");
  debugPrintln("Shockburst:");

  for(int i = 0; i < 6; i++) {
    debugPrint("  ENAA_P"); debugPrintDec(i);
    debugPrint(": ");
    debugPrintln(nrf24_shockburstIsEnabled(i) ? "Enabled" : "Disabled");
  }

  debugPrintln("RX Data Pipes:");
  for(int i = 0; i < 6; i++) {
    debugPrint("  ERX_P"); debugPrintDec(i); debugPrint(": ");
    debugPrintln(nrf24_dataPipeIsEnabled(i) ? "Enabled" : "Disabled");
  }

  debugPrintln("RX Payload sizes: ");
  for(int i = 0; i < 6; i++) {
    debugPrint("  RX_PW_P"); debugPrintDec(i); debugPrint(": ");
    debugPrintDec(nrf24_getPayloadSize(i)); debugPrintln("");
  }

  debugPrint("Address Width: "); debugPrintDec(nrf24_getAddressWidths()); debugPrintln("");
  debugPrintln("RX Addresses: ");

  uint8_t rxAddr[5];
  for(int pipeId = 0; pipeId < 6; pipeId++) {
    nrf24_getRxAddress(pipeId, rxAddr);
    debugPrint("  RX_ADDR_P"); debugPrintDec(pipeId); debugPrint(": ");

    for(int i = 0; i < nrf24_getAddressWidths(); i++) {
      debugPrintHex(rxAddr[i]);
      debugPrint(" ");
    }

    debugPrintln("");
  }

  debugPrintln("TX Address: ");
  uint8_t txAddr[5];
  nrf24_getTxAddress(txAddr);
  debugPrint("  TX_ADDR: ");

  for(int i = 0; i < nrf24_getAddressWidths(); i++) {
    debugPrintHex(txAddr[i]);
    debugPrint(" ");
  }

  debugPrintln("");

  uint8_t fifoIsFull = nrf24_txFifoIsFull();
  debugPrint("TX_FIFO: "); debugPrintln(fifoIsFull ? "Full" : "Free");
}

static void initClocks() {
  rcc_clock_setup_in_hse_8mhz_out_72mhz();

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_SPI1);
  rcc_periph_clock_enable(RCC_AFIO);
}

static void initGpio() {
/*
  gpio_set_mode(RCC_GPIOB,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO0 | GPIO1); // DC / RST

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
*/

  gpio_set_mode(GPIO_LED_PORT,
                GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL,
                GPIO_LED_PIN);
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

void setup_nrf24() {
  nrf24_init();

  // uint8_t rxaddr1[] = { 0x00, 0x53, 0x00, 0x46, 0x09 };
  // uint8_t txaddr1[] = { 0x00, 0x53, 0x00, 0x46, 0x09 };
  // uint8_t rxaddr2[] = { 0x09, 0x46, 0x00, 0x53, 0x00 };
  // uint8_t txaddr2[] = { 0x09, 0x46, 0x00, 0x53, 0x00 };
  uint8_t rxaddr[] = {0x01, 0x02, 0x03, 0x02, 0x01 };
  uint8_t txaddr[] = {0x01, 0x02, 0x03, 0x02, 0x01 };

  nrf24_setRxAddress(PIPE_0, rxaddr);
  nrf24_setTxAddress(txaddr);
  nrf24_enableCRC(CRC_MODE_OFF);
  nrf24_enableDataPipe(PIPE_0, TRUE);
  nrf24_enableDataPipe(PIPE_1, FALSE);
  nrf24_enableShockburst(PIPE_0, FALSE);
  nrf24_setAddressWidth(5);
  nrf24_listenMode(TRUE);
  nrf24_setDataRate(SPEED_2M);
  nrf24_setPayloadSize(PIPE_0, 16);
  nrf24_setRFChannel(81);
  nrf24_powerUp(TRUE);

  debugPrintln("NRF24L01+ Library Example");
  debugPrintln("-------------------------");
  printFullConfig();
}

void init() {
  initClocks();
  initGpio();
//   initSpi();
//  nrf24_init();
//  setup_nrf24();
}

/*
void init_pwm() {
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

static void usbSample() {
  int i;
  const char* line;

  while (1) {
    usbd_poll(_pUsbd_dev);

    line = usb_serial_rx();
    if (line) {
      usb_serial_tx(_pUsbd_dev, "Roflcopter: ");
      usb_serial_tx(_pUsbd_dev, line);
      usb_serial_tx(_pUsbd_dev, "\r\n");
    }

    i++;
  }
}
*/

// ----------------

void delayMs(int timeMs) {
  for(int i = 0; i < timeMs; ++i) {
    for (int k = 0; k < 10250; k++) {
      __asm__("nop");
    }
  }
}

int main(void) {
  init();

  while(1) {
    gpio_clear(GPIO_LED_PORT, GPIO_LED_PIN);
    delayMs(500);
    gpio_set(GPIO_LED_PORT, GPIO_LED_PIN);
    delayMs(500);
  }

  const char* line;

  usbd_device *usbd_dev = usb_serial_init();

  while (1) {
    usbd_poll(usbd_dev);
    line = usb_serial_rx();

    if (line) {
      usb_serial_tx(usbd_dev, "Received a line: ");
      usb_serial_tx(usbd_dev, line);
      usb_serial_tx(usbd_dev, "\r\n");
    }

//    sendUartText("lol\n");
  }

//----------

  init();
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
  _pUsbd_dev = usb_serial_init();

  while(true) {
    sendUartText("lol\n");
  }

  init();

  int32_t recvByteCount;
  char recvBuffer[NRF_MAX_PAYLOAD_SIZE + 1];

  while(TRUE) {
    recvByteCount = nrf24_recvPacket(recvBuffer);
    recvBuffer[recvByteCount] = '\0';

    if(recvByteCount != NRF_NO_DATA_AVAILABLE) {
      // blink();

      debugPrint("Received [");
      debugPrintDec(recvByteCount);
      debugPrint("]: ");

      for(int i = 0; i < recvByteCount; ++i) {
        debugPrintHex(recvBuffer[i]);
        debugPrint(" ");
      }

      debugPrintln("");
    }
  }

  return 0;
}

