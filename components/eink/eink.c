/**
 ******************************************************************************
 *  file           : eink.c
 *  brief          : E-Ink Driver for the Vision Master E290
 ******************************************************************************
 *  Copyright (C) 2024 Florian Brandner
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "eink.h"

#include <stdio.h>
#include <string.h>

#include "../../main/sysconfig.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define LCD_HOST SPI2_HOST

#define TIMEOUT_HW_RESET 1000 // [ms] Timeout hardware reset
#define TIMEOUT_SW_RESET 1000 // [ms] Timeout software reset
#define TIMEOUT_REFRESH 5000  // [ms] Timeout full display refresh
#define TIMEOUT_POWERON 250   // [ms] Timeout for power on
#define TIMEOUT_POWEROFF 250  // [ms] Timeout for power off

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static const char* TAG = "EINK";
static spi_device_handle_t spi;
static uint8_t* pBuffer      = NULL;
static uint16_t bufferLength = 0;

/* Private function prototypes -----------------------------------------------*/

static void spi_pre_transfer_callback(spi_transaction_t* t);
static void hw_reset();
static void sw_reset();
static esp_err_t wait4busylow(uint16_t busy_timeout, const char* funcname);
static void write_cmd(const uint8_t cmd, bool keep_cs_active);
static void write_dat(const uint8_t* data, int len);
static void write_cmddat(const uint8_t cmd, const uint8_t* data, int len);
static void voltage_on();
static void voltage_off();
static void set_params();
static void update_full();
static void power_on();
static void power_off();
static void send_buffer();

/* Private user code ---------------------------------------------------------*/

/**
 * @brief Turn display voltage ON
 */
static void voltage_on() {
  ESP_LOGD(TAG, "Voltage ON");
  gpio_set_level(PIN_VE_CTRL, 1);
  vTaskDelay(10 / portTICK_PERIOD_MS);
}

/**
 * @brief Turn display voltage OFF
 */
static void voltage_off() {
  ESP_LOGD(TAG, "Voltage OFF");
  gpio_set_level(PIN_VE_CTRL, 0);
  vTaskDelay(10 / portTICK_PERIOD_MS);
}

/**
 * @brief Send a command
 *
 * @param cmd Command to send
 * @param keep_cs_active Keep CS active after transmission
 */
static void write_cmd(const uint8_t cmd, bool keep_cs_active) {
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t)); // Zero out the transaction
  t.length    = 8;          // Command is 8 bits
  t.tx_buffer = &cmd;       // The data is the cmd itself
  t.user      = (void*)0;   // D/C needs to be set to 0
  if (keep_cs_active) {
    t.flags = SPI_TRANS_CS_KEEP_ACTIVE; // Keep CS active after data transfer
  }
  ret = spi_device_polling_transmit(spi, &t); // Transmit!
  assert(ret == ESP_OK);                      // Should have had no issues.
}

/**
 * @brief Send command and data
 *
 * @param cmd Command to send
 * @param data Pointer to the data
 * @param len Length of data in byte
 */
static void write_cmddat(const uint8_t cmd, const uint8_t* data, int len) {
  ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));
  write_cmd(cmd, true);
  write_dat(data, len);
  spi_device_release_bus(spi);
}

/**
 * @brief Transmit data
 *
 * @param data Pointer to the data
 * @param len Length of data in byte
 */
static void write_dat(const uint8_t* data, int len) {
  esp_err_t ret;
  spi_transaction_t t;
  if (len == 0) {
    return; // no need to send anything
  }
  memset(&t, 0, sizeof(t));                           // Zero out transaction
  t.length    = len * 8;                              // Transaction length in bits
  t.tx_buffer = data;                                 // Data
  t.user      = (void*)1;                             // D/C needs to be set to 1
  ret         = spi_device_polling_transmit(spi, &t); // Transmit
  assert(ret == ESP_OK);                              // Should have had no issues
}

/**
 * @brief Called just before a transmission start to take care of D/C
 *
 * @note in irq context
 */
static void spi_pre_transfer_callback(spi_transaction_t* t) {
  int dc = (int)t->user;
  gpio_set_level(PIN_DC, dc);
}

/**
 * @brief Reset the display by hardware pin
 */
static void hw_reset() {
  ESP_LOGD(TAG, "HW Reset");
  gpio_set_level(PIN_RST, 0);
  vTaskDelay(20 / portTICK_PERIOD_MS);
  gpio_set_level(PIN_RST, 1);
  vTaskDelay(20 / portTICK_PERIOD_MS);
  wait4busylow(TIMEOUT_HW_RESET, __FUNCTION__);
  vTaskDelay(10 / portTICK_PERIOD_MS);
}

/**
 * @brief Reset the display by software
 */
static void sw_reset() {
  ESP_LOGD(TAG, "SW Reset");
  write_cmd(0x12, false);
  wait4busylow(TIMEOUT_SW_RESET, __FUNCTION__);
  vTaskDelay(10 / portTICK_PERIOD_MS);
}

/**
 * @brief Wait for BUSY going low
 *
 * @param busy_timeout Timeout in ms
 * @param funcname Name of calling func (for debugging)
 * @return esp_err_t ESP_OK on success, ESP_FAIL otherwise
 */
static esp_err_t wait4busylow(uint16_t busy_timeout, const char* funcname) {
  const long timeout = xTaskGetTickCount() + (busy_timeout / portTICK_PERIOD_MS);
  while (1) {
    if (gpio_get_level(PIN_BUSY) == 0) {
      return ESP_OK;
    }

    vTaskDelay(1);

    if (xTaskGetTickCount() >= timeout) {
      ESP_LOGW(TAG, "Wait for busy TIMEOUT in '%s'", funcname);
      return ESP_FAIL;
    }
  }
  return ESP_OK;
}

/**
 * @brief Full display update
 */
static void update_full() {
  ESP_LOGD(TAG, "Full Update");
  uint8_t data = 0xf7;
  write_cmddat(0x22, &data, 1);
  wait4busylow(TIMEOUT_SW_RESET, __FUNCTION__);

  write_cmd(0x20, false); // Master activation
  wait4busylow(TIMEOUT_REFRESH, __FUNCTION__);
  vTaskDelay(100 / portTICK_PERIOD_MS);
}

/**
 * @brief Set Display Configuration
 */
static void set_params() {
  uint8_t data[4];

  ESP_LOGD(TAG, "Setting Parameters");

  // Driver output control
  data[0] = 0x27; // Set A=0x127
  data[1] = 0x01; // Set B=0x00
  data[2] = 0x00;
  write_cmddat(0x01, &data[0], 3);

  // BorderWaveform
  data[0] = 0x05;
  write_cmddat(0x3C, &data[0], 1);

  //  Display update control
  data[0] = 0x00;
  data[1] = 0x80;
  write_cmddat(0x21, &data[0], 2);

  // Temperature Sensor Control
  data[0] = 0x80; // Internal Temp Sensor
  write_cmddat(0x18, &data[0], 1);

  // RAM parameters
  uint8_t XStart  = 0;
  uint8_t XEnd    = (EINK_SIZE_X - 1) / 8;
  uint16_t YStart = (EINK_SIZE_Y - 1);
  uint16_t YEnd   = 0;
  uint8_t XCount  = 0x00;
  uint16_t YCount = EINK_SIZE_Y;

  // RAM X Start/End
  data[0] = XStart;
  data[1] = XEnd;
  write_cmddat(0x44, &data[0], 2);

  // RAM Y Start/End
  data[0] = YStart % 256;
  data[1] = YStart / 256;
  data[2] = YEnd % 256;
  data[3] = YEnd / 256;
  write_cmddat(0x45, &data[0], 4);

  // RAM X address counter
  write_cmddat(0x4E, &XCount, 1);

  // RAM Y address counter
  data[0] = YCount % 256;
  data[1] = YCount / 256;
  write_cmddat(0x4F, &data[0], 2);
}

/**
 * @brief Power ON display
 */
static void power_on(void) {
  uint8_t data;
  data = 0xc0;

  write_cmddat(0x22, &data, 1);
  write_cmd(0x20, false);

  wait4busylow(TIMEOUT_POWERON, __FUNCTION__);
  vTaskDelay(10 / portTICK_PERIOD_MS);
}

/**
 * @brief Power OFF display
 */
static void power_off(void) {
  uint8_t data;

  write_cmd(0x22, true);
  data = 0x83;
  write_dat(&data, 1);
  write_cmd(0x20, false);
  //   wait4busylow(TIMEOUT_POWEROFF, __FUNCTION__);
}

/**
 * @brief Send the buffer to the display
 * @note The Data is transmitted with Y reversed
 */
static void send_buffer() {
  if (NULL == pBuffer)
    return;
  if (0 == bufferLength)
    return;

  static uint16_t tx_size = EINK_SIZE_X / 8;        // Transmit ONE line
  uint8_t* pData          = pBuffer + bufferLength; // Start at the end

  ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));
  write_cmd(0x24, true);
  do {
    pData -= tx_size;
    write_dat(pData, tx_size);
  } while (pData > pBuffer);
  spi_device_release_bus(spi);
  wait4busylow(TIMEOUT_REFRESH, __FUNCTION__);
}

/* Public user code ----------------------------------------------------------*/

void eink_fillscreen(const uint8_t value) {
  voltage_on();
  hw_reset();
  sw_reset();
  power_on();
  set_params();

  ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));
  write_cmd(0x24, true);
  for (uint16_t i = 0; i < (EINK_SIZE_X / 8 * EINK_SIZE_Y); i++) {
    write_dat(&value, 1);
  }
  spi_device_release_bus(spi);
  wait4busylow(TIMEOUT_REFRESH, __FUNCTION__);
  update_full();
  voltage_off();
}

void eink_setbuffer(uint8_t* pBuf, const uint16_t len) {
  // The buffer size MUST match!
  assert(len == (EINK_SIZE_X / 8 * EINK_SIZE_Y));
  pBuffer      = pBuf;
  bufferLength = len;
}

void eink_update() {
  ESP_LOGD(TAG, "Updating");

  // Power ON Display
  voltage_on();

  // Reset the display
  hw_reset();
  sw_reset();
  power_on();

  // Set displays params
  set_params();

  send_buffer();
  update_full();

  // Power OFF Display
  voltage_off();
}

void eink_init(void) {
  esp_err_t ret;
  spi_bus_config_t buscfg = {
      .miso_io_num   = PIN_MISO,
      .mosi_io_num   = PIN_MOSI,
      .sclk_io_num   = PIN_CLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
  };
  spi_device_interface_config_t devcfg = {
      .clock_speed_hz = 10 * 1000 * 1000,          // Clock out at 10 MHz
      .mode           = 0,                         // SPI mode 0
      .spics_io_num   = PIN_CS,                    // CS pin
      .queue_size     = 7,                         // We want to be able to queue 7 transactions at a time
      .pre_cb         = spi_pre_transfer_callback, // Specify pre-transfer callback to handle D/C line
  };

  // Initialize the SPI bus
  ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);

  // Attach the Display to the SPI bus
  ret = spi_bus_add_device(LCD_HOST, &devcfg, &spi);
  ESP_ERROR_CHECK(ret);

  // D/C OUT
  ESP_ERROR_CHECK(gpio_reset_pin(PIN_DC));
  ESP_ERROR_CHECK(gpio_set_direction(PIN_DC, GPIO_MODE_OUTPUT));

  // Voltage Control
  ESP_ERROR_CHECK(gpio_reset_pin(PIN_VE_CTRL));
  ESP_ERROR_CHECK(gpio_set_direction(PIN_VE_CTRL, GPIO_MODE_OUTPUT));

  // Reset OUT
  ESP_ERROR_CHECK(gpio_reset_pin(PIN_RST));
  ESP_ERROR_CHECK(gpio_set_direction(PIN_RST, GPIO_MODE_OUTPUT));

  // Busy IN
  ESP_ERROR_CHECK(gpio_reset_pin(PIN_BUSY));
  ESP_ERROR_CHECK(gpio_set_pull_mode(PIN_BUSY, GPIO_PULLUP_ONLY));
  ESP_ERROR_CHECK(gpio_set_direction(PIN_BUSY, GPIO_MODE_INPUT));

  ESP_LOGI(TAG, "Initialised");
}
