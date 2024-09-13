/**
 ******************************************************************************
 *  file           : bluetooth.c
 *  brief          : Bluetooth component for the Victron Display
 ******************************************************************************
 *  Copyright (C) 2024 Florian Brandner
 *
 */

/* Includes ------------------------------------------------------------------*/

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../../main/sysconfig.h"
#include "aes/esp_aes.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "victron.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define SCAN_TIME 60    // [s] Scanning time
#define MAX_NAME_LEN 30 // Max length of name

/* Private macro -------------------------------------------------------------*/

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

/* Private variables ---------------------------------------------------------*/
static const char* TAG = "BLUE";

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type          = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type      = BLE_ADDR_TYPE_RPA_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval      = 0x50,
    .scan_window        = 0x30,
    .scan_duplicate     = BLE_SCAN_DUPLICATE_ENABLE, // TODO: Test necessary!
};

/* Private function prototypes -----------------------------------------------*/

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);
bool isVictronFrame(const VICTRON_BLE_RECORD* pFrame);
void parseVictronFrame(const VICTRON_BLE_RECORD* pFrame, const uint16_t frameLen, const char* pName,
                       const esp_bd_addr_t BLEAddr);

/* Private user code ---------------------------------------------------------*/

// Test if a frame is a valid victron device data frame
bool isVictronFrame(const VICTRON_BLE_RECORD* pFrame) {
  if (pFrame->manu_id != MANUFACTURER_ID) {
    ESP_LOGI(TAG, "Manufacturer 0x%04x is invalid", pFrame->manu_id);
    return false;
  }
  if (pFrame->manu_record_type != MANUFACTURER_RTYPE) {
    ESP_LOGI(TAG, "Record Type 0x%02x is invalid", pFrame->manu_record_type);
    return false;
  }
  if (pFrame->manu_record_length != MANUFACTURER_RLEN) {
    ESP_LOGI(TAG, "Record Length 0x%02x is invalid", pFrame->manu_record_length);
    return false;
  }
  ESP_LOGI(TAG, "Frame is a Victron frame!");
  return true;
}

// Parses the data in a Victron Frame
void parseVictronFrame(const VICTRON_BLE_RECORD* pFrame, const uint16_t frameLen, const char* pName,
                       const esp_bd_addr_t BLEAddr) {
  char address[18]   = {0};
  uint8_t encKey[16] = {0};

  const int16_t encLen    = (1 + frameLen - sizeof(VICTRON_BLE_RECORD));
  const uint8_t* pEncData = &pFrame->encData;

  snprintf(&address[0], 18, "%02x-%02x-%02x-%02x-%02x-%02x", BLEAddr[0], BLEAddr[1], BLEAddr[2], BLEAddr[3], BLEAddr[4],
           BLEAddr[5]);

  // Debug-Print content
  ESP_LOGI(TAG, "          Name = '%s'", pName);
  ESP_LOGI(TAG, "          Addr = '%s'", address);
  ESP_LOGI(TAG, "    Product ID = 0x%04x", pFrame->product_id);
  ESP_LOGI(TAG, "   Record Type = 0x%02x", pFrame->record_type);
  ESP_LOGI(TAG, "      Data Cnt = 0x%02x:%02x", pFrame->data_counter_msb, pFrame->data_counter_lsb);
  ESP_LOGI(TAG, "   Encr Key B0 = 0x%02x", pFrame->encryption_key_0);

  if (encLen <= 0) {
    ESP_LOGW(TAG, "Length of encrypted data %d invalid!", encLen);
    return;
  }

  ESP_LOGI(TAG, "Encrypted Data:");
  ESP_LOG_BUFFER_HEXDUMP(TAG, pEncData, encLen, ESP_LOG_INFO);

  // Find encryption key in NVS
  // TODO: Implement me!

  // Decrypt data
  // TODO: Move to func
  {
    esp_aes_context ctx;
    esp_aes_init(&ctx);
    int status = esp_aes_setkey(&ctx, encKey, sizeof(encKey) * 8);
    if (status != 0) {
      ESP_LOGE(TAG, "Error %i when setting key!", status);
      esp_aes_free(&ctx);
      return;
    }

    size_t nc_offset          = 0;
    uint8_t nonce_counter[16] = {0};
    uint8_t stream_block[16]  = {0};
    uint8_t decData[16]       = {0};

    nonce_counter[0] = pFrame->data_counter_lsb;
    nonce_counter[1] = pFrame->data_counter_msb;

    status = esp_aes_crypt_ctr(&ctx, encLen, &nc_offset, nonce_counter, stream_block, pEncData, &decData[0]);
    if (status != 0) {
      ESP_LOGE(TAG, "Error %i at esp_aes_crypt_ctr", status);
      esp_aes_free(&ctx);
      return;
    }

    esp_aes_free(&ctx);

    ESP_LOGI(TAG, "Decrypted Data:");
    ESP_LOG_BUFFER_HEXDUMP(TAG, &decData[0], encLen, ESP_LOG_INFO);
  }

  // TODO: Update Data Model

  return;
}

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
  switch (event) {
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
      //   ESP_LOGI(TAG, "Scan result received");
      esp_ble_gap_cb_param_t* scan_result = (esp_ble_gap_cb_param_t*)param;
      switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT: {
          uint8_t* adv_name     = NULL;
          uint8_t* manu_data    = NULL;
          uint8_t adv_name_len  = 0;
          uint8_t manu_data_len = 0;

          adv_name  = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
          manu_data = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE,
                                               &manu_data_len);

          if ((adv_name_len == 0) || (adv_name == NULL)) {
            ESP_LOGD(TAG, "Advertisement not relevant (No name)");
          } else if ((manu_data_len == 0) || (manu_data == NULL)) {
            ESP_LOGD(TAG, "Advertisement not relevant (No manufacturer data)");
          } else {
            if (manu_data_len >= sizeof(VICTRON_BLE_RECORD)) {
              // Test if valid frame and parse
              if (isVictronFrame((VICTRON_BLE_RECORD*)manu_data)) {
#if 0
                ESP_LOGW(TAG, "Advertisement received:");
                ESP_LOG_BUFFER_HEXDUMP(TAG, scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len,
                                       ESP_LOG_INFO);
                ESP_LOGW(TAG, "Manufacturer Data:");
                ESP_LOG_BUFFER_HEXDUMP(TAG, manu_data, manu_data_len, ESP_LOG_INFO);
#endif
                char devName[MAX_NAME_LEN];
                memset(&devName[0], 0x00, MAX_NAME_LEN);
                memcpy(&devName[0], adv_name, MIN(adv_name_len, MAX_NAME_LEN));
                parseVictronFrame((VICTRON_BLE_RECORD*)manu_data, manu_data_len, devName, scan_result->scan_rst.bda);
              }
            } else {
              ESP_LOGI(TAG, "Manufacturer data too small! %d < %d", manu_data_len, sizeof(VICTRON_BLE_RECORD));
            }
          }
          break;
        }
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
          ESP_LOGI(TAG, "Scanning loop ended, restarting!");
          esp_ble_gap_start_scanning(SCAN_TIME);
          break;
        default:
          ESP_LOGW(TAG, "Unknown search event 0x%x", scan_result->scan_rst.search_evt);

          break;
      } // switch scan_result->scan_rst.search_evt
      break;
    }

    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
      ESP_LOGD(TAG, "Scan started");
      break;
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
      ESP_LOGD(TAG, "Scan params set");
      break;
    default:
      ESP_LOGW(TAG, "Unsupported GAP event %d!", (uint16_t)event);
      break;
  } // switch event
}

/* Public user code ----------------------------------------------------------*/

void blue_init() {
  esp_err_t ret;

  // BT Controller Init
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

  ret = esp_bt_controller_init(&bt_cfg);
  if (ret) {
    ESP_LOGE(TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret) {
    ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
    return;
  }

  // Bluedroid Stack Init
  ret = esp_bluedroid_init();
  if (ret) {
    ESP_LOGE(TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_bluedroid_enable();
  if (ret) {
    ESP_LOGE(TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
    return;
  }

  // Register GAP callback
  ret = esp_ble_gap_register_callback(gap_cb);
  if (ret) {
    ESP_LOGE(TAG, "%s gap register error, error code = %x", __func__, ret);
    return;
  }

  // Configure Scan
  esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
  if (scan_ret) {
    ESP_LOGE(TAG, "set scan params error, error code = %x", scan_ret);
  }

  // Start Scan
  esp_ble_gap_start_scanning(SCAN_TIME);

  ESP_LOGI(TAG, "Initialised");
}