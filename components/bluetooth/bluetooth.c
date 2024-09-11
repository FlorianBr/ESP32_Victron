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

#define SCAN_TIME 30 // [s] Scanning time

/* Private macro -------------------------------------------------------------*/

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

/* Private user code ---------------------------------------------------------*/

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
  switch (event) {
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
      //   ESP_LOGI(TAG, "Scan result received");
      esp_ble_gap_cb_param_t* scan_result = (esp_ble_gap_cb_param_t*)param;
      switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
          char address[18];
          snprintf(&address[0], 18, "%02x-%02x-%02x-%02x-%02x-%02x", scan_result->scan_rst.bda[0],
                   scan_result->scan_rst.bda[1], scan_result->scan_rst.bda[2], scan_result->scan_rst.bda[3],
                   scan_result->scan_rst.bda[4], scan_result->scan_rst.bda[5]);

          ESP_LOGW(TAG, "Advertisement received from '%s':", address);
          ESP_LOG_BUFFER_HEXDUMP(TAG, scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len, ESP_LOG_INFO);

          // TODO: Test minimal length

          // TODO: Test manufacturer ID

          // TODO: Call parser with manufacturer data

          // uint8_t* adv_name    = NULL;
          // uint8_t adv_name_len = 0;

          // adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL,
          // &adv_name_len);

          // if (adv_name_len > 0) {
          //   ESP_LOGI(TAG, "Found device:");
          //   ESP_LOGI(TAG, "   Type = %d", scan_result->scan_rst.dev_type);

          //   if (adv_name != NULL) {
          //     ESP_LOGI(TAG, "   Name = '%s'", adv_name);
          //   }
          //   ESP_LOGI(TAG, "   Addr:");
          //   ESP_LOG_BUFFER_HEXDUMP(TAG, scan_result->scan_rst.bda, 6, ESP_LOG_INFO);

          //   ESP_LOGI(TAG, "   ADV Data:");
          //   ESP_LOG_BUFFER_HEXDUMP(TAG, scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len,
          //                          ESP_LOG_INFO);
          // }
          break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
          ESP_LOGI(TAG, "Scanning loop ended, restarting!");
          esp_ble_gap_start_scanning(SCAN_TIME);
          break;
        default:
          ESP_LOGW(TAG, "Unknown search event 0x%x", scan_result->scan_rst.search_evt);

          break;
      }
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
  }
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