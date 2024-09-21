/**
 ******************************************************************************
 *  file           : bluetooth.c
 *  brief          : Bluetooth component for the Victron Display
 ******************************************************************************
 *  Copyright (C) 2024 Florian Brandner
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "./include/bluetooth.h"

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

/* Private define ------------------------------------------------------------*/

#define SCAN_TIME 0     // [s] Scanning time (0=forever)
#define MAX_NAME_LEN 30 // Max length of name
#define KEY_SIZE 16     // size of key in byte

/* Private typedef -----------------------------------------------------------*/

typedef struct {
  esp_bd_addr_t Addr;    // The Bluetooth address
  uint8_t Key[KEY_SIZE]; // The encryption key
} ENC_KEY_ENTRY;

/* Private macro -------------------------------------------------------------*/

#define MIN(a, b) (((a) < (b)) ? (a) : (b))

/* Private variables ---------------------------------------------------------*/

static const char* TAG = "BTLE";

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type          = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type      = BLE_ADDR_TYPE_RPA_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval      = 0x50,
    .scan_window        = 0x30,
    .scan_duplicate     = BLE_SCAN_DUPLICATE_DISABLE,
};

// Note: The Addresses and encryption keys depend on the setup!
static ENC_KEY_ENTRY enc_keys[] = {
    // SmartShunt
    {{0xde, 0x4b, 0xad, 0x98, 0xa0, 0xf2},
     {0x5c, 0x2d, 0x00, 0x0d, 0x3c, 0x1b, 0x8f, 0xe7, 0xf3, 0xcc, 0x0a, 0x86, 0x7a, 0x82, 0x0a, 0x60}},
    // OrionSmart
    {{0xfe, 0x88, 0xb0, 0x11, 0x11, 0x79},
     {0xb7, 0x35, 0xd0, 0xe4, 0x2e, 0x30, 0xd5, 0x0f, 0x41, 0xa4, 0x13, 0x62, 0x14, 0x02, 0xf0, 0xe8}},
    // SmartSolar
    {{0xdc, 0x4d, 0x47, 0x2a, 0x9b, 0x3e},
     {0xb5, 0x10, 0xa3, 0x56, 0xf0, 0x70, 0xb3, 0x86, 0x88, 0xdc, 0x9e, 0x2a, 0x94, 0x34, 0x55, 0x11}},
};

blue_sshunt_cb_t cb_shunt = NULL;
blue_ssolar_cb_t cb_solar = NULL;
blue_dcdc_cb_t cb_dcdc    = NULL;

/* Private function prototypes -----------------------------------------------*/

bool isVictronFrame(const VICTRON_BLE_RECORD* pFrame);
uint8_t* getEncKey(const esp_bd_addr_t Addr);
bool decrytData(const uint8_t* pEncData, uint8_t* pDecData, size_t length, const uint8_t cnt1, const uint8_t cnt2,
                const uint8_t* pKey);
void parseVictronFrame(const VICTRON_BLE_RECORD* pFrame, const uint16_t frameLen, const esp_bd_addr_t BLEAddr);
static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief Test if a frame is a valid victron device data frame
 *
 * @param pFrame Pointer to the frame data
 * @return true if a valid Victron Frame
 */
bool isVictronFrame(const VICTRON_BLE_RECORD* pFrame) {
  if (pFrame->manu_id != MANUFACTURER_ID) {
    ESP_LOGW(TAG, "Manufacturer 0x%04x is invalid", pFrame->manu_id);
    return false;
  }
  if (pFrame->manu_record_type != MANUFACTURER_RTYPE) {
    ESP_LOGW(TAG, "Record Type 0x%02x is invalid", pFrame->manu_record_type);
    return false;
  }
#if 0
  // Some frames have a different value but are still valid
  // The SmartSolar has a 0x00 instead of a 0x00 for example
  if (pFrame->manu_record_length != MANUFACTURER_RLEN) {
    ESP_LOGW(TAG, "Record Length 0x%02x is invalid", pFrame->manu_record_length);
    return false;
  }
#endif
  return true;
}

/**
 * @brief Get a encrytion key
 *
 * @param Addr BLE Address of the device
 * @return uint8_t* Pointer to the 16-Byte key or NULL
 */
uint8_t* getEncKey(const esp_bd_addr_t Addr) {
  for (size_t i = 0; i < (sizeof(enc_keys) / sizeof(ENC_KEY_ENTRY)); i++) {
    if (memcmp(&Addr[0], &enc_keys[i].Addr[0], sizeof(esp_bd_addr_t)) == 0) {
      ESP_LOGD(TAG, "Found key at index %d!", i);
      return &enc_keys[i].Key[0];
    }
  }
  ESP_LOGW(TAG, "Encryption key not found!");
  return NULL;
}

/**
 * @brief Decrypt the data with a specific key
 *
 * @param pEncData Pointer to the ENcrypted data
 * @param pDecData Pointer to the buffer to store the DEcrypted data
 * @param length Length of the data
 * @param cnt1 Nounce / Counter
 * @param cnt2 Nounce / Counter
 * @param pKey Pointer to the encryption key
 * @return true if decoded, false on error
 */
bool decrytData(const uint8_t* pEncData, uint8_t* pDecData, size_t length, const uint8_t cnt1, const uint8_t cnt2,
                const uint8_t* pKey) {
  esp_aes_context ctx;
  size_t nc_offset          = 0;
  uint8_t nonce_counter[16] = {0};
  uint8_t stream_block[16]  = {0};

  esp_aes_init(&ctx);
  int status = esp_aes_setkey(&ctx, pKey, KEY_SIZE * 8);
  if (status != 0) {
    ESP_LOGE(TAG, "Error %i when setting key!", status);
    esp_aes_free(&ctx);
    return false;
  }

  nonce_counter[0] = cnt2;
  nonce_counter[1] = cnt1;

  status = esp_aes_crypt_ctr(&ctx, length, &nc_offset, nonce_counter, stream_block, pEncData, pDecData);
  if (status != 0) {
    ESP_LOGE(TAG, "Error %i at esp_aes_crypt_ctr", status);
    esp_aes_free(&ctx);
    return false;
  }
  esp_aes_free(&ctx);

  return true;
}

/**
 * @brief Parses the data in a Victron Frame
 *
 * @param pFrame Pointer to the frame data
 * @param frameLen Length of Frame Data
 * @param BLEAddr BluetoothLE Address
 */
void parseVictronFrame(const VICTRON_BLE_RECORD* pFrame, const uint16_t frameLen, const esp_bd_addr_t BLEAddr) {
  const int16_t encLen = (1 + frameLen - sizeof(VICTRON_BLE_RECORD));

  // Debug-Print content
  ESP_LOGD(TAG, "Victron Frame Header:");
  ESP_LOGD(TAG, "    Product ID = 0x%04x", pFrame->product_id);
  ESP_LOGD(TAG, "   Record Type = 0x%02x", pFrame->record_type);
  ESP_LOGD(TAG, "      Data Cnt = 0x%02x:%02x", pFrame->data_counter_msb, pFrame->data_counter_lsb);
  ESP_LOGD(TAG, "   Encr Key B0 = 0x%02x", pFrame->encryption_key_0);

  if (encLen <= 0) {
    ESP_LOGW(TAG, "Length of encrypted data %d invalid!", encLen);
    return;
  }

  // Find encryption key and decrypt data
  uint8_t* pKey = getEncKey(BLEAddr);
  if (NULL != pKey) {
    if (*pKey != pFrame->encryption_key_0) {
      ESP_LOGW(TAG, "Key Byte 0 mismatch: 0x%02x != 0x%02x (PID=0x%04x)", *pKey, pFrame->encryption_key_0,
               pFrame->product_id);
      return;
    }

    uint8_t* pDecData = malloc(encLen);

    if (decrytData(&pFrame->encData, pDecData, encLen, pFrame->data_counter_msb, pFrame->data_counter_lsb, pKey)) {
      ESP_LOGD(TAG, "Decrypted Data:");
      ESP_LOG_BUFFER_HEXDUMP(TAG, pDecData, encLen, ESP_LOG_DEBUG);

      switch (pFrame->record_type) {
        case VREG_RTYPE_SOLAR_CHARGER: {
          if (encLen == sizeof(victron_solar_charger_t)) {
            victron_solar_charger_t* pData = (victron_solar_charger_t*)pDecData;
            if (cb_solar != NULL) {
              cb_solar(pData->dev_state, pData->bat_voltage, pData->bat_current, pData->pv_power,
                       (pData->charger_error != 0));
            }
          } else {
            ESP_LOGW(TAG, "Record size mismatch for Solar Charger! %d != %d", encLen, sizeof(victron_solar_charger_t));
          }
          break;
        }
        case VREG_RTYPE_BATTERY_MONITOR: {
          if (encLen == sizeof(victron_battery_monitor_t)) {
            uint16_t temp                    = 0;
            victron_battery_monitor_t* pData = (victron_battery_monitor_t*)pDecData;

            if (pData->alarm != 0) {
              ESP_LOGW(TAG, "SmartShunt signals an Alarm = 0x%04x", pData->alarm);
            }
            if (pData->aux_in == 2) {
              temp = pData->aux_value;
            }

            // Update Mains Data Model
            if (cb_shunt != NULL) {
              cb_shunt(pData->bat_voltage, pData->bat_current, pData->soc, temp, pData->ttg, (pData->alarm != 0));
            }
          } else {
            ESP_LOGW(TAG, "Record size mismatch for Battery Monitor! %d != %d", encLen,
                     sizeof(victron_battery_monitor_t));
          }
          break;
        }
        case VREG_RTYPE_DCDC_CONVERTER: {
          if (encLen == sizeof(victron_dcdc_t)) {
            victron_dcdc_t* pData = (victron_dcdc_t*)pDecData;
            if (pData->charger_error != 0) {
              ESP_LOGW(TAG, "DC/DC Converter signals an Alarm = 0x%02x", pData->charger_error);
            }
            // Update Mains Data Model
            if (cb_dcdc != NULL) {
              cb_dcdc(pData->dev_state, pData->in_voltage, pData->out_voltage, pData->offreason,
                      (pData->charger_error != 0));
            }
          } else {
            ESP_LOGW(TAG, "Record size mismatch for DC-DC Converter! %d != %d", encLen, sizeof(victron_dcdc_t));
          }
          break;
        }
        default:
          ESP_LOGW(TAG, "Record type 0x%02x not supported!", pFrame->record_type);
          break;
      }
    } else {
      ESP_LOGE(TAG, "Unable to decrypted Data!");
    }
    free(pDecData);
  }
  return;
}

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
  switch (event) {
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
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
                char address[18] = {0};
                snprintf(&address[0], 18, "%02x-%02x-%02x-%02x-%02x-%02x", scan_result->scan_rst.bda[0],
                         scan_result->scan_rst.bda[1], scan_result->scan_rst.bda[2], scan_result->scan_rst.bda[3],
                         scan_result->scan_rst.bda[4], scan_result->scan_rst.bda[5]);

                ESP_LOGD(TAG, "Found '%s' with address '%s'", adv_name, address);
                parseVictronFrame((VICTRON_BLE_RECORD*)manu_data, manu_data_len, scan_result->scan_rst.bda);
              } else {
                ESP_LOGW(TAG, "Device '%s' is not sending a valid victron frame!", adv_name);
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
  } // switch event
}

/* Public user code ----------------------------------------------------------*/

void blue_setcb_sshunt(blue_sshunt_cb_t cb) {
  cb_shunt = cb;
}

void blue_setcb_ssolar(blue_ssolar_cb_t cb) {
  cb_solar = cb;
}

void blue_setcb_dcdc(blue_dcdc_cb_t cb) {
  cb_dcdc = cb;
}

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