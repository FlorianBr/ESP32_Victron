/**
 ******************************************************************************
 *  file           : main.c
 *  brief          : Main for the Victron Display
 ******************************************************************************
 *  Copyright (C) 2024 Florian Brandner
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#include "bluetooth.h"
#include "driver/gpio.h"
#include "eink.h"
#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "sysconfig.h"

/* Private typedef -----------------------------------------------------------*/

typedef struct {
  uint16_t voltage; // [0.1V] Voltage
  int32_t current;  // [0.1A] Charge/discharge current
  uint16_t soc;     // [0.1%] Battery capacity
  uint16_t temp;    // [0.1dC] Temperature
  uint16_t remain;  // [min] Remaining time
  TickType_t time;  // [Tick] Last update
} data_shunt_t;

/* Private define ------------------------------------------------------------*/

#define LOOP_TIME 5000            // [ms] Main loop time
#define OSSTATS_ARRAY_SIZE_OFFS 5 // OS-Statistics: Increase this if print_real_time_stats returns ESP_ERR_INVALID_SIZE
#define OSSTATS_TIME 30000        // [ms] OS-Statistics cycle time

#define EINK_BUFFER_SIZE ((EINK_SIZE_X / 8 * EINK_SIZE_Y) + 8) // Buffer Size: 1 bit per pixel plus 8 byte for LVGL

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static const char* TAG = "MAIN";
static uint8_t einkBuffer[EINK_BUFFER_SIZE];
static data_shunt_t data_shunt;

/* Private function prototypes -----------------------------------------------*/

static void TaskOSStats(void* pvParameters);
void shunt_cb(const uint16_t volt, const int32_t curr, const uint16_t soc, const uint16_t temp, const uint16_t remain);
void solar_cb(const uint8_t state, const uint16_t volt, const uint16_t curr);
void flush_cb(lv_display_t* display, const lv_area_t* area, uint8_t* px_map);

/* Private user code ---------------------------------------------------------*/

// Task to print OS statistics
static void TaskOSStats(void* pvParameters) {
  while (1) {
    TaskStatus_t *start_array = NULL, *end_array = NULL;
    UBaseType_t start_array_size, end_array_size;
    uint32_t start_run_time, end_run_time;

    // Allocate array to store current task states
    start_array_size = uxTaskGetNumberOfTasks() + OSSTATS_ARRAY_SIZE_OFFS;
    start_array      = malloc(sizeof(TaskStatus_t) * start_array_size);
    if (start_array == NULL) {
      ESP_LOGE(TAG, "OSStats: Out of memory!\r\n");
      goto exit;
    }

    // Get current task states
    start_array_size = uxTaskGetSystemState(start_array, start_array_size, &start_run_time);
    if (start_array_size == 0) {
      ESP_LOGE(TAG, "OSStats: Invalid size!\r\n");
      goto exit;
    }

    vTaskDelay(OSSTATS_TIME / portTICK_PERIOD_MS);

    // Allocate array to store tasks states post delay
    end_array_size = uxTaskGetNumberOfTasks() + OSSTATS_ARRAY_SIZE_OFFS;
    end_array      = malloc(sizeof(TaskStatus_t) * end_array_size);
    if (end_array == NULL) {
      ESP_LOGE(TAG, "OSStats: Out of memory!\r\n");
      goto exit;
    }

    // Get post delay task states
    end_array_size = uxTaskGetSystemState(end_array, end_array_size, &end_run_time);
    if (end_array_size == 0) {
      ESP_LOGE(TAG, "OSStats: Invalid size!\r\n");
      goto exit;
    }

    // Calculate total_elapsed_time in units of run time stats clock period.
    uint32_t total_elapsed_time = (end_run_time - start_run_time);
    if (total_elapsed_time == 0) {
      ESP_LOGE(TAG, "OSStats: Invalid state!\r\n");
      goto exit;
    }

    printf(
        "+------------------------------------------------------------------"
        "-+\n");
    printf(
        "| Task              |   Run Time  | Percentage | State |      "
        "Stack |\n");
    printf(
        "+------------------------------------------------------------------"
        "-+\n");
    // Match each task in start_array to those in the end_array
    for (int i = 0; i < start_array_size; i++) {
      int k = -1;
      for (int j = 0; j < end_array_size; j++) {
        if (start_array[i].xHandle == end_array[j].xHandle) {
          k = j;
          // Mark that task have been matched by overwriting their handles
          start_array[i].xHandle = NULL;
          end_array[j].xHandle   = NULL;
          break;
        }
      }
      // Check if matching task found
      if (k >= 0) {
        char OutputLine[80];
        char charbuffer[20];

        uint32_t task_elapsed_time = end_array[k].ulRunTimeCounter - start_array[i].ulRunTimeCounter;
        uint32_t percentage_time   = (task_elapsed_time * 100UL) / (total_elapsed_time * portNUM_PROCESSORS);

        memset(&OutputLine[0], ' ', sizeof(OutputLine));

        // Task Name
        OutputLine[0] = '|';
        memcpy(&OutputLine[2], start_array[i].pcTaskName, strlen(start_array[i].pcTaskName));

        // Time (absolute)
        snprintf(&charbuffer[0], sizeof(charbuffer), "| %10lu", task_elapsed_time);
        memcpy(&OutputLine[20], &charbuffer[0], strlen(charbuffer));

        // Time (percentage)
        snprintf(&charbuffer[0], sizeof(charbuffer), "| %5lu", percentage_time);
        memcpy(&OutputLine[34], &charbuffer[0], strlen(charbuffer));

        // Task State
        OutputLine[47] = '|';
        switch (start_array[i].eCurrentState) {
          case eRunning:
            memcpy(&OutputLine[49], "Run", 3);
            break;
          case eReady:
            memcpy(&OutputLine[49], "Rdy", 3);
            break;
          case eBlocked:
            memcpy(&OutputLine[49], "Blk", 3);
            break;
          case eSuspended:
            memcpy(&OutputLine[49], "Sus", 3);
            break;
          case eDeleted:
            memcpy(&OutputLine[49], "Del", 3);
            break;
          default:
            memcpy(&OutputLine[49], "Ukn", 3);
            break;
        } // switch

        // Stack Usage
        snprintf(&charbuffer[0], sizeof(charbuffer), "| %10lu", start_array[i].usStackHighWaterMark);
        memcpy(&OutputLine[55], &charbuffer[0], strlen(charbuffer));

        OutputLine[68] = '|';
        OutputLine[69] = '\n';
        OutputLine[70] = '\0';

        printf(&OutputLine[0]);
      }
    } // for

    // Print unmatched tasks
    for (int i = 0; i < start_array_size; i++) {
      if (start_array[i].xHandle != NULL) {
        printf("| %s Deleted\n", start_array[i].pcTaskName);
      }
    }
    for (int i = 0; i < end_array_size; i++) {
      if (end_array[i].xHandle != NULL) {
        printf("| %s Created\n", end_array[i].pcTaskName);
      }
    }
#if 1
    printf(
        "+------------------------------------------------------------------"
        "-+\n");

    printf("| HEAP           Free    MinFree     MaxBlk\n");
    printf("| All      %10d %10d %10d\n", heap_caps_get_free_size(MALLOC_CAP_8BIT),
           heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    printf("| Internal %10d %10d %10d\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL), heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    printf("| SPI      %10d %10d %10d\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
           heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM), heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));
    printf(
        "+------------------------------------------------------------------"
        "-+\n\n");
#endif
  exit: // Common return path
    free(start_array);
    free(end_array);
  } // while (1)
} // TaskOSStats()

// Send LVGL data to the display
void flush_cb(lv_display_t* display, const lv_area_t* area, uint8_t* px_map) {
  eink_setbuffer(px_map + 8, EINK_BUFFER_SIZE - 8);
  eink_update();

  // Inform LVGL that you are ready with the flushing and buf is not used anymore
  lv_display_flush_ready(display);
}

// Callback to set smartshunts measurement data
void shunt_cb(const uint16_t volt, const int32_t curr, const uint16_t soc, const uint16_t temp, const uint16_t remain) {
  data_shunt.voltage = volt;
  data_shunt.current = curr;
  data_shunt.temp    = temp;
  data_shunt.soc     = soc;
  data_shunt.remain  = remain;
  data_shunt.time    = xTaskGetTickCount();
}

// Callback to set solar charger  measurement data
void solar_cb(const uint8_t state, const uint16_t volt, const uint16_t curr) {
  ESP_LOGI(TAG, "SmartSolar callback!");
}

/* Public user code ----------------------------------------------------------*/

void app_main(void) {
  esp_err_t ret = ESP_OK;

#if 1
  // Print Chip Info
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  ESP_LOGI(TAG, "------------------------------------- System Info:");
  ESP_LOGI(TAG, "    %s chip with %d CPU cores, WiFi%s%s%s%s%s%s, ", CONFIG_IDF_TARGET, chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "", (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "/FLASH" : "",
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "/WiFi" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? "/WPAN" : "",
           (chip_info.features & CHIP_FEATURE_EMB_PSRAM) ? "/PSRAM" : "");
  ESP_LOGI(TAG, "    Heap: %lu", esp_get_free_heap_size());
  ESP_LOGI(TAG, "    Reset reason: %d", esp_reset_reason());

  // Print Partition Info
  esp_ota_img_states_t ota_state;
  esp_partition_t* part_info = (esp_partition_t*)esp_ota_get_boot_partition();
  esp_ota_get_state_partition(part_info, &ota_state);
  if (NULL != part_info) {
    ESP_LOGI(TAG, "------------------------------------- Current partition:");
    ESP_LOGI(TAG, "    Label = %s, state = %d", part_info->label, ota_state);
    ESP_LOGI(TAG, "    Address=0x%lx, size=0x%lx", part_info->address, part_info->size);
  }
  ESP_LOGI(TAG, "-------------------------------------");

  // Initialize NVS, format it if necessary
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "Erasing NVS!");
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  ESP_LOGI(TAG, "NVS init returned %d", ret);

  // Print out NVS statistics
  nvs_stats_t nvs_stats;
  nvs_get_stats("nvs", &nvs_stats);
  ESP_LOGI(TAG, "-------------------------------------");
  ESP_LOGI(TAG, "NVS Statistics:");
  ESP_LOGI(TAG, "NVS Used = %d", nvs_stats.used_entries);
  ESP_LOGI(TAG, "NVS Free = %d", nvs_stats.free_entries);
  ESP_LOGI(TAG, "NVS All = %d", nvs_stats.total_entries);

  nvs_iterator_t iter = NULL;
  esp_err_t res       = nvs_entry_find("nvs", NULL, NVS_TYPE_ANY, &iter);
  while (res == ESP_OK) {
    nvs_entry_info_t info;
    nvs_entry_info(iter, &info);
    ESP_LOGI(TAG, "Key '%s', Type '%d'", info.key, info.type);
    res = nvs_entry_next(&iter);
  }
  nvs_release_iterator(iter);
  ESP_LOGI(TAG, "-------------------------------------");
#endif

#if 0 // Set NVS values here
  {
    nvs_handle_t handle;
    ESP_ERROR_CHECK(nvs_open("SETTINGS", NVS_READWRITE, &handle));
    ESP_ERROR_CHECK(nvs_set_str(handle, "WIFI_SSID", "<SSID>"));
    ESP_ERROR_CHECK(nvs_set_str(handle, "WIFI_PASS", "<Secret!>"));
    nvs_close(handle);
  }
#endif

  // Start OS Statistics Task
  xTaskCreate(TaskOSStats, "FreeRTOS Stats", 4096, NULL, tskIDLE_PRIORITY, NULL);

  // The LED Output
  gpio_reset_pin(PIN_LED);
  gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);

  // Init EINK SPI driver
  eink_init();

  // Bluetooth
  blue_init();
  blue_setcb_sshunt(shunt_cb);
  blue_setcb_ssolar(solar_cb);

  // Init LVGL
  lv_init();
  lv_display_t* display = lv_display_create(EINK_SIZE_X, EINK_SIZE_Y);
  lv_display_set_flush_cb(display, flush_cb);
  lv_display_set_buffers(display, &einkBuffer[0], NULL, EINK_BUFFER_SIZE, LV_DISPLAY_RENDER_MODE_FULL);
  // lv_display_set_rotation(display, LV_DISPLAY_ROTATION_90);
  lv_obj_set_style_bg_color(lv_screen_active(), lv_color_white(), LV_PART_ANY);
  lv_obj_set_style_text_color(lv_screen_active(), lv_color_black(), LV_PART_ANY);

#if 0 // Rotated Hello
  lv_obj_t* helloLabel = lv_label_create(lv_screen_active());
  lv_label_set_text(helloLabel, "Hello");
  lv_obj_set_style_transform_rotation(helloLabel, -900, 0);
  lv_obj_set_width(helloLabel, EINK_SIZE_X);
  lv_obj_set_style_text_align(helloLabel, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(helloLabel, LV_ALIGN_CENTER, 50, -10);
#endif

#if 1 // Regular World
  lv_obj_t* helloLabel2 = lv_label_create(lv_screen_active());
  lv_label_set_text(helloLabel2, "World");
  lv_obj_set_width(helloLabel2, EINK_SIZE_X);
  lv_obj_set_style_text_align(helloLabel2, LV_TEXT_ALIGN_CENTER, 0);
  // lv_obj_align(helloLabel2, LV_ALIGN_CENTER, 0, -10);
#endif

#if 1 // Outer Frame
  {
    static lv_point_precise_t line_points[] = {
        {0, 0}, {EINK_SIZE_X - 2, 0}, {EINK_SIZE_X - 2, EINK_SIZE_Y - 2}, {0, EINK_SIZE_Y - 2}, {0, 0}, {20, 0},
        {0, 20}};

    static lv_style_t style_line;
    lv_style_init(&style_line);
    lv_style_set_line_width(&style_line, 1);

    lv_obj_t* line1;
    line1 = lv_line_create(lv_screen_active());
    lv_line_set_points(line1, line_points, 7);
    lv_obj_add_style(line1, &style_line, 0);
    lv_obj_center(line1);
  }
#endif

#if 1 // Diagonal lines
  {
    static lv_point_precise_t line_points[] = {
        {0, 0}, {EINK_SIZE_X - 2, EINK_SIZE_Y - 2}, {0, EINK_SIZE_Y - 2}, {EINK_SIZE_X - 2, 0}};

    static lv_style_t style_line;
    lv_style_init(&style_line);
    lv_style_set_line_width(&style_line, 1);

    lv_obj_t* line1;
    line1 = lv_line_create(lv_screen_active());
    lv_line_set_points(line1, line_points, 4);
    lv_obj_add_style(line1, &style_line, 0);
    lv_obj_center(line1);
  }
#endif

#if 0 // Chart Test
  lv_obj_t* chart;
  chart = lv_chart_create(lv_screen_active());
  lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
  lv_obj_set_size(chart, 50, 100);
  lv_obj_set_style_pad_all(chart, 0, 0);
  lv_obj_set_style_radius(chart, 0, 0);
  lv_obj_center(chart);
  lv_chart_set_div_line_count(chart, 10, 10);

  lv_chart_series_t* ser1 = lv_chart_add_series(chart, lv_color_black(), LV_CHART_AXIS_PRIMARY_Y);

  for (uint32_t i = 0; i < 10; i++) {
    /*Set the next points on 'ser1'*/
    lv_chart_set_next_value(chart, ser1, i * 10);
  }
  lv_chart_set_next_value(chart, ser1, 100);
  lv_chart_set_next_value(chart, ser1, 100);
  lv_obj_set_style_transform_rotation(chart, -900, 0);
  lv_chart_refresh(chart);
#endif

  // lv_obj_t* TickLabel = lv_label_create(lv_screen_active());
  while (1) {
    static uint16_t TickCnt = 0;

    ESP_LOGI(TAG, "Shunt:");
    ESP_LOGI(TAG, "  U=%d I=%ld", data_shunt.voltage, data_shunt.current);
    ESP_LOGI(TAG, "  T=%d L=%d", data_shunt.temp, data_shunt.soc);

    uint8_t hours = data_shunt.soc / 60;
    uint8_t mins  = data_shunt.soc - 60 * hours;
    ESP_LOGI(TAG, "  R=%d:%d", hours, mins);

    ESP_LOGI(TAG, "  U=%lu ms ago", portTICK_PERIOD_MS * (xTaskGetTickCount() - data_shunt.time));

    // TODO: Move eInk Update to Task
    // char cBuffer[50];
    // snprintf(&cBuffer[0], 50, "Tick: %u", TickCnt);
    // lv_label_set_text(TickLabel, cBuffer);
    // lv_obj_set_width(TickLabel, EINK_SIZE_X);
    // lv_obj_set_style_text_align(TickLabel, LV_TEXT_ALIGN_CENTER, 0);
    // lv_obj_set_pos(TickLabel, 0, 10);

    // gpio_set_level(PIN_LED, 1);
    // lv_refr_now(display);
    // vTaskDelay(250 / portTICK_PERIOD_MS);
    // gpio_set_level(PIN_LED, 0);

    vTaskDelay(LOOP_TIME / portTICK_PERIOD_MS);
    TickCnt++;
  }
}
