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
#include "esp_random.h"
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
  bool error;       // Error occured
  TickType_t time;  // [Tick] Last update
} data_shunt_t;

typedef struct {
  uint8_t state;   // Device State
  uint16_t v_in;   // [0.1V] Input Voltage
  uint16_t v_out;  // [0.1V] Output Voltage
  uint32_t offr;   // Off Reason
  bool error;      // Error occured
  TickType_t time; // [Tick] Last update
} data_dcdc_t;

typedef struct {
  uint8_t state;    // Device State
  uint16_t voltage; // [0.01V] Voltage
  uint16_t current; // [0.1A] current
  uint16_t power;   // [W] Power
  bool error;       // Error occured
  TickType_t time;  // [Tick] Last update
} data_solar_t;

/* Private define ------------------------------------------------------------*/

// #define ENABLE_TASK_STATS          // Enable task statistic output
#define LOOP_TIME (5 * 1000)       // [ms] Main loop time
#define EINK_UPD_TIME (15 * 1000)  // [ms] Update time for the display
#define GRAPH_UPD_TIME (60 * 1000) // [ms] Update time for the graphs
#define EINK_MAX_LINESIZE 50       // Max size of one Line
#define OSSTATS_ARRAY_SIZE_OFFS 5  // OS-Statistics: Increase this if print_real_time_stats returns ESP_ERR_INVALID_SIZE
#define OSSTATS_TIME 30000         // [ms] OS-Statistics cycle time

#define EINK_BUFFER_SIZE ((EINK_SIZE_X / 8 * EINK_SIZE_Y) + 8) // Buffer Size: 1 bit per pixel plus 8 byte for LVGL

#define FONT_HEIGHT 9 // [px] Height of the default font
#define Y_SPACE 10    // [px] Y distance between blocks

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static const char* TAG = "MAIN";

static uint8_t einkBuffer[EINK_BUFFER_SIZE];     // Pixel Buffer for the panel
static data_shunt_t data_shunt;                  // Data for the shunt / battery monitor
static data_dcdc_t data_dcdc;                    // Data for DC/DC converter
static data_solar_t data_solar;                  // Data for solar loader
static lv_obj_t* bat_chart               = NULL; // The battery capacity chart
static lv_chart_series_t* bat_cap_data   = NULL; // The battery capacity chart data
static lv_obj_t* solar_chart             = NULL; // The solar current chart
static lv_chart_series_t* solar_cur_data = NULL; // The solar current chart data
static SemaphoreHandle_t eink_sem;               // EInk semaphore handle

/* Private function prototypes -----------------------------------------------*/

#ifdef ENABLE_TASK_STATS
static void TaskOSStats(void* pvParameters);
#endif
static void TaskEInkUpdate(void* pvParameters);
static void TaskGraphUpdate(void* pvParameters);

void shunt_cb(const uint16_t volt, const int32_t curr, const uint16_t soc, const uint16_t temp, const uint16_t remain,
              const bool error);
void solar_cb(const uint8_t state, const uint16_t volt, const uint16_t curr, const uint16_t power, const bool error);
void dcdc_cb(const uint8_t state, const uint16_t in, const uint16_t out, const uint32_t off, const bool error);

void flush_cb(lv_display_t* display, const lv_area_t* area, uint8_t* px_map);

char* solar_state2text(uint8_t state);

/* Private user code ---------------------------------------------------------*/

// Covert the numeric state of a SmartSolar to text
// TODO: Move to victron component
char* solar_state2text(uint8_t state) {
  switch (state) {
    case 0:
      return "Off";
      break;
    case 1:
      return "Low Power";
      break;
    case 2:
      return "Fault";
      break;
    case 3:
      return "Bulk";
      break;
    case 4:
      return "Absorption";
      break;
    case 5:
      return "Float";
      break;
    default:
      return "Unknown";
      break;
  }
}

#ifdef ENABLE_TASK_STATS
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
#endif

// Update the Graph with the current data
static void TaskGraphUpdate(void* pvParameters) {
  while (1) {
    // Append current battery capacity
    if ((bat_chart != NULL) && (bat_cap_data != NULL)) {
      ESP_LOGI(TAG, "Updating Battery Graph");
      xSemaphoreTake(eink_sem, portMAX_DELAY);
      lv_chart_set_next_value(bat_chart, bat_cap_data, data_shunt.soc);
      lv_chart_refresh(bat_chart);
      xSemaphoreGive(eink_sem);
    }

    // Append current solar current
    if ((solar_chart != NULL) && (solar_cur_data != NULL)) {
      ESP_LOGI(TAG, "Updating Solar Graph");
      xSemaphoreTake(eink_sem, portMAX_DELAY);
      lv_chart_set_next_value(solar_chart, solar_cur_data, data_solar.current);
      lv_chart_refresh(solar_chart);
      xSemaphoreGive(eink_sem);
    }

    vTaskDelay(GRAPH_UPD_TIME / portTICK_PERIOD_MS);
  }
}

// Task to update the EInk content
static void TaskEInkUpdate(void* pvParameters) {
  char cBuffer[EINK_MAX_LINESIZE];

  xSemaphoreTake(eink_sem, portMAX_DELAY);

  // Init LVGL
  lv_init();
  lv_display_t* display = lv_display_create(EINK_SIZE_X, EINK_SIZE_Y);
  lv_display_set_flush_cb(display, flush_cb);
  lv_display_set_buffers(display, &einkBuffer[0], NULL, EINK_BUFFER_SIZE, LV_DISPLAY_RENDER_MODE_FULL);
  lv_obj_set_style_bg_color(lv_screen_active(), lv_color_white(), LV_PART_ANY);
  lv_obj_set_style_text_color(lv_screen_active(), lv_color_black(), LV_PART_ANY);

  eink_fillscreen(0x00);

  // Header text style
  lv_style_t style_header;
  lv_style_init(&style_header);
  lv_style_set_text_color(&style_header, lv_color_white());
  lv_style_set_bg_color(&style_header, lv_color_black());
  lv_style_set_bg_opa(&style_header, 0xFF);

  // Default Line style
  static lv_style_t style_thinline;
  lv_style_init(&style_thinline);
  lv_style_set_line_width(&style_thinline, 1);
  lv_style_set_line_color(&style_thinline, lv_color_black());

  // TODO: Refactoring, move all the labels into a struct and create/update in a loop

  // Draw outer frame
  static lv_point_precise_t frame_points[] = {
      {0, 0}, {EINK_SIZE_X - 1, 0}, {EINK_SIZE_X - 1, EINK_SIZE_Y - 1}, {0, EINK_SIZE_Y - 1}, {0, 0}};

  lv_obj_t* frame_line;
  frame_line = lv_line_create(lv_screen_active());
  lv_line_set_points(frame_line, frame_points, sizeof(frame_points) / sizeof(lv_point_precise_t));
  lv_obj_add_style(frame_line, &style_thinline, 0);

  // The Labels
  lv_obj_t* Bat_Header   = lv_label_create(lv_screen_active());
  lv_obj_t* Bat_Cap      = lv_label_create(lv_screen_active());
  lv_obj_t* Bat_Volt     = lv_label_create(lv_screen_active());
  lv_obj_t* Bat_Cur      = lv_label_create(lv_screen_active());
  lv_obj_t* Bat_Remain   = lv_label_create(lv_screen_active());
  lv_obj_t* Solar_Header = lv_label_create(lv_screen_active());
  lv_obj_t* Solar_Power  = lv_label_create(lv_screen_active());
  lv_obj_t* Solar_Cur    = lv_label_create(lv_screen_active());
  lv_obj_t* Solar_State  = lv_label_create(lv_screen_active());

  lv_obj_add_style(Bat_Header, &style_header, 0);
  lv_obj_add_style(Solar_Header, &style_header, 0);

  // Battery Data
  snprintf(&cBuffer[0], EINK_MAX_LINESIZE, "Battery");
  lv_label_set_text(Bat_Header, cBuffer);
  lv_obj_set_width(Bat_Header, lv_pct(100));
  lv_obj_set_style_text_align(Bat_Header, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(Bat_Header, 0, 0);

  lv_label_set_text(Bat_Cap, "---");
  lv_obj_set_width(Bat_Cap, lv_pct(50));
  lv_obj_set_style_text_align(Bat_Cap, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(Bat_Cap, 0, FONT_HEIGHT);

  lv_label_set_text(Bat_Volt, "---");
  lv_obj_set_width(Bat_Volt, lv_pct(50));
  lv_obj_set_style_text_align(Bat_Volt, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(Bat_Volt, lv_pct(50), FONT_HEIGHT);

  lv_label_set_text(Bat_Cur, "---");
  lv_obj_set_width(Bat_Cur, lv_pct(50));
  lv_obj_set_style_text_align(Bat_Cur, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(Bat_Cur, 0, 2 * FONT_HEIGHT);

  lv_label_set_text(Bat_Remain, "---");
  lv_obj_set_width(Bat_Remain, lv_pct(50));
  lv_obj_set_style_text_align(Bat_Remain, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(Bat_Remain, lv_pct(50), 2 * FONT_HEIGHT);

  // Battery capacity graph
  const uint8_t margin  = 4;
  const uint16_t width  = EINK_SIZE_X - (2 * margin);
  uint16_t y_pos        = 3 * FONT_HEIGHT + margin; // will be reused!
  const uint16_t height = (EINK_SIZE_Y / 2) - y_pos - (2 * margin);

  bat_chart = lv_chart_create(lv_screen_active());
  lv_chart_set_range(bat_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 1000);          // Y-Range: 0 .. 100.0%
  lv_chart_set_update_mode(bat_chart, LV_CHART_UPDATE_MODE_SHIFT);          // Shifting Mode
  lv_obj_set_size(bat_chart, width, height);                                // Width: 120px
  lv_chart_set_point_count(bat_chart, 120);                                 // 120 data points
  lv_chart_set_div_line_count(bat_chart, 5, 5);                             // 5 Divs
  lv_obj_set_style_size(bat_chart, 0, 0, LV_PART_INDICATOR);                // Do not show Indicator
  lv_obj_add_style(bat_chart, &style_thinline, 0);                          // Use thin Lines
  lv_obj_set_style_bg_color(bat_chart, lv_color_white(), LV_PART_MAIN);     // Background: White
  lv_obj_set_style_border_color(bat_chart, lv_color_black(), LV_PART_MAIN); // Foreground: Black
  lv_obj_set_style_radius(bat_chart, 0, 0);                                 // No Radius on corners
  lv_obj_set_pos(bat_chart, margin, y_pos);

  bat_cap_data = lv_chart_add_series(bat_chart, lv_palette_main(LV_PALETTE_NONE), LV_CHART_AXIS_PRIMARY_Y);

  // Solar Data
  snprintf(&cBuffer[0], EINK_MAX_LINESIZE, "Solar");
  lv_label_set_text(Solar_Header, cBuffer);
  lv_obj_set_width(Solar_Header, lv_pct(100));
  lv_obj_set_style_text_align(Solar_Header, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(Solar_Header, 0, (EINK_SIZE_Y / 2));

  lv_label_set_text(Solar_Power, "---");
  lv_obj_set_width(Solar_Power, lv_pct(50));
  lv_obj_set_style_text_align(Solar_Power, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(Solar_Power, 0, (EINK_SIZE_Y / 2) + (1 * FONT_HEIGHT));

  lv_label_set_text(Solar_Cur, "---");
  lv_obj_set_width(Solar_Cur, lv_pct(50));
  lv_obj_set_style_text_align(Solar_Cur, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(Solar_Cur, lv_pct(50), (EINK_SIZE_Y / 2) + (1 * FONT_HEIGHT));

  lv_label_set_text(Solar_State, "---");
  lv_obj_set_width(Solar_State, lv_pct(100));
  lv_obj_set_style_text_align(Solar_State, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(Solar_State, 0, (EINK_SIZE_Y / 2) + (2 * FONT_HEIGHT));

  // Solar current capacity graph
  y_pos = (EINK_SIZE_Y / 2) + (3 * FONT_HEIGHT) + margin;

  solar_chart = lv_chart_create(lv_screen_active());
  lv_chart_set_range(solar_chart, LV_CHART_AXIS_PRIMARY_Y, 0, 100);           // Y-Range: 0 .. 10.0 A
  lv_chart_set_update_mode(solar_chart, LV_CHART_UPDATE_MODE_SHIFT);          // Shifting Mode
  lv_obj_set_size(solar_chart, width, height);                                // Width: 120px
  lv_chart_set_point_count(solar_chart, 120);                                 // 120 data points
  lv_chart_set_div_line_count(solar_chart, 3, 5);                             // 5 Divs
  lv_obj_set_style_size(solar_chart, 0, 0, LV_PART_INDICATOR);                // Do not show Indicator
  lv_obj_add_style(solar_chart, &style_thinline, 0);                          // Use thin Lines
  lv_obj_set_style_bg_color(solar_chart, lv_color_white(), LV_PART_MAIN);     // Background: White
  lv_obj_set_style_border_color(solar_chart, lv_color_black(), LV_PART_MAIN); // Foreground: Black
  lv_obj_set_style_radius(solar_chart, 0, 0);                                 // No Radius on corners
  lv_obj_set_pos(solar_chart, margin, y_pos);

  solar_cur_data = lv_chart_add_series(solar_chart, lv_palette_main(LV_PALETTE_NONE), LV_CHART_AXIS_PRIMARY_Y);

  lv_refr_now(display);

  while (1) {
    xSemaphoreGive(eink_sem);
    vTaskDelay(EINK_UPD_TIME / portTICK_PERIOD_MS);
    xSemaphoreTake(eink_sem, portMAX_DELAY);

    // Update Battery Data
    snprintf(&cBuffer[0], EINK_MAX_LINESIZE, "%d%%", (data_shunt.soc / 10));
    lv_label_set_text(Bat_Cap, cBuffer);

    snprintf(&cBuffer[0], EINK_MAX_LINESIZE, "%.1fV", ((double)data_shunt.voltage / 100));
    lv_label_set_text(Bat_Volt, cBuffer);

    if (data_shunt.current > 0) {
      snprintf(&cBuffer[0], EINK_MAX_LINESIZE, "%.2fA", ((double)data_shunt.current / 1000));
    } else {
      snprintf(&cBuffer[0], EINK_MAX_LINESIZE, "%.2fA", ((double)data_shunt.current / 1000));
    }
    lv_label_set_text(Bat_Cur, cBuffer);

    if (data_shunt.current > 0) {
      snprintf(&cBuffer[0], EINK_MAX_LINESIZE, "-");
    } else {
      uint8_t hours = data_shunt.soc / 60;
      uint8_t mins  = data_shunt.soc - 60 * hours;
      snprintf(&cBuffer[0], EINK_MAX_LINESIZE, "%d:%02d", hours, mins);
    }
    lv_label_set_text(Bat_Remain, cBuffer);

    // Update Solar Data
    snprintf(&cBuffer[0], EINK_MAX_LINESIZE, "%dW", data_solar.power);
    lv_label_set_text(Solar_Power, cBuffer);

    snprintf(&cBuffer[0], EINK_MAX_LINESIZE, "%.2fA", ((double)data_solar.current / 10));
    lv_label_set_text(Solar_Cur, cBuffer);

    snprintf(&cBuffer[0], EINK_MAX_LINESIZE, "%s", solar_state2text(data_solar.state));
    lv_label_set_text(Solar_State, cBuffer);

    // Update Display
    gpio_set_level(PIN_LED, 1);
    lv_refr_now(display);
    gpio_set_level(PIN_LED, 0);
  }
}

// Send LVGL data to the display
void flush_cb(lv_display_t* display, const lv_area_t* area, uint8_t* px_map) {
  eink_setbuffer(px_map + 8, EINK_BUFFER_SIZE - 8);
  eink_update();

  // Inform LVGL that you are ready with the flushing and buf is not used anymore
  lv_display_flush_ready(display);
}

// Callback to set smartshunts measurement data
void shunt_cb(const uint16_t volt, const int32_t curr, const uint16_t soc, const uint16_t temp, const uint16_t remain,
              const bool error) {
  data_shunt.voltage = volt;
  data_shunt.current = curr;
  data_shunt.soc     = soc;
  data_shunt.temp    = temp;
  data_shunt.remain  = remain;
  data_shunt.error   = error;
  data_shunt.time    = xTaskGetTickCount();
}

// Callback to set DC/DC Converter Data
void dcdc_cb(const uint8_t state, const uint16_t in, const uint16_t out, const uint32_t off, const bool error) {
  data_dcdc.state = state;
  data_dcdc.v_in  = in;
  data_dcdc.v_out = out;
  data_dcdc.offr  = off;
  data_dcdc.error = error;
  data_dcdc.time  = xTaskGetTickCount();
}

// Callback to set solar charger measurement data
void solar_cb(const uint8_t state, const uint16_t volt, const uint16_t curr, const uint16_t power, const bool error) {
  data_solar.state   = state;
  data_solar.voltage = volt;
  data_solar.current = curr;
  data_solar.power   = power;
  data_solar.error   = error;
  data_solar.time    = xTaskGetTickCount();
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

#ifdef ENABLE_TASK_STATS
  // Start OS Statistics Task
  xTaskCreate(TaskOSStats, "FreeRTOS Stats", 4096, NULL, tskIDLE_PRIORITY, NULL);
#endif

  // The LED Output
  gpio_reset_pin(PIN_LED);
  gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);

  // Init EINK SPI driver
  eink_sem = xSemaphoreCreateMutex();
  eink_init();

  // Bluetooth and Callbacks
  blue_init();
  blue_setcb_sshunt(shunt_cb);
  blue_setcb_ssolar(solar_cb);
  blue_setcb_dcdc(dcdc_cb);

  // Start Panel Update Task
  xTaskCreate(TaskEInkUpdate, "EInk Update", 4096, NULL, tskIDLE_PRIORITY, NULL);

  // Start graph updater
  xTaskCreate(TaskGraphUpdate, "Graph Update", 4096, NULL, tskIDLE_PRIORITY, NULL);

  while (1) {
#if 1
    // Create simulation data
    shunt_cb(1234, -150, (esp_random() % 1000), 0xFF, 90, 0);
    solar_cb(3, 1345, (esp_random() % 100), 34, 0);
    dcdc_cb(0, 1234, 0, 0, 0);
#endif

    // Print current Data
    uint8_t hours = data_shunt.soc / 60;
    uint8_t mins  = data_shunt.soc - 60 * hours;
    ESP_LOGI(TAG, "Shunt: U=%d I=%ld T=%d L=%d E=0x%02x R=%02d:%0d (%lu)", data_shunt.voltage, data_shunt.current,
             data_shunt.temp, data_shunt.soc, data_shunt.error, hours, mins,
             portTICK_PERIOD_MS * (xTaskGetTickCount() - data_shunt.time));
    ESP_LOGI(TAG, "DCDC:  I=%d O=%d Off=0x%lx E=0x%02x S=0x%02x (%lu)", data_dcdc.v_in, data_dcdc.v_out, data_dcdc.offr,
             data_dcdc.error, data_dcdc.state, portTICK_PERIOD_MS * (xTaskGetTickCount() - data_dcdc.time));
    ESP_LOGI(TAG, "Solar: U=%d I=%d P=%d S=0x%02x E=0x%02x (%lu)", data_solar.voltage, data_solar.current,
             data_solar.power, data_solar.state, data_solar.error,
             portTICK_PERIOD_MS * (xTaskGetTickCount() - data_solar.time));

    vTaskDelay(LOOP_TIME / portTICK_PERIOD_MS);
  }
}
