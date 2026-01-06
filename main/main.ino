/**
 * Merged Smart Heater Firmware
 * Combined functionalities of main.ino (Logic) and main_modified.ino (Non-blocking WiFi)
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include "Adafruit_MLX90614.h"
#include "driver/pcnt.h"
#include <TFT_eSPI.h>
#include <Ticker.h>
#include "ui_manager.h"
#include <Preferences.h>
#include "esp_task_wdt.h"

// FreeRTOS Headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// OTA
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>

// ========== [Pin Definitions] ==========
#define MAXDO 48   // MISO
#define MAXCLK 47  // SCK
#define MAXCS1 15  // CS1
#define MAXCS2 16  // CS2
#define MAXCS3 17  // CS3
#define MAXCS4 18  // CS4

#define ENCODER_A 6
#define ENCODER_B 7
#define ENCODER_SW 8

#define SDA_PIN 4
#define SCL_PIN 5
#define BUZZER 41
#define PCF_ADDR 0x20

#define IR1_ADDR 0x10
#define IR2_ADDR 0x11
#define SSR_PIN1 42
#define SSR_PIN2 2
#define SSR_PIN3 1

#define WINDOW_MS_INIT 1000
#define ENCODER_DIVIDER 2

#define BEEP_MODE_READY 2
#define BEEP_MODE_ALARM 3

#define MAIN_HEATER_INDEX 1  //(0=Heater1, 1=Heater2, 2=Heater3)

// ========== [Auto Mode Control Sensor Selection] ==========
// 0 = Thermocouple, 1 = IR1
#define AUTO_CONTROL_SENSOR 1 

// ========== [WiFi Configuration] ==========
const char* ssid = "NNTT24";
const char* password = "TeraE-01";

// WiFi Status - For UI
enum WiFiConnectionStatus {
  WIFI_STATUS_DISCONNECTED = 0,
  WIFI_STATUS_CONNECTING,
  WIFI_STATUS_CONNECTED
};
volatile WiFiConnectionStatus wifiStatus = WIFI_STATUS_DISCONNECTED;
volatile int wifiSignalStrength = 0;

// WiFi Config
#define WIFI_RECONNECT_INTERVAL_MS 30000   
#define WIFI_CHECK_INTERVAL_MS 5000        

// ========== [Shared Resources] ==========
SemaphoreHandle_t spiMutex;
SemaphoreHandle_t dataMutex;
SemaphoreHandle_t serialMutex;

// Thread-Safe Data Model
struct SharedData {
  float tc_temps[3];
  float displayed_temps[3];
  float tc_probe_temp;
  float tc_probe_peak;
  float cj_temps[3];
  uint8_t tc_faults[3];
  float ir_temps[2];
  float ir_ambient[2];
  bool heater_cutoff[3];
  bool heater_ready[3]; // True if stable for 10s
  uint8_t auto_step;    // 0=Off, 1=Cycle1, 2=Cycle2, 3=Cycle3
  bool auto_mode_enabled;
  bool auto_was_started;        
  bool manual_was_started;
  bool manual_preset_running;
  uint8_t manual_preset_index;  // 0-2
  bool manual_mode_enabled;
};

SharedData sysState;

// ========== [Frequency Monitoring Counters] ==========
volatile uint32_t freq_max_cnt = 0;
volatile uint32_t freq_mlx_cnt = 0;
volatile uint32_t freq_ctl_cnt = 0;
volatile uint32_t freq_disp_cnt = 0;
volatile uint32_t freq_input_cnt = 0;

// ========== [Objects & Config] ==========
const int TC_CS_PINS[] = { MAXCS1, MAXCS2, MAXCS3 };
const int TC_PROBE_PIN = MAXCS4;
const int NUM_THERMOCOUPLES = 3;
SPISettings maxSettings(1000000, MSBFIRST, SPI_MODE0);

TFT_eSPI tft = TFT_eSPI();
Preferences preferences;
void saveConfig(const ConfigState& config);
UIManager ui(&tft, saveConfig);
ConfigState config;
Adafruit_MLX90614 mlx1 = Adafruit_MLX90614();
Adafruit_MLX90614 mlx2 = Adafruit_MLX90614();
Ticker tpo_ticker;

// Global Runtime Vars
float go_to = NAN;
bool has_go_to = false;
volatile int beep_queue = 0;
volatile bool beeper_active = false;
volatile int beep_mode = 0;
uint32_t t_next_beep_action = 0;
const int BEEP_ON_MS = 60;
const int BEEP_OFF_MS = 100;

// [OTA] Web Server Object
AsyncWebServer server(80);
bool serverStarted = false;

// ========== [PID Vars] ==========
float Kp[3] = { 1.2f, 2.5f, 0.5f };
float Ki[3] = { 0.02f, 0.5f, 0.01f };
float Kd[3] = { 0.2f, 0.01f, 0.8f };
float pid_integral[3] = { 0.0f, 0.0f, 0.0f };
float pid_prev_error[3] = { 0.0f, 0.0f, 0.0f };
uint32_t pid_last_time[3] = { 0, 0, 0 };

// TPO
portMUX_TYPE tpoMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t tpo_window_period_ticks = WINDOW_MS_INIT;
volatile uint32_t tpo_on_ticks[3] = { 0, 0, 0 };
volatile uint32_t tpo_tick_counter = 0;

// PCF
struct PCF_State {
  bool btn_pressed[4] = { false, false, false, false };
  bool led_state[4] = { false, false, false, false };
  uint32_t last_press[4] = { 0, 0, 0, 0 };
};
PCF_State pcf_state;
long lastEncoderValue = 0;

// ========== [Forward Declarations] ==========
void IRAM_ATTR tpo_isr();
void setup_encoder_fixed();
void read_hardware_encoder(float* delta_out);
void max31855_init_pins();
void loadConfig(ConfigState& cfg);
void tpo_set_percent(int index, float percent);
float applyEmissivity(float t_obj_sensor, float t_amb, float emissivity, float sensor_emissivity = 0.95f);
void setupWebServer(); 

// Add specific helper functions
const char* getWiFiSSID() {
  if (config.wifi_config.use_custom && strlen(config.wifi_config.ssid) > 0) {
    return config.wifi_config.ssid;
  }
  return "NNTT24";  // Default fallback
}

const char* getWiFiPassword() {
  if (config.wifi_config.use_custom && strlen(config.wifi_config.password) > 0) {
    return config.wifi_config.password;
  }
  return "TeraE-01";  // Default fallback
}

void triggerWiFiReconnect() {
  Serial.println("[WiFi] Manual reconnect triggered");
  WiFi.disconnect();
  vTaskDelay(pdMS_TO_TICKS(500));
  WiFi.begin(getWiFiSSID(), getWiFiPassword());
  wifiStatus = WIFI_STATUS_CONNECTING;
}

// ========== [Tasks] ==========

// 0. WiFi Manager Task (Core 0, Non-Blocking)
void TaskWiFiManager(void* pvParameters) {
  const TickType_t xCheckInterval = pdMS_TO_TICKS(WIFI_CHECK_INTERVAL_MS);
  uint32_t lastReconnectAttempt = 0;
  bool wasConnected = false;
  
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  
  Serial.println("[WiFi] Starting initial connection attempt...");
  // WiFi.begin(ssid, password);
  WiFi.begin(getWiFiSSID(), getWiFiPassword());
  wifiStatus = WIFI_STATUS_CONNECTING;
  
  for (;;) {
    wl_status_t currentStatus = WiFi.status();
    switch (currentStatus) {
      case WL_CONNECTED:
        wifiStatus = WIFI_STATUS_CONNECTED;
        if (!wasConnected) {
          Serial.println("\n[WiFi] Connected!");
          Serial.print("[WiFi] IP Address: ");
          Serial.println(WiFi.localIP());
          wifiSignalStrength = WiFi.RSSI();
          if (!serverStarted) {
            setupWebServer();
            serverStarted = true;
          }
          wasConnected = true;
        }
        wifiSignalStrength = WiFi.RSSI();
        break;
        
      case WL_DISCONNECTED:
      case WL_CONNECTION_LOST:
      case WL_CONNECT_FAILED:
        if (wasConnected) {
          Serial.println("[WiFi] Connection lost!");
          wasConnected = false;
        }
        wifiStatus = WIFI_STATUS_DISCONNECTED;
        wifiSignalStrength = 0;
        
        if (millis() - lastReconnectAttempt > WIFI_RECONNECT_INTERVAL_MS) {
          Serial.println("[WiFi] Attempting to reconnect...");
          wifiStatus = WIFI_STATUS_CONNECTING;
          WiFi.disconnect();
          vTaskDelay(pdMS_TO_TICKS(100));
          WiFi.begin(ssid, password);
          lastReconnectAttempt = millis();
        }
        break;
      case WL_IDLE_STATUS:
      case WL_NO_SSID_AVAIL:
        wifiStatus = WIFI_STATUS_CONNECTING;
        if (millis() - lastReconnectAttempt > WIFI_RECONNECT_INTERVAL_MS) {
          Serial.println("[WiFi] Network not found, retrying...");
          WiFi.disconnect();
          vTaskDelay(pdMS_TO_TICKS(100));
          // WiFi.begin(ssid, password);
          WiFi.begin(getWiFiSSID(), getWiFiPassword());
          lastReconnectAttempt = millis();
        }
        break;
      default:
        break;
    }
    vTaskDelay(xCheckInterval);
  }
}

// 1. MAX31855 TASK
void TaskMAX(void* pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(100);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  const int MA_WINDOW = 10;
  static float ma_buffer[4][MA_WINDOW];
  static int ma_idx[4] = { 0, 0, 0, 0 };
  static int ma_count[4] = { 0, 0, 0, 0 };
  static uint32_t prev_raw_data[4] = { 0, 0, 0, 0 };
  static uint32_t last_change_time[4] = { 0, 0, 0, 0 };
  const uint32_t STUCK_THRESHOLD_MS = 2000;
  static uint32_t fault_start_time[4] = { 0, 0, 0, 0 };
  const uint32_t FAULT_HOLD_MS = 500;

  SPIClass* vspi = new SPIClass(VSPI);
  vspi->begin(MAXCLK, MAXDO, -1, -1);
  SPISettings maxSettings(1000000, MSBFIRST, SPI_MODE0);

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < MA_WINDOW; j++) ma_buffer[i][j] = 0.0f;
    last_change_time[i] = millis();
  }

  for (;;) {
    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
      digitalWrite(TFT_CS, HIGH);
      vspi->beginTransaction(maxSettings);

      for (int i = 0; i < 4; i++) {
        uint32_t d = 0;
        int cs_pin = (i < 3) ? TC_CS_PINS[i] : TC_PROBE_PIN;

        digitalWrite(cs_pin, LOW);
        d = vspi->transfer32(0);
        digitalWrite(cs_pin, HIGH);

        float raw_temp = NAN;
        bool is_fault = false;

        if (d == 0 || d == 0xFFFFFFFF) {
          is_fault = true;
        } else if (d & 0x10000) {
          if (fault_start_time[i] == 0) {
            fault_start_time[i] = millis();
          }
          if (millis() - fault_start_time[i] > FAULT_HOLD_MS) {
            is_fault = true;
          }
        } else {
          fault_start_time[i] = 0;
        }

        if (!is_fault) {
          if (d == prev_raw_data[i]) {
            if (millis() - last_change_time[i] > STUCK_THRESHOLD_MS) {
              is_fault = true;
            }
          } else {
            prev_raw_data[i] = d;
            last_change_time[i] = millis();
          }
        } else {
          prev_raw_data[i] = d;
          last_change_time[i] = millis();
        }

        if (is_fault) {
          raw_temp = NAN;
          ma_count[i] = 0;
          ma_idx[i] = 0;
        } else {
          int32_t v = (d >> 18) & 0x3FFF;
          if (d & 0x20000000) v -= 16384;
          raw_temp = v * 0.25f;
        }

        float final_temp = NAN;
        if (!is_fault) {
          ma_buffer[i][ma_idx[i]] = raw_temp;
          ma_idx[i] = (ma_idx[i] + 1) % MA_WINDOW;
          if (ma_count[i] < MA_WINDOW) ma_count[i]++;

          float sum = 0;
          for (int k = 0; k < ma_count[i]; k++) sum += ma_buffer[i][k];
          final_temp = sum / ma_count[i];
          if (final_temp < 25.0f) final_temp = 25.0f;
        }

        if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
          if (i < 3) {
            sysState.tc_temps[i] = final_temp;
            sysState.tc_faults[i] = is_fault ? 1 : 0;
          } else {
            if (isnan(final_temp)) {
              sysState.tc_probe_temp = NAN;
            } else {
              sysState.tc_probe_temp = final_temp + config.tc_probe_offset;
            }
          }
          xSemaphoreGive(dataMutex);
        }
      }
      vspi->endTransaction();
      xSemaphoreGive(spiMutex);
    }
    freq_max_cnt++;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 2. MLX90614 TASK
void TaskMLX(void* pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(100);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    float ir1_obj = mlx1.readObjectTempC();
    float ir1_amb = mlx1.readAmbientTempC();
    float ir2_obj = mlx2.readObjectTempC();
    float ir2_amb = mlx2.readAmbientTempC();

    float ir1_corrected = applyEmissivity(ir1_obj, ir1_amb, config.ir_emissivity[0], 0.95f);
    float ir2_corrected = applyEmissivity(ir2_obj, ir2_amb, config.ir_emissivity[1], 0.95f);

    if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
      sysState.ir_temps[0] = ir1_corrected;
      sysState.ir_ambient[0] = ir1_amb;
      sysState.ir_temps[1] = ir2_corrected;
      sysState.ir_ambient[1] = ir2_amb;
      xSemaphoreGive(dataMutex);
    }
    freq_mlx_cnt++;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 3. CONTROL TASK (High Priority)
void TaskControl(void* pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(50);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static float ready_stable_start[3] = { 0, 0, 0 };
  const float READY_THRESHOLD = 5.0f;
  const uint32_t READY_HOLD_MS = 5000;
  static uint8_t current_auto_step = 0;

  static float peak_temp[3] = { 0, 0, 0 };
  static bool has_reached_target[3] = { false, false, false };
  static uint32_t cycle_change_debounce = 0;
  static uint32_t snapshot_timer[3] = { 0, 0, 0 };
  static float last_displayed_val[3] = { 0, 0, 0 };
  static float wire_max_tracker = -999.0f;
  static uint32_t wire_max_timer = 0;
  static float wire_max_display = 0.0f;
  static bool prev_ready_state[3] = { false, false, false };

  for (;;) {
    float current_temps[3];
    float ir1_temp = NAN;
    float current_wire_temp = NAN;

    bool local_auto_started = false;
    bool local_manual_started = false;
    bool local_manual_preset_running = false;
    uint8_t local_preset_idx = 0;

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      for (int i = 0; i < 3; i++) current_temps[i] = sysState.tc_temps[i];
      ir1_temp = sysState.ir_temps[0];
      current_auto_step = sysState.auto_step;
      current_wire_temp = sysState.tc_probe_temp;
      local_auto_started = sysState.auto_was_started;
      local_manual_started = sysState.manual_was_started;
      local_manual_preset_running = sysState.manual_preset_running;
      local_preset_idx = sysState.manual_preset_index;
      xSemaphoreGive(dataMutex);
    }

    if (!isnan(current_wire_temp)) {
      if (current_wire_temp > wire_max_tracker) {
        wire_max_tracker = current_wire_temp;
      }
    }

    if (millis() - wire_max_timer > 5000) {
      wire_max_display = wire_max_tracker;
      wire_max_tracker = -999.0f;
      wire_max_timer = millis();
      if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
        sysState.tc_probe_peak = wire_max_display;
        xSemaphoreGive(dataMutex);
      }
    }

    bool auto_is_running = (local_auto_started && current_auto_step > 0);
    bool system_run = has_go_to;

    for (int i = 0; i < 3; i++) {
      float current_t = current_temps[i];
      bool is_active = config.heater_active[i];
      float target_t = config.target_temps[i];
      float max_t = config.max_temps[i];

      bool is_auto_controlled = false;
      // 1. AUTO MODE OVERRIDE
      if (i == MAIN_HEATER_INDEX && auto_is_running) {
        is_active = true;
        is_auto_controlled = true;
        int cycle_idx = current_auto_step - 1;
        target_t = config.auto_target_temps[cycle_idx];
        max_t = config.auto_max_temps[cycle_idx];
#if AUTO_CONTROL_SENSOR == 1
        current_t = ir1_temp;
#endif
      }
      // 2. MANUAL PRESET MODE
      else if (i == MAIN_HEATER_INDEX && local_manual_preset_running && system_run) {
        is_active = true;
        is_auto_controlled = true;
        target_t = config.manual_target_temps[local_preset_idx];
        max_t = config.manual_max_temps[local_preset_idx];
#if AUTO_CONTROL_SENSOR == 1
        current_t = ir1_temp;
#endif
      }
      // 3. STANDBY / BASIC MANUAL
      else {
        if (!local_manual_started) {
          is_active = false;
        }
        if (i == MAIN_HEATER_INDEX && (auto_is_running || local_manual_preset_running)) {
          // Blocked by Auto/Preset
        }
      }

      // === PID & OUTPUT LOGIC ===
      float output_percent = 0.0f;
      bool cutoff_active = false;
      bool is_ready_status = false;

      if (has_go_to && is_active && !isnan(current_t)) {
        if (current_t >= max_t) {
          cutoff_active = true;
          output_percent = 0.0f;
          pid_integral[i] = 0;
        } else {
          float error = target_t - current_t;
          uint32_t now = millis();
          float dt = (pid_last_time[i] == 0) ? 0.05f : (now - pid_last_time[i]) / 1000.0f;
          if (dt > 0.5f) dt = 0.05f;
          pid_last_time[i] = now;

          pid_integral[i] += error * dt;
          pid_integral[i] = constrain(pid_integral[i], -100, 100);
          float derivative = (dt > 0) ? ((error - pid_prev_error[i]) / dt) : 0;
          pid_prev_error[i] = error;
          float P = Kp[i] * error;
          float I = Ki[i] * pid_integral[i];
          float D = Kd[i] * derivative;
          output_percent = P + I + D;
          output_percent = constrain(output_percent, 0, 100);
          if (fabs(error) <= READY_THRESHOLD) {
            if (ready_stable_start[i] == 0) ready_stable_start[i] = millis();
            if (millis() - ready_stable_start[i] >= READY_HOLD_MS) {
              is_ready_status = true;
            }
          } else {
            ready_stable_start[i] = 0;
          }
        }
      } else {
        output_percent = 0.0f;
        pid_integral[i] = 0;
        pid_prev_error[i] = 0;
        pid_last_time[i] = 0;
        ready_stable_start[i] = 0;
      }

      if (is_ready_status && !prev_ready_state[i]) {
        beep_mode = BEEP_MODE_READY;
        beep_queue = 6;
      }
      prev_ready_state[i] = is_ready_status;

      tpo_set_percent(i, output_percent);
      if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
        sysState.heater_cutoff[i] = cutoff_active;
        xSemaphoreGive(dataMutex);
      }

      // === AUTO MODE CYCLE TRANSITION ===
      if (i == MAIN_HEATER_INDEX && auto_is_running && has_go_to) {
#if AUTO_CONTROL_SENSOR == 1
        float control_temp = ir1_temp;
#else
        float control_temp = current_temps[i];
#endif
        if (!isnan(control_temp)) {
          if (is_ready_status) {
            has_reached_target[i] = true;
            if (control_temp > peak_temp[i]) peak_temp[i] = control_temp;
          }

          if (has_reached_target[i] && (peak_temp[i] - control_temp) > 5.0f) {
            if (millis() - cycle_change_debounce > 1000) {
              if (current_auto_step < 3) {
                current_auto_step++;
              } else {
                current_auto_step = 1;
              }
              if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
                sysState.auto_step = current_auto_step;
                xSemaphoreGive(dataMutex);
              }
              peak_temp[i] = 0;
              has_reached_target[i] = false;
              cycle_change_debounce = millis();
            }
          }
        }
      }

      if (!has_go_to) {
        if (current_auto_step != 0) {
          if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
            sysState.auto_step = 0;
            xSemaphoreGive(dataMutex);
          }
          current_auto_step = 0;
        }
        peak_temp[i] = 0;
        has_reached_target[i] = false;
        cycle_change_debounce = 0;
      }

      // === Display Freeze Logic ===
      float val_to_display = current_temps[i];
      if (is_ready_status) {
        if (millis() - snapshot_timer[i] > 10000) {
          val_to_display = current_temps[i];
          snapshot_timer[i] = millis();
        } else {
          val_to_display = last_displayed_val[i];
        }
      } else {
        val_to_display = current_temps[i];
        snapshot_timer[i] = millis() - 10000;
      }
      last_displayed_val[i] = val_to_display;
      if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
        sysState.heater_ready[i] = is_ready_status;
        sysState.displayed_temps[i] = val_to_display;
        sysState.auto_step = current_auto_step;
        xSemaphoreGive(dataMutex);
      }
    }

    freq_ctl_cnt++;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 4. INPUT TASK
void TaskInput(void* pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(20);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static int lastBtn = HIGH;
  static uint32_t press_start_time = 0;
  static bool ignore_release = false;
  const uint32_t LONG_PRESS_MS = 1000;
  for (;;) {
    float delta = 0;
    read_hardware_encoder(&delta);
    if (delta != 0) {
      if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        ui.handleEncoderRotation(delta, config);
        xSemaphoreGive(dataMutex);
      }
    }

    int curBtn = digitalRead(ENCODER_SW);
    if (lastBtn == HIGH && curBtn == LOW) {
      press_start_time = millis();
      ignore_release = false;
    } else if (lastBtn == LOW && curBtn == LOW) {
      if (!ignore_release && (millis() - press_start_time > LONG_PRESS_MS)) {
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
          UIScreen scr = ui.getScreen();
          if (scr == SCREEN_STANDBY) {
            ui.enterQuickEdit();
          } else if (scr == SCREEN_AUTO_MODE) {
            ui.enterQuickEditAuto();
          } else if (scr == SCREEN_MANUAL_MODE) {
            ui.enterQuickEditManual();
          } else {
            ui.handleButtonDoubleClick(config);
          }
          xSemaphoreGive(dataMutex);
        }
        beep_queue += 2;
        ignore_release = true;
      }
    } else if (lastBtn == LOW && curBtn == HIGH) {
      if (!ignore_release) {
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
          UIScreen scr = ui.getScreen();
          ui.handleButtonSingleClick(config, go_to, has_go_to);
          if (scr == SCREEN_MANUAL_MODE && sysState.manual_preset_running && has_go_to) {
            sysState.manual_preset_index = ui.getManualConfirmedPreset();
          }
          xSemaphoreGive(dataMutex);
        }
        beep_queue += 2;
      }
    }
    lastBtn = curBtn;

    Wire.requestFrom(PCF_ADDR, 1);
    if (Wire.available()) {
      uint8_t input_data = Wire.read();
      bool btn[4] = { !(input_data & 1), !(input_data & 2), !(input_data & 4), !(input_data & 8) };
      for (int i = 0; i < 4; i++) {
        if (btn[i] && !pcf_state.btn_pressed[i]) {
          beep_queue += 2;
          if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            // Button 0: Settings
            if (i == 0) {
              UIScreen scr = ui.getScreen();
              if (scr == SCREEN_STANDBY || scr == SCREEN_AUTO_MODE || scr == SCREEN_MANUAL_MODE) {
                ui.openSettings();
              } else {
                ui.exitSettings();
              }
            }
            // Button 1: Start/Stop
            else if (i == 1) {
              UIScreen current = ui.getScreen();
              if (current == SCREEN_STANDBY) {
                sysState.manual_was_started = !sysState.manual_was_started;
                has_go_to = sysState.manual_was_started || sysState.auto_was_started || sysState.manual_preset_running;
              } else if (current == SCREEN_AUTO_MODE) {
                if (sysState.auto_was_started) {
                  sysState.auto_was_started = false;
                  sysState.auto_step = 0;
                } else {
                  sysState.auto_was_started = true;
                  sysState.auto_step = 1;
                }
                has_go_to = sysState.manual_was_started || sysState.auto_was_started || sysState.manual_preset_running;
              } else if (current == SCREEN_MANUAL_MODE) {
                if (sysState.manual_preset_running) {
                  sysState.manual_preset_running = false;
                } else {
                  sysState.manual_preset_running = true;
                  sysState.manual_preset_index = ui.getManualConfirmedPreset();
                }
                has_go_to = sysState.manual_was_started || sysState.auto_was_started || sysState.manual_preset_running;
              }
            }
            // Button 3: Mode Toggle
            else if (i == 2) {
              UIScreen current = ui.getScreen();
              if (current == SCREEN_STANDBY || current == SCREEN_QUICK_EDIT) {
                ui.switchToAutoMode();
                sysState.auto_mode_enabled = true;
                sysState.manual_mode_enabled = false;
              } else if (current == SCREEN_AUTO_MODE || current == SCREEN_QUICK_EDIT_AUTO) {
                ui.switchToManualMode();
                sysState.auto_mode_enabled = false;
                sysState.manual_mode_enabled = true;
              } else if (current == SCREEN_MANUAL_MODE || current == SCREEN_QUICK_EDIT_MANUAL) {
                ui.switchToStandby();
                sysState.auto_mode_enabled = false;
                sysState.manual_mode_enabled = false;
              } else {
                ui.switchToStandby();
                sysState.auto_mode_enabled = false;
                sysState.manual_mode_enabled = false;
              }
            }
            saveConfig(config);
            xSemaphoreGive(dataMutex);
          }
        }
        pcf_state.btn_pressed[i] = btn[i];
      }

      uint8_t led_out = 0xF0;
      if (config.heater_active[0]) led_out &= ~(1 << 4);
      if (config.heater_active[1]) led_out &= ~(1 << 5);
      if (config.heater_active[2]) led_out &= ~(1 << 6);
      if (has_go_to) led_out &= ~(1 << 7);

      Wire.beginTransmission(PCF_ADDR);
      Wire.write(led_out | 0x0F);
      Wire.endTransmission();
    }
    freq_input_cnt++;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 5. DISPLAY TASK
void TaskDisplay(void* pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(100);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    AppState st;
    bool alarm_active = false;
    bool lock_active = false;

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      if (ui.checkInactivity(config, has_go_to, go_to)) {
        beep_mode = 1;
        beep_queue = 6;
      }

      for (int i = 0; i < 3; i++) {
        st.tc_temps[i] = sysState.displayed_temps[i];
        st.tc_faults[i] = sysState.tc_faults[i];
        st.heater_cutoff_state[i] = sysState.heater_cutoff[i];
        if (st.heater_cutoff_state[i]) {
          lock_active = true;
        }
        st.heater_ready[i] = sysState.heater_ready[i];
      }
      st.auto_step = sysState.auto_step;
      st.auto_mode_enabled = sysState.auto_mode_enabled;
      st.auto_running_background = (has_go_to && sysState.auto_was_started && sysState.auto_step > 0);
      st.manual_running_background = (has_go_to && sysState.manual_was_started);
      st.manual_preset_running = sysState.manual_preset_running && has_go_to;
      st.manual_was_started = sysState.manual_was_started;
      st.manual_preset_index = sysState.manual_preset_index;
      st.manual_mode_enabled = sysState.manual_mode_enabled;
      st.tc_probe_temp = sysState.tc_probe_temp;
      st.tc_probe_peak = sysState.tc_probe_peak;
      st.ir_temps[0] = sysState.ir_temps[0];
      st.ir_temps[1] = sysState.ir_temps[1];
      st.ir_ambient[0] = sysState.ir_ambient[0];
      st.ir_ambient[1] = sysState.ir_ambient[1];

      if (st.ir_ambient[0] > 80.0f || st.ir_ambient[1] > 80.0f) {
        alarm_active = true;
      }
      xSemaphoreGive(dataMutex);
    }

    if (getWiFiStatus() == WIFI_STATUS_CONNECTED) {
       String ip = WiFi.localIP().toString();
       strncpy(st.ip_address, ip.c_str(), 19);
       st.ip_address[19] = '\0';
    } else {
       strcpy(st.ip_address, "---");
    }

    st.is_heating_active = has_go_to;
    st.target_temp = go_to;
    st.temp_unit = config.temp_unit;
    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
      ui.draw(st, config);
      xSemaphoreGive(spiMutex);
    }

    if (lock_active) {
      beep_mode = 3;
      if (beep_queue == 0) beep_queue = 2;
    } else if (alarm_active) {
      if (beep_queue == 0) beep_queue = 2;
    }

    freq_disp_cnt++;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 6. DEBUG TASK
void TaskDebug(void* pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(1000);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
      Serial.println("\n=== [System Health Monitor] ===");
      Serial.printf("Task Frequency (Hz):\n");
      Serial.printf("  MAX31855: %d Hz (Target: 10)\n", freq_max_cnt);
      Serial.printf("  MLX90614: %d Hz (Target: 10)\n", freq_mlx_cnt);
      Serial.printf("  Control : %d Hz (Target: 20)\n", freq_ctl_cnt);
      Serial.printf("  Input   : %d Hz (Target: 50)\n", freq_input_cnt);
      Serial.printf("  Display : %d Hz (Target: 10)\n", freq_disp_cnt);

      freq_max_cnt = 0; freq_mlx_cnt = 0; freq_ctl_cnt = 0;
      freq_input_cnt = 0; freq_disp_cnt = 0;

      Serial.println("--- [Module Data Log] ---");
      if (xSemaphoreTake(dataMutex, 100) == pdTRUE) {
        Serial.printf("  TC1: %.2f | TC2: %.2f | TC3: %.2f\n", sysState.tc_temps[0], sysState.tc_temps[1], sysState.tc_temps[2]);
        Serial.printf("  IR1: Obj %.2f / Amb %.2f | IR2: Obj %.2f / Amb %.2f\n",
                      sysState.ir_temps[0], sysState.ir_ambient[0],
                      sysState.ir_temps[1], sysState.ir_ambient[1]);
        Serial.printf("  Target: %.2f | Active: %s\n", go_to, has_go_to ? "YES" : "NO");
        Serial.printf("  Auto Step: %d | Auto Started: %s\n", sysState.auto_step, sysState.auto_was_started ? "YES" : "NO");
        xSemaphoreGive(dataMutex);
      }
      Serial.println("===============================");
      xSemaphoreGive(serialMutex);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 7. SOUND TASK
void TaskSound(void* pvParameters) {
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  for (;;) {
    if (beep_queue > 0) {
      int on_time = 60;
      int off_time = 100;
      if (beep_mode == 1) {
        on_time = 250; off_time = 250;
      } else if (beep_mode == 3) {
        on_time = 3000; off_time = 100;
      }

      if (config.sound_on) {
        digitalWrite(BUZZER, HIGH);
      }

      vTaskDelay(pdMS_TO_TICKS(on_time));
      digitalWrite(BUZZER, LOW);
      vTaskDelay(pdMS_TO_TICKS(off_time));

      if (beep_queue >= 2) beep_queue -= 2;
      else beep_queue = 0;

      if (beep_queue == 0) beep_mode = 0;
    } else {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

// ========== [Setup & Loop] ==========
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n=== Smart Heater Booting ===");

  spiMutex = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();

  // Initialize sysState
  for (int i = 0; i < 3; i++) {
    sysState.tc_temps[i] = NAN;
    sysState.displayed_temps[i] = NAN;
    sysState.cj_temps[i] = NAN;
    sysState.tc_faults[i] = 0;
    sysState.heater_cutoff[i] = false;
    sysState.heater_ready[i] = false;
  }
  sysState.tc_probe_temp = NAN;
  sysState.tc_probe_peak = NAN;
  sysState.ir_temps[0] = NAN;
  sysState.ir_temps[1] = NAN;
  sysState.ir_ambient[0] = NAN;
  sysState.ir_ambient[1] = NAN;
  sysState.auto_step = 0;
  sysState.auto_mode_enabled = false;
  sysState.auto_was_started = false;
  sysState.manual_was_started = false;
  sysState.manual_preset_running = false;
  sysState.manual_preset_index = 0;
  sysState.manual_mode_enabled = false;

  // Setup WiFi in background (Non-Blocking)
  Serial.println("[Setup] WiFi will be managed by background task");

  preferences.begin("app_config", false);
  loadConfig(config);

  if (config.auto_target_temps[0] == 0.0f) {
    for (int i = 0; i < 3; i++) {
      config.auto_target_temps[i] = 100.0f;
      config.auto_max_temps[i] = 250.0f;
    }
    for (int i = 0; i < 4; i++) {
      config.manual_target_temps[i] = 100.0f;
      config.manual_max_temps[i] = 250.0f;
    }
    saveConfig(config);
  }

  if (config.max_temp_lock == 0.0f) {
    Serial.println("Config Invalid. Loading Defaults...");
    config.max_temp_lock = 400.0f;
    config.temp_unit = 'C';
    config.idle_off_mode = IDLE_OFF_30_MIN;
    config.startup_mode = STARTUP_OFF;
    config.sound_on = true;
    config.ir_emissivity[0] = 1.0f;
    config.ir_emissivity[1] = 1.0f;
    config.tc_probe_offset = 0.0f;
    for (int i = 0; i < 3; i++) {
      config.target_temps[i] = 200.0f;
      config.max_temps[i] = 250.0f;
      config.tc_offsets[i] = 0.0f;
      config.heater_active[i] = false;
    }
    saveConfig(config);
  }

  if (config.startup_mode == STARTUP_OFF) {
    has_go_to = false;
  }
  if (config.startup_mode == STARTUP_AUTORUN) {
    bool any_active = false;
    for (int i = 0; i < 3; i++) {
      if (config.heater_active[i]) any_active = true;
    }
    if (any_active) {
      has_go_to = true;
      sysState.manual_was_started = true;
      Serial.println("Startup: Auto-Run Triggered");
    }
  } else {
    has_go_to = false;
  }

  pinMode(SSR_PIN1, OUTPUT); digitalWrite(SSR_PIN1, LOW);
  pinMode(SSR_PIN2, OUTPUT); digitalWrite(SSR_PIN2, LOW);
  pinMode(SSR_PIN3, OUTPUT); digitalWrite(SSR_PIN3, LOW);
  pinMode(BUZZER, OUTPUT);   digitalWrite(BUZZER, LOW);
  pinMode(TFT_CS, OUTPUT);   digitalWrite(TFT_CS, HIGH);
  pinMode(ENCODER_SW, INPUT_PULLUP);

  max31855_init_pins();
  setup_encoder_fixed();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  mlx1.begin(IR1_ADDR, &Wire);
  mlx2.begin(IR2_ADDR, &Wire);

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  ui.begin();
  ui.setWiFiReconnectCallback(triggerWiFiReconnect);
  Serial.println("Hardware initialized. Starting tasks...");

  // Core 0: WiFi
  xTaskCreatePinnedToCore(TaskWiFiManager, "WiFiMgr", 4096, NULL, 1, NULL, 0);
  
  // Core 1: Application
  xTaskCreatePinnedToCore(TaskControl, "Control", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskMAX, "MAX31855", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskMLX, "MLX90614", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskInput, "Input", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskDisplay, "Display", 8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskDebug, "Debug", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskSound, "Sound", 2048, NULL, 1, NULL, 1);

  tpo_ticker.attach_ms(1, tpo_isr);
  Serial.println("=== System Started Successfully ===");
}

void loop() {
  ElegantOTA.loop();
  vTaskDelay(pdMS_TO_TICKS(20));
}

// ========== [WiFi & Web Server Functions] ==========
void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
    html += "<title>Smart Heater</title>";
    html += "<style>";
    html += "body{font-family:Arial;margin:20px;background:#1a1a2e;color:#eee}";
    html += "h2{color:#0ff}";
    html += ".val{color:#0f0;font-weight:bold}";
    html += ".warn{color:#f80}";
    html += ".err{color:#f00}";
    html += "table{width:100%;border-collapse:collapse}";
    html += "th,td{padding:8px;text-align:left;border:1px solid #444}";
    html += "th{background:#333}";
    html += "button{padding:10px 20px;margin:5px;cursor:pointer}";
    html += "</style></head><body>";
    html += "<h2>Smart Heater Status</h2>";
    
    html += "<h3>WiFi Status</h3>";
    html += "<p>Signal Strength: <span class='val'>" + String(wifiSignalStrength) + " dBm</span></p>";
    html += "<p>IP: <span class='val'>" + WiFi.localIP().toString() + "</span></p>";
    
    float tcs[3] = {0}, irs[2] = {0};
    bool manual_started = false, auto_started = false;
    uint8_t a_step = 0;
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      for (int i = 0; i < 3; i++) tcs[i] = sysState.tc_temps[i];
      for (int i = 0; i < 2; i++) irs[i] = sysState.ir_temps[i];
      manual_started = sysState.manual_was_started;
      auto_started = sysState.auto_was_started;
      a_step = sysState.auto_step;
      xSemaphoreGive(dataMutex);
    }
    
    html += "<hr><h3>Temperature Readings</h3>";
    html += "<table>";
    html += "<tr><th>Sensor</th><th>Value</th></tr>";
    for (int i = 0; i < 3; i++) {
      html += "<tr><td>TC" + String(i+1) + "</td>";
      html += "<td class='val'>" + String(tcs[i], 1) + " &deg;C</td></tr>";
    }
    for (int i = 0; i < 2; i++) {
      html += "<tr><td>IR" + String(i+1) + "</td>";
      html += "<td class='val'>" + String(irs[i], 1) + " &deg;C</td></tr>";
    }
    html += "</table>";
    html += "<hr><h3>System Status</h3>";
    html += "<p>Manual Started: <span class='" + String(manual_started ? "val" : "warn") + "'>";
    html += String(manual_started ? "YES" : "NO") + "</span></p>";
    html += "<p>Auto Started: <span class='" + String(auto_started ? "val" : "warn") + "'>";
    html += String(auto_started ? "YES" : "NO") + "</span></p>";
    html += "<p>Auto Step: " + String(a_step) + "</p>";
    html += "<br><a href='/update'><button>OTA Update</button></a>";
    html += "</body></html>";
    
    request->send(200, "text/html", html);
  });
  
  ElegantOTA.begin(&server);
  server.begin();
  Serial.println("[WiFi] HTTP Server Started");
}

WiFiConnectionStatus getWiFiStatus() {
  return wifiStatus;
}

int getWiFiSignalStrength() {
  return wifiSignalStrength;
}

// ========== [Hardware Helper Functions] ==========
float applyEmissivity(float t_obj_sensor, float t_amb, float emissivity, float sensor_emissivity) {
  if (isnan(t_obj_sensor) || isnan(t_amb)) return NAN;
  if (fabs(emissivity - sensor_emissivity) < 0.01f) return t_obj_sensor;

  float Tk_obj = t_obj_sensor + 273.15f;
  float Tk_amb = t_amb + 273.15f;
  float Tk_true_4 = pow(Tk_amb, 4) + (pow(Tk_obj, 4) - pow(Tk_amb, 4)) * (sensor_emissivity / emissivity);

  float Tk_real = pow(Tk_true_4, 0.25f);
  return Tk_real - 273.15f;
}

void IRAM_ATTR tpo_isr() {
  uint32_t pos;
  uint32_t onTicks[3];
  portENTER_CRITICAL_ISR(&tpoMux);
  pos = tpo_tick_counter;
  for (int i = 0; i < 3; i++) onTicks[i] = tpo_on_ticks[i];
  tpo_tick_counter++;
  if (tpo_tick_counter >= tpo_window_period_ticks) tpo_tick_counter = 0;
  portEXIT_CRITICAL_ISR(&tpoMux);
  digitalWrite(SSR_PIN1, (pos < onTicks[0]) ? HIGH : LOW);
  digitalWrite(SSR_PIN2, (pos < onTicks[1]) ? HIGH : LOW);
  digitalWrite(SSR_PIN3, (pos < onTicks[2]) ? HIGH : LOW);
}

void tpo_set_percent(int index, float percent) {
  if (index < 0 || index >= 3) return;
  percent = constrain(percent, 0, 100);
  uint32_t onTicks = (uint32_t)((percent / 100.0f) * (float)tpo_window_period_ticks + 0.5f);
  portENTER_CRITICAL(&tpoMux);
  tpo_on_ticks[index] = onTicks;
  portEXIT_CRITICAL(&tpoMux);
}

void max31855_init_pins() {
  for (int i = 0; i < 3; i++) {
    pinMode(TC_CS_PINS[i], OUTPUT);
    digitalWrite(TC_CS_PINS[i], HIGH);
  }
  pinMode(TC_PROBE_PIN, OUTPUT);
  digitalWrite(TC_PROBE_PIN, HIGH);
}

void setup_encoder_fixed() {
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pcnt_config_t pc_config = {};
  pc_config.pulse_gpio_num = ENCODER_A;
  pc_config.ctrl_gpio_num = ENCODER_B;
  pc_config.channel = PCNT_CHANNEL_0;
  pc_config.unit = PCNT_UNIT_0;
  pc_config.pos_mode = PCNT_COUNT_DEC;
  pc_config.neg_mode = PCNT_COUNT_INC;
  pc_config.lctrl_mode = PCNT_MODE_REVERSE;
  pc_config.hctrl_mode = PCNT_MODE_KEEP;
  pc_config.counter_h_lim = 32000;
  pc_config.counter_l_lim = -32000;
  pcnt_unit_config(&pc_config);
  pcnt_set_filter_value(PCNT_UNIT_0, 100);
  pcnt_filter_enable(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);
}

void read_hardware_encoder(float* delta_out) {
  int16_t val = 0;
  if (pcnt_get_counter_value(PCNT_UNIT_0, &val) == ESP_OK) {
    long diff = (long)val - lastEncoderValue;
    if (abs(diff) >= ENCODER_DIVIDER) {
      int steps = diff / ENCODER_DIVIDER;
      if (steps != 0) {
        *delta_out = (float)steps;
        lastEncoderValue += (steps * ENCODER_DIVIDER);
      }
    }
    if (abs(val) > 20000) {
      pcnt_counter_clear(PCNT_UNIT_0);
      lastEncoderValue = 0;
    }
  }
}

void saveConfig(const ConfigState& cfg) {
  preferences.putBytes("config", &cfg, sizeof(cfg));
}

void loadConfig(ConfigState& cfg) {
  if (preferences.getBytesLength("config") == sizeof(cfg))
    preferences.getBytes("config", &cfg, sizeof(cfg));

  if (cfg.wifi_config.ssid[0] == 0xFF) { 
   memset(&cfg.wifi_config, 0, sizeof(WiFiConfig));
   cfg.wifi_config.use_custom = false;
}
}