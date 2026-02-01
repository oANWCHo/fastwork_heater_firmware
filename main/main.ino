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
#define TFT_BL 38

#define ENCODER_A 6
#define ENCODER_B 7
#define ENCODER_SW 8

#define SDA_PIN 4
#define SCL_PIN 5
#define BUZZER 41
#define BUZZER_CHANNEL 2  // ใช้ Channel 2 (จะได้ไม่ชนกับ Backlight ที่มักใช้ 0 หรือ 1)
#define BUZZER_FREQ 2700
#define BUZZER_RES 8  // ความละเอียด 8-bit
#define PCF_ADDR 0x20
#define PCF_INT 21

#define IR1_ADDR 0x10
#define IR2_ADDR 0x11
#define SSR_PIN1 42
#define SSR_PIN2 2
#define SSR_PIN3 1

#define WINDOW_MS_INIT 1000
#define ENCODER_DIVIDER 2

#define BEEP_MODE_READY 2
#define BEEP_MODE_ALARM 3

#define MAIN_HEATER_INDEX 0  //(0=Heater1, 1=Heater2, 2=Heater3)

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
  bool heater_ready[3];  // True if stable for 10s
  uint8_t auto_step;     // 0=Off, 1=Cycle1, 2=Cycle2, 3=Cycle3
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
float Kp[3] = { 1.2f, 1.2f, 1.2f };
float Ki[3] = { 0.02f, 0.02f, 0.02f };
float Kd[3] = { 0.2f, 0.2f, 0.2f };
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
void setBrightness(uint8_t val);
WiFiConnectionStatus getWiFiStatus();
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
          if (final_temp < 0.0f) final_temp = 0.0f;
        }

        if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
          if (i < 3) {
            sysState.tc_temps[i] = final_temp;
            sysState.tc_faults[i] = is_fault ? 1 : 0;
            if (is_fault) {
              // 1. สั่งปิด Heater ตัวที่เสีย (เอาติ๊กถูกออก)
              config.heater_active[i] = false;

              // 2. ถ้าเป็น Heater 1 (Main) ให้สั่งหยุด Auto และ Preset ทันที
              if (i == 0) {
                sysState.auto_was_started = false;       // ปิด Auto Mode
                sysState.manual_preset_running = false;  // ปิด Manual Preset
                sysState.auto_step = 0;

                // 3. ถ้าเครื่องกำลังรันอยู่ ให้ Force Stop และร้องเตือน
                if (has_go_to) {
                  has_go_to = false;                    // สั่ง Stop
                  sysState.manual_was_started = false;  // ปิด Manual (Standby run)

                  // สั่งร้องเตือน (Alarm)
                  beep_mode = 3;
                  beep_queue = 10;
                }
              }
            }
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


// ========== [HEATER CONTROL TASKS - Separated into 3 tasks] ==========

// Shared state for auto mode cycle control (only used by Heater 1)
static volatile uint32_t cycle_change_debounce = 0;

// Wire probe peak tracking (shared, updated by Heater 1 task)
static volatile float wire_max_tracker = -999.0f;
static volatile uint32_t wire_max_timer = 0;

// Constants shared by all heater tasks
const float READY_THRESHOLD = 5.0f;
const uint32_t READY_HOLD_MS = 5000;

// 3a. HEATER 1 CONTROL TASK (Main Heater - handles auto mode)
void TaskHeater1Control(void* pvParameters) {
  const int HEATER_IDX = 0;
  const TickType_t xFrequency = pdMS_TO_TICKS(50);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Local state for this heater
  float ready_stable_start = 0;
  float peak_temp = 0;
  bool has_reached_target = false;
  uint32_t snapshot_timer = 0;
  float last_displayed_val = 0;
  bool prev_ready_state = false;

  uint32_t auto_low_temp_timer = 0;

  for (;;) {
    float current_t = NAN;
    float ir1_temp = NAN;
    float current_wire_temp = NAN;
    uint8_t current_auto_step = 0;
    bool local_auto_started = false;
    bool local_manual_started = false;
    bool local_manual_preset_running = false;
    uint8_t local_preset_idx = 0;

    // Read shared data
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      current_t = sysState.tc_temps[HEATER_IDX];
      ir1_temp = sysState.ir_temps[0];
      current_auto_step = sysState.auto_step;
      current_wire_temp = sysState.tc_probe_temp;
      local_auto_started = sysState.auto_was_started;
      local_manual_started = sysState.manual_was_started;
      local_manual_preset_running = sysState.manual_preset_running;
      local_preset_idx = sysState.manual_preset_index;
      xSemaphoreGive(dataMutex);
    }

    // Wire probe peak tracking (Heater 1 handles this)
    if (!isnan(current_wire_temp)) {
      if (current_wire_temp > wire_max_tracker) {
        wire_max_tracker = current_wire_temp;
      }
    }
    if (millis() - wire_max_timer > 5000) {
      float wire_max_display = wire_max_tracker;
      wire_max_tracker = -999.0f;
      wire_max_timer = millis();
      if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
        sysState.tc_probe_peak = wire_max_display;
        xSemaphoreGive(dataMutex);
      }
    }

    bool auto_is_running = (local_auto_started && current_auto_step > 0);
    bool system_run = has_go_to;

    bool is_active = config.heater_active[HEATER_IDX];
    float target_t = config.target_temps[HEATER_IDX];
    float max_t = config.max_temps[HEATER_IDX];
    bool is_auto_controlled = false;

    // 1. AUTO MODE OVERRIDE (only Heater 1)
    if (auto_is_running) {
      is_active = true;
      is_auto_controlled = true;
      int cycle_idx = current_auto_step - 1;
      target_t = config.auto_target_temps[cycle_idx];
      max_t = config.auto_max_temps[cycle_idx];
#if AUTO_CONTROL_SENSOR == 1
      current_t = ir1_temp;
#endif
    }
    // 2. MANUAL PRESET MODE (only Heater 1)
    else if (local_manual_preset_running && system_run) {
      is_active = true;
      is_auto_controlled = true;
      target_t = config.manual_target_temps[local_preset_idx];
      max_t = config.manual_max_temps[local_preset_idx];
    }
    // 3. STANDBY / BASIC MANUAL
    else {
      if (!local_manual_started) {
        is_active = false;
      }
    }
    if (has_go_to && is_active && isnan(current_t)) {
      if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        has_go_to = false;
        sysState.manual_was_started = false;
        sysState.auto_was_started = false;
        sysState.manual_preset_running = false;
        sysState.auto_step = 0;
        xSemaphoreGive(dataMutex);
      }
      beep_mode = 3;
      beep_queue = 10;
    }
    // === PID & OUTPUT LOGIC ===
    float output_percent = 0.0f;
    bool cutoff_active = false;
    bool is_ready_status = false;

    if (has_go_to && is_active && !isnan(current_t)) {
      if (current_t >= max_t) {
        cutoff_active = true;
        output_percent = 0.0f;
        pid_integral[HEATER_IDX] = 0;
      } else {
        float error = target_t - current_t;
        uint32_t now = millis();
        float dt = (pid_last_time[HEATER_IDX] == 0) ? 0.05f : (now - pid_last_time[HEATER_IDX]) / 1000.0f;
        if (dt > 0.5f) dt = 0.05f;
        pid_last_time[HEATER_IDX] = now;

        pid_integral[HEATER_IDX] += error * dt;
        pid_integral[HEATER_IDX] = constrain(pid_integral[HEATER_IDX], -100, 100);
        float derivative = (dt > 0) ? ((error - pid_prev_error[HEATER_IDX]) / dt) : 0;
        pid_prev_error[HEATER_IDX] = error;
        float P = Kp[HEATER_IDX] * error;
        float I = Ki[HEATER_IDX] * pid_integral[HEATER_IDX];
        float D = Kd[HEATER_IDX] * derivative;
        output_percent = P + I + D;
        output_percent = constrain(output_percent, 0, 100);

        if (fabs(error) <= READY_THRESHOLD) {
          if (ready_stable_start == 0) ready_stable_start = millis();
          if (millis() - ready_stable_start >= READY_HOLD_MS) {
            is_ready_status = true;
          }
        } else {
          ready_stable_start = 0;
        }
      }
    } else {
      output_percent = 0.0f;
      pid_integral[HEATER_IDX] = 0;
      pid_prev_error[HEATER_IDX] = 0;
      pid_last_time[HEATER_IDX] = 0;
      ready_stable_start = 0;
    }

    // Ready beep
    if (is_ready_status && !prev_ready_state) {
      beep_mode = BEEP_MODE_READY;
      beep_queue = 6;
    }
    prev_ready_state = is_ready_status;

    tpo_set_percent(HEATER_IDX, output_percent);

    if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
      sysState.heater_cutoff[HEATER_IDX] = cutoff_active;
      xSemaphoreGive(dataMutex);
    }

    // === AUTO MODE CYCLE TRANSITION (only Heater 1) ===
    if (auto_is_running && has_go_to) {
#if AUTO_CONTROL_SENSOR == 1
      float control_temp = ir1_temp;
#else
      float control_temp = sysState.tc_temps[HEATER_IDX];
#endif
      if (!isnan(control_temp)) {
        if (is_ready_status) {
          has_reached_target = true;
          if (control_temp > peak_temp) peak_temp = control_temp;
        }

        if (has_reached_target && (control_temp < 40.0f)) {
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
            peak_temp = 0;
            has_reached_target = false;
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
      }
      peak_temp = 0;
      has_reached_target = false;
      cycle_change_debounce = 0;
      auto_low_temp_timer = 0;
    }

    if (auto_is_running && has_go_to) {
      if (!isnan(ir1_temp) && ir1_temp < 40.0f) {
        
        if (auto_low_temp_timer == 0) {
          auto_low_temp_timer = millis(); 
        } 
        else if (millis() - auto_low_temp_timer > 30000) {
          
          // สั่ง Stop การทำงาน
          if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            has_go_to = false;               
            sysState.auto_was_started = false; 
            sysState.auto_step = 0;        
            
            sysState.manual_was_started = false;
            sysState.manual_preset_running = false;
            
            xSemaphoreGive(dataMutex);
          }
          
          beep_mode = 3; 
          beep_queue = 4; 

          auto_low_temp_timer = 0; 
        }
      } else {
        // ถ้าอุณหภูมิเกิน 40 หรืออ่านค่าไม่ได้ ให้รีเซ็ตเวลา
        auto_low_temp_timer = 0; 
      }
    } else {
      // ถ้าไม่ได้รัน Auto ก็รีเซ็ตเวลาทิ้ง
      auto_low_temp_timer = 0;
    }

    // === Display Freeze Logic ===
    float tc_temp = NAN;
    if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
      tc_temp = sysState.tc_temps[HEATER_IDX];
      xSemaphoreGive(dataMutex);
    }
    float val_to_display = tc_temp;
    if (is_ready_status) {
      if (millis() - snapshot_timer > 10000) {
        val_to_display = tc_temp;
        snapshot_timer = millis();
      } else {
        val_to_display = last_displayed_val;
      }
    } else {
      snapshot_timer = millis() - 10000;
    }
    last_displayed_val = val_to_display;

    if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
      sysState.heater_ready[HEATER_IDX] = is_ready_status;
      sysState.displayed_temps[HEATER_IDX] = val_to_display;
      sysState.auto_step = current_auto_step;
      xSemaphoreGive(dataMutex);
    }

    freq_ctl_cnt++;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 3b. HEATER 2 CONTROL TASK (Secondary Heater)
void TaskHeater2Control(void* pvParameters) {
  const int HEATER_IDX = 1;
  const TickType_t xFrequency = pdMS_TO_TICKS(50);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Local state for this heater
  float ready_stable_start = 0;
  uint32_t snapshot_timer = 0;
  float last_displayed_val = 0;
  bool prev_ready_state = false;

  for (;;) {
    float current_t = NAN;
    bool local_manual_started = false;
    bool local_auto_running = false;
    bool local_preset_running = false;

    // Read shared data
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      current_t = sysState.tc_temps[HEATER_IDX];
      local_manual_started = sysState.manual_was_started;
      local_auto_running = sysState.auto_was_started;
      local_preset_running = sysState.manual_preset_running;
      xSemaphoreGive(dataMutex);
    }

    bool is_active = config.heater_active[HEATER_IDX];
    float target_t = config.target_temps[HEATER_IDX];
    float max_t = config.max_temps[HEATER_IDX];

    if (local_auto_running || local_preset_running) {
      is_active = false;
    } else if (!local_manual_started) {
      is_active = false;
    }
    if (has_go_to && is_active && isnan(current_t)) {
      if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        has_go_to = false;
        sysState.manual_was_started = false;
        sysState.auto_was_started = false;
        sysState.manual_preset_running = false;
        xSemaphoreGive(dataMutex);
      }
      beep_mode = 3;
      beep_queue = 10;
    }
    // === PID & OUTPUT LOGIC ===
    float output_percent = 0.0f;
    bool cutoff_active = false;
    bool is_ready_status = false;

    if (has_go_to && is_active && !isnan(current_t)) {
      if (current_t >= max_t) {
        cutoff_active = true;
        output_percent = 0.0f;
        pid_integral[HEATER_IDX] = 0;
      } else {
        float error = target_t - current_t;
        uint32_t now = millis();
        float dt = (pid_last_time[HEATER_IDX] == 0) ? 0.05f : (now - pid_last_time[HEATER_IDX]) / 1000.0f;
        if (dt > 0.5f) dt = 0.05f;
        pid_last_time[HEATER_IDX] = now;

        pid_integral[HEATER_IDX] += error * dt;
        pid_integral[HEATER_IDX] = constrain(pid_integral[HEATER_IDX], -100, 100);
        float derivative = (dt > 0) ? ((error - pid_prev_error[HEATER_IDX]) / dt) : 0;
        pid_prev_error[HEATER_IDX] = error;
        float P = Kp[HEATER_IDX] * error;
        float I = Ki[HEATER_IDX] * pid_integral[HEATER_IDX];
        float D = Kd[HEATER_IDX] * derivative;
        output_percent = P + I + D;
        output_percent = constrain(output_percent, 0, 100);

        if (fabs(error) <= READY_THRESHOLD) {
          if (ready_stable_start == 0) ready_stable_start = millis();
          if (millis() - ready_stable_start >= READY_HOLD_MS) {
            is_ready_status = true;
          }
        } else {
          ready_stable_start = 0;
        }
      }
    } else {
      output_percent = 0.0f;
      pid_integral[HEATER_IDX] = 0;
      pid_prev_error[HEATER_IDX] = 0;
      pid_last_time[HEATER_IDX] = 0;
      ready_stable_start = 0;
    }

    // Ready beep
    if (is_ready_status && !prev_ready_state) {
      beep_mode = BEEP_MODE_READY;
      beep_queue = 6;
    }
    prev_ready_state = is_ready_status;

    tpo_set_percent(HEATER_IDX, output_percent);

    if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
      sysState.heater_cutoff[HEATER_IDX] = cutoff_active;
      xSemaphoreGive(dataMutex);
    }

    // === Display Freeze Logic ===
    float tc_temp = NAN;
    if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
      tc_temp = sysState.tc_temps[HEATER_IDX];
      xSemaphoreGive(dataMutex);
    }
    float val_to_display = tc_temp;
    if (is_ready_status) {
      if (millis() - snapshot_timer > 10000) {
        val_to_display = tc_temp;
        snapshot_timer = millis();
      } else {
        val_to_display = last_displayed_val;
      }
    } else {
      snapshot_timer = millis() - 10000;
    }
    last_displayed_val = val_to_display;

    if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
      sysState.heater_ready[HEATER_IDX] = is_ready_status;
      sysState.displayed_temps[HEATER_IDX] = val_to_display;
      xSemaphoreGive(dataMutex);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 3c. HEATER 3 CONTROL TASK (Tertiary Heater)
void TaskHeater3Control(void* pvParameters) {
  const int HEATER_IDX = 2;
  const TickType_t xFrequency = pdMS_TO_TICKS(50);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  bool local_auto_running = false;
  bool local_preset_running = false;

  // Local state for this heater
  float ready_stable_start = 0;
  uint32_t snapshot_timer = 0;
  float last_displayed_val = 0;
  bool prev_ready_state = false;

  for (;;) {
    float current_t = NAN;
    bool local_manual_started = false;

    // Read shared data
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      current_t = sysState.tc_temps[HEATER_IDX];
      local_manual_started = sysState.manual_was_started;
      local_auto_running = sysState.auto_was_started;
      local_preset_running = sysState.manual_preset_running;
      xSemaphoreGive(dataMutex);
    }

    bool is_active = config.heater_active[HEATER_IDX];
    float target_t = config.target_temps[HEATER_IDX];
    float max_t = config.max_temps[HEATER_IDX];

    if (local_auto_running || local_preset_running) {
      is_active = false;
    } else if (!local_manual_started) {
      is_active = false;
    }
    if (has_go_to && is_active && isnan(current_t)) {
      if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        has_go_to = false;
        sysState.manual_was_started = false;
        sysState.auto_was_started = false;
        sysState.manual_preset_running = false;
        xSemaphoreGive(dataMutex);
      }
      beep_mode = 3;
      beep_queue = 10;
    }
    // === PID & OUTPUT LOGIC ===
    float output_percent = 0.0f;
    bool cutoff_active = false;
    bool is_ready_status = false;

    if (has_go_to && is_active && !isnan(current_t)) {
      if (current_t >= max_t) {
        cutoff_active = true;
        output_percent = 0.0f;
        pid_integral[HEATER_IDX] = 0;
      } else {
        float error = target_t - current_t;
        uint32_t now = millis();
        float dt = (pid_last_time[HEATER_IDX] == 0) ? 0.05f : (now - pid_last_time[HEATER_IDX]) / 1000.0f;
        if (dt > 0.5f) dt = 0.05f;
        pid_last_time[HEATER_IDX] = now;

        pid_integral[HEATER_IDX] += error * dt;
        pid_integral[HEATER_IDX] = constrain(pid_integral[HEATER_IDX], -100, 100);
        float derivative = (dt > 0) ? ((error - pid_prev_error[HEATER_IDX]) / dt) : 0;
        pid_prev_error[HEATER_IDX] = error;
        float P = Kp[HEATER_IDX] * error;
        float I = Ki[HEATER_IDX] * pid_integral[HEATER_IDX];
        float D = Kd[HEATER_IDX] * derivative;
        output_percent = P + I + D;
        output_percent = constrain(output_percent, 0, 100);

        if (fabs(error) <= READY_THRESHOLD) {
          if (ready_stable_start == 0) ready_stable_start = millis();
          if (millis() - ready_stable_start >= READY_HOLD_MS) {
            is_ready_status = true;
          }
        } else {
          ready_stable_start = 0;
        }
      }
    } else {
      output_percent = 0.0f;
      pid_integral[HEATER_IDX] = 0;
      pid_prev_error[HEATER_IDX] = 0;
      pid_last_time[HEATER_IDX] = 0;
      ready_stable_start = 0;
    }

    // Ready beep
    if (is_ready_status && !prev_ready_state) {
      beep_mode = BEEP_MODE_READY;
      beep_queue = 6;
    }
    prev_ready_state = is_ready_status;

    tpo_set_percent(HEATER_IDX, output_percent);

    if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
      sysState.heater_cutoff[HEATER_IDX] = cutoff_active;
      xSemaphoreGive(dataMutex);
    }

    // === Display Freeze Logic ===
    float tc_temp = NAN;
    if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
      tc_temp = sysState.tc_temps[HEATER_IDX];
      xSemaphoreGive(dataMutex);
    }
    float val_to_display = tc_temp;
    if (is_ready_status) {
      if (millis() - snapshot_timer > 10000) {
        val_to_display = tc_temp;
        snapshot_timer = millis();
      } else {
        val_to_display = last_displayed_val;
      }
    } else {
      snapshot_timer = millis() - 10000;
    }
    last_displayed_val = val_to_display;

    if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
      sysState.heater_ready[HEATER_IDX] = is_ready_status;
      sysState.displayed_temps[HEATER_IDX] = val_to_display;
      xSemaphoreGive(dataMutex);
    }

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
            ui.handleButtonHold(config);
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
          if (scr == SCREEN_MANUAL_MODE) {
            sysState.manual_preset_index = ui.getManualConfirmedPreset();

            sysState.manual_preset_running = true;
            sysState.manual_was_started = true;
            sysState.auto_was_started = false;
            sysState.auto_step = 0;
            has_go_to = true;
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
            if (ui.getScreen() == SCREEN_SLEEP) {
              ui.switchToStandby();
              ui.resetInactivityTimer();
              xSemaphoreGive(dataMutex);
              pcf_state.btn_pressed[i] = btn[i];
              continue;
            }
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

              bool isAnyRunning = sysState.manual_was_started || sysState.auto_was_started || sysState.manual_preset_running;

              if (isAnyRunning) {
                // --- CASE: STOP ---
                sysState.manual_was_started = false;
                sysState.auto_was_started = false;
                sysState.auto_step = 0;
                sysState.manual_preset_running = false;
                has_go_to = false;

              } else {
                bool h1_fault = false;
                if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
                    h1_fault = (sysState.tc_faults[0] != 0);
                    xSemaphoreGive(dataMutex);
                }
                if (h1_fault) {
                  beep_mode = 3;  // เสียง Alarm สั้นๆ หรือแบบเตือน
                  beep_queue = 2;
                } else {
                  // --- CASE: START ---
                  if (current == SCREEN_STANDBY) {
                    sysState.manual_was_started = true;
                    has_go_to = true;

                  } else if (current == SCREEN_AUTO_MODE) {
                    sysState.auto_was_started = true;
                    sysState.auto_step = 1;
                    has_go_to = true;

                  } else if (current == SCREEN_MANUAL_MODE) {
                    sysState.manual_preset_running = true;
                    sysState.manual_preset_index = ui.getManualConfirmedPreset();
                    has_go_to = true;
                  }
                }
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
    bool all_sensors_ok = true;

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      if (ui.checkInactivity(config, has_go_to, go_to)) {
        beep_mode = 1;
        beep_queue = 6;
      }

      for (int i = 0; i < 3; i++) {
        st.tc_temps[i] = sysState.displayed_temps[i];
        if (isnan(st.tc_temps[i])) all_sensors_ok = false;
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
    
    st.is_warning = alarm_active || (beep_mode == 3);
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
    } else if (beep_mode == 3 && all_sensors_ok) {
      beep_queue = 0;
      beep_mode = 0;
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

      freq_max_cnt = 0;
      freq_mlx_cnt = 0;
      freq_ctl_cnt = 0;
      freq_input_cnt = 0;
      freq_disp_cnt = 0;

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
  ledcAttach(BUZZER, BUZZER_FREQ, BUZZER_RES);

  for (;;) {
    if (beep_queue > 0) {
      int on_time = 60;
      int off_time = 100;

      if (beep_mode == 1) {
        on_time = 250;
        off_time = 250;
      } else if (beep_mode == 3) {
        on_time = 3000;
        off_time = 100;
      }

      // --- ขับเสียงด้วย PWM (API v3.0+) ---
      if (config.sound_volume > 0) {
        int duty = map(config.sound_volume, 0, 100, 0, 128);

        // แก้ไข: ส่งค่าไปที่ขา BUZZER โดยตรง (ไม่ต้องใส่ Channel แล้ว)
        ledcWrite(BUZZER, duty);
      } else {
        ledcWrite(BUZZER, 0);
      }

      vTaskDelay(pdMS_TO_TICKS(on_time));

      ledcWrite(BUZZER, 0);  // ปิดเสียง
      vTaskDelay(pdMS_TO_TICKS(off_time));

      if (beep_queue >= 2) beep_queue -= 2;
      else beep_queue = 0;

      if (beep_queue == 0) beep_mode = 0;
    } else {
      ledcWrite(BUZZER, 0);  // เงียบเมื่อไม่มีคิว
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
    config.sound_volume = 100;
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

  pinMode(SSR_PIN1, OUTPUT);
  digitalWrite(SSR_PIN1, LOW);
  pinMode(SSR_PIN2, OUTPUT);
  digitalWrite(SSR_PIN2, LOW);
  pinMode(SSR_PIN3, OUTPUT);
  digitalWrite(SSR_PIN3, LOW);
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  max31855_init_pins();
  setup_encoder_fixed();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  mlx1.begin(IR1_ADDR, &Wire);
  mlx2.begin(IR2_ADDR, &Wire);

  setBrightness(config.brightness);

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  ui.begin();
  ui.setWiFiReconnectCallback(triggerWiFiReconnect);
  Serial.println("Hardware initialized. Starting tasks...");

  // Core 0: WiFi
  xTaskCreatePinnedToCore(TaskWiFiManager, "WiFiMgr", 4096, NULL, 1, NULL, 0);

  // Core 1: Application
  xTaskCreatePinnedToCore(TaskHeater1Control, "Heater1", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskHeater2Control, "Heater2", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskHeater3Control, "Heater3", 4096, NULL, 3, NULL, 1);
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
  
  // ========== [1. JSON API ENDPOINT - For 1-second updates] ==========
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* request) {
    String json = "{";
    
    float tcs[3] = {0}, irs[2] = {0}, ir_amb[2] = {0};
    float displayed[3] = {0};
    float probe_temp = 0, probe_peak = 0;
    uint8_t faults[3] = {0};
    bool cutoff[3] = {false}, ready[3] = {false};
    bool manual_started = false, auto_started = false, preset_running = false;
    uint8_t a_step = 0, preset_idx = 0;
    uint32_t on_ticks[3] = {0};
    
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      for (int i = 0; i < 3; i++) {
        tcs[i] = sysState.tc_temps[i];
        displayed[i] = sysState.displayed_temps[i];
        faults[i] = sysState.tc_faults[i];
        cutoff[i] = sysState.heater_cutoff[i];
        ready[i] = sysState.heater_ready[i];
      }
      for (int i = 0; i < 2; i++) {
        irs[i] = sysState.ir_temps[i];
        ir_amb[i] = sysState.ir_ambient[i];
      }
      probe_temp = sysState.tc_probe_temp;
      probe_peak = sysState.tc_probe_peak;
      manual_started = sysState.manual_was_started;
      auto_started = sysState.auto_was_started;
      preset_running = sysState.manual_preset_running;
      preset_idx = sysState.manual_preset_index;
      a_step = sysState.auto_step;
      xSemaphoreGive(dataMutex);
    }
    
    // Get TPO on_ticks for heater power %
    portENTER_CRITICAL(&tpoMux);
    for (int i = 0; i < 3; i++) on_ticks[i] = tpo_on_ticks[i];
    uint32_t window = tpo_window_period_ticks;
    portEXIT_CRITICAL(&tpoMux);
    
    // Calculate power percentages
    float power[3];
    for (int i = 0; i < 3; i++) {
      power[i] = (window > 0) ? (on_ticks[i] * 100.0f / window) : 0;
    }
    
    // Determine current mode
    String mode = "STANDBY";
    if (has_go_to) {
      if (auto_started && a_step > 0) {
        mode = "AUTO_CYCLE_" + String(a_step);
      } else if (preset_running) {
        mode = "PRESET_" + String(preset_idx + 1);
      } else if (manual_started) {
        mode = "MANUAL";
      }
    }
    
    // Build JSON
    json += "\"mode\":\"" + mode + "\",";
    json += "\"timestamp\":" + String(millis()) + ",";
    
    // Temperatures
    json += "\"temperatures\":{";
    json += "\"tc\":[";
    for (int i = 0; i < 3; i++) {
      json += isnan(tcs[i]) ? "null" : String(tcs[i], 1);
      if (i < 2) json += ",";
    }
    json += "],\"displayed\":[";
    for (int i = 0; i < 3; i++) {
      json += isnan(displayed[i]) ? "null" : String(displayed[i], 1);
      if (i < 2) json += ",";
    }
    json += "],\"ir\":[";
    for (int i = 0; i < 2; i++) {
      json += isnan(irs[i]) ? "null" : String(irs[i], 1);
      if (i < 1) json += ",";
    }
    json += "],\"ir_ambient\":[";
    for (int i = 0; i < 2; i++) {
      json += isnan(ir_amb[i]) ? "null" : String(ir_amb[i], 1);
      if (i < 1) json += ",";
    }
    json += "],\"probe\":" + (isnan(probe_temp) ? String("null") : String(probe_temp, 1));
    json += ",\"probe_peak\":" + (isnan(probe_peak) ? String("null") : String(probe_peak, 1));
    json += "},";
    
    // Heaters status
    json += "\"heaters\":[";
    for (int i = 0; i < 3; i++) {
      json += "{";
      json += "\"active\":" + String(config.heater_active[i] ? "true" : "false") + ",";
      json += "\"target\":" + String(config.target_temps[i], 1) + ",";
      json += "\"max\":" + String(config.max_temps[i], 1) + ",";
      json += "\"power\":" + String(power[i], 1) + ",";
      json += "\"cutoff\":" + String(cutoff[i] ? "true" : "false") + ",";
      json += "\"ready\":" + String(ready[i] ? "true" : "false") + ",";
      json += "\"fault\":" + String(faults[i]);
      json += "}";
      if (i < 2) json += ",";
    }
    json += "],";
    
    // Warnings
    json += "\"warnings\":[";
    bool first_warn = true;
    for (int i = 0; i < 3; i++) {
      if (faults[i]) {
        if (!first_warn) json += ",";
        json += "\"TC" + String(i+1) + " FAULT\"";
        first_warn = false;
      }
      if (cutoff[i]) {
        if (!first_warn) json += ",";
        json += "\"Heater " + String(i+1) + " CUTOFF\"";
        first_warn = false;
      }
    }
    json += "],";
    
    // WiFi
    json += "\"wifi\":{";
    json += "\"rssi\":" + String(wifiSignalStrength) + ",";
    json += "\"ip\":\"" + WiFi.localIP().toString() + "\"";
    json += "}";
    
    json += "}";
    
    request->send(200, "application/json", json);
  });

  // ========== [2. MAIN DASHBOARD PAGE] ==========
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Smart Heater Dashboard</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { 
      font-family: 'Segoe UI', Arial, sans-serif; 
      background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
      color: #eee; 
      min-height: 100vh;
      padding: 20px;
    }
    .container { max-width: 1200px; margin: 0 auto; }
    h1 { 
      text-align: center; 
      color: #0ff; 
      margin-bottom: 20px;
      text-shadow: 0 0 10px rgba(0,255,255,0.5);
    }
    .status-bar {
      display: flex;
      justify-content: space-between;
      align-items: center;
      background: rgba(0,0,0,0.3);
      padding: 10px 20px;
      border-radius: 10px;
      margin-bottom: 20px;
    }
    .mode-badge {
      padding: 8px 20px;
      border-radius: 20px;
      font-weight: bold;
      font-size: 1.1em;
    }
    .mode-STANDBY { background: #666; }
    .mode-MANUAL { background: #2ecc71; }
    .mode-AUTO { background: #e74c3c; }
    .mode-PRESET { background: #f39c12; }
    
    .grid { 
      display: grid; 
      grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); 
      gap: 20px; 
      margin-bottom: 20px;
    }
    .card {
      background: rgba(255,255,255,0.05);
      border-radius: 15px;
      padding: 20px;
      border: 1px solid rgba(255,255,255,0.1);
    }
    .card h3 { 
      color: #0ff; 
      margin-bottom: 15px;
      border-bottom: 1px solid rgba(0,255,255,0.3);
      padding-bottom: 10px;
    }
    
    .heater-card {
      position: relative;
      overflow: hidden;
    }
    .heater-card.active { border-color: #2ecc71; }
    .heater-card.fault { border-color: #e74c3c; animation: pulse-red 1s infinite; }
    .heater-card.cutoff { border-color: #f39c12; }
    .heater-card.ready { border-color: #2ecc71; box-shadow: 0 0 20px rgba(46,204,113,0.3); }
    
    @keyframes pulse-red {
      0%, 100% { box-shadow: 0 0 5px rgba(231,76,60,0.5); }
      50% { box-shadow: 0 0 20px rgba(231,76,60,0.8); }
    }
    
    .temp-display {
      font-size: 2.5em;
      font-weight: bold;
      text-align: center;
      padding: 15px;
      background: rgba(0,0,0,0.3);
      border-radius: 10px;
      margin: 10px 0;
    }
    .temp-display.fault { color: #e74c3c; }
    .temp-display.normal { color: #2ecc71; }
    .temp-display.hot { color: #f39c12; }
    
    .power-bar {
      height: 20px;
      background: #333;
      border-radius: 10px;
      overflow: hidden;
      margin: 10px 0;
    }
    .power-fill {
      height: 100%;
      background: linear-gradient(90deg, #2ecc71, #f39c12, #e74c3c);
      transition: width 0.3s ease;
    }
    
    .info-row {
      display: flex;
      justify-content: space-between;
      padding: 5px 0;
      border-bottom: 1px solid rgba(255,255,255,0.1);
    }
    .info-label { color: #888; }
    .info-value { font-weight: bold; }
    .info-value.ok { color: #2ecc71; }
    .info-value.warn { color: #f39c12; }
    .info-value.err { color: #e74c3c; }
    
    .warnings-panel {
      background: rgba(231,76,60,0.2);
      border: 1px solid #e74c3c;
      border-radius: 10px;
      padding: 15px;
      margin-bottom: 20px;
      display: none;
    }
    .warnings-panel.visible { display: block; }
    .warnings-panel h3 { color: #e74c3c; }
    .warning-item {
      padding: 8px;
      margin: 5px 0;
      background: rgba(231,76,60,0.3);
      border-radius: 5px;
    }
    
    #chart-container {
      background: rgba(0,0,0,0.3);
      border-radius: 10px;
      padding: 15px;
      height: 300px;
    }
    
    .footer {
      text-align: center;
      margin-top: 20px;
      color: #666;
    }
    .footer a {
      color: #0ff;
      text-decoration: none;
      margin: 0 10px;
    }
    
    .update-indicator {
      width: 10px;
      height: 10px;
      border-radius: 50%;
      background: #2ecc71;
      display: inline-block;
      margin-right: 5px;
    }
    .update-indicator.updating { animation: blink 0.5s infinite; }
    @keyframes blink { 50% { opacity: 0.3; } }
  </style>
</head>
<body>
  <div class="container">
    <h1>Heater Dashboard</h1>
    
    <div class="status-bar">
      <div>
        <span class="update-indicator" id="updateIndicator"></span>
        <span id="lastUpdate">Connecting...</span>
      </div>
      <div id="modeDisplay" class="mode-badge mode-STANDBY">STANDBY</div>
      <div>WiFi: <span id="wifiRssi">--</span> dBm</div>
    </div>
    
    <div class="warnings-panel" id="warningsPanel">
      <h3>⚠️ Warnings</h3>
      <div id="warningsList"></div>
    </div>
    
    <div class="grid">
      <!-- Heater 1 -->
      <div class="card heater-card" id="heater1">
        <h3>Heater 1 (Main)</h3>
        <div class="temp-display" id="temp1">-- °C</div>
        <div class="info-row">
          <span class="info-label">Target</span>
          <span class="info-value" id="target1">--</span>
        </div>
        <div class="info-row">
          <span class="info-label">Max</span>
          <span class="info-value" id="max1">--</span>
        </div>
        <div class="info-row">
          <span class="info-label">Power</span>
          <span class="info-value" id="power1">0%</span>
        </div>
        <div class="power-bar"><div class="power-fill" id="powerBar1" style="width:0%"></div></div>
        <div class="info-row">
          <span class="info-label">Status</span>
          <span class="info-value" id="status1">Inactive</span>
        </div>
      </div>
      
      <!-- Heater 2 -->
      <div class="card heater-card" id="heater2">
        <h3>Heater 2</h3>
        <div class="temp-display" id="temp2">-- °C</div>
        <div class="info-row">
          <span class="info-label">Target</span>
          <span class="info-value" id="target2">--</span>
        </div>
        <div class="info-row">
          <span class="info-label">Max</span>
          <span class="info-value" id="max2">--</span>
        </div>
        <div class="info-row">
          <span class="info-label">Power</span>
          <span class="info-value" id="power2">0%</span>
        </div>
        <div class="power-bar"><div class="power-fill" id="powerBar2" style="width:0%"></div></div>
        <div class="info-row">
          <span class="info-label">Status</span>
          <span class="info-value" id="status2">Inactive</span>
        </div>
      </div>
      
      <!-- Heater 3 -->
      <div class="card heater-card" id="heater3">
        <h3>Heater 3</h3>
        <div class="temp-display" id="temp3">-- °C</div>
        <div class="info-row">
          <span class="info-label">Target</span>
          <span class="info-value" id="target3">--</span>
        </div>
        <div class="info-row">
          <span class="info-label">Max</span>
          <span class="info-value" id="max3">--</span>
        </div>
        <div class="info-row">
          <span class="info-label">Power</span>
          <span class="info-value" id="power3">0%</span>
        </div>
        <div class="power-bar"><div class="power-fill" id="powerBar3" style="width:0%"></div></div>
        <div class="info-row">
          <span class="info-label">Status</span>
          <span class="info-value" id="status3">Inactive</span>
        </div>
      </div>
      
      <!-- IR Sensors -->
      <div class="card">
        <h3>IR Sensors</h3>
        <div class="info-row">
          <span class="info-label">IR1 Object</span>
          <span class="info-value" id="ir1">-- °C</span>
        </div>
        <div class="info-row">
          <span class="info-label">IR1 Ambient</span>
          <span class="info-value" id="ir1amb">-- °C</span>
        </div>
        <div class="info-row">
          <span class="info-label">IR2 Object</span>
          <span class="info-value" id="ir2">-- °C</span>
        </div>
        <div class="info-row">
          <span class="info-label">IR2 Ambient</span>
          <span class="info-value" id="ir2amb">-- °C</span>
        </div>
        <div class="info-row" style="margin-top:15px; border-top: 2px solid #0ff; padding-top:10px;">
          <span class="info-label">Wire Probe</span>
          <span class="info-value" id="probe">-- °C</span>
        </div>
        <div class="info-row">
          <span class="info-label">Probe Peak</span>
          <span class="info-value" id="probePeak">-- °C</span>
        </div>
      </div>
    </div>
    
    <!-- Temperature Chart -->
    <div class="card">
      <h3>📈 Temperature History (Last 60 seconds)</h3>
      <div id="chart-container">
        <canvas id="tempChart"></canvas>
      </div>
    </div>
    
    <div class="footer">
      <a href="/update">🔄 OTA Update</a>
      <span>|</span>
      <span>IP: <span id="ipAddr">--</span></span>
    </div>
  </div>

  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <script>
    // Chart setup
    const ctx = document.getElementById('tempChart').getContext('2d');
    const maxDataPoints = 60;
    const chartData = {
      labels: [],
      datasets: [
        { label: 'TC1', data: [], borderColor: '#e74c3c', fill: false, tension: 0.3 },
        { label: 'TC2', data: [], borderColor: '#2ecc71', fill: false, tension: 0.3 },
        { label: 'TC3', data: [], borderColor: '#3498db', fill: false, tension: 0.3 },
        { label: 'IR1', data: [], borderColor: '#f39c12', fill: false, tension: 0.3, borderDash: [5,5] },
        { label: 'IR2', data: [], borderColor: '#9b59b6', fill: false, tension: 0.3, borderDash: [5,5] }
      ]
    };
    
    const chart = new Chart(ctx, {
      type: 'line',
      data: chartData,
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: { duration: 0 },
        scales: {
          y: { 
            beginAtZero: false,
            grid: { color: 'rgba(255,255,255,0.1)' },
            ticks: { color: '#888' }
          },
          x: { 
            grid: { color: 'rgba(255,255,255,0.05)' },
            ticks: { color: '#888', maxTicksLimit: 10 }
          }
        },
        plugins: {
          legend: { labels: { color: '#eee' } }
        }
      }
    });
    
    function addChartData(tc, ir) {
      const now = new Date().toLocaleTimeString();
      chartData.labels.push(now);
      chartData.datasets[0].data.push(tc[0]);
      chartData.datasets[1].data.push(tc[1]);
      chartData.datasets[2].data.push(tc[2]);
      chartData.datasets[3].data.push(ir[0]);
      chartData.datasets[4].data.push(ir[1]);
      
      if (chartData.labels.length > maxDataPoints) {
        chartData.labels.shift();
        chartData.datasets.forEach(ds => ds.data.shift());
      }
      chart.update('none');
    }
    
    function updateDashboard(data) {
      // Update mode
      const modeEl = document.getElementById('modeDisplay');
      modeEl.textContent = data.mode;
      modeEl.className = 'mode-badge';
      if (data.mode.includes('AUTO')) modeEl.classList.add('mode-AUTO');
      else if (data.mode.includes('PRESET')) modeEl.classList.add('mode-PRESET');
      else if (data.mode === 'MANUAL') modeEl.classList.add('mode-MANUAL');
      else modeEl.classList.add('mode-STANDBY');
      
      // Update heaters
      data.heaters.forEach((h, i) => {
        const idx = i + 1;
        const card = document.getElementById('heater' + idx);
        const tempEl = document.getElementById('temp' + idx);
        
        // Temperature
        const temp = data.temperatures.tc[i];
        tempEl.textContent = (temp !== null) ? temp.toFixed(1) + ' °C' : 'FAULT';
        tempEl.className = 'temp-display';
        if (h.fault) tempEl.classList.add('fault');
        else if (temp > h.max * 0.9) tempEl.classList.add('hot');
        else tempEl.classList.add('normal');
        
        // Card status
        card.className = 'card heater-card';
        if (h.fault) card.classList.add('fault');
        else if (h.cutoff) card.classList.add('cutoff');
        else if (h.ready) card.classList.add('ready');
        else if (h.active) card.classList.add('active');
        
        // Info
        document.getElementById('target' + idx).textContent = h.target.toFixed(1) + ' °C';
        document.getElementById('max' + idx).textContent = h.max.toFixed(1) + ' °C';
        document.getElementById('power' + idx).textContent = h.power.toFixed(0) + '%';
        document.getElementById('powerBar' + idx).style.width = h.power + '%';
        
        // Status text
        const statusEl = document.getElementById('status' + idx);
        if (h.fault) { statusEl.textContent = 'FAULT'; statusEl.className = 'info-value err'; }
        else if (h.cutoff) { statusEl.textContent = 'CUTOFF'; statusEl.className = 'info-value warn'; }
        else if (h.ready) { statusEl.textContent = 'READY'; statusEl.className = 'info-value ok'; }
        else if (h.active && h.power > 0) { statusEl.textContent = 'Heating'; statusEl.className = 'info-value warn'; }
        else if (h.active) { statusEl.textContent = 'Active'; statusEl.className = 'info-value ok'; }
        else { statusEl.textContent = 'Inactive'; statusEl.className = 'info-value'; }
      });
      
      // IR sensors
      document.getElementById('ir1').textContent = (data.temperatures.ir[0] !== null) ? data.temperatures.ir[0].toFixed(1) + ' °C' : '--';
      document.getElementById('ir2').textContent = (data.temperatures.ir[1] !== null) ? data.temperatures.ir[1].toFixed(1) + ' °C' : '--';
      document.getElementById('ir1amb').textContent = (data.temperatures.ir_ambient[0] !== null) ? data.temperatures.ir_ambient[0].toFixed(1) + ' °C' : '--';
      document.getElementById('ir2amb').textContent = (data.temperatures.ir_ambient[1] !== null) ? data.temperatures.ir_ambient[1].toFixed(1) + ' °C' : '--';
      document.getElementById('probe').textContent = (data.temperatures.probe !== null) ? data.temperatures.probe.toFixed(1) + ' °C' : '--';
      document.getElementById('probePeak').textContent = (data.temperatures.probe_peak !== null) ? data.temperatures.probe_peak.toFixed(1) + ' °C' : '--';
      
      // WiFi
      document.getElementById('wifiRssi').textContent = data.wifi.rssi;
      document.getElementById('ipAddr').textContent = data.wifi.ip;
      
      // Warnings
      const warnPanel = document.getElementById('warningsPanel');
      const warnList = document.getElementById('warningsList');
      if (data.warnings.length > 0) {
        warnPanel.classList.add('visible');
        warnList.innerHTML = data.warnings.map(w => '<div class="warning-item">⚠️ ' + w + '</div>').join('');
      } else {
        warnPanel.classList.remove('visible');
      }
      
      // Add to chart
      addChartData(data.temperatures.tc, data.temperatures.ir);
      
      // Update timestamp
      document.getElementById('lastUpdate').textContent = 'Updated: ' + new Date().toLocaleTimeString();
    }
    
    function fetchData() {
      const indicator = document.getElementById('updateIndicator');
      indicator.classList.add('updating');
      
      fetch('/api/status')
        .then(res => res.json())
        .then(data => {
          updateDashboard(data);
          indicator.classList.remove('updating');
        })
        .catch(err => {
          console.error('Fetch error:', err);
          document.getElementById('lastUpdate').textContent = 'Connection lost...';
          indicator.classList.remove('updating');
        });
    }
    
    // Initial fetch and start interval
    fetchData();
    setInterval(fetchData, 1000); // Update every 1 second
  </script>
</body>
</html>
)rawliteral";
    
    request->send(200, "text/html", html);
  });

  // Start OTA and server
  ElegantOTA.begin(&server);
  server.begin();
  Serial.println("[WiFi] HTTP Server Started with Dashboard");
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

void setBrightness(uint8_t val) {
  int duty = map(val, 0, 100, 0, 255);
  analogWrite(TFT_BL, duty);
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
  setBrightness(cfg.brightness);
}

void loadConfig(ConfigState& cfg) {
  if (preferences.getBytesLength("config") == sizeof(cfg))
    preferences.getBytes("config", &cfg, sizeof(cfg));

  if (cfg.wifi_config.ssid[0] == 0xFF) {
    memset(&cfg.wifi_config, 0, sizeof(WiFiConfig));
    cfg.wifi_config.use_custom = false;
  }

  if (cfg.brightness == 0 && cfg.wifi_config.use_custom == false) cfg.brightness = 100;
}