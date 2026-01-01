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

// ========== [Pin Definitions] ==========
// #define MAXDO   13   // MISO
// #define MAXCLK  12   // SCK
#define MAXDO 48   // MISO
#define MAXCLK 47  // SCK
#define MAXCS1 15  // CS1
#define MAXCS2 16  // CS2
#define MAXCS3 17  // CS3
#define MAXCS4 18  // CS4
// #define TFT_CS  10   // Guard Pin

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

// ========== [Shared Resources] ==========
SemaphoreHandle_t spiMutex;     // Protects SPI Bus
SemaphoreHandle_t dataMutex;    // Protects Shared Data
SemaphoreHandle_t serialMutex;  // Protects Serial Monitor (prevent mixed text)

// Thread-Safe Data Model
struct SharedData {
  float tc_temps[3];
  float tc_probe_temp;
  float cj_temps[3];
  uint8_t tc_faults[3];
  float ir_temps[2];
  float ir_ambient[2];
  bool heater_cutoff[3];
  bool heater_ready[3];  // New: True if stable for 10s
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

// ========== [PID Vars - UPDATED with D term] ==========
float Kp = 1.2f, Ki = 0.02f, Kd = 0.5f;  // Added proper Kd value
float pid_integral[3] = { 0.0f, 0.0f, 0.0f };
float pid_prev_error[3] = { 0.0f, 0.0f, 0.0f };  // Previous error for D term
uint32_t pid_last_time[3] = { 0, 0, 0 };         // Last time for dt calculation

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

// ========== [Tasks] ==========

// 1. MAX31855 TASK - Using Hardware SPI (HSPI from TFT)
void TaskMAX(void* pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(100);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // --- [Config Moving Average] ---
  const int MA_WINDOW = 10;
  static float ma_buffer[4][MA_WINDOW];  // Changed to 4 (0-2 for H, 3 for Probe)
  static int ma_idx[4] = { 0, 0, 0, 0 };
  static int ma_count[4] = { 0, 0, 0, 0 };

  // --- [Watchdog / Stuck Data Tracking] ---
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

      // Loop 0-2 (Heaters) + 3 (Probe)
      for (int i = 0; i < 4; i++) {
        uint32_t d = 0;
        int cs_pin = (i < 3) ? TC_CS_PINS[i] : TC_PROBE_PIN;  // Select CS

        digitalWrite(cs_pin, LOW);
        d = vspi->transfer32(0);
        digitalWrite(cs_pin, HIGH);

        // --- [FAULT DETECTION LOGIC] ---
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

        // --- [Moving Average] ---
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

        // --- [Update System State] ---
        if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
          if (i < 3) {
            // Heaters
            if (isnan(final_temp)) {
              sysState.tc_temps[i] = NAN;
            } else {
              sysState.tc_temps[i] = final_temp + config.tc_offsets[i];
            }
            sysState.tc_faults[i] = (is_fault) ? (d & 0x07) : 0;
          } else {
            // Probe (Index 3)
            if (isnan(final_temp)) {
              sysState.tc_probe_temp = NAN;
            } else {
              sysState.tc_probe_temp = final_temp + config.tc_probe_offset;  // Apply Probe Offset
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

// 2. MLX90614 TASK (I2C - Medium Priority)
void TaskMLX(void* pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(100);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    float raw_obj[2] = { NAN, NAN };
    float raw_amb[2] = { NAN, NAN };

    // Check IR1 Connection
    Wire.beginTransmission(IR1_ADDR);
    if (Wire.endTransmission() == 0) {
      raw_obj[0] = mlx1.readObjectTempC();
      raw_amb[0] = mlx1.readAmbientTempC();
      if (raw_obj[0] > 1000) raw_obj[0] = NAN;
    }

    // Check IR2 Connection
    Wire.beginTransmission(IR2_ADDR);
    if (Wire.endTransmission() == 0) {
      raw_obj[1] = mlx2.readObjectTempC();
      raw_amb[1] = mlx2.readAmbientTempC();
      if (raw_obj[1] > 1000) raw_obj[1] = NAN;
    }

    // Update Data
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      for (int i = 0; i < 2; i++) {
        sysState.ir_ambient[i] = raw_amb[i];
        sysState.ir_temps[i] = applyEmissivity(raw_obj[i], raw_amb[i], config.ir_emissivity[i]);
      }
      xSemaphoreGive(dataMutex);
    }

    freq_mlx_cnt++;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 3. CONTROL TASK (PID - High Priority) - UPDATED WITH FULL PID
void TaskControl(void* pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(50);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  uint32_t stable_start_time[3] = { 0, 0, 0 };
  bool beep_triggered[3] = { false, false, false };

  // Initialize PID timing
  for (int i = 0; i < 3; i++) {
    pid_last_time[i] = millis();
    pid_prev_error[i] = 0.0f;
  }

  for (;;) {
    float current_temps[3];
    bool active_flags[3];
    bool cutoff_flags[3];

    // 1. Read all shared data safely
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      memcpy(current_temps, sysState.tc_temps, sizeof(current_temps));
      for (int i = 0; i < 3; i++) {
        active_flags[i] = config.heater_active[i];
        cutoff_flags[i] = sysState.heater_cutoff[i];
      }
      xSemaphoreGive(dataMutex);
    }

    for (int i = 0; i < 3; i++) {

      // --- CASE 1: Heater is OFF (Inactive) or Invalid Sensor ---
      if (!active_flags[i] || isnan(current_temps[i])) {
        tpo_set_percent(i, 0);
        stable_start_time[i] = 0;
        // Reset PID state when heater is off
        pid_integral[i] = 0.0f;
        pid_prev_error[i] = 0.0f;
        pid_last_time[i] = millis();
        continue;
      }

      // --- CASE 2: Heater is LOCKED (Overheat Protection) ---
      // FIX: Auto-unlock when temperature drops below target temp
      if (cutoff_flags[i]) {
        if (current_temps[i] <= config.target_temps[i]) {
          // Temperature has cooled down below target - auto-unlock
          if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            sysState.heater_cutoff[i] = false;  // Release the lock
            // Re-enable the heater so it can resume operation
            //  config.heater_active[i] = true;
            has_go_to = false;
            // Reset PID to prevent windup issues
            pid_integral[i] = 0.0f;
            pid_prev_error[i] = 0.0f;
            pid_last_time[i] = millis();
            xSemaphoreGive(dataMutex);
          }
          // Don't set has_go_to to false - let it continue running
          // The heater will resume heating automatically
        }

        tpo_set_percent(i, 0);  // Keep power off while locked
        continue;
      }

      // --- System Run Check ---
      if (!has_go_to) {
        // Active, Safe, but System Stopped -> Standby Mode
        tpo_set_percent(i, 0);
        stable_start_time[i] = 0;
        beep_triggered[i] = false;
        // Reset PID state in standby
        pid_integral[i] = 0.0f;
        pid_prev_error[i] = 0.0f;
        pid_last_time[i] = millis();

        if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
          sysState.heater_ready[i] = false;
          xSemaphoreGive(dataMutex);
        }
        continue;
      }

      // --- Active Heating Logic ---

      // 1. Overheat Check (Safety)
      if (current_temps[i] >= config.max_temps[i]) {
        tpo_set_percent(i, 0);
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
          // config.heater_active[i] = false;   // Turn OFF
          sysState.heater_cutoff[i] = true;  // Set LOCK
          // Reset PID
          pid_integral[i] = 0.0f;
          pid_prev_error[i] = 0.0f;
          xSemaphoreGive(dataMutex);
        }
        beep_mode = BEEP_MODE_ALARM;
        beep_queue = 1;
        continue;
      }

      // 2. FULL PID Control (with D term)
      uint32_t now = millis();
      float dt = (now - pid_last_time[i]) / 1000.0f;  // Convert to seconds
      if (dt <= 0) dt = 0.05f;                        // Minimum dt to prevent division issues

      float err = config.target_temps[i] - current_temps[i];

      // P term
      float P = Kp * err;

      // I term with anti-windup
      pid_integral[i] += err * dt;
      pid_integral[i] = constrain(pid_integral[i], -500.0f, 500.0f);
      float I = Ki * pid_integral[i];

      // D term (derivative of error)
      float derivative = (err - pid_prev_error[i]) / dt;
      float D = Kd * derivative;

      // Store for next iteration
      pid_prev_error[i] = err;
      pid_last_time[i] = now;

      // Calculate total output
      float u = P + I + D;

      // Clamp output to valid range
      u = constrain(u, 0.0f, 100.0f);

      tpo_set_percent(i, u);

      // 3. Ready / Stability Logic
      if (abs(err) <= 2.0f) {
        if (stable_start_time[i] == 0) stable_start_time[i] = millis();
        if (millis() - stable_start_time[i] > 10000) {
          if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
            sysState.heater_ready[i] = true;
            xSemaphoreGive(dataMutex);
          }
          if (!beep_triggered[i]) {
            beep_mode = BEEP_MODE_READY;
            beep_queue = 4;
            beep_triggered[i] = true;
          }
        }
      } else {
        stable_start_time[i] = 0;
        beep_triggered[i] = false;
        if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
          sysState.heater_ready[i] = false;
          xSemaphoreGive(dataMutex);
        }
      }
    }  // End for loop

    freq_ctl_cnt++;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 4. INPUT TASK (Medium Priority)
void TaskInput(void* pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(20);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Vars for Long Press Logic
  static int lastBtn = HIGH;
  static uint32_t press_start_time = 0;
  static bool ignore_release = false;  // Set true if long press happened
  const uint32_t LONG_PRESS_MS = 1000;

  for (;;) {
    // 1. Encoder Rotation
    float delta = 0;
    read_hardware_encoder(&delta);
    if (delta != 0) {
      if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        ui.handleEncoderRotation(delta, config);
        xSemaphoreGive(dataMutex);
      }
    }

    // 2. Encoder Switch (Long Press = Back, Short Click = Select)
    int curBtn = digitalRead(ENCODER_SW);

    // --- Button State Machine ---
    if (lastBtn == HIGH && curBtn == LOW) {
      // PRESS STARTED
      press_start_time = millis();
      ignore_release = false;
    } else if (lastBtn == LOW && curBtn == LOW) {
      // HOLDING
      if (!ignore_release && (millis() - press_start_time > LONG_PRESS_MS)) {
        // >> LONG PRESS DETECTED (Trigger Back/Return)
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
          if (ui.getScreen() == SCREEN_STANDBY) {
            ui.enterQuickEdit();  // Standby -> Quick Edit
          } else {
            ui.handleButtonDoubleClick(config);  // Others -> Back
          }
          xSemaphoreGive(dataMutex);
        }
        beep_queue += 2;        // Beep
        ignore_release = true;  // Flag to ignore the release action
      }
    } else if (lastBtn == LOW && curBtn == HIGH) {
      // RELEASED
      if (!ignore_release) {
        // >> SHORT CLICK DETECTED (Trigger Select)
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
          ui.handleButtonSingleClick(config, go_to, has_go_to);
          xSemaphoreGive(dataMutex);
        }
        beep_queue += 2;  // Beep
      }
    }
    lastBtn = curBtn;

    // 3. PCF8574 (External Buttons)
    Wire.requestFrom(PCF_ADDR, 1);
    if (Wire.available()) {
      uint8_t input_data = Wire.read();
      bool btn[4] = { !(input_data & 1), !(input_data & 2), !(input_data & 4), !(input_data & 8) };

      for (int i = 0; i < 4; i++) {
        if (btn[i] && !pcf_state.btn_pressed[i]) {
          beep_queue += 2;

          if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {

            if (i == 0) {
              // Button 1: Toggle Settings
              if (ui.getScreen() == SCREEN_STANDBY) {
                ui.openSettings();  // Open
              } else {
                ui.exitSettings();
              }
            } else if (i == 1) {
              if (ui.getScreen() == SCREEN_STANDBY) {  // <-- ADD THIS CHECK
                has_go_to = !has_go_to;
              }
            }

            saveConfig(config);
            xSemaphoreGive(dataMutex);
          }
        }
        pcf_state.btn_pressed[i] = btn[i];
      }

      // Update LEDs
      uint8_t led_out = 0xF0;
      if (config.heater_active[0]) led_out &= ~(1 << 4);
      if (config.heater_active[1]) led_out &= ~(1 << 5);
      if (config.heater_active[2]) led_out &= ~(1 << 6);

      // [FIXED] ใช้ตัวแปร has_go_to
      if (has_go_to) led_out &= ~(1 << 7);  // LED สถานะ Run

      Wire.beginTransmission(PCF_ADDR);
      Wire.write(led_out | 0x0F);
      Wire.endTransmission();
    }

    freq_input_cnt++;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 5. DISPLAY TASK (Low Priority)
void TaskDisplay(void* pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(100);  // Target 100ms (10Hz)
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    AppState st;
    bool alarm_active = false;

    // 1. Prepare Data (Fast)
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      // Check inactivity
      if (ui.checkInactivity(config, has_go_to, go_to)) {
        beep_mode = 1;
        beep_queue = 6;
      }

      // Copy State
      for (int i = 0; i < 3; i++) {
        st.tc_temps[i] = sysState.tc_temps[i];
        st.tc_faults[i] = sysState.tc_faults[i];
        st.heater_cutoff_state[i] = sysState.heater_cutoff[i];
        st.heater_ready[i] = sysState.heater_ready[i];
      }
      st.tc_probe_temp = sysState.tc_probe_temp;
      st.ir_temps[0] = sysState.ir_temps[0];
      st.ir_temps[1] = sysState.ir_temps[1];
      st.ir_ambient[0] = sysState.ir_ambient[0];
      st.ir_ambient[1] = sysState.ir_ambient[1];

      if (st.ir_ambient[0] > 80.0f || st.ir_ambient[1] > 80.0f) {
        alarm_active = true;
      }
      xSemaphoreGive(dataMutex);
    }

    st.is_heating_active = has_go_to;
    st.target_temp = go_to;
    st.temp_unit = config.temp_unit;

    // 2. Draw to Screen (Slow - SPI Bus Heavy)
    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
      ui.draw(st, config);  // This takes ~50-60ms
      xSemaphoreGive(spiMutex);
    }

    if (alarm_active) {
      if (beep_queue == 0) beep_queue = 2;
    }

    freq_disp_cnt++;

    // FIX: Use vTaskDelayUntil to ensure fixed 10Hz regardless of draw time
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 6. DEBUG & LOGGING TASK (Runs once per second)
void TaskDebug(void* pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(1000);  // 1Hz
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
        xSemaphoreGive(dataMutex);
      }
      Serial.println("===============================");
      xSemaphoreGive(serialMutex);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void TaskSound(void* pvParameters) {
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  for (;;) {
    // If we have beeps in the queue
    if (beep_queue > 0) {

      int on_time = 60;
      int off_time = 100;

      // --- NEW LOGIC START ---
      if (beep_mode == 1) {
        // Sleep Mode (Slow pulsing)
        on_time = 250;
        off_time = 250;
      } else if (beep_mode == 3) {
        // ALARM Mode (Overheat) - Long 5 second beep
        on_time = 5000;
        off_time = 100;
      }
      // Mode 2 (Ready) uses default fast beeps, which is fine
      // --- NEW LOGIC END ---

      // Perform Beep
      if (config.sound_on) {
        digitalWrite(BUZZER, HIGH);
      }

      // Blocking delay is safe here (Task has its own stack)
      vTaskDelay(pdMS_TO_TICKS(on_time));

      digitalWrite(BUZZER, LOW);
      vTaskDelay(pdMS_TO_TICKS(off_time));

      // Decrease queue
      if (beep_queue >= 2) beep_queue -= 2;
      else beep_queue = 0;

      // Reset mode if done
      if (beep_queue == 0) beep_mode = 0;

    } else {
      // No beeps needed, sleep to save CPU
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

// ========== [Setup & Loop] ==========
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting...");

  // 1. Create Mutexes
  spiMutex = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();

  // 2. Hardware Init
  preferences.begin("app_config", false);
  loadConfig(config);

  if (config.max_temp_lock == 0.0f) {
    Serial.println("Config Invalid (Zeros). Loading Defaults...");

    config.max_temp_lock = 400.0f;  // Allow up to 400C by default
    config.temp_unit = 'C';
    config.idle_off_mode = IDLE_OFF_30_MIN;
    config.startup_mode = STARTUP_OFF;
    config.sound_on = true;
    config.ir_emissivity[0] = 1.0f;
    config.ir_emissivity[1] = 1.0f;
    config.tc_probe_offset = 0.0f;

    for (int i = 0; i < 3; i++) {
      config.target_temps[i] = 200.0f;  // Safe default
      config.max_temps[i] = 250.0f;     // Safe default
      config.tc_offsets[i] = 0.0f;
      config.heater_active[i] = false;
    }
    saveConfig(config);  // Save these defaults so they persist
  }


  if (config.startup_mode == STARTUP_OFF) {
    // Mode 1: Restore Selection, but stay STOPPED
    // (This matches standard behavior, no change needed)
    has_go_to = false;
  }

  if (config.startup_mode == STARTUP_AUTORUN) {
    bool any_active = false;
    for (int i = 0; i < 3; i++) {
      if (config.heater_active[i]) any_active = true;
    }
    if (any_active) {
      has_go_to = true;
    } else {
      has_go_to = false;
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
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  pinMode(ENCODER_SW, INPUT_PULLUP);

  // MAX31855 CS pins
  max31855_init_pins();

  setup_encoder_fixed();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  mlx1.begin(IR1_ADDR, &Wire);
  mlx2.begin(IR2_ADDR, &Wire);

  // TFT init - this sets up HSPI on pins 11, 12, 13
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  ui.begin();

  Serial.println("Hardware initialized. Starting tasks...");

  // 3. Start Tasks
  xTaskCreatePinnedToCore(TaskControl, "Control", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskMAX, "MAX31855", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskMLX, "MLX90614", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskInput, "Input", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskDisplay, "Display", 8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskDebug, "Debug", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskSound, "Sound", 2048, NULL, 1, NULL, 1);

  // 4. Start TPO Interrupt
  tpo_ticker.attach_ms(1, tpo_isr);

  Serial.println("FreeRTOS System Started.");
}

void loop() {
  vTaskDelete(NULL);
}

// ========== [Hardware Helper Functions] ==========
// 2. Helper Function: Software Emissivity Correction
// Stefan-Boltzmann Law: T_obj^4 = T_amb^4 + (T_sensor^4 - T_amb^4) / emissivity
float applyEmissivity(float t_obj_sensor, float t_amb, float emissivity) {
  if (isnan(t_obj_sensor) || isnan(t_amb)) return NAN;
  if (emissivity >= 0.99f) return t_obj_sensor;  // Optimization for 1.0

  // Convert to Kelvin
  float Tk_obj = t_obj_sensor + 273.15f;
  float Tk_amb = t_amb + 273.15f;

  float term1 = pow(Tk_obj, 4);
  float term2 = pow(Tk_amb, 4);

  // Correction
  float Tk_real_4 = term2 + ((term1 - term2) / emissivity);

  // Back to Celsius (Fourth root)
  float Tk_real = sqrt(sqrt(Tk_real_4));
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
}
