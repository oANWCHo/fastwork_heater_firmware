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

// ========== [Shared Resources] ==========
SemaphoreHandle_t spiMutex;     // Protects SPI Bus
SemaphoreHandle_t dataMutex;    // Protects Shared Data
SemaphoreHandle_t serialMutex;  // Protects Serial Monitor (prevent mixed text)

// Thread-Safe Data Model
struct SharedData {
  float tc_temps[3];
  float cj_temps[3];
  uint8_t tc_faults[3];
  float ir_temps[2];
  bool heater_cutoff[3];
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

// PID Vars
float Kp = 1.2f, Ki = 0.02f, Kd = 0.01f;
float pid_integral[3] = { 0.0f, 0.0f, 0.0f };

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
  const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 10Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // --- [Config Moving Average] ---
  const int MA_WINDOW = 10;
  static float ma_buffer[3][MA_WINDOW];
  static int ma_idx[3] = { 0, 0, 0 };
  static int ma_count[3] = { 0, 0, 0 };

  // --- [Watchdog / Stuck Data Tracking] ---
  static uint32_t prev_raw_data[3] = { 0, 0, 0 };
  static uint32_t last_change_time[3] = { 0, 0, 0 };
  const uint32_t STUCK_THRESHOLD_MS = 2000;  // 2 Seconds

  static uint32_t fault_start_time[3] = { 0, 0, 0 };
  const uint32_t FAULT_HOLD_MS = 500;

  // --- [Hardware SPI Setup] ---
  // Use VSPI (ID 3) because TFT is using HSPI
  SPIClass* vspi = new SPIClass(VSPI);

  // begin(SCLK, MISO, MOSI, SS)
  // MOSI and SS are set to -1 (unused)
  vspi->begin(MAXCLK, MAXDO, -1, -1);

  // Define SPI Settings: 1MHz, MSB First, Mode 0
  SPISettings maxSettings(1000000, MSBFIRST, SPI_MODE0);

  // Clear buffers
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < MA_WINDOW; j++) ma_buffer[i][j] = 0.0f;
    last_change_time[i] = millis();
  }

  for (;;) {
    // Lock SPI Mutex to prevent conflict if other tasks try to use VSPI (unlikely here, but safe)
    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {

      digitalWrite(TFT_CS, HIGH);  // Safety: Deselect TFT

      // Start Transaction
      vspi->beginTransaction(maxSettings);

      for (int i = 0; i < 3; i++) {
        uint32_t d = 0;

        digitalWrite(TC_CS_PINS[i], LOW);  // Select MAX31855

        // --- [HARDWARE SPI READ] ---
        // Transfer 32 bits (sends 0s, reads data)
        d = vspi->transfer32(0);

        digitalWrite(TC_CS_PINS[i], HIGH);  // Deselect MAX31855

        // --- [FAULT DETECTION LOGIC] ---
        float raw_temp = NAN;
        bool is_fault = false;

        // 1. SPI Bus Failure (All 0 or All 1)
        if (d == 0 || d == 0xFFFFFFFF) {
          is_fault = true;
        }
        // 2. Internal MAX31855 Fault Bit (D16)
        else if (d & 0x10000) {
          if (fault_start_time[i] == 0) {
            fault_start_time[i] = millis();  // เริ่มจับเวลา
          }

          // ถ้าเวลาผ่านไปนานกว่ากำหนด ให้ถือว่าเป็น Fault จริงๆ
          if (millis() - fault_start_time[i] > FAULT_HOLD_MS) {
            is_fault = true;
          }
        } else {
          // ถ้าสถานะปกติ (ไม่มี D16 Fault) ให้รีเซ็ตเวลา
          fault_start_time[i] = 0;
        }

        // --- [Watchdog: Check for Frozen Data] ---
        if (!is_fault) {
          if (d == prev_raw_data[i]) {
            // Bits are exactly the same. Check time.
            if (millis() - last_change_time[i] > STUCK_THRESHOLD_MS) {
              is_fault = true;  // FORCE FAULT
            }
          } else {
            // Data changed. Reset tracker.
            prev_raw_data[i] = d;
            last_change_time[i] = millis();
          }
        } else {
          // Already faulted, just update tracker
          prev_raw_data[i] = d;
          last_change_time[i] = millis();
        }

        // --- [Processing] ---
        if (is_fault) {
          raw_temp = NAN;
          ma_count[i] = 0;  // Reset average on fault
          ma_idx[i] = 0;
        } else {
          // Extract Temp (Bits 31-18)
          int32_t v = (d >> 18) & 0x3FFF;
          if (d & 0x20000000) v -= 16384;  // Negative handling
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
          if (isnan(final_temp)) {
            sysState.tc_temps[i] = NAN;
          } else {
            sysState.tc_temps[i] = final_temp + config.tc_offsets[i];
          }
          // Store fault code (masked for display if needed)
          sysState.tc_faults[i] = (is_fault) ? (d & 0x07) : 0;
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
    float ir1 = NAN;
    float ir2 = NAN;

    // Check IR1 Connection
    Wire.beginTransmission(IR1_ADDR);
    if (Wire.endTransmission() == 0) {
      ir1 = mlx1.readObjectTempC();
      // Filter out standard library error values if any (some return 1037.55 on error)
      if (ir1 > 1000) ir1 = NAN;
    }

    // Check IR2 Connection
    Wire.beginTransmission(IR2_ADDR);
    if (Wire.endTransmission() == 0) {
      ir2 = mlx2.readObjectTempC();
      if (ir2 > 1000) ir2 = NAN;
    }

    // Update Data
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      sysState.ir_temps[0] = ir1;
      sysState.ir_temps[1] = ir2;
      xSemaphoreGive(dataMutex);
    }

    freq_mlx_cnt++;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 3. CONTROL TASK (PID - High Priority)
void TaskControl(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(50);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    float current_temps[3];
    bool active_flags[3];
    bool cutoff_flags[3];

    // 1. Read all shared data safely
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      memcpy(current_temps, sysState.tc_temps, sizeof(current_temps));
      for(int i=0; i<3; i++) {
        active_flags[i] = config.heater_active[i];
        cutoff_flags[i] = sysState.heater_cutoff[i];
      }
      xSemaphoreGive(dataMutex);
    }

    for (int i = 0; i < 3; i++) {
        // Only try to clear if we have a valid temperature (not NAN)
        if (cutoff_flags[i] && !isnan(current_temps[i])) {
            // Check if temp has dropped below the setpoint
            if (current_temps[i] < config.target_temps[i]) {
                 if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
                     sysState.heater_cutoff[i] = false; // Stop Blinking
                     xSemaphoreGive(dataMutex);
                 }
            }
        }
    }

    if (!has_go_to) {
      // System is STOPPED
      for (int i=0; i<3; i++) tpo_set_percent(i, 0);
    } else {
      // System is RUNNING
      for (int i = 0; i < 3; i++) {
        

        if (!active_flags[i] || isnan(current_temps[i])) { 
          tpo_set_percent(i, 0);
          continue; 
        }

        // Overheat Logic
        if (current_temps[i] >= config.max_temps[i]) {
          tpo_set_percent(i, 0);
          if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
             config.heater_active[i] = false;
             sysState.heater_cutoff[i] = true; // Trigger red blinking
             xSemaphoreGive(dataMutex);
          }
          continue;
        }

        // PID Logic
        float err = config.target_temps[i] - current_temps[i];
        pid_integral[i] = constrain(pid_integral[i] + err, -500, 500);
        float u = (Kp * err) + (Ki * pid_integral[i]);
        tpo_set_percent(i, u);
      }
    }
    
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
          // Re-using the double click handler as it contains the "Back" logic
          ui.handleButtonDoubleClick(config);
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
            if (i < 3) config.heater_active[i] = !config.heater_active[i];
            else {
              if (!has_go_to) ui.handleButtonSingleClick(config, go_to, has_go_to);
              else {
                has_go_to = false;
                go_to = NAN;
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
      if (has_go_to) led_out &= ~(1 << 7);
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
  for (;;) {
    AppState st;

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {

      if (ui.checkInactivity(config, has_go_to, go_to)) {
          // Trigger 3 Beeps at 2Hz
          beep_mode = 1;  // Set Slow Mode
          beep_queue = 6; // 3 Beeps (3 ON + 3 OFF actions)
      }

      for (int i = 0; i < 3; i++) {
        st.tc_temps[i] = sysState.tc_temps[i];
        st.tc_faults[i] = sysState.tc_faults[i];
        st.heater_cutoff_state[i] = sysState.heater_cutoff[i];
      }
      st.ir_temps[0] = sysState.ir_temps[0];
      st.ir_temps[1] = sysState.ir_temps[1];
      xSemaphoreGive(dataMutex);
    }

    st.is_heating_active = has_go_to;
    st.target_temp = go_to;
    st.temp_unit = config.temp_unit;

    // Draw (MUST LOCK SPI)
    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
      ui.draw(st, config);
      xSemaphoreGive(spiMutex);
    }

    freq_disp_cnt++;
    vTaskDelay(pdMS_TO_TICKS(100));  // 10 FPS
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
        Serial.printf("  IR1: %.2f | IR2: %.2f\n", sysState.ir_temps[0], sysState.ir_temps[1]);
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
      
      // Determine timing based on mode
      int on_time, off_time;
      if (beep_mode == 1) { 
        // Sleep Mode (Slow: 2Hz => 250ms ON / 250ms OFF)
        on_time = 250;
        off_time = 250;
      } else {
        // Normal Mode (Fast Click)
        on_time = 60;   // BEEP_ON_MS
        off_time = 100; // BEEP_OFF_MS
      }

      // Perform Beep (Blocking is okay here because it's in its own task!)
      if (config.sound_on) {
          digitalWrite(BUZZER, HIGH);
      }
      vTaskDelay(pdMS_TO_TICKS(on_time));

      digitalWrite(BUZZER, LOW);
      vTaskDelay(pdMS_TO_TICKS(off_time));

      // Decrease queue (we just did 1 full beep cycle)
      // Note: In your old code "queue" was actions (ON+OFF), so 6 = 3 beeps.
      // Here, let's treat 1 queue = 1 full beep for simplicity?
      // OR keep your existing logic: 1 beep = 2 actions (ON then OFF).
      
      // Let's stick to your existing logic where queue = actions
      // But since we did ON *and* OFF above, we subtract 2.
      if (beep_queue >= 2) beep_queue -= 2;
      else beep_queue = 0;

      // Reset mode if done
      if (beep_queue == 0) beep_mode = 0;

    } else {
      // No beeps needed, sleep for a bit to save CPU
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
