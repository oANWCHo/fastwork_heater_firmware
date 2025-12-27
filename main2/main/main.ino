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
#define MAXDO   48   // MISO
#define MAXCLK  47   // SCK
#define MAXCS1  15   // CS1
#define MAXCS2  16   // CS2
#define MAXCS3  17   // CS3
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
SemaphoreHandle_t spiMutex;  // Protects SPI Bus
SemaphoreHandle_t dataMutex; // Protects Shared Data
SemaphoreHandle_t serialMutex; // Protects Serial Monitor (prevent mixed text)

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
  bool btn_pressed[4] = {false, false, false, false};
  bool led_state[4] = {false, false, false, false};
  uint32_t last_press[4] = {0, 0, 0, 0};
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
// void TaskMAX(void *pvParameters) {
//   const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10Hz
//   TickType_t xLastWakeTime = xTaskGetTickCount();
  
//   // Get TFT's SPI instance (HSPI)
//   SPIClass& hspi = tft.getSPIinstance();

//   for (;;) {
//     float local_tc[3];
//     uint8_t local_fault[3];
    
//     if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
      
//       for (int i = 0; i < 3; i++) {
//         // Guard TFT
//         digitalWrite(TFT_CS, HIGH);
        
//         // Read MAX31855 using hardware SPI
//         hspi.beginTransaction(maxSettings);
//         digitalWrite(TC_CS_PINS[i], LOW);
//         delayMicroseconds(1);
        
//         uint32_t raw = 0;
//         raw |= ((uint32_t)hspi.transfer(0x00)) << 24;
//         raw |= ((uint32_t)hspi.transfer(0x00)) << 16;
//         raw |= ((uint32_t)hspi.transfer(0x00)) << 8;
//         raw |= ((uint32_t)hspi.transfer(0x00));
        
//         digitalWrite(TC_CS_PINS[i], HIGH);
//         hspi.endTransaction();
        
//         // Parse
//         if (raw == 0 || raw == 0xFFFFFFFF) {
//           local_tc[i] = NAN;
//           local_fault[i] = 0xFF; // Wiring error
//         } 
//         else if (raw & 0x10000) {
//           local_tc[i] = NAN;
//           local_fault[i] = (raw & 0x07);
//         }
//         else {
//           int32_t tc14 = (int32_t)((raw >> 18) & 0x3FFF);
//           if (raw & 0x20000000) tc14 -= 16384;
//           local_tc[i] = (float)tc14 * 0.25f;
//           local_fault[i] = 0;
//         }
//       }
      
//       xSemaphoreGive(spiMutex);
//     }

//     // Update shared state outside SPI mutex
//     if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
//       for(int i = 0; i < 3; i++) {
//         if(!isnan(local_tc[i])) {
//           sysState.tc_temps[i] = local_tc[i] + config.tc_offsets[i];
//         } else {
//           sysState.tc_temps[i] = NAN;
//         }
//         sysState.tc_faults[i] = local_fault[i];
//       }
//       xSemaphoreGive(dataMutex);
//     }
    
//     freq_max_cnt++;
//     vTaskDelayUntil(&xLastWakeTime, xFrequency);
//   }
// }

// void TaskMAX(void *pvParameters) {
//   const TickType_t xFrequency = pdMS_TO_TICKS(100);
//   TickType_t xLastWakeTime = xTaskGetTickCount();

//   for (;;) {
//     if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
      
//       // Setup bit-bang pins
//       digitalWrite(TFT_CS, HIGH);
//       pinMode(MAXCLK, OUTPUT);
//       digitalWrite(MAXCLK, LOW);  // CRITICAL: Start LOW
//       pinMode(MAXDO, INPUT);

//       for (int i = 0; i < 3; i++) {
//         uint32_t d = 0;
        
//         digitalWrite(TC_CS_PINS[i], LOW);
//         delayMicroseconds(2);
        
//         for (int j = 31; j >= 0; j--) {
//           digitalWrite(MAXCLK, HIGH);
//           delayMicroseconds(2);
//           d <<= 1;
//           if (digitalRead(MAXDO)) d |= 1;
//           digitalWrite(MAXCLK, LOW);
//           delayMicroseconds(2);
//         }
        
//         digitalWrite(TC_CS_PINS[i], HIGH);
//         delayMicroseconds(5);
        
//         // Parse
//         float temp = NAN;
//         if (d != 0 && d != 0xFFFFFFFF && !(d & 0x10000)) {
//           int32_t v = (d >> 18) & 0x3FFF;
//           if (d & 0x20000000) v -= 16384;
//           temp = v * 0.25f;
//         }
        
//         // Update state
//         if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
//           sysState.tc_temps[i] = isnan(temp) ? NAN : temp + config.tc_offsets[i];
//           sysState.tc_faults[i] = (d & 0x10000) ? (d & 0x07) : 0;
//           xSemaphoreGive(dataMutex);
//         }
//       }
      
//       xSemaphoreGive(spiMutex);
//     }
    
//     freq_max_cnt++;
//     vTaskDelayUntil(&xLastWakeTime, xFrequency);
//   }
// }

void TaskMAX(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // --- [Config Moving Average] ---
  const int MA_WINDOW = 10;     // จำนวนค่าที่จะนำมาเฉลี่ย (ยิ่งเยอะยิ่งนิ่ง แต่ตอบสนองช้า)
  static float ma_buffer[3][MA_WINDOW]; // Buffer เก็บค่า
  static int ma_idx[3] = {0, 0, 0};     // Index ปัจจุบัน
  static int ma_count[3] = {0, 0, 0};   // นับจำนวนข้อมูลที่มี (สำหรับการเริ่มต้น)

  // เคลียร์ค่าเริ่มต้นเป็น 0
  for(int i=0; i<3; i++) {
     for(int j=0; j<MA_WINDOW; j++) ma_buffer[i][j] = 0.0f;
  }

  for (;;) {
    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
      
      // Setup bit-bang pins
      digitalWrite(TFT_CS, HIGH);
      pinMode(MAXCLK, OUTPUT);
      digitalWrite(MAXCLK, LOW);
      pinMode(MAXDO, INPUT);

      for (int i = 0; i < 3; i++) {
        uint32_t d = 0;
        digitalWrite(TC_CS_PINS[i], LOW);
        delayMicroseconds(2);
        
        // Read 32 bits
        for (int j = 31; j >= 0; j--) {
          digitalWrite(MAXCLK, HIGH);
          delayMicroseconds(2);
          d <<= 1;
          if (digitalRead(MAXDO)) d |= 1;
          digitalWrite(MAXCLK, LOW);
          delayMicroseconds(2);
        }
        
        digitalWrite(TC_CS_PINS[i], HIGH);
        delayMicroseconds(5);

        // Parse Raw Temp
        float raw_temp = NAN;
        bool is_fault = false;

        if (d == 0 || d == 0xFFFFFFFF || (d & 0x10000)) {
           is_fault = true;
           raw_temp = NAN;
           // Reset MA counter เมื่อ Sensor หลุด เพื่อไม่ให้เอาค่าเก่ามาเฉลี่ย
           ma_count[i] = 0; 
           ma_idx[i] = 0;
        } else {
           int32_t v = (d >> 18) & 0x3FFF;
           if (d & 0x20000000) v -= 16384;
           raw_temp = v * 0.25f; // คูณ 0.25 ตามสูตรมาตรฐาน
        }
        
        // --- [Calculated Moving Average] ---
        float final_temp = NAN;

        if (!is_fault) {
           // 1. เก็บค่าลง Buffer
           ma_buffer[i][ma_idx[i]] = raw_temp;
           
           // 2. เลื่อน Index และนับจำนวน
           ma_idx[i] = (ma_idx[i] + 1) % MA_WINDOW;
           if (ma_count[i] < MA_WINDOW) ma_count[i]++;

           // 3. หาค่าเฉลี่ย
           float sum = 0;
           for(int k=0; k < ma_count[i]; k++) {
              sum += ma_buffer[i][k];
           }
           final_temp = sum / ma_count[i];

           // --- [Low Temp Logic (< 50)] ---
           if (final_temp < 25.0f) {
              
              // Action 1: ถ้าต่ำกว่า 0 ให้ปัดเป็น 0 (แก้ปัญหาค่าติดลบที่ผิดปกติ)
              // if (final_temp < 0.0f) final_temp = 0.0f;
              final_temp = 25.0f;

              // Action 2 (Optional): ถ้าต้องการ Offset พิเศษเฉพาะช่วงอุณหภูมิต่ำ
              // final_temp += 2.0f; // ตัวอย่าง: บวกเพิ่ม 2 องศาถ้ามันอ่านต่ำไป
           }
        }

        // Update state
        if (xSemaphoreTake(dataMutex, 10) == pdTRUE) {
          if (isnan(final_temp)) {
             sysState.tc_temps[i] = NAN;
          } else {
             sysState.tc_temps[i] = final_temp + config.tc_offsets[i]; // บวก Offset รวมอีกที
          }
          
          sysState.tc_faults[i] = (d & 0x10000) ? (d & 0x07) : 0;
          xSemaphoreGive(dataMutex);
        }
      }
      
      xSemaphoreGive(spiMutex);
    }
    
    freq_max_cnt++;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 2. MLX90614 TASK (I2C - Medium Priority)
void TaskMLX(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    // Read I2C
    float ir1 = mlx1.readObjectTempC();
    float ir2 = mlx2.readObjectTempC();

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
  const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    float current_temps[3];
    bool active_flags[3];

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      memcpy(current_temps, sysState.tc_temps, sizeof(current_temps));
      for(int i=0; i<3; i++) active_flags[i] = config.heater_active[i];
      xSemaphoreGive(dataMutex);
    }

    if (!has_go_to) {
      for (int i=0; i<3; i++) tpo_set_percent(i, 0);
    } else {
      for (int i = 0; i < 3; i++) {
        if (!active_flags[i] || isnan(current_temps[i])) { 
          tpo_set_percent(i, 0); 
          continue; 
        }

        if (current_temps[i] >= config.max_temps[i]) {
          tpo_set_percent(i, 0);
          if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
             config.heater_active[i] = false;
             sysState.heater_cutoff[i] = true;
             xSemaphoreGive(dataMutex);
          }
          continue;
        }

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
void TaskInput(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz Check
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // ตัวแปรสำหรับ Double Click Logic
  static uint32_t last_click_time = 0;
  static bool awaiting_double = false;
  const uint32_t DOUBLE_CLICK_MS = 400; // ระยะเวลาในการรอคลิกที่ 2

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

    // 2. Encoder Switch (Double Click Logic)
    static int lastBtn = HIGH;
    int curBtn = digitalRead(ENCODER_SW);
    
    // ตรวจจับการกด (Falling Edge)
    if (lastBtn == HIGH && curBtn == LOW) {
      if (awaiting_double) {
         // --- DOUBLE CLICK DETECTED ---
         if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            ui.handleButtonDoubleClick(config); // เรียกฟังก์ชันถอยกลับ
            xSemaphoreGive(dataMutex);
         }
         beep_queue += 2; // Beep
         awaiting_double = false; // Reset สถานะ
      } else {
         // --- FIRST CLICK DETECTED ---
         awaiting_double = true;
         last_click_time = millis();
         beep_queue += 2; // Beep ตอบสนองทันทีที่กดครั้งแรก
      }
    }
    lastBtn = curBtn;

    // ตรวจสอบ Timeout สำหรับ Single Click
    if (awaiting_double && (millis() - last_click_time > DOUBLE_CLICK_MS)) {
        // หมดเวลาแล้วไม่มีการกดซ้ำ -> ยืนยันว่าเป็น Single Click
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
           ui.handleButtonSingleClick(config, go_to, has_go_to);
           xSemaphoreGive(dataMutex);
        }
        awaiting_double = false;
    }

    // 3. PCF8574 (ปุ่มแยก)
    Wire.requestFrom(PCF_ADDR, 1);
    if (Wire.available()) {
       uint8_t input_data = Wire.read();
       bool btn[4] = { !(input_data & 1), !(input_data & 2), !(input_data & 4), !(input_data & 8) };
       for(int i=0; i<4; i++) {
         if(btn[i] && !pcf_state.btn_pressed[i]) {
           beep_queue += 2; // เพิ่มทีละ 2 เพื่อให้ Buzzer Logic แบบเก่าทำงานได้
           if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
             if(i < 3) config.heater_active[i] = !config.heater_active[i];
             else { 
               if(!has_go_to) ui.handleButtonSingleClick(config, go_to, has_go_to);
               else { has_go_to = false; go_to = NAN; }
             }
             saveConfig(config);
             xSemaphoreGive(dataMutex);
           }
         }
         pcf_state.btn_pressed[i] = btn[i];
       }
       
       // Update LEDs
       uint8_t led_out = 0xF0;
       if(config.heater_active[0]) led_out &= ~(1<<4);
       if(config.heater_active[1]) led_out &= ~(1<<5);
       if(config.heater_active[2]) led_out &= ~(1<<6);
       if(has_go_to)               led_out &= ~(1<<7);
       Wire.beginTransmission(PCF_ADDR);
       Wire.write(led_out | 0x0F);
       Wire.endTransmission();
    }
    
    freq_input_cnt++;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// 5. DISPLAY TASK (Low Priority)
void TaskDisplay(void *pvParameters) {
  for (;;) {
    AppState st;

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
       for(int i=0; i<3; i++) {
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
    vTaskDelay(pdMS_TO_TICKS(100)); // 10 FPS
  }
}

// 6. DEBUG & LOGGING TASK (Runs once per second)
void TaskDebug(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 1Hz
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

// ========== [Setup & Loop] ==========
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting...");

  // 1. Create Mutexes
  spiMutex    = xSemaphoreCreateMutex();
  dataMutex   = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();

  // 2. Hardware Init
  preferences.begin("app_config", false);
  loadConfig(config);

  pinMode(SSR_PIN1, OUTPUT); digitalWrite(SSR_PIN1, LOW);
  pinMode(SSR_PIN2, OUTPUT); digitalWrite(SSR_PIN2, LOW);
  pinMode(SSR_PIN3, OUTPUT); digitalWrite(SSR_PIN3, LOW);
  pinMode(BUZZER, OUTPUT);   digitalWrite(BUZZER, LOW);
  pinMode(TFT_CS, OUTPUT);   digitalWrite(TFT_CS, HIGH); 
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
  xTaskCreatePinnedToCore(TaskMAX,     "MAX31855",4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskMLX,     "MLX90614",4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskInput,   "Input",   4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskDisplay, "Display", 8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskDebug,   "Debug",   4096, NULL, 1, NULL, 1);

  // 4. Start TPO Interrupt
  tpo_ticker.attach_ms(1, tpo_isr);
  
  Serial.println("FreeRTOS System Started.");
}

void loop() {
  // --- Buzzer Logic (ยืมจาก Old Main) ---
  if (beep_queue > 0) {
    if (!beeper_active) {
       // เริ่มต้นทำงาน Beep
       beeper_active = true;
       t_next_beep_action = millis();
    }

    if (millis() >= t_next_beep_action) {
      // เช็คว่าเป็นจังหวะเปิด (คู่) หรือ ปิด (คี่)
      // ตัวอย่าง: queue=2 (ON) -> queue=1 (OFF) -> queue=0 (Done)
      bool is_on_cycle = (beep_queue % 2 == 0); 
      
      if (is_on_cycle) {
        if(config.sound_on) digitalWrite(BUZZER, HIGH);
        t_next_beep_action = millis() + BEEP_ON_MS;
      } else {
        digitalWrite(BUZZER, LOW);
        t_next_beep_action = millis() + BEEP_OFF_MS;
      }
      beep_queue--; // ลดคิวลง
    }
  } else {
    // คิวหมด ปิดถาวร
    if (beeper_active) {
       digitalWrite(BUZZER, LOW);
       beeper_active = false;
    }
  }

  // Code เดิม...
  vTaskDelay(pdMS_TO_TICKS(50));
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
