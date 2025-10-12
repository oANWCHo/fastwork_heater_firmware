/****************************************************
 * Project: ESP32 + MAX31855 + MLX90614 + Rotary Encoder + ILI9341
 * Feature: แสดงค่าจาก encoder, switch, และ sensor ทั้งหมดบนจอ TFT
 * อ่านค่าอุณหภูมิจาก MAX31855 และ MLX90614 x2
 * แสดงผลอุณหภูมิจาก MLX90614 และค่าเฉลี่ยทาง Serial Monitor
 ****************************************************/

// ========== [1) Include Libraries] ==========
#include <SPI.h>
#include <Wire.h>
#include "Adafruit_MAX31855.h"
#include "Adafruit_MLX90614.h"
#include "driver/pcnt.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

// ========== [2) Pin Definitions] ==========
// --- MAX31855 Thermocouple ---
#define MAXDO     19
#define MAXCS     5
#define MAXCLK    18
// --- Relay Output ---
#define RELAY_PIN 15
// --- Rotary Encoder (ใช้ PCNT) ---
#define ENCODER_A 36
#define ENCODER_B 39
#define ENCODER_SW 34
// --- TFT ILI9341 ---
#define TFT_DC    27
#define TFT_CS    26
#define TFT_MOSI  13
#define TFT_MISO  12
#define TFT_CLK   14
#define TFT_RST   4
// --- MLX90614 I2C Addresses ---
#define IR1_ADDR 0x10
#define IR2_ADDR 0x11

// ========== [3) Objects Initialization] ==========
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);
Adafruit_ILI9341  tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);
Adafruit_MLX90614 mlx1 = Adafruit_MLX90614();
Adafruit_MLX90614 mlx2 = Adafruit_MLX90614();

// ========== [4) Global Variables] ==========
// สำหรับ Serial monitor/ลอจิก
long lastEncoderValue = 0;
int  lastSwitchState  = HIGH;

// --- สำหรับค่าที่จะ "แสดงผลบนจอ" --- // <-- CHANGED
float   max_temp_display = NAN;       // สำหรับ MAX31855
float   ir1_temp_display = NAN;       // สำหรับ MLX90614 #1
float   ir2_temp_display = NAN;       // สำหรับ MLX90614 #2
int16_t encoder_count_display = 0;
int     switch_state_display  = HIGH;

// สีส้มสำหรับตัวอักษรบนจอ
uint16_t COLOR_ORANGE = 0;

// ฟังก์ชันแสดงผลบนจอ (ประกาศล่วงหน้า)
void updateDisplay();

// ========== [5) Setup] ==========
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(1); }

  pinMode(ENCODER_SW, INPUT_PULLUP);

  pcnt_config_t pcnt_config = {};
  pcnt_config.pulse_gpio_num = ENCODER_A;
  pcnt_config.ctrl_gpio_num  = ENCODER_B;
  pcnt_config.channel        = PCNT_CHANNEL_0;
  pcnt_config.unit           = PCNT_UNIT_0;
  pcnt_config.pos_mode       = PCNT_COUNT_DEC;
  pcnt_config.neg_mode       = PCNT_COUNT_INC;
  pcnt_config.lctrl_mode     = PCNT_MODE_REVERSE;
  pcnt_config.hctrl_mode     = PCNT_MODE_KEEP;
  pcnt_unit_config(&pcnt_config);
  pcnt_set_filter_value(PCNT_UNIT_0, 1000);
  pcnt_filter_enable(PCNT_UNIT_0);
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  Wire.begin();
  Serial.println("Initializing MLX90614 sensors...");
  if (!mlx1.begin(IR1_ADDR)) { Serial.println("Error connecting to MLX sensor #1"); while (1); };
  if (!mlx2.begin(IR2_ADDR)) { Serial.println("Error connecting to MLX sensor #2"); while (1); };
  Serial.println("MLX sensors connected!");

  delay(500);
  Serial.print("Initializing MAX31855 sensor...");
  if (!thermocouple.begin()) { Serial.println("ERROR."); while (1) delay(10); }
  Serial.println("DONE.");

  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(ILI9341_BLACK);
  COLOR_ORANGE = tft.color565(255, 165, 0);
  updateDisplay(); // วาดครั้งแรก
}

// ========== [6) Main Loop] ==========
void loop() {
  read_heater_input();
  heater_read();
  read_mlx_sensors();
  read_hardware_encoder();
  check_encoder_switch();

  updateDisplay(); // <-- เรียกฟังก์ชันอัปเดตจอในทุกๆ loop
  delay(100);
}

// ========== [7) Sensor Reading Section] ==========
void heater_read() {
  double c = thermocouple.readCelsius();
  max_temp_display = c; // <-- ADDED: อัปเดตค่าสำหรับแสดงผลบนจอ
  if (!isnan(c)) {
      Serial.print("MAX31855 Temp: ");
      Serial.print(c);
      Serial.println(" *C");
  }
}

void read_mlx_sensors() {
  float tempC1 = mlx1.readObjectTempC();
  float tempC2 = mlx2.readObjectTempC();
  
  ir1_temp_display = tempC1; // <-- ADDED: อัปเดตค่าสำหรับแสดงผลบนจอ
  ir2_temp_display = tempC2; // <-- ADDED: อัปเดตค่าสำหรับแสดงผลบนจอ

  if (!isnan(tempC1)) {
    Serial.print("MLX IR1 (0x10): Object = "); Serial.print(tempC1); Serial.println(" *C");
  }
  if (!isnan(tempC2)) {
    Serial.print("MLX IR2 (0x11): Object = "); Serial.print(tempC2); Serial.println(" *C");
  }
  if (!isnan(tempC1) && !isnan(tempC2)) {
      float avgC = (tempC1 + tempC2) / 2.0;
      Serial.print("*********** Average MLX Temp: ");
      Serial.print(avgC);
      Serial.print(" *C ***********\n\n");
  }
}

void read_heater_input() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.equalsIgnoreCase("ON")) {
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println("Relay turned ON");
    } else if (command.equalsIgnoreCase("OFF")) {
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("Relay turned OFF");
    }
  }
}

// ========== [8) Encoder (PCNT) Section] ==========
void read_hardware_encoder() {
  int16_t current_count = 0;
  pcnt_get_counter_value(PCNT_UNIT_0, &current_count);
  encoder_count_display = current_count;
  if (current_count != lastEncoderValue) {
    Serial.print("Encoder Value: ");
    Serial.println(current_count);
    lastEncoderValue = current_count;
  }
}

void check_encoder_switch() {
  int currentSwitchState = digitalRead(ENCODER_SW);
  switch_state_display = currentSwitchState;
  if (lastSwitchState == HIGH && currentSwitchState == LOW) {
    Serial.println("Encoder Switch Pressed!");
    delay(50);
  }
  lastSwitchState = currentSwitchState;
}

// ========== [9) TFT Display Section] ========== // <-- REWORKED
void updateDisplay() {
  static bool ui_inited = false;
  static int yLine[5], xPos; // Array to hold Y positions of 5 lines, and one X position for left alignment

  // --- ตัวแปรสำหรับจำค่าล่าสุดที่แสดงผลไปแล้ว ---
  static float   prev_max    = NAN;
  static float   prev_ir1    = NAN;
  static float   prev_ir2    = NAN;
  static int16_t prev_count  = INT16_MIN;
  static int     prev_switch = -1;

  // --- ตั้งค่าฟอนต์/ระยะห่าง ---
  const uint8_t TXT_SIZE = 2;
  const int     GAP      = 8; // ระยะห่างระหว่างบรรทัด

  // --- คำนวณตำแหน่งและตั้งค่า UI แค่ครั้งเดียว ---
  if (!ui_inited) {
    tft.setTextWrap(false);
    tft.setTextSize(TXT_SIZE);
    tft.setTextColor(COLOR_ORANGE, ILI9341_BLACK); // วาดทับพร้อมพื้นหลังดำ

    const int CHAR_H = 8 * TXT_SIZE; // ความสูงของตัวอักษร
    int yStart = 15; // เริ่มวาดจากขอบบน ห่างลงมา 15 pixels
    xPos = 10;       // เริ่มวาดจากขอบซ้าย ห่างเข้ามา 10 pixels

    for(int i = 0; i < 5; i++) {
        yLine[i] = yStart + i * (CHAR_H + GAP);
    }
    ui_inited = true;
  }
  
  // --- ตรวจสอบว่ามีค่าใดๆ เปลี่ยนแปลงหรือไม่ ---
  bool maxChanged = (max_temp_display != prev_max && !isnan(max_temp_display));
  bool ir1Changed = (ir1_temp_display != prev_ir1 && !isnan(ir1_temp_display));
  bool ir2Changed = (ir2_temp_display != prev_ir2 && !isnan(ir2_temp_display));
  bool encChanged = (encoder_count_display != prev_count);
  bool swChanged  = (switch_state_display != prev_switch);
  
  if (maxChanged || ir1Changed || ir2Changed || encChanged || swChanged) {

    // --- อัปเดตบรรทัด MAX31855 ---
    if (maxChanged) {
      prev_max = max_temp_display;
      char buffer[20];
      snprintf(buffer, sizeof(buffer), "MAX: %.1f C ", prev_max); // จัดรูปแบบทศนิยม 1 ตำแหน่ง
      tft.setCursor(xPos, yLine[0]);
      tft.print(buffer);
    }

    // --- อัปเดตบรรทัด MLX90614 #1 ---
    if (ir1Changed) {
      prev_ir1 = ir1_temp_display;
      char buffer[20];
      snprintf(buffer, sizeof(buffer), "IR1: %.1f C ", prev_ir1);
      tft.setCursor(xPos, yLine[1]);
      tft.print(buffer);
    }
    
    // --- อัปเดตบรรทัด MLX90614 #2 ---
    if (ir2Changed) {
      prev_ir2 = ir2_temp_display;
      char buffer[20];
      snprintf(buffer, sizeof(buffer), "IR2: %.1f C ", prev_ir2);
      tft.setCursor(xPos, yLine[2]);
      tft.print(buffer);
    }

    // --- อัปเดตบรรทัด Encoder ---
    if (encChanged) {
      prev_count = encoder_count_display;
      char buffer[20];
      snprintf(buffer, sizeof(buffer), "ENC: %-7d", prev_count); // %-7d คือจัดชิดซ้าย กว้าง 7 ตัวอักษร
      tft.setCursor(xPos, yLine[3]);
      tft.print(buffer);
    }
    
    // --- อัปเดตบรรทัด Switch ---
    if (swChanged) {
      prev_switch = switch_state_display;
      char buffer[20];
      if (prev_switch == LOW) {
        strcpy(buffer, "SW: PRESSED ");
      } else {
        strcpy(buffer, "SW: RELEASED");
      }
      tft.setCursor(xPos, yLine[4]);
      tft.print(buffer);
    }
  }
}