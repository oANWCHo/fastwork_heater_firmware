/****************************************************
 * Project: ESP32 + MAX31855 + MLX90614 + Rotary Encoder + ILI9341
 * Feature:
 * - แสดงค่า encoder, switch, MAX31855, MLX90614 x2 บนจอ ILI9341
 * - แสดงค่าเฉลี่ย MLX (avgC) ทาง Serial (รูปแบบ: "MAX31855,avgC")
 * - เพิ่ม setpoint (float) ปรับด้วย encoder
 * - กดปุ่ม encoder เพื่อยืนยัน setpoint เป็น go_to
 * - ทำ PID (P=1, I=0, D=0) เพื่อขับ PWM ไปที่ GPIO 0 ไล่ avgC -> go_to
 ****************************************************/

// ========== [1) Include Libraries] ==========
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include "Adafruit_MAX31855.h"
#include "Adafruit_MLX90614.h"
#include "driver/pcnt.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "driver/ledc.h"

// ========== [2) Pin Definitions] ==========
// --- MAX31855 Thermocouple ---
#define MAXDO     19
#define MAXCS     5
#define MAXCLK    18
// --- Relay Output (เดิม; ยังคงไว้แต่ไม่ได้ใช้ในโหมด PWM) ---
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

// --- PWM Output (LEDC - ESP-IDF API) ---
#define PWM_PIN        2       // GPIO 2 (โปรดตรวจสายบูต ถ้าไม่สะดวก เปลี่ยนขาได้)
#define PWM_FREQ       5000     // 5 kHz
#define PWM_RES_BITS   8        // 8-bit (duty 0..255)
#define LEDC_MODE      LEDC_LOW_SPEED_MODE
#define LEDC_TIMER     LEDC_TIMER_0
#define LEDC_CHANNEL   LEDC_CHANNEL_0

// ========== [3) Objects Initialization] ==========
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);
Adafruit_ILI9341  tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);
Adafruit_MLX90614 mlx1 = Adafruit_MLX90614();
Adafruit_MLX90614 mlx2 = Adafruit_MLX90614();

// ========== [4) Global Variables] ==========
// สำหรับ Serial monitor/ลอจิก
long lastEncoderValue = 0;
int  lastSwitchState  = HIGH;

// --- ค่าที่ "แสดงผลบนจอ" ---
float   max_temp_display = NAN;       // จาก MAX31855
float   ir1_temp_display = NAN;       // จาก MLX90614 #1
float   ir2_temp_display = NAN;       // จาก MLX90614 #2
float   avg_temp_display = NAN;       // เฉลี่ย MLX #1 และ #2
int16_t encoder_count_display = 0;
int     switch_state_display  = HIGH;

// --- Setpoint / Go-To Control ---
float setpoint     = 25.0f;           // ค่าเริ่มต้น setpoint
float go_to        = NAN;             // กดสวิตช์เพื่อยืนยัน setpoint -> go_to
bool  has_go_to    = false;

// การปรับ setpoint ด้วย encoder
const int   ENCODER_COUNTS_PER_STEP = 4;    // จำนวนพัลส์ต่อหนึ่ง “สเต็ป”
const float STEP_SIZE               = 0.5f; // ขนาดการเปลี่ยนต่อสเต็ป (องศา C)

// PID (เริ่มต้นตามโจทย์)
float Kp = 1.0f, Ki = 0.0f, Kd = 0.0f;
float pid_integral = 0.0f;
float pid_prev_err = 0.0f;
const float PID_OUT_MIN = 0.0f;
const float PID_OUT_MAX = (float)((1 << PWM_RES_BITS) - 1); // เช่น 255 เมื่อ 8-bit
const float INTEGRAL_CLAMP = PID_OUT_MAX; // ป้องกัน windup แบบง่าย

// สีส้มสำหรับตัวอักษรบนจอ
uint16_t COLOR_ORANGE = 0;

// ฟังก์ชันแสดงผลบนจอ (ประกาศล่วงหน้า)
void updateDisplay();

// ฟังก์ชันควบคุม (PID + PWM)
void updateControl();

// helper เขียนค่า PWM (ESP-IDF LEDC)
static inline void pwmWriteDuty(uint32_t duty) {
  uint32_t maxd = (1u << PWM_RES_BITS) - 1u;
  if (duty > maxd) duty = maxd;
  ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
  ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

// ========== [5) Setup] ==========
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(1); }

  pinMode(ENCODER_SW, INPUT_PULLUP);

  // ตั้งค่า PCNT สำหรับ encoder
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

  // PWM (LEDC) Setup - ESP-IDF style
  ledc_timer_config_t ledc_timer = {};
  ledc_timer.speed_mode       = LEDC_MODE;
  ledc_timer.timer_num        = LEDC_TIMER;
  ledc_timer.duty_resolution  = (ledc_timer_bit_t)PWM_RES_BITS; // 8-bit
  ledc_timer.freq_hz          = PWM_FREQ;                       // 5 kHz
  ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channel_cfg = {};
  ledc_channel_cfg.gpio_num   = PWM_PIN;
  ledc_channel_cfg.speed_mode = LEDC_MODE;
  ledc_channel_cfg.channel    = LEDC_CHANNEL;
  ledc_channel_cfg.intr_type  = LEDC_INTR_DISABLE;
  ledc_channel_cfg.timer_sel  = LEDC_TIMER;
  ledc_channel_cfg.duty       = 0;  // เริ่ม 0%
  ledc_channel_cfg.hpoint     = 0;
  ledc_channel_config(&ledc_channel_cfg);

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
  read_heater_input();     // เดิม: รับคำสั่ง ON/OFF ผ่าน Serial (ไม่เกี่ยว PWM)
  heater_read();           // อ่าน MAX31855
  read_mlx_sensors();      // อ่าน MLX90614 x2 และอัปเดต avg_temp_display
  read_hardware_encoder(); // อ่านค่า encoder และปรับ setpoint ตามพัลส์
  check_encoder_switch();  // กดยืนยัน setpoint -> go_to

  updateControl();         // ทำ PID + PWM ตาม go_to และ avgC
  updateDisplay();         // อัปเดตจอ

  delay(100);
}

// ========== [7) Sensor Reading Section] ==========
void heater_read() {
  double c = thermocouple.readCelsius();
  max_temp_display = c;
  if (!isnan(c)) {
      Serial.print(c);
      Serial.print(",");
  }
}

void read_mlx_sensors() {
  float tempC1 = mlx1.readObjectTempC();
  float tempC2 = mlx2.readObjectTempC();

  ir1_temp_display = tempC1;
  ir2_temp_display = tempC2;

  if (!isnan(tempC1) && !isnan(tempC2)) {
      float avgC = (tempC1 + tempC2) / 2.0f;
      avg_temp_display = avgC;
      Serial.println(avgC); // คู่นี้คือ "MAX31855, avgC"
  } else {
      avg_temp_display = NAN;
      Serial.println("nan"); // ให้รูปแบบบรรทัดยังครบ
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

  // แสดงบนจอ/Serial
  encoder_count_display = current_count;

  // คำนวณ delta เพื่อปรับ setpoint
  long delta = (long)current_count - lastEncoderValue;
  if (delta != 0) {
    // แปลงพัลส์ -> “สเต็ป”
    float steps = (float)delta / (float)ENCODER_COUNTS_PER_STEP;
    // ปรับ setpoint ตาม STEP_SIZE
    setpoint += steps * STEP_SIZE;

    // จำกัดขอบเขต setpoint (ถ้าต้องการ)
    if (setpoint < -1000.0f) setpoint = -1000.0f;
    if (setpoint >  1000.0f) setpoint =  1000.0f;

    lastEncoderValue = current_count;
  }
}

void check_encoder_switch() {
  int currentSwitchState = digitalRead(ENCODER_SW);
  switch_state_display = currentSwitchState;

  // กดเพื่อ "ยืนยัน" setpoint -> go_to
  if (lastSwitchState == HIGH && currentSwitchState == LOW) {
    go_to     = setpoint;
    has_go_to = true;
    // รีเซ็ตสถานะ PID สำหรับรอบใหม่
    pid_integral = 0.0f;
    pid_prev_err = 0.0f;

    Serial.println("Encoder Switch Pressed! Set go_to = setpoint");
    delay(50);
  }
  lastSwitchState = currentSwitchState;
}

// ========== [9) Control (PID + PWM) Section] ==========
void updateControl() {
  if (!has_go_to) {
    // ยังไม่ยืนยัน setpoint → ปล่อย PWM เป็น 0
    pwmWriteDuty(0);
    return;
  }
  if (isnan(avg_temp_display)) {
    // ไม่มีค่าเฉลี่ยอุณหภูมิ → ปล่อย PWM เป็น 0 เพื่อความปลอดภัย
    pwmWriteDuty(0);
    return;
  }

  // error = เป้าหมาย - ปัจจุบัน
  float error = go_to - avg_temp_display;

  // เลือก deadband เล็กน้อยกันสั่น (เช่น 0.1 C)
  const float DEADBAND = 0.1f;
  if (fabsf(error) < DEADBAND) {
    error = 0.0f;
  }

  // คำนวณ PID (ตั้งต้น I=D=0 ตามโจทย์; เก็บ integral/derivative เผื่อปรับภายหลัง)
  pid_integral += error;
  // anti-windup อย่างง่าย
  if (pid_integral > INTEGRAL_CLAMP)  pid_integral = INTEGRAL_CLAMP;
  if (pid_integral < -INTEGRAL_CLAMP) pid_integral = -INTEGRAL_CLAMP;

  float derivative = (error - pid_prev_err);
  pid_prev_err = error;

  float u = Kp * error + Ki * pid_integral + Kd * derivative;

  // ระบบให้ความร้อนทางเดียว → ไม่ใช้ค่าติดลบ
  if (u < 0.0f) u = 0.0f;

  // แปลงเป็น duty PWM
  if (u > PID_OUT_MAX) u = PID_OUT_MAX;
  uint32_t duty = (uint32_t)(u + 0.5f);

  pwmWriteDuty(duty);
}

// ========== [10) TFT Display Section] ==========
void updateDisplay() {
  static bool ui_inited = false;
  static int yLine[7], xPos; // 7 บรรทัด: SP, go_to, MAX, IR1, IR2, ENC, SW

  // --- ตัวแปรสำหรับจำค่าล่าสุดที่แสดงผลไปแล้ว ---
  static float   prev_sp     = NAN;
  static float   prev_go     = NAN;
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
    int yStart = 10; // เริ่มวาดจากขอบบน
    xPos = 10;       // เริ่มวาดจากขอบซ้าย

    for(int i = 0; i < 7; i++) {
        yLine[i] = yStart + i * (CHAR_H + GAP);
    }
    ui_inited = true;

    // วาดหัวข้อคงที่รอบแรก (ฝั่ง label) เพื่อกันฟลิกเกอร์
    tft.setCursor(xPos, yLine[0]); tft.print("SP: ");
    tft.setCursor(xPos, yLine[1]); tft.print("go_to: ");
    tft.setCursor(xPos, yLine[2]); tft.print("MAX: ");
    tft.setCursor(xPos, yLine[3]); tft.print("IR1: ");
    tft.setCursor(xPos, yLine[4]); tft.print("IR2: ");
    tft.setCursor(xPos, yLine[5]); tft.print("ENC: ");
    tft.setCursor(xPos, yLine[6]); tft.print("SW:  ");
  }

  // Helper lambda สำหรับวาดค่าหลัง label ให้ทับพื้นหลัง (ลด ghost text)
  auto printValueAt = [&](int lineY, const char* label, const char* valueFmt, float val) {
    int xVal = xPos + 80; // ตำแหน่งค่า หลัง label
    tft.fillRect(xVal, lineY, 120, 16 + 2, ILI9341_BLACK);
    tft.setCursor(xVal, lineY);
    char buf[24];
    snprintf(buf, sizeof(buf), valueFmt, val);
    tft.print(buf);
  };

  auto printTextAt = [&](int lineY, const char* label, const char* text) {
    int xVal = xPos + 80;
    tft.fillRect(xVal, lineY, 120, 16 + 2, ILI9341_BLACK);
    tft.setCursor(xVal, lineY);
    tft.print(text);
  };

  // SP (setpoint)
  if (setpoint != prev_sp) {
    prev_sp = setpoint;
    printValueAt(yLine[0], "SP: ", "%.1f C", prev_sp);
  }

  // go_to (แสดงเฉพาะเมื่อมีการยืนยันแล้ว)
  float go_disp = has_go_to ? go_to : NAN;
  if (go_disp != prev_go) {
    prev_go = go_disp;
    if (has_go_to) {
      printValueAt(yLine[1], "go_to: ", "%.1f C", prev_go);
    } else {
      printTextAt(yLine[1], "go_to: ", "---");
    }
  }

  // MAX31855
  if (!isnan(max_temp_display) && max_temp_display != prev_max) {
    prev_max = max_temp_display;
    printValueAt(yLine[2], "MAX: ", "%.1f C", prev_max);
  }

  // IR1
  if (!isnan(ir1_temp_display) && ir1_temp_display != prev_ir1) {
    prev_ir1 = ir1_temp_display;
    printValueAt(yLine[3], "IR1: ", "%.1f C", prev_ir1);
  }

  // IR2
  if (!isnan(ir2_temp_display) && ir2_temp_display != prev_ir2) {
    prev_ir2 = ir2_temp_display;
    printValueAt(yLine[4], "IR2: ", "%.1f C", prev_ir2);
  }

  // Encoder
  if (encoder_count_display != prev_count) {
    prev_count = encoder_count_display;
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "%-7d", prev_count);
    int xVal = xPos + 80;
    tft.fillRect(xVal, yLine[5], 120, 16 + 2, ILI9341_BLACK);
    tft.setCursor(xVal, yLine[5]);
    tft.print(buffer);
  }

  // Switch
  if (switch_state_display != prev_switch) {
    prev_switch = switch_state_display;
    if (prev_switch == LOW) {
      printTextAt(yLine[6], "SW: ", "PRESSED ");
    } else {
      printTextAt(yLine[6], "SW: ", "RELEASED");
    }
  }
}
