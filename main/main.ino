/****************************************************
 * ESP32 + MAX31855 x2 + MLX90614 x2 + Encoder + ILI9341
 * Heater control via SSR using Time-Proportioning (Burst Fire)
 *
 * [MODIFIED]:
 * - Integrated new UI State Machine from OvenUI.h
 * - Decoupled Input, Control, and Display logic.
 * - Added Single/Double Click detection.
 ****************************************************/

// ========== [1) Include Libraries] ==========
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include "Adafruit_MLX90614.h"
#include "driver/pcnt.h"
#include <TFT_eSPI.h>
#include <Ticker.h>

// --- Our New UI Library ---
#include "OvenData.h"  // ต้องมาก่อน
#include "OvenUI.h"    // นี่คือไฟล์ UI ที่เราสร้าง

// ========== [2) Pin Definitions] ==========
// ... (เหมือนเดิมทุกประการ) ... [cite: 74]
#define MAXDO 19   // MISO (Shared)
#define MAXCLK 18  // SCK (Shared)
#define MAXCS1 5   // CS for sensor 1
#define MAXCS2 33  // CS for sensor 2

#define ENCODER_A 36
#define ENCODER_B 39
#define ENCODER_SW 34

#define IR1_ADDR 0x10
#define IR2_ADDR 0x11
#define SSR_PIN 25
#define TPO_TICK_US 1000
#define WINDOW_MS_INIT 1000
#ifndef MAX_GUARD_US
#define MAX_GUARD_US 50
#define ENCODER_COUNTS_PER_STEP 4
#endif

// ========== [3) Objects & Global Data] ==========

static const int TC_CS_PINS[] = { MAXCS1, MAXCS2 };
const int NUM_THERMOCOUPLES = sizeof(TC_CS_PINS) / sizeof(TC_CS_PINS[0]);  // [cite: 76]

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);  // [cite: 76]

Adafruit_MLX90614 mlx1 = Adafruit_MLX90614();
Adafruit_MLX90614 mlx2 = Adafruit_MLX90614();  // [cite: 77]
Ticker tpo_ticker;

// --- Global Data Structs ---
OvenSettings g_settings;  // เก็บค่าที่ตั้งค่าไว้ทั้งหมด
SensorData g_sensors;     // เก็บค่าเซ็นเซอร์ที่อ่านได้

// --- PID & Control Vars ---
// (ย้าย Kp, Ki, Kd และตัวแปร PID มาไว้ตรงนี้)
float Kp = 1.2f, Ki = 0.02f, Kd = 0.01f;  // [cite: 83]
float pid_integral = 0.0f;
float pid_prev_err = 0.0f;
float last_pwm_percent = 0.0f;  // [cite: 84]

// ========== [4) Input Handling (New)] ==========
long lastEncoderValue = 0;

// ฟังก์ชันอ่านค่า Encoder ที่เปลี่ยนไป (Delta)
int get_encoder_delta() {
  int16_t current = 0;
  pcnt_get_counter_value(PCNT_UNIT_0, &current);  // [cite: 135]
  long delta = (long)current - lastEncoderValue;

  if (delta != 0) {
    lastEncoderValue = current;
    return delta;
  }
  return 0;
}

// ฟังก์ชันตรวจจับ Single/Double Click (แบบง่าย)
ClickType get_click_event() {
  static uint32_t last_press_time = 0;
  static uint32_t last_click_time = 0;
  static int click_count = 0;
  static int last_state = HIGH;

  const int DEBOUNCE_MS = 50;
  const int DOUBLE_CLICK_MS = 400;

  ClickType event = CLICK_NONE;
  int current_state = digitalRead(ENCODER_SW);  //

  if (current_state == LOW && last_state == HIGH) {  // เพิ่งกด
    if (millis() - last_press_time > DEBOUNCE_MS) {
      last_press_time = millis();
    }
  } else if (current_state == HIGH && last_state == LOW) {  // เพิ่งปล่อย
    if (millis() - last_press_time > DEBOUNCE_MS) {
      click_count++;
      last_click_time = millis();
    }
  }

  // ตรวจสอบว่าหมดเวลา Double Click หรือยัง
  if (click_count > 0 && (millis() - last_click_time > DOUBLE_CLICK_MS)) {
    if (click_count == 1) {
      event = CLICK_SINGLE;
    } else {
      event = CLICK_DOUBLE;
    }
    click_count = 0;
  }

  last_state = current_state;
  return event;
}


// ========== [5) TPO via Hardware Timer] ==========
// ... (เหมือนเดิมทุกประการ) ... [cite: 85-94]
portMUX_TYPE tpoMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t tpo_window_period_ticks = WINDOW_MS_INIT;
volatile uint32_t tpo_on_ticks = 0;
volatile uint32_t tpo_tick_counter = 0;

void tpo_set_percent(float percent) {
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  last_pwm_percent = percent;  // [cite: 89]
  uint32_t onTicks = (uint32_t)((percent / 100.0f) * (float)tpo_window_period_ticks + 0.5f);
  portENTER_CRITICAL(&tpoMux);
  tpo_on_ticks = onTicks;
  portEXIT_CRITICAL(&tpoMux);
}
void tpo_set_window_ms(uint32_t window_ms) {
  if (window_ms < 100)  window_ms = 100;
  if (window_ms > 5000) window_ms = 5000;
  portENTER_CRITICAL(&tpoMux);
  tpo_window_period_ticks = window_ms;
  if (tpo_tick_counter >= tpo_window_period_ticks) tpo_tick_counter = 0;
  tpo_on_ticks = (uint32_t)((last_pwm_percent / 100.0f) * (float)window_ms + 0.5f);
  portEXIT_CRITICAL(&tpoMux);
}
void IRAM_ATTR tpo_isr() {
  uint32_t pos;
  uint32_t onTicks;

  portENTER_CRITICAL_ISR(&tpoMux);
  pos = tpo_tick_counter;
  onTicks = tpo_on_ticks;
  tpo_tick_counter++;
  if (tpo_tick_counter >= tpo_window_period_ticks) {
    tpo_tick_counter = 0;
  }
  portEXIT_CRITICAL_ISR(&tpoMux);

  bool on = (pos < onTicks);
  digitalWrite(SSR_PIN, on ? HIGH : LOW);
}


// ========== [6) Forward Declarations] ==========
void updateControl(OvenSettings& settings);  // (ปรับปรุง)
void read_heater_input();
void heater_read();       // (ปรับปรุง)
void read_mlx_sensors();  // (ปรับปรุง)

// ... (Manual MAX31855 driver ... เหมือนเดิมทุกประการ) ... [cite: 95-97]
void max31855_init_pins();
static inline void all_cs_idle();
uint32_t max31855_read_raw(int csPin);
bool max31855_decode(uint32_t raw, float* tc_c, float* tj_c, uint8_t* faults);
bool max31855_read_celsius(int csPin, float* tc_c, float* tj_c, uint8_t* faults, uint32_t* raw_out);
static inline int32_t sign_extend(int32_t v, uint8_t nbits);

// ========== [7) Setup] ==========
void setup() {
  Serial.begin(115200);
  while (!Serial) { /* no delay */
  }

  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);  // [cite: 99]
  pinMode(ENCODER_SW, INPUT_PULLUP);

  // PCNT (encoder)
  pcnt_config_t pcnt_config = {};
  pcnt_config.pulse_gpio_num = ENCODER_A;
  pcnt_config.ctrl_gpio_num = ENCODER_B;
  pcnt_config.channel = PCNT_CHANNEL_0;   // [cite: 100]
  pcnt_config.unit = PCNT_UNIT_0;         // [cite: 101]
  pcnt_config.pos_mode = PCNT_COUNT_DEC;  // [cite: 102]
  pcnt_config.neg_mode = PCNT_COUNT_INC;
  pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;  // [cite: 103]
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_unit_config(&pcnt_config);
  // pcnt_set_filter_value(PCNT_UNIT_0, 100);
  // pcnt_filter_enable(PCNT_UNIT_0);
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);  // [cite: 104]

  Wire.begin();
  if (!mlx1.begin(IR1_ADDR, &Wire)) {
    Serial.println("Error connecting to MLX #1 (Addr 0x10)");
    while (1) {}  // [cite: 105]
  }
  if (!mlx2.begin(IR2_ADDR, &Wire)) {
    Serial.println("Error connecting to MLX #2 (Addr 0x11)");
    while (1) {}  // [cite: 106]
  }

  max31855_init_pins();  // [cite: 106]
  Serial.println("Using manual MAX31855 driver.");

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  spr.createSprite(tft.width(), tft.height());  // [cite: 108]

  // TPO setup
  tpo_set_window_ms(WINDOW_MS_INIT);
  tpo_set_percent(0.0f);
  tpo_ticker.attach_ms(1, tpo_isr);  // [cite: 109]
  Serial.println("Ready. UI State Machine active.");
}

// ========== [8) Main Loop: Refactored] ==========
void loop() {
  // 1. อ่าน Serial (สำหรับ Debug/Override)
  read_heater_input();

  // 2. อ่าน Input จาก Encoder และปุ่ม
  int encoder_delta = get_encoder_delta();
  ClickType click_event = get_click_event();

  // 3. อัปเดต UI State (ส่ง Input เข้าไป)
  ui_handle_input(encoder_delta, click_event, g_settings);

  // 4. อ่านเซ็นเซอร์ (ตามรอบเวลา)
  static uint32_t t_sens = 0;
  uint32_t now = millis();
  if (now - t_sens >= 100) {  // [cite: 112]
    t_sens = now;
    heater_read();       // อัปเดต g_sensors
    read_mlx_sensors();  // อัปเดต g_sensors
  }

  // 5. อัปเดต Control Loop (ตามรอบเวลา)
  static uint32_t t_ctl = 0;
  if (now - t_ctl >= 50) {  // [cite: 113]
    t_ctl = now;
    updateControl(g_settings);  // ส่ง g_settings เข้าไป
  }

  // 6. อัปเดตหน้าจอ (ตามรอบเวลา)
  static uint32_t t_disp = 0;
  if (now - t_disp >= 50) {  // [cite: 114]
    t_disp = now;
    // เรียกฟังก์ชันวาดจอหลักจาก OvenUI.h
    ui_draw(spr, g_sensors, g_settings);
  }

  // 7. ส่งค่าไป Serial Plotter (ตามรอบเวลา)
  static uint32_t t_plot = 0;
  if (now - t_plot >= 200) {  // [cite: 115]
    t_plot = now;
    printForSerialPlotter();
  }
}

// ========== [9) Sensor Reading (Modified)] ==========

void heater_read() {
  // ... (โค้ดอ่าน MAX31855 เหมือนเดิม) ... [cite: 116-119]
  for (int i = 0; i < NUM_THERMOCOUPLES; ++i) {
    float tc, tj;
    uint8_t fb;
    uint32_t raw;
    bool ok = false;
    for (int attempt = 0; attempt < 2 && !ok; ++attempt) {
      ok = max31855_read_celsius(TC_CS_PINS[i], &tc, &tj, &fb, &raw);
      if (!ok) delayMicroseconds(MAX_GUARD_US);
    }

    // [MODIFIED] อัปเดตค่าไปยัง Global Struct
    if (i == 0) {
      g_sensors.TC1 = ok ? tc : NAN;  // [cite: 119-121]
      g_sensors.TC1_CJ = ok ? tj : NAN;
    } else if (i == 1) {
      g_sensors.TC2 = ok ? tc : NAN;
      g_sensors.TC2_CJ = ok ? tj : NAN;
    }
  }
}

void read_mlx_sensors() {
  float t1 = mlx1.readObjectTempC();
  float t2 = mlx2.readObjectTempC();
  // [MODIFIED] อัปเดตค่าไปยัง Global Struct
  if (!isnan(t1) && !isnan(t2)) g_sensors.IR_Avg = (t1 + t2) / 2.0f;  //
  else g_sensors.IR_Avg = NAN;
}

// ========== [10) Serial Commands] ==========
// ... (เหมือนเดิมทุกประการ) ... [cite: 123-134]
// (หมายเหตุ: Serial commands จะ override UI)
bool pwm_override_enabled = false;
float pwm_override_percent = 0.0f;
void read_heater_input() {
if (Serial.available() <= 0) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  String up = cmd; up.toUpperCase();

  if (up == "AUTO") {
    pwm_override_enabled = false;
    Serial.println("MODE=AUTO(PID)");
    return;
  }

  if (up == "PWM?" || up == "STATUS?" || up == "DUTY?") {
    Serial.print("MODE: ");
    Serial.println(pwm_override_enabled ? "OVERRIDE" : "AUTO(PID)");
    Serial.print("PWM%: "); Serial.println(last_pwm_percent, 2);
    Serial.print("WINDOW_MS: ");
    uint32_t w; portENTER_CRITICAL(&tpoMux); w = tpo_window_period_ticks; portEXIT_CRITICAL(&tpoMux);
    Serial.println(w);
    return;
  }

  if (up.startsWith("WIN")) {
    String val = cmd;
    val.replace("WIN", ""); val.replace("win", "");
    val.replace(":", " ");
    val.replace("=", " ");
    val.trim();
    uint32_t w = (uint32_t) val.toInt();
    if (w < 100)  w = 100;
    if (w > 5000) w = 5000;
    tpo_set_window_ms(w);
    Serial.print("WINDOW_MS="); Serial.println(w);
    return;
  }

  if (up.startsWith("PWM")) {
    String val = cmd;
    val.replace("PWM", ""); val.replace("pwm", "");
    val.replace(":" , " ");
    val.replace("=", " ");
    val.trim();
    float pct = val.toFloat();
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    pwm_override_enabled = true;
    pwm_override_percent = pct;
    tpo_set_percent(pct);
    Serial.print("MODE=OVERRIDE, PWM%="); Serial.println(pct, 2);
    return;
  }
}


// ========== [11) Encoder] ==========
// (ฟังก์ชันเดิมถูกย้ายและแทนที่ด้วย get_encoder_delta() และ get_click_event())


// ========== [12) Control (Modified)] ==========
void updateControl(OvenSettings& settings) {
  if (pwm_override_enabled) {  // Serial override ชนะทุกสิ่ง [cite: 141]
    return;
  }

  // *** นี่คือส่วนที่ต้องขยาย ***
  // ปัจจุบันโค้ดจะคุมเฉพาะ Heater 1 เท่านั้น

  // ถ้า Heater 1 ไม่ได้สั่งทำงาน
  if (!settings.heater_running[0]) {
    tpo_set_percent(0);
    return;
  }

  // ใช้ TC1 เป็นเซ็นเซอร์ควบคุม [cite: 142]
  if (isnan(g_sensors.TC1)) {
    tpo_set_percent(0);
    return;
  }

  // ใช้ค่า "Start Temp" จาก settings (แทน go_to เดิม)
  float error = settings.heater_start_temp[0] - g_sensors.TC1;

  // ... (PID Logic เหมือนเดิม) ... [cite: 144-150]
  const float DEADBAND = 0.1f;
  if (fabsf(error) < DEADBAND) error = 0.0f;
  if (Ki != 0.0f) {
    pid_integral += error;
    const float INTEGRAL_CLAMP = 1000.0f;
    if (pid_integral > INTEGRAL_CLAMP) pid_integral = INTEGRAL_CLAMP;
    if (pid_integral < -INTEGRAL_CLAMP) pid_integral = -INTEGRAL_CLAMP;
  } else {
    pid_integral = 0.0f;
  }
  float derivative = (error - pid_prev_err);
  pid_prev_err = error;
  float u = (Kp * error) + (Ki * pid_integral) + (Kd * derivative);
  if (u < 0.0f) u = 0.0f;
  const float U_SCALE = 100.0f;
  float percent = u * (100.0f / U_SCALE);
  if (percent > 100.0f) percent = 100.0f;

  tpo_set_percent(percent);
}

// ========== [13) Display] ==========
// (ฟังก์ชัน updateDisplay() เดิม [cite: 151] ถูกลบออกทั้งหมด และแทนที่ด้วย ui_draw() ใน OvenUI.h)

void printForSerialPlotter() {
  // [MODIFIED] อ่านค่าจาก g_sensors และ g_settings
  float temp1 = g_sensors.TC1;                                                    // [cite: 173]
  float temp2 = g_sensors.TC2;                                                    // [cite: 174]
  float sp = g_settings.heater_running[0] ? g_settings.heater_start_temp[0] : NAN;  // [cite: 174]

  if (isnan(sp)) sp = 0.0;  // Plotter ไม่ชอบ NAN

  if (isnan(temp1)) Serial.print(0.0);
  else Serial.print(temp1, 2);  // [cite: 176]

  Serial.print(",");
  if (isnan(temp2)) Serial.print(0.0);
  else Serial.print(temp2, 2);  // [cite: 177]
  Serial.print(",");
  Serial.println(sp, 2);  // [cite: 178]
}


// ========== [14) Manual MAX31855 Driver Functions] ==========
// ... (เหมือนเดิมทุกประการ) ... [cite: 179-197]
static inline int32_t sign_extend(int32_t v, uint8_t nbits) {
  int32_t shift = 32 - nbits;
  return (v << shift) >> shift;
}
void max31855_init_pins() {
  pinMode(MAXCLK, OUTPUT);
  pinMode(MAXDO, INPUT_PULLUP);
  digitalWrite(MAXCLK, LOW);
  for (int i = 0; i < NUM_THERMOCOUPLES; ++i) {
    pinMode(TC_CS_PINS[i], OUTPUT);
    digitalWrite(TC_CS_PINS[i], HIGH);
  }
}
static inline void all_cs_idle() {
  for (int i = 0; i < NUM_THERMOCOUPLES; ++i) digitalWrite(TC_CS_PINS[i], HIGH);
}
uint32_t max31855_read_raw(int csPin) {
  uint32_t v = 0;
  all_cs_idle();
  delayMicroseconds(MAX_GUARD_US);
  noInterrupts();
  digitalWrite(csPin, LOW);
  delayMicroseconds(2);
  for (int i = 31; i >= 0; --i) {
    digitalWrite(MAXCLK, HIGH);
    delayMicroseconds(2);
    int bit = digitalRead(MAXDO);
    v |= (uint32_t(bit) << i);
    digitalWrite(MAXCLK, LOW);
    delayMicroseconds(2);
  }
  digitalWrite(csPin, HIGH);
  interrupts();
  delayMicroseconds(MAX_GUARD_US);
  return v;
}
bool max31855_decode(uint32_t raw, float* tc_c, float* tj_c, uint8_t* faults) {
  bool fault = (raw & (1UL << 16));
  int32_t tc14 = (raw >> 18) & 0x3FFF;
  tc14 = sign_extend(tc14, 14);
  float tc = (float)tc14 * 0.25f;
  int32_t tj12 = (raw >> 4) & 0x0FFF;
  tj12 = sign_extend(tj12, 12);
  float tj = (float)tj12 * 0.0625f;
  uint8_t fbits = (uint8_t)(raw & 0x07);
  if (fault) fbits |= 0x80;
  if (tc_c) *tc_c = tc;
  if (tj_c) *tj_c = tj;
  if (faults) *faults = fbits;
  return !fault;
}
bool max31855_read_celsius(int csPin, float* tc_c, float* tj_c, uint8_t* faults, uint32_t* raw_out) {
  for (int attempt = 0; attempt < 3; ++attempt) {
    uint32_t raw = max31855_read_raw(csPin);
    if (raw_out) *raw_out = raw;
    if (raw == 0x00000000UL || raw == 0xFFFFFFFFUL) {
      delayMicroseconds(MAX_GUARD_US);
      continue;
    }
    return max31855_decode(raw, tc_c, tj_c, faults);
  }
  if (tc_c) *tc_c = NAN;
  if (tj_c) *tj_c = NAN;
  if (faults) *faults = 0xFF;
  return false;
}
