/****************************************************
 * ESP32 + MAX31855 x2 + MLX90614 x2 + Encoder + ILI9341
 * Heater control via SSR using Time-Proportioning (Burst Fire)
 * - Uses Ticker library for stable 1ms ISR
 * - [REFACTOR] Uses an array for MAX31855 sensors
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
#include <Ticker.h>

// ========== [2) Pin Definitions] ==========
// [REFACTOR] Renamed for array
#define MAXDO     19 // MISO (Shared)
#define MAXCLK    18 // SCK (Shared)
#define MAXCS1    5  // CS for sensor 1
#define MAXCS2    33 // CS for sensor 2

#define ENCODER_A 36
#define ENCODER_B 39
#define ENCODER_SW 32

#define TFT_DC    27
#define TFT_CS    26
#define TFT_MOSI  13
#define TFT_MISO  12
#define TFT_CLK   14
#define TFT_RST   4

#define IR1_ADDR 0x10
#define IR2_ADDR 0x11

// --- SSR Output (Burst-Fire) ---
#define SSR_PIN         25
#define TPO_TICK_US     1000
#define WINDOW_MS_INIT  1000

#define MAX_GUARD_US  50

// ========== [3) Objects] ==========
// [REFACTOR] Use an array for thermocouples
Adafruit_MAX31855 thermocouples[] = {
  Adafruit_MAX31855(MAXCLK, MAXCS1, MAXDO),
  Adafruit_MAX31855(MAXCLK, MAXCS2, MAXDO)
};
// [NEW] Helper to get the number of TCs
const int NUM_THERMOCOUPLES = sizeof(thermocouples) / sizeof(thermocouples[0]);

static const int TC_CS_PINS[] = { MAXCS1, MAXCS2 }; // ถ้ามี >2 ตัว ให้เพิ่มพินในลิสต์นี้
static_assert(sizeof(TC_CS_PINS)/sizeof(TC_CS_PINS[0]) == NUM_THERMOCOUPLES, "TC_CS_PINS size must match NUM_THERMOCOUPLES");

Adafruit_ILI9341  tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);
Adafruit_MLX90614 mlx1 = Adafruit_MLX90614();
Adafruit_MLX90614 mlx2 = Adafruit_MLX90614();
Ticker tpo_ticker;

// ========== [4) Runtime / Display Vars] ==========
long lastEncoderValue = 0;
int  lastSwitchState  = HIGH;

// [REFACTOR] Use an array for TC temps
float   tc_temp_display[NUM_THERMOCOUPLES] = { NAN, NAN }; // Index 0 is TC1, Index 1 is TC2

float   ir1_temp_display = NAN;
float   ir2_temp_display = NAN;
float   avg_temp_display = NAN;
int16_t encoder_count_display = 0;
int     switch_state_display  = HIGH;

float setpoint  = 25.0f;
float go_to     = NAN;
bool  has_go_to = false;

const int   ENCODER_COUNTS_PER_STEP = 4;
const float STEP_SIZE               = 0.5f;

float Kp = 1.0f, Ki = 0.0f, Kd = 0.0f;
float pid_integral = 0.0f;
float pid_prev_err = 0.0f;

bool  pwm_override_enabled = false;
float pwm_override_percent = 0.0f;
float last_pwm_percent     = 0.0f;

uint16_t COLOR_ORANGE = 0;

// ========== [5) TPO via Hardware Timer] ==========
portMUX_TYPE tpoMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t tpo_window_period_ticks = WINDOW_MS_INIT;
volatile uint32_t tpo_on_ticks            = 0;
volatile uint32_t tpo_tick_counter        = 0;

void tpo_set_percent(float percent) {
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  last_pwm_percent = percent;
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
void updateDisplay();
void updateControl();
void read_heater_input();
void heater_read();
void read_mlx_sensors();
void read_hardware_encoder();
void check_encoder_switch();

static inline void tc_all_cs_idle_init_once();
static inline void tc_all_cs_idle();
// ========== [7) Setup] ==========
void setup() {
  Serial.begin(115200);
  while (!Serial) { /* no delay */ }

  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);
  pinMode(ENCODER_SW, INPUT_PULLUP);

  pinMode(MAXCS1, OUTPUT);
  pinMode(MAXCS2, OUTPUT);
  digitalWrite(MAXCS1, HIGH);
  digitalWrite(MAXCS2, HIGH);
  // PCNT (encoder)
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

  Wire.begin();
  if (!mlx1.begin(IR1_ADDR, &Wire)) { Serial.println("Error connecting to MLX #1 (Addr 0x10)"); while (1) {} }
  if (!mlx2.begin(IR2_ADDR, &Wire)) { Serial.println("Error connecting to MLX #2 (Addr 0x11)"); while (1) {} }

  // [REFACTOR] Initialize all thermocouples in a loop
  for (int i = 0; i < NUM_THERMOCOUPLES; i++) {
    if (!thermocouples[i].begin()) {
      Serial.printf("MAX31855 #%d init ERROR\n", i + 1);
      while (1) {}
    }
  }

  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(ILI9341_BLACK);
  COLOR_ORANGE = tft.color565(255, 165, 0);
  updateDisplay();

  // TPO setup
  tpo_set_window_ms(WINDOW_MS_INIT);
  tpo_set_percent(0.0f);
  tpo_ticker.attach_ms(1, tpo_isr);

  Serial.println("Ready (Ticker 1ms). Use: PWM <0..100>, AUTO, PWM?, WIN=ms");
}

// ========== [8) Main Loop: cooperative, no delay()] ==========
void loop() {
  read_heater_input();
  read_hardware_encoder();
  check_encoder_switch();

  static uint32_t t_sens = 0, t_disp = 0, t_ctl = 0, t_plot = 0;
  uint32_t now = millis();

  // Read sensors ~100 ms
  if (now - t_sens >= 100) {
    t_sens = now;
    heater_read();
    read_mlx_sensors();
  }

  // Update control ~50 ms
  if (now - t_ctl >= 50) {
    t_ctl = now;
    updateControl();
  }

  // Update display ~50 ms
  if (now - t_disp >= 50) {
    t_disp = now;
    updateDisplay();
  }

  if (now - t_plot >= 200) {
    t_plot = now;
    printForSerialPlotter();
  }
}

// ========== [9) Sensor Reading] ==========
void heater_read() {
  tc_all_cs_idle_init_once();

  for (int i = 0; i < NUM_THERMOCOUPLES; ++i) {
    // ปล่อยทุก CS ขึ้นสูงค้างสักแป๊บให้เฟรมก่อนหน้าจบสนิท
    tc_all_cs_idle();
    delayMicroseconds(MAX_GUARD_US);

    // อ่านทีละตัว พร้อมกัน ISR ไม่ให้มารบกวน timing ของ SCK/CS
    float t = NAN;
    for (int attempt = 0; attempt < 3; ++attempt) {   // เผื่อเฟรมพลาด ลองซ้ำเล็กน้อย
      noInterrupts();
      t = thermocouples[i].readCelsius();            // ไลบรารีจะกด/ปล่อย CS ของตัวที่ i ให้เอง
      interrupts();

      if (!isnan(t)) break;
      delayMicroseconds(MAX_GUARD_US);
      tc_all_cs_idle();                               // ย้ำ idle ก่อนลองใหม่
      delayMicroseconds(MAX_GUARD_US);
    }

    tc_temp_display[i] = t;

    // เว้นช่องไฟก่อนจะไปตัวถัดไป
    tc_all_cs_idle();
    delayMicroseconds(MAX_GUARD_US);
  }
}

void read_mlx_sensors() {
  float t1 = mlx1.readObjectTempC();
  float t2 = mlx2.readObjectTempC();
  ir1_temp_display = t1;
  ir2_temp_display = t2;
  if (!isnan(t1) && !isnan(t2)) avg_temp_display = (t1 + t2) / 2.0f;
  else                        avg_temp_display = NAN;
}

// ========== [10) Serial Commands: override/auto/status/window] ==========
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
    Serial.print("MODE: "); Serial.println(pwm_override_enabled ? "OVERRIDE" : "AUTO(PID)");
    Serial.print("PWM%: "); Serial.println(last_pwm_percent, 2);
    Serial.print("WINDOW_MS: ");
    uint32_t w; portENTER_CRITICAL(&tpoMux); w = tpo_window_period_ticks; portEXIT_CRITICAL(&tpoMux);
    Serial.println(w);
    return;
  }

  if (up.startsWith("WIN")) {
    String val = cmd;
    val.replace("WIN", ""); val.replace("win", "");
    val.replace(":", " ");  val.replace("=", " ");
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
    val.replace(":", " ");  val.replace("=", " ");
    val.trim();
    float pct = val.toFloat();
    if (pct < 0) pct = 0; if (pct > 100) pct = 100;
    pwm_override_enabled = true;
    pwm_override_percent = pct;
    tpo_set_percent(pct);
    Serial.print("MODE=OVERRIDE, PWM%="); Serial.println(pct, 2);
    return;
  }

  Serial.println("Unknown. Use: PWM <0..100>, AUTO, PWM?, WIN=<100..5000>");
}

// ========== [11) Encoder] ==========
void read_hardware_encoder() {
  int16_t current = 0;
  pcnt_get_counter_value(PCNT_UNIT_0, &current);
  encoder_count_display = current;

  long delta = (long)current - lastEncoderValue;
  if (delta != 0) {
    float steps = (float)delta / (float)ENCODER_COUNTS_PER_STEP;
    setpoint += steps * STEP_SIZE;
    if (setpoint < -1000.0f) setpoint = -1000.0f;
    if (setpoint >  1000.0f) setpoint =  1000.0f;
    lastEncoderValue = current;
  }
}

void check_encoder_switch() {
  int cur = digitalRead(ENCODER_SW);
  switch_state_display = cur;
  if (lastSwitchState == HIGH && cur == LOW) {
    go_to     = setpoint;
    has_go_to = true;
    pid_integral = 0.0f;
    pid_prev_err = 0.0f;
    Serial.println("Commit go_to = setpoint");
  }
  lastSwitchState = cur;
}

// ========== [12) Control (PID or Override -> percent)] ==========
void updateControl() {
  if (pwm_override_enabled) {
    return;
  }
  if (!has_go_to) { tpo_set_percent(0); return; }

  // [REFACTOR] Use TC #1 (index 0) as the control sensor
  if (isnan(tc_temp_display[0])) { tpo_set_percent(0); return; }

  // Calculate error using the first K-type sensor
  float error = go_to - tc_temp_display[0];

  const float DEADBAND = 0.1f;
  if (fabsf(error) < DEADBAND) error = 0.0f;

  if (Ki != 0.0f) {
    pid_integral += error;
    const float INTEGRAL_CLAMP = 1000.0f;
    if (pid_integral >  INTEGRAL_CLAMP) pid_integral =  INTEGRAL_CLAMP;
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
void updateDisplay() {
  static bool ui_inited = false;
  static int yLine[8], xPos;

  // Renamed prev_tc1/2 for clarity, matches the array access
  static float   prev_sp = NAN, prev_go = NAN, prev_tc1 = NAN, prev_tc2 = NAN, prev_ir1 = NAN, prev_ir2 = NAN;
  static int     prev_mode  = -1;
  static int     prev_pwm   = -1;

  const uint8_t TXT_SIZE = 2;
  const int     GAP      = 8;

  if (!ui_inited) {
    tft.setTextWrap(false);
    tft.setTextSize(TXT_SIZE);
    tft.setTextColor(COLOR_ORANGE, ILI9341_BLACK);

    const int CHAR_H = 8 * TXT_SIZE;
    int yStart = 4;
    xPos = 6;
    for (int i=0;i<8;i++) yLine[i] = yStart + i * (CHAR_H + GAP);
    ui_inited = true;

    // Using the re-ordered layout from previous request
    tft.setCursor(xPos, yLine[0]); tft.print("MODE: ");
    tft.setCursor(xPos, yLine[1]); tft.print("PWM%: ");
    tft.setCursor(xPos, yLine[2]); tft.print("SP: ");
    tft.setCursor(xPos, yLine[3]); tft.print("go_to: ");
    tft.setCursor(xPos, yLine[4]); tft.print("TC1: ");
    tft.setCursor(xPos, yLine[5]); tft.print("TC2: ");
    tft.setCursor(xPos, yLine[6]); tft.print("IR1: ");
    tft.setCursor(xPos, yLine[7]); tft.print("IR2: ");
  }

  int xVal = xPos + 90;
  const int VAL_W = 140;
  const int VAL_H = 18;

  auto printVal = [&](int y, float v, const char* fmt="%.1f C") {
    tft.fillRect(xVal, y, VAL_W, VAL_H, ILI9341_BLACK);
    tft.setCursor(xVal, y);
    char buf[24]; snprintf(buf, sizeof(buf), fmt, v); tft.print(buf);
  };
  auto printTxt = [&](int y, const char* s) {
    tft.fillRect(xVal, y, VAL_W, VAL_H, ILI9341_BLACK);
    tft.setCursor(xVal, y); tft.print(s);
  };

  int mode_now = pwm_override_enabled ? 1 : 0;
  if (mode_now != prev_mode) { prev_mode = mode_now; printTxt(yLine[0], pwm_override_enabled ? "OVERRIDE" : "AUTO(PID)"); }

  int pwm_now = (int)(last_pwm_percent + 0.5f);
  if (pwm_now != prev_pwm) {
    prev_pwm = pwm_now;
    tft.fillRect(xVal, yLine[1], VAL_W, VAL_H, ILI9341_BLACK);
    tft.setCursor(xVal, yLine[1]); tft.print(pwm_now); tft.print(" %");
  }

  if (setpoint != prev_sp) { prev_sp = setpoint; printVal(yLine[2], prev_sp); }

  float go_disp = has_go_to ? go_to : NAN;
  if (go_disp != prev_go) {
    prev_go = go_disp;
    if (has_go_to) printVal(yLine[3], prev_go);
    else           printTxt(yLine[3], "---");
  }

  // [REFACTOR] Update display using the temperature array
  if (tc_temp_display[0] != prev_tc1) {
    prev_tc1 = tc_temp_display[0];
    if(isnan(prev_tc1)) printTxt(yLine[4], "ERR"); else printVal(yLine[4], prev_tc1);
  }

  if (tc_temp_display[1] != prev_tc2) {
    prev_tc2 = tc_temp_display[1];
    if(isnan(prev_tc2)) printTxt(yLine[5], "ERR"); else printVal(yLine[5], prev_tc2);
  }

  if (ir1_temp_display != prev_ir1) {
    prev_ir1 = ir1_temp_display;
    if(isnan(prev_ir1)) printTxt(yLine[6], "ERR"); else printVal(yLine[6], prev_ir1);
  }

  if (ir2_temp_display != prev_ir2) {
    prev_ir2 = ir2_temp_display;
    if(isnan(prev_ir2)) printTxt(yLine[7], "ERR"); else printVal(yLine[7], prev_ir2);
  }
}

void printForSerialPlotter() {
  // [REFACTOR] Get values from the temperature array
  float temp1 = tc_temp_display[0];
  float temp2 = tc_temp_display[1];
  float sp = has_go_to ? go_to : NAN;

  if (isnan(sp)) {
    sp = setpoint;
  }

  // Print "Temp1,Temp2,Setpoint"
  if (isnan(temp1)) Serial.print(0.0);
  else              Serial.print(temp1, 2);

  Serial.print(",");

  if (isnan(temp2)) Serial.print(0.0);
  else              Serial.print(temp2, 2);

  Serial.print(",");
  Serial.println(sp, 2);
}

static inline void tc_all_cs_idle_init_once() {
  static bool inited = false;
  if (!inited) {
    for (int k = 0; k < NUM_THERMOCOUPLES; ++k) {
      pinMode(TC_CS_PINS[k], OUTPUT);
      digitalWrite(TC_CS_PINS[k], HIGH); // idle = ไม่เลือกชิป
    }
    inited = true;
  }
}

static inline void tc_all_cs_idle() {
  for (int k = 0; k < NUM_THERMOCOUPLES; ++k) {
    digitalWrite(TC_CS_PINS[k], HIGH);
  }
}