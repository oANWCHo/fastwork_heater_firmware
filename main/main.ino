/****************************************************
 * ESP32 + MAX31855 x2 + MLX90614 x2 + Encoder + ILI9341
 * Heater control via SSR using Time-Proportioning (Burst Fire)
 *
 * [MODIFIED]:
 * - Removed Adafruit_MAX31855 library
 * - Integrated manual bit-bang SPI logic from max_seperate.ino
 * - [CHANGED] Replaced Adafruit_ILI9341 with TFT_eSPI + Sprite for fast,
 * flicker-free display.
****************************************************/

// ========== [1) Include Libraries] ==========
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include "Adafruit_MLX90614.h"
#include "driver/pcnt.h"
#include <TFT_eSPI.h>
#include <Ticker.h>

// ========== [2) Pin Definitions] ==========
#define MAXDO     19 // MISO (Shared)
#define MAXCLK    18 // SCK (Shared)
#define MAXCS1    5  // CS for sensor 1
#define MAXCS2    33 // CS for sensor 2

#define ENCODER_A 36
#define ENCODER_B 39
#define ENCODER_SW 34

#define IR1_ADDR 0x10
#define IR2_ADDR 0x11

// --- SSR Output (Burst-Fire) ---
#define SSR_PIN     25
#define TPO_TICK_US     1000
#define WINDOW_MS_INIT  1000

// --- MAX31855 Manual Driver Config ---
#ifndef MAX_GUARD_US
#define MAX_GUARD_US 50   // gap between CS changes;
#endif

// ========== [3) Objects & Manual MAX31855 Config] ==========

static const int TC_CS_PINS[] = { MAXCS1, MAXCS2 };
const int NUM_THERMOCOUPLES = sizeof(TC_CS_PINS) / sizeof(TC_CS_PINS[0]);

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);

Adafruit_MLX90614 mlx1 = Adafruit_MLX90614();
Adafruit_MLX90614 mlx2 = Adafruit_MLX90614();
Ticker tpo_ticker;

// ========== [4) Runtime / Display Vars] ==========
long lastEncoderValue = 0;
int  lastSwitchState  = HIGH;

float   tc_temp_display[NUM_THERMOCOUPLES]; // thermocouple temperature (°C)
float   cj_temp_display[NUM_THERMOCOUPLES];
uint8_t tc_fault_bits[NUM_THERMOCOUPLES];   // fault summary per channel

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
float Kp = 1.2f, Ki = 0.02f, Kd = 0.01f;
float pid_integral = 0.0f;
float pid_prev_err = 0.0f;
bool  pwm_override_enabled = false;
float pwm_override_percent = 0.0f;
float last_pwm_percent     = 0.0f;

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
// This will be the manual version
void read_mlx_sensors();
void read_hardware_encoder();
void check_encoder_switch();

void max31855_init_pins();
static inline void all_cs_idle();
uint32_t max31855_read_raw(int csPin);
bool max31855_decode(uint32_t raw, float* tc_c, float* tj_c, uint8_t* faults);
bool max31855_read_celsius(int csPin, float* tc_c, float* tj_c, uint8_t* faults, uint32_t* raw_out);
static inline int32_t sign_extend(int32_t v, uint8_t nbits);


// ========== [7) Setup] ==========
void setup() {
  Serial.begin(115200);
  while (!Serial) { /* no delay */ }

  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);
  pinMode(ENCODER_SW, INPUT_PULLUP);

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
  if (!mlx1.begin(IR1_ADDR, &Wire)) {
    Serial.println("Error connecting to MLX #1 (Addr 0x10)");
    while (1) {}
  }
  if (!mlx2.begin(IR2_ADDR, &Wire)) {
    Serial.println("Error connecting to MLX #2 (Addr 0x11)");
    while (1) {}
  }

  max31855_init_pins();
  Serial.println("Using manual MAX31855 driver.");

  for (int i = 0; i < NUM_THERMOCOUPLES; i++) {
    tc_temp_display[i] = NAN;
    cj_temp_display[i] = NAN;
    tc_fault_bits[i] = 0;
  }

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  spr.createSprite(tft.width(), tft.height());
  spr.setSwapBytes(true); // [CHANGED] For printf compatibility

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
  if (now - t_sens >= 100) {
    t_sens = now;
    heater_read();
    read_mlx_sensors();
  }

  if (now - t_ctl >= 50) {
    t_ctl = now;
    updateControl();
  }

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
  for (int i = 0; i < NUM_THERMOCOUPLES; ++i) {
    float tc, tj;
    uint8_t fb;
    uint32_t raw; // For debug

    // Retry a couple times (max31855_read_celsius also has internal retries for 0x0/0xFF)
    bool ok = false;
    for (int attempt = 0; attempt < 2 && !ok; ++attempt) {
      ok = max31855_read_celsius(TC_CS_PINS[i], &tc, &tj, &fb, &raw);
      if (!ok) delayMicroseconds(MAX_GUARD_US);
    }

    if (ok) {
      tc_temp_display[i] = tc;
      cj_temp_display[i] = tj;
    } else {
      tc_temp_display[i] = NAN;
      cj_temp_display[i] = NAN;
    }
    tc_fault_bits[i] = fb;
  }
}


void read_mlx_sensors() {
  float t1 = mlx1.readObjectTempC();
  float t2 = mlx2.readObjectTempC();
  ir1_temp_display = t1;
  ir2_temp_display = t2;
  if (!isnan(t1) && !isnan(t2)) avg_temp_display = (t1 + t2) / 2.0f;
  else avg_temp_display = NAN;
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
    go_to = setpoint;
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
// [CHANGED] This function is completely refactored to use the Sprite.
// This is MUCH simpler and 100% flicker-free.
void updateDisplay() {

  // [REMOVED] All static "prev_" variables and `ui_inited` check.
  // We will redraw everything to the sprite buffer every time.

  // Define layout constants
  const uint8_t TXT_SIZE = 2;
  const int     GAP      = 8;
  const int     CHAR_H   = 8 * TXT_SIZE;
  const int     xPos     = 6;
  const int     xVal     = xPos + 90;
  int yStart = 4;
  int yLine[8];
  for (int i = 0; i < 8; i++) {
    yLine[i] = yStart + i * (CHAR_H + GAP);
  }

  // 1. Clear the sprite (fill with black)
  spr.fillSprite(TFT_BLACK);

  // 2. Set text properties
  spr.setTextWrap(false);
  spr.setTextSize(TXT_SIZE);
  spr.setTextColor(TFT_SILVER, TFT_BLACK); // Use built-in TFT_ORANGE

  // 3. Draw all labels and values to the sprite buffer

  // --- MODE ---
  spr.setCursor(xPos, yLine[0]); spr.print("MODE: ");
  spr.setCursor(xVal, yLine[0]);
  spr.print(pwm_override_enabled ? "OVERRIDE" : "AUTO(PID)");

  // --- PWM% ---
  spr.setCursor(xPos, yLine[1]); spr.print("PWM%: ");
  spr.setCursor(xVal, yLine[1]);
  spr.printf("%d %%", (int)(last_pwm_percent + 0.5f));

  // --- Setpoint (SP) ---
  spr.setCursor(xPos, yLine[2]); spr.print("SP: ");
  spr.setCursor(xVal, yLine[2]);
  spr.printf("%.1f C", setpoint);

  // --- Go To ---
  spr.setCursor(xPos, yLine[3]); spr.print("go_to: ");
  spr.setCursor(xVal, yLine[3]);
  if (has_go_to) spr.printf("%.1f C", go_to);
  else           spr.print("---");

  // --- TC1 ---
  spr.setCursor(xPos, yLine[4]); spr.print("TC1: ");
  spr.setCursor(xVal, yLine[4]);
  if (!isnan(tc_temp_display[0])) spr.printf("%.1f C", tc_temp_display[0]);
  else                            spr.print("---");

  // --- TC2 ---
  spr.setCursor(xPos, yLine[5]); spr.print("TC2: ");
  spr.setCursor(xVal, yLine[5]);
  if (!isnan(tc_temp_display[1])) spr.printf("%.1f C", tc_temp_display[1]);
  else                            spr.print("---");

  // --- IR1 ---
  spr.setCursor(xPos, yLine[6]); spr.print("IR1: ");
  spr.setCursor(xVal, yLine[6]);
  if (!isnan(ir1_temp_display)) spr.printf("%.1f C", ir1_temp_display);
  else                          spr.print("ERR"); // [CHANGED] Keep "ERR" as per original code

  // --- IR2 ---
  spr.setCursor(xPos, yLine[7]); spr.print("IR2: ");
  spr.setCursor(xVal, yLine[7]);
  if (!isnan(ir2_temp_display)) spr.printf("%.1f C", ir2_temp_display);
  else                          spr.print("ERR"); // [CHANGED] Keep "ERR" as per original code

  // 4. Push the completed sprite to the screen in one DMA transfer
  spr.pushSprite(0, 0);
}


void printForSerialPlotter() {
  // [REFACTOR] Get values from the temperature array (This code requires no changes)
  float temp1 = tc_temp_display[0];
  float temp2 = tc_temp_display[1];
  float sp = has_go_to ? go_to : NAN;
  if (isnan(sp)) {
    sp = setpoint;
  }

  // Print "Temp1,Temp2,Setpoint"
  if (isnan(temp1)) Serial.print(0.0);
  else               Serial.print(temp1, 2);

  Serial.print(",");

  if (isnan(temp2)) Serial.print(0.0);
  else               Serial.print(temp2, 2);

  Serial.print(",");
  Serial.println(sp, 2);
}


// ========== [14) Manual MAX31855 Driver Functions] ==========
// All functions below are copied from max_seperate.ino

// Helper to sign-extend a value
static inline int32_t sign_extend(int32_t v, uint8_t nbits) {
  int32_t shift = 32 - nbits;
  return (v << shift) >> shift;
}

// Initialize all MAX31855-related pins
void max31855_init_pins() {
  pinMode(MAXCLK, OUTPUT);
  pinMode(MAXDO, INPUT_PULLUP);
  digitalWrite(MAXCLK, LOW);
  // CPOL=0 idle LOW

  for (int i = 0; i < NUM_THERMOCOUPLES; ++i) {
    pinMode(TC_CS_PINS[i], OUTPUT);
    digitalWrite(TC_CS_PINS[i], HIGH); // idle = not selected
  }
}

// Set all CS pins to idle (HIGH)
static inline void all_cs_idle() {
  for (int i = 0; i < NUM_THERMOCOUPLES; ++i) digitalWrite(TC_CS_PINS[i], HIGH);
}

// Shift out 32-bit frame from MAX31855 (MSB first), sample on SCK rising edge
uint32_t max31855_read_raw(int csPin) {
  uint32_t v = 0;
  all_cs_idle();
  delayMicroseconds(MAX_GUARD_US);

  noInterrupts();
  digitalWrite(csPin, LOW);
  delayMicroseconds(2);               // tCSS: allow CS to settle before clocking

  for (int i = 31; i >= 0; --i) {
    // Rising edge: sample DO
    digitalWrite(MAXCLK, HIGH);
    delayMicroseconds(2);
    int bit = digitalRead(MAXDO);
    v |= (uint32_t(bit) << i);
    // Falling edge: device shifts next bit
    digitalWrite(MAXCLK, LOW);
    delayMicroseconds(2);
  }

  digitalWrite(csPin, HIGH);
  interrupts();

  delayMicroseconds(MAX_GUARD_US);
  return v;
}

/*
  Decode RAW:
   - [31:18] 14-bit signed thermocouple temp, 0.25°C/LSB
   - [16]    fault flag (1 = fault)
   - [15:4]  12-bit signed cold-junction temp, 0.0625°C/LSB
   - [2] SCV, [1] SCG, [0] OC
  Returns true if no global fault flag, but still fills outputs for diagnostics.
*/
bool max31855_decode(uint32_t raw, float* tc_c, float* tj_c, uint8_t* faults) {
  bool fault = (raw & (1UL << 16));
  int32_t tc14 = (raw >> 18) & 0x3FFF;   // 14-bit
  tc14 = sign_extend(tc14, 14);
  float tc = (float)tc14 * 0.25f;

  int32_t tj12 = (raw >> 4) & 0x0FFF; // 12-bit
  tj12 = sign_extend(tj12, 12);
  float tj = (float)tj12 * 0.0625f;

  uint8_t fbits = (uint8_t)(raw & 0x07);
  if (fault) fbits |= 0x80; // mark global fault in bit7

  if (tc_c) *tc_c = tc;
  if (tj_c) *tj_c = tj;
  if (faults) *faults = fbits;

  return !fault;
}

// Read one channel by CS pin, with retries for bad frames
bool max31855_read_celsius(int csPin, float* tc_c, float* tj_c, uint8_t* faults, uint32_t* raw_out) {
  for (int attempt = 0; attempt < 3; ++attempt) {
    uint32_t raw = max31855_read_raw(csPin);
    if (raw_out) *raw_out = raw;

    // Filter bad frames (all 0s or all 1s usually indicates MISO floating)
    if (raw == 0x00000000UL || raw == 0xFFFFFFFFUL) {
      delayMicroseconds(MAX_GUARD_US);
      continue;  // Try again
    }

    return max31855_decode(raw, tc_c, tj_c, faults);
  }

  // If all 3 attempts failed
  if (tc_c) *tc_c = NAN;
  if (tj_c) *tj_c = NAN;
  if (faults) *faults = 0xFF; // All fault bits
  return false;
}