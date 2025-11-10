/****************************************************
 * ESP32 + MAX31855 x3 + MLX910614 x2 + Encoder + ILI9341
 * Heater control via SSR using Time-Proportioning (Burst Fire)
 *
 * Final version with complete UI Manager.
 ****************************************************/

// ========== [1) Include Libraries] ==========
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include "Adafruit_MLX90614.h"
#include "driver/pcnt.h"
#include <TFT_eSPI.h>
#include <Ticker.h>
#include "ui_manager.h"
#include <Preferences.h>

// ========== [2) Pin Definitions] ==========
#define MAXDO 19   // MISO (Shared)
#define MAXCLK 18  // SCK (Shared)
#define MAXCS1 32  // CS for sensor 1
#define MAXCS2 5   // CS for sensor 2
#define MAXCS3 33  // CS for sensor 3

#define ENCODER_A 36
#define ENCODER_B 39
#define ENCODER_SW 34

#define BUZZER 23

#define IR1_ADDR 0x10
#define IR2_ADDR 0x11

// --- SSR Output (Burst-Fire) ---
#define SSR_PIN1 25
#define SSR_PIN2 26
#define SSR_PIN3 27

#define TPO_TICK_US 1000
#define WINDOW_MS_INIT 1000

// --- MAX31855 Manual Driver Config ---
#ifndef MAX_GUARD_US
#define MAX_GUARD_US 50
#endif

// ========== [3) Objects & Manual MAX31855 Config] ==========
static const int TC_CS_PINS[] = { MAXCS1, MAXCS2, MAXCS3 };
const int NUM_THERMOCOUPLES = sizeof(TC_CS_PINS) / sizeof(TC_CS_PINS[0]);
TFT_eSPI tft = TFT_eSPI();

Preferences preferences;                     // <-- ADD THIS
void saveConfig(const ConfigState& config);  // <-- ADD THIS (Forward declaration)
UIManager ui(&tft, saveConfig);

ConfigState config;
Adafruit_MLX90614 mlx1 = Adafruit_MLX90614();
Adafruit_MLX90614 mlx2 = Adafruit_MLX90614();
Ticker tpo_ticker;
// ========== [4) Runtime / Display Vars] ==========
long lastEncoderValue = 0;
int lastSwitchState = HIGH;

float tc_temp_display[NUM_THERMOCOUPLES];
float cj_temp_display[NUM_THERMOCOUPLES];
uint8_t tc_fault_bits[NUM_THERMOCOUPLES];

enum ClickState { IDLE,
                  AWAITING_SECOND_CLICK };
ClickState click_state = IDLE;
uint32_t first_click_time = 0;
const uint32_t DOUBLE_CLICK_WINDOW_MS = 400;

float ir1_temp_display = NAN;
float ir2_temp_display = NAN;
float avg_temp_display = NAN;
int16_t encoder_count_display = 0;
int switch_state_display = HIGH;

float go_to = NAN;
bool has_go_to = false;
const int ENCODER_COUNTS_PER_STEP = 2;

float Kp = 1.2f, Ki = 0.02f, Kd = 0.01f;
float pid_integral[NUM_THERMOCOUPLES] = { 0.0f, 0.0f, 0.0f };
float pid_prev_err[NUM_THERMOCOUPLES] = { 0.0f, 0.0f, 0.0f };
bool pwm_override_enabled = false;
float pwm_override_percent = 0.0f;
float last_pwm_percent[NUM_THERMOCOUPLES] = { 0.0f, 0.0f, 0.0f };

volatile int beep_queue = 0;
volatile uint32_t t_next_beep_action = 0;
volatile bool beeper_active = false;
const int BEEP_ON_MS = 60;
const int BEEP_OFF_MS = 100;

// ** ADDED ** State for max temp cutoff blinking
bool heater_cutoff_state[NUM_THERMOCOUPLES] = { false, false, false };
uint32_t heater_cutoff_start_time[NUM_THERMOCOUPLES] = { 0, 0, 0 };
const uint32_t CUTOFF_BLINK_DURATION_MS = 10000;  // 10 seconds

// --- MODIFIED FOR MAX TEMP BEEP ---
bool heater_near_max_state[NUM_THERMOCOUPLES] = { false, false, false }; // Renamed from heater_near_target_state
const float NEAR_MAX_THRESHOLD_C = 5.0f; // Beep when within 5 degrees C of max temp
// --- END MODIFICATION ---

// ========== [5) TPO via Hardware Timer] ==========
portMUX_TYPE tpoMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t tpo_window_period_ticks = WINDOW_MS_INIT;
volatile uint32_t tpo_on_ticks[NUM_THERMOCOUPLES] = { 0, 0, 0 };
volatile uint32_t tpo_tick_counter = 0;

void tpo_set_percent(int index, float percent) {
  if (index < 0 || index >= NUM_THERMOCOUPLES) return;  // Safety check
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;

  last_pwm_percent[index] = percent;
  uint32_t onTicks = (uint32_t)((percent / 100.0f) * (float)tpo_window_period_ticks + 0.5f);

  portENTER_CRITICAL(&tpoMux);
  tpo_on_ticks[index] = onTicks;
  portEXIT_CRITICAL(&tpoMux);
}

void tpo_set_window_ms(uint32_t window_ms) {
  if (window_ms < 100) window_ms = 100;
  if (window_ms > 5000) window_ms = 5000;

  portENTER_CRITICAL(&tpoMux);
  tpo_window_period_ticks = window_ms;
  if (tpo_tick_counter >= tpo_window_period_ticks) tpo_tick_counter = 0;

  // Update on-ticks for all channels based on their last percentages
  for (int i = 0; i < NUM_THERMOCOUPLES; i++) {
    tpo_on_ticks[i] = (uint32_t)((last_pwm_percent[i] / 100.0f) * (float)window_ms + 0.5f);
  }

  portEXIT_CRITICAL(&tpoMux);
}

void IRAM_ATTR tpo_isr() {
  uint32_t pos;
  uint32_t onTicks[NUM_THERMOCOUPLES];  // Array to hold values

  portENTER_CRITICAL_ISR(&tpoMux);
  pos = tpo_tick_counter;

  // Read all on-tick values inside critical section
  for (int i = 0; i < NUM_THERMOCOUPLES; i++) {
    onTicks[i] = tpo_on_ticks[i];
  }

  tpo_tick_counter++;
  if (tpo_tick_counter >= tpo_window_period_ticks) {
    tpo_tick_counter = 0;
  }
  portEXIT_CRITICAL_ISR(&tpoMux);

  // Write to all three SSRs
  digitalWrite(SSR_PIN1, (pos < onTicks[0]) ? HIGH : LOW);
  digitalWrite(SSR_PIN2, (pos < onTicks[1]) ? HIGH : LOW);
  digitalWrite(SSR_PIN3, (pos < onTicks[2]) ? HIGH : LOW);
}
void queue_beeps(int count) {
  if (beeper_active || !config.sound_on) return; // Don't interrupt, or if sound is off
  beep_queue = count * 2; // (on, off, on, off, on, off)
  t_next_beep_action = millis();
  beeper_active = true;
}
void saveConfig(const ConfigState& config) {
  // Save the entire config struct as a blob of bytes
  preferences.putBytes("config", &config, sizeof(config));
  Serial.println("Configuration saved to flash.");
}

void loadConfig(ConfigState& config) {
  // Check if a saved config exists and is the correct size
  if (preferences.getBytesLength("config") == sizeof(config)) {
    preferences.getBytes("config", &config, sizeof(config));
    Serial.println("Configuration loaded from flash.");
  } else {
    Serial.println("No valid config found, using defaults and saving.");
    // If no config is found, save the current defaults for next time
    saveConfig(config);
  }
}

// ========== [6) Forward Declarations] ==========
void updateControl();
void read_heater_input();
void heater_read();
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
  preferences.begin("app_config", false);
  while (!Serial) {}

  config.target_temps[0] = 50.0f;
  config.target_temps[1] = 50.0f;
  config.target_temps[2] = 50.0f;
  config.max_temps[0] = 200.0f;
  config.max_temps[1] = 200.0f;
  config.max_temps[2] = 200.0f;
  config.max_temp_lock = 300.0f;
  config.temp_unit = 'C';
  config.idle_off_mode = IDLE_OFF_ALWAYS_ON;
  config.light_on = true;
  config.sound_on = true;
  config.heater_active[0] = false;
  config.heater_active[1] = false;
  config.heater_active[2] = false;

  config.tc_offsets[0] = 0.0f;
  config.tc_offsets[1] = 0.0f;
  config.tc_offsets[2] = 0.0f;
  loadConfig(config);

  pinMode(SSR_PIN1, OUTPUT);
  digitalWrite(SSR_PIN1, LOW);
  pinMode(SSR_PIN2, OUTPUT);
  digitalWrite(SSR_PIN2, LOW);
  pinMode(SSR_PIN3, OUTPUT);
  digitalWrite(SSR_PIN3, LOW);

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  pinMode(ENCODER_SW, INPUT_PULLUP);

  // PCNT (encoder)
  pcnt_config_t pcnt_config = {};
  pcnt_config.pulse_gpio_num = ENCODER_A;
  pcnt_config.ctrl_gpio_num = ENCODER_B;
  pcnt_config.channel = PCNT_CHANNEL_0;
  pcnt_config.unit = PCNT_UNIT_0;
  pcnt_config.pos_mode = PCNT_COUNT_DEC;
  pcnt_config.neg_mode = PCNT_COUNT_INC;
  pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
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
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);

  ui.begin();

  // TPO setup
  tpo_set_window_ms(WINDOW_MS_INIT);
  tpo_set_percent(0, 0.0f);  // Set H1 to 0%
  tpo_set_percent(1, 0.0f);  // Set H2 to 0%
  tpo_set_percent(2, 0.0f);  // Set H3 to 0%
  tpo_ticker.attach_ms(1, tpo_isr);
  Serial.println("Ready (Ticker 1ms). Use: PWM <0..100>, AUTO, PWM?, WIN=ms");
}

// ========== [8) Main Loop: cooperative, no delay()] ==========
void loop() {
  if (beeper_active && millis() >= t_next_beep_action) {
    if (beep_queue > 0) {
      bool is_on_cycle = (beep_queue % 2 == 0);
      if (is_on_cycle) {
        digitalWrite(BUZZER, HIGH);
        t_next_beep_action = millis() + BEEP_ON_MS;
      } else {
        digitalWrite(BUZZER, LOW);
        t_next_beep_action = millis() + BEEP_OFF_MS;
      }
      beep_queue--;
    } else {
      digitalWrite(BUZZER, LOW);
      beeper_active = false;
    }
  }

  // ** ADDED ** Manage the cutoff blink state timer
  for (int i = 0; i < NUM_THERMOCOUPLES; i++) {
    if (heater_cutoff_state[i]) {
      if (millis() - heater_cutoff_start_time[i] > CUTOFF_BLINK_DURATION_MS) {
        heater_cutoff_state[i] = false;  // Stop blinking after 10 seconds
      }
    }
  }

  if (click_state == AWAITING_SECOND_CLICK && millis() - first_click_time > DOUBLE_CLICK_WINDOW_MS) {
    ui.handleButtonSingleClick(config, go_to, has_go_to);
    click_state = IDLE;
  }

  read_heater_input();
  read_hardware_encoder();
  check_encoder_switch();

  ui.checkInactivity();
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
    AppState current_state;
    for (int i = 0; i < NUM_THERMOCOUPLES; i++) {
      current_state.tc_temps[i] = tc_temp_display[i];
      current_state.tc_faults[i] = tc_fault_bits[i];
      // ** ADDED ** Pass cutoff state to UI
      current_state.heater_cutoff_state[i] = heater_cutoff_state[i];
    }
    current_state.ir_temps[0] = ir1_temp_display;
    current_state.ir_temps[1] = ir2_temp_display;
    current_state.is_heating_active = has_go_to;
    current_state.target_temp = go_to;
    current_state.temp_unit = config.temp_unit;

    ui.draw(current_state, config);
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
    uint32_t raw;
    bool ok = false;
    for (int attempt = 0; attempt < 2 && !ok; ++attempt) {
      ok = max31855_read_celsius(TC_CS_PINS[i], &tc, &tj, &fb, &raw);
      if (!ok) delayMicroseconds(MAX_GUARD_US);
    }
    if (ok) {
      tc_temp_display[i] = tc + config.tc_offsets[i];
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

// ========== [10) Serial Commands] ==========
void read_heater_input() {
  if (Serial.available() <= 0) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  String up = cmd;
  up.toUpperCase();
  if (up == "AUTO") {
    pwm_override_enabled = false;
    Serial.println("MODE=AUTO(PID)");
    return;
  }
  if (up == "PWM?" || up == "STATUS?" || up == "DUTY?") {
    Serial.print("MODE: ");
    Serial.println(pwm_override_enabled ? "OVERRIDE" : "AUTO(PID)");
    Serial.print("PWM%: ");
    Serial.println(last_pwm_percent[0], 2);
    Serial.print("WINDOW_MS: ");
    uint32_t w;
    portENTER_CRITICAL(&tpoMux);
    w = tpo_window_period_ticks;
    portEXIT_CRITICAL(&tpoMux);
    Serial.println(w);
    return;
  }
  if (up.startsWith("WIN")) {
    String val = cmd;
    val.replace("WIN", "");
    val.replace("win", "");
    val.replace(":", " ");
    val.replace("=", " ");
    val.trim();
    uint32_t w = (uint32_t)val.toInt();
    if (w < 100) w = 100;
    if (w > 5000) w = 5000;
    tpo_set_window_ms(w);
    Serial.print("WINDOW_MS=");
    Serial.println(w);
    return;
  }
  if (up.startsWith("PWM")) {
    String val = cmd;
    val.replace("PWM", "");
    val.replace("pwm", "");
    val.replace(":", " ");
    val.replace("=", " ");
    val.trim();
    float pct = val.toFloat();
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    pwm_override_enabled = true;
    pwm_override_percent = pct;
    tpo_set_percent(0, pct);
    Serial.print("MODE=OVERRIDE, PWM%=");
    Serial.println(pct, 2);
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
    ui.handleEncoderRotation(steps, config);
    lastEncoderValue = current;
  }
}

void check_encoder_switch() {
  int cur = digitalRead(ENCODER_SW);
  switch_state_display = cur;
  if (lastSwitchState == HIGH && cur == LOW) {  // On button press
    queue_beeps(1);
    if (click_state == IDLE) {
      click_state = AWAITING_SECOND_CLICK;
      first_click_time = millis();
    } else {
      ui.handleButtonDoubleClick(config);
      click_state = IDLE;
    }
  }
  lastSwitchState = cur;
}

// ========== [12) Control] ==========
void updateControl() {
  if (pwm_override_enabled) {
    // Manual override is on.
    // We already set H1 in read_heater_input().
    // Set H2 and H3 to 0% to prevent them from running.
    tpo_set_percent(1, 0.0f);
    tpo_set_percent(2, 0.0f);
    return;
  }

  // Master heating switch (from UI "start" button) is OFF.
  // Turn off all heaters and reset PID integrals.
  if (!has_go_to) {
    for (int i = 0; i < NUM_THERMOCOUPLES; i++) {
      tpo_set_percent(i, 0);
      pid_integral[i] = 0.0f;
      pid_prev_err[i] = 0.0f;
    }
    return;
  }

  // --- Main PID Loop for all heaters ---
  // If has_go_to is TRUE, we loop through each heater.
  for (int i = 0; i < NUM_THERMOCOUPLES; i++) {

    // 1. Check if this specific heater is active (toggled on in UI)
    if (!config.heater_active[i]) {
      tpo_set_percent(i, 0);  // Not active, make sure it's off
      continue;               // Move to the next heater
    }

    // 2. Check for bad sensor
    if (isnan(tc_temp_display[i])) {
      tpo_set_percent(i, 0);  // Sensor fault, turn off
      continue;               // Move to the next heater
    }

    // --- LOGIC MOVED AND MODIFIED ---
    // 3a. NEW: Check for near max temp warning
    float max_temp_diff = config.max_temps[i] - tc_temp_display[i];
    if (max_temp_diff < NEAR_MAX_THRESHOLD_C)
    {
      if (!heater_near_max_state[i])
      {
        heater_near_max_state[i] = true;
        // Beep 3 times as a warning
        queue_beeps(3);
      }
    }
    else if (max_temp_diff > (NEAR_MAX_THRESHOLD_C + 2.0f)) // Hysteresis (2 degrees)
    {
      // We've moved away from the max temp, reset the state
      heater_near_max_state[i] = false;
    }

    // 3b. Safety Max Temp Cutoff (Original section 3)
    if (tc_temp_display[i] >= config.max_temps[i]) {
      tpo_set_percent(i, 0);
      pid_integral[i] = 0.0f;
      pid_prev_err[i] = 0.0f;

      // This heater hit its max. Disable it, start blink
      config.heater_active[i] = false;
      heater_cutoff_state[i] = true;
      heater_cutoff_start_time[i] = millis();
      heater_near_max_state[i] = false; // Reset state on cutoff

      // Check if any heaters are left active. If not, turn off master switch.
      bool any_active = false;
      for (int j = 0; j < NUM_THERMOCOUPLES; j++) {
        if (config.heater_active[j]) any_active = true;
      }
      if (!any_active) {
        has_go_to = false;
        go_to = NAN;  // Clear display temp
      }

      continue;  // Move to the next heater
    }
    // --- END OF MOVED LOGIC ---

    // 4. PID Calculation
    float error = config.target_temps[i] - tc_temp_display[i];
    const float DEADBAND = 0.1f;
    if (fabsf(error) < DEADBAND) error = 0.0f;

    if (Ki != 0.0f) {
      pid_integral[i] += error;
      const float INTEGRAL_CLAMP = 1000.0f;
      if (pid_integral[i] > INTEGRAL_CLAMP) pid_integral[i] = INTEGRAL_CLAMP;
      if (pid_integral[i] < -INTEGRAL_CLAMP) pid_integral[i] = -INTEGRAL_CLAMP;
    } else {
      pid_integral[i] = 0.0f;
    }

    float derivative = (error - pid_prev_err[i]);
    pid_prev_err[i] = error;

    float u = (Kp * error) + (Ki * pid_integral[i]) + (Kd * derivative);

    if (u < 0.0f) u = 0.0f;
    const float U_SCALE = 100.0f;
    float percent = u * (100.0f / U_SCALE);
    if (percent > 100.0f) percent = 100.0f;
    
    // --- OLD BEEP LOGIC REMOVED ---
    // float temp_error = fabsf(config.target_temps[i] - tc_temp_display[i]);
    // if (temp_error < NEAR_TARGET_THRESHOLD_C) ...
    // else if (temp_error > (NEAR_TARGET_THRESHOLD_C + 1.0f)) ...
    // --- END REMOVED LOGIC ---

    // 5. Set TPO percentage for this specific heater
    tpo_set_percent(i, percent);
  }
}


// ========== [13) Serial Plotter] ==========
void printForSerialPlotter() {
  float temp1 = tc_temp_display[0];
  float temp2 = tc_temp_display[1];
  float sp = has_go_to ? go_to : NAN;

  if (isnan(temp1)) Serial.print(0.0);
  else Serial.print(temp1, 2);
  Serial.print(",");
  if (isnan(temp2)) Serial.print(0.0);
  else Serial.print(temp2, 2);
  Serial.print(",");

  if (isnan(sp)) Serial.println(0.0);
  else Serial.println(sp, 2);
}

// ========== [14) Manual MAX31855 Driver Functions] ==========
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