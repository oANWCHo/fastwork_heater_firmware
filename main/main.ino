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

// ========== [2) Pin Definitions] ==========
// SPI Pins for MAX31855
#define MAXDO   13   // MISO
#define MAXCLK  12   // SCK
#define MAXCS1  15   // CS1
#define MAXCS2  16   // CS2
#define MAXCS3  17   // CS3
#define TFT_CS  10   // Guard Pin

#define ENCODER_A 6
#define ENCODER_B 7
#define ENCODER_SW 8

#define SDA_PIN 4
#define SCL_PIN 5
#define BUZZER 41
#define PCF_ADDR 0x20 

#define IR1_ADDR 0x10
#define IR2_ADDR 0x11

// --- SSR Output (Burst-Fire) ---
#define SSR_PIN1 42
#define SSR_PIN2 2
#define SSR_PIN3 1

#define WINDOW_MS_INIT 1000
#ifndef MAX_GUARD_US
#define MAX_GUARD_US 50
#endif

// ========== [3) Objects & Configuration] ==========
static const int TC_CS_PINS[] = { MAXCS1, MAXCS2, MAXCS3 };
const int NUM_THERMOCOUPLES = 3;

// Configure SPI for MAX31855: 1MHz, MSB First, Mode 0
SPISettings maxSettings(1000000, MSBFIRST, SPI_MODE0);

TFT_eSPI tft = TFT_eSPI();

Preferences preferences;
void saveConfig(const ConfigState& config);
UIManager ui(&tft, saveConfig);

ConfigState config;
Adafruit_MLX90614 mlx1 = Adafruit_MLX90614();
Adafruit_MLX90614 mlx2 = Adafruit_MLX90614();
Ticker tpo_ticker;

// ========== [4) Runtime Variables] ==========
long lastEncoderValue = 0;
int lastSwitchState = HIGH;
uint32_t last_encoder_read = 0;
const uint32_t ENCODER_READ_INTERVAL = 10; 
const int ENCODER_COUNTS_PER_STEP = 1; 

float tc_temp_display[NUM_THERMOCOUPLES];
float cj_temp_display[NUM_THERMOCOUPLES];
uint8_t tc_fault_bits[NUM_THERMOCOUPLES];

enum ClickState { IDLE, AWAITING_SECOND_CLICK };
ClickState click_state = IDLE;
uint32_t first_click_time = 0;
const uint32_t DOUBLE_CLICK_WINDOW_MS = 400;

float ir1_temp_display = NAN;
float ir2_temp_display = NAN;
float go_to = NAN;
bool has_go_to = false;

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
bool heater_cutoff_state[NUM_THERMOCOUPLES] = { false, false, false };
uint32_t heater_cutoff_start_time[NUM_THERMOCOUPLES] = { 0, 0, 0 };
const uint32_t CUTOFF_BLINK_DURATION_MS = 10000;

bool heater_near_max_state[NUM_THERMOCOUPLES] = { false, false, false };
const float NEAR_MAX_THRESHOLD_C = 5.0f;

// PCF8574 State Tracking
struct PCF_State {
  bool btn_pressed[4] = {false, false, false, false};
  bool led_state[4] = {false, false, false, false};
  uint32_t last_press[4] = {0, 0, 0, 0};
  const uint32_t DEBOUNCE_MS = 50;
};
PCF_State pcf_state;

// ========== [5) TPO Logic (ISR)] ==========
portMUX_TYPE tpoMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t tpo_window_period_ticks = WINDOW_MS_INIT;
volatile uint32_t tpo_on_ticks[NUM_THERMOCOUPLES] = { 0, 0, 0 };
volatile uint32_t tpo_tick_counter = 0;

void tpo_set_percent(int index, float percent) {
  if (index < 0 || index >= NUM_THERMOCOUPLES) return;
  percent = constrain(percent, 0, 100);
  last_pwm_percent[index] = percent;
  uint32_t onTicks = (uint32_t)((percent / 100.0f) * (float)tpo_window_period_ticks + 0.5f);
  portENTER_CRITICAL(&tpoMux);
  tpo_on_ticks[index] = onTicks;
  portEXIT_CRITICAL(&tpoMux);
}

void IRAM_ATTR tpo_isr() {
  uint32_t pos;
  uint32_t onTicks[NUM_THERMOCOUPLES];
  portENTER_CRITICAL_ISR(&tpoMux);
  pos = tpo_tick_counter;
  for (int i = 0; i < NUM_THERMOCOUPLES; i++) onTicks[i] = tpo_on_ticks[i];
  tpo_tick_counter++;
  if (tpo_tick_counter >= tpo_window_period_ticks) tpo_tick_counter = 0;
  portEXIT_CRITICAL_ISR(&tpoMux);
  digitalWrite(SSR_PIN1, (pos < onTicks[0]) ? HIGH : LOW);
  digitalWrite(SSR_PIN2, (pos < onTicks[1]) ? HIGH : LOW);
  digitalWrite(SSR_PIN3, (pos < onTicks[2]) ? HIGH : LOW);
}

void queue_beeps(int count) {
  if (beeper_active || !config.sound_on) return;
  beep_queue = count * 2;
  t_next_beep_action = millis();
  beeper_active = true;
}

// ========== [6) PCF8574 Individual Control Logic] ==========
void handle_pcf8574_fixed() {
  static uint32_t last_pcf_read = 0;
  uint32_t now = millis();
  if (now - last_pcf_read < 50) return;
  last_pcf_read = now;

  Wire.requestFrom(PCF_ADDR, 1);
  if (!Wire.available()) return;
  uint8_t input_data = Wire.read();
  bool btn_current[4] = {
    !(input_data & 0x01), // P0
    !(input_data & 0x02), // P1
    !(input_data & 0x04), // P2
    !(input_data & 0x08)  // P3
  };
  for (int i = 0; i < 4; i++) {
    if (btn_current[i] && !pcf_state.btn_pressed[i]) {
      if (now - pcf_state.last_press[i] > pcf_state.DEBOUNCE_MS) {
        pcf_state.led_state[i] = !pcf_state.led_state[i];
        pcf_state.last_press[i] = now;
        queue_beeps(1);
        
        switch(i) {
          case 0: config.heater_active[0] = !config.heater_active[0];
                  saveConfig(config); break;
          case 1: config.heater_active[1] = !config.heater_active[1]; saveConfig(config); break;
          case 2: config.heater_active[2] = !config.heater_active[2]; saveConfig(config); break;
          case 3: 
            if (!has_go_to) ui.handleButtonSingleClick(config, go_to, has_go_to);
            else { has_go_to = false; go_to = NAN; }
            break;
        }
      }
    }
    pcf_state.btn_pressed[i] = btn_current[i];
  }

  uint8_t led_output = 0xF0; 
  if (config.heater_active[0]) led_output &= ~(1 << 4);
  if (config.heater_active[1]) led_output &= ~(1 << 5);
  if (config.heater_active[2]) led_output &= ~(1 << 6);
  if (has_go_to)               led_output &= ~(1 << 7);
  Wire.beginTransmission(PCF_ADDR);
  Wire.write(led_output | 0x0F);
  Wire.endTransmission();
}

// ========== [7) Forward Declarations] ==========
void updateControl();
void heater_read();
void read_mlx_sensors();
void read_hardware_encoder();
void check_encoder_switch();
void max31855_init_pins();
uint32_t max31855_read_raw(int csPin);
bool max31855_read_celsius(int csPin, float* tc, float* tj, uint8_t* fb, uint32_t* raw);
void loadConfig(ConfigState& cfg);
void setup_encoder_fixed();

// ========== [8) Setup] ==========
void setup() {
  Serial.begin(115200);
  delay(100);
  
  esp_task_wdt_config_t wdt_config = { .timeout_ms = 10000, .idle_core_mask = 0, .trigger_panic = true };
  esp_task_wdt_reconfigure(&wdt_config);
  esp_task_wdt_add(NULL);

  preferences.begin("app_config", false);
  loadConfig(config);

  pinMode(SSR_PIN1, OUTPUT); digitalWrite(SSR_PIN1, LOW);
  pinMode(SSR_PIN2, OUTPUT); digitalWrite(SSR_PIN2, LOW);
  pinMode(SSR_PIN3, OUTPUT); digitalWrite(SSR_PIN3, LOW);
  pinMode(BUZZER, OUTPUT);   digitalWrite(BUZZER, LOW);
  
  // Guard pin to prevent TFT SPI conflict
  pinMode(TFT_CS, OUTPUT);   digitalWrite(TFT_CS, HIGH);
  pinMode(ENCODER_SW, INPUT_PULLUP);

  setup_encoder_fixed();
  esp_task_wdt_reset();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  esp_task_wdt_reset();

  mlx1.begin(IR1_ADDR, &Wire);
  mlx2.begin(IR2_ADDR, &Wire);

  // Initialize Hardware SPI for MAX31855
  // Note: We use -1 for MOSI since MAX31855 is Read-Only
  SPI.begin(MAXCLK, MAXDO, -1, -1);
  max31855_init_pins(); // Just sets up CS pins now

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  esp_task_wdt_reset();

  ui.begin(); 
  esp_task_wdt_reset();

  tpo_ticker.attach_ms(1, tpo_isr);
  Serial.println("System Ready.");
}

// ========== [9) Main Loop] ==========
void loop() {
  esp_task_wdt_reset();
  handle_pcf8574_fixed();

  if (beeper_active && millis() >= t_next_beep_action) {
    if (beep_queue > 0) {
      digitalWrite(BUZZER, (beep_queue % 2 == 0));
      t_next_beep_action = millis() + (beep_queue % 2 == 0 ? BEEP_ON_MS : BEEP_OFF_MS);
      beep_queue--;
    } else { 
      digitalWrite(BUZZER, LOW);
      beeper_active = false; 
    }
  }

  if (click_state == AWAITING_SECOND_CLICK && millis() - first_click_time > DOUBLE_CLICK_WINDOW_MS) {
    ui.handleButtonSingleClick(config, go_to, has_go_to);
    click_state = IDLE;
  }

  static uint32_t t_sens = 0, t_disp = 0, t_ctl = 0;
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
    AppState st;
    for (int i = 0; i < 3; i++) {
      st.tc_temps[i] = tc_temp_display[i];
      st.tc_faults[i] = tc_fault_bits[i];
      st.heater_cutoff_state[i] = heater_cutoff_state[i];
    }
    st.ir_temps[0] = ir1_temp_display;
    st.ir_temps[1] = ir2_temp_display;
    st.is_heating_active = has_go_to;
    st.target_temp = go_to;
    st.temp_unit = config.temp_unit;
    ui.draw(st, config);
  }

  read_hardware_encoder();
  check_encoder_switch();
}

// ========== [10) Sub-functions Implementation] ==========

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

void read_hardware_encoder() {
  uint32_t now = millis();
  if (now - last_encoder_read < ENCODER_READ_INTERVAL) return;
  last_encoder_read = now;
  int16_t val = 0;
  if (pcnt_get_counter_value(PCNT_UNIT_0, &val) == ESP_OK) {
    if (abs(val) > 30000) { 
      pcnt_counter_clear(PCNT_UNIT_0);
      lastEncoderValue = 0; return; 
    }
    long delta = (long)val - lastEncoderValue;
    if (abs(delta) >= ENCODER_COUNTS_PER_STEP) {
      ui.handleEncoderRotation((float)delta / ENCODER_COUNTS_PER_STEP, config);
      lastEncoderValue = val;
    }
  }
}

void check_encoder_switch() {
  int cur = digitalRead(ENCODER_SW);
  if (lastSwitchState == HIGH && cur == LOW) {
    queue_beeps(1);
    if (click_state == IDLE) { 
      first_click_time = millis(); 
      click_state = AWAITING_SECOND_CLICK; 
    } else { 
      ui.handleButtonDoubleClick(config);
      click_state = IDLE; 
    }
  }
  lastSwitchState = cur;
}

void heater_read() {
  for (int i = 0; i < NUM_THERMOCOUPLES; ++i) {
    float tc, tj;
    uint8_t fb; 
    uint32_t raw;
    if (max31855_read_celsius(TC_CS_PINS[i], &tc, &tj, &fb, &raw)) {
      tc_temp_display[i] = tc + config.tc_offsets[i];
    } else {
      tc_temp_display[i] = NAN;
    }
    tc_fault_bits[i] = fb;
  }
}

void read_mlx_sensors() {
  ir1_temp_display = mlx1.readObjectTempC();
  ir2_temp_display = mlx2.readObjectTempC();
}

void updateControl() {
  if (!has_go_to) { 
    for (int i=0; i<3; i++) tpo_set_percent(i, 0); 
    return;
  }
  for (int i = 0; i < NUM_THERMOCOUPLES; i++) {
    if (!config.heater_active[i] || isnan(tc_temp_display[i])) { 
      tpo_set_percent(i, 0); 
      continue; 
    }
    if (tc_temp_display[i] >= config.max_temps[i]) {
      tpo_set_percent(i, 0); 
      config.heater_active[i] = false;
      heater_cutoff_state[i] = true; 
      heater_cutoff_start_time[i] = millis();
      continue;
    }
    float err = config.target_temps[i] - tc_temp_display[i];
    pid_integral[i] = constrain(pid_integral[i] + err, -500, 500);
    float u = (Kp * err) + (Ki * pid_integral[i]);
    tpo_set_percent(i, u);
  }
}

void max31855_init_pins() {
  // CLK and DO are handled by SPI.begin()
  // We only need to configure CS pins
  for (int i = 0; i < NUM_THERMOCOUPLES; i++) {
    pinMode(TC_CS_PINS[i], OUTPUT);
    digitalWrite(TC_CS_PINS[i], HIGH);
  }
}

uint32_t max31855_read_raw(int csPin) {
  uint32_t v = 0;
  
  // Guard against TFT SPI conflict
  digitalWrite(TFT_CS, HIGH); 
  
  // Start Hardware SPI Transaction
  SPI.beginTransaction(maxSettings);
  digitalWrite(csPin, LOW);
  
  // Transfer dummy byte 0x00 to push clock and read MISO
  v |= ((uint32_t)SPI.transfer(0x00)) << 24;
  v |= ((uint32_t)SPI.transfer(0x00)) << 16;
  v |= ((uint32_t)SPI.transfer(0x00)) << 8;
  v |= ((uint32_t)SPI.transfer(0x00));
  
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  
  return v;
}

bool max31855_read_celsius(int csPin, float* tc, float* tj, uint8_t* fb, uint32_t* raw_out) {
  uint32_t raw = max31855_read_raw(csPin);
  if (raw_out) *raw_out = raw;
  if (raw == 0 || raw == 0xFFFFFFFF) return false;
  
  int32_t tc14 = (int32_t)((raw >> 18) & 0x3FFF);
  if (raw & 0x20000000) tc14 -= 16384;
  
  // POLARITY FIX: Change 0.25f to -0.25f here if you cannot swap wires
  *tc = (float)tc14 * -0.25f; 
  
  int32_t tj12 = (int32_t)((raw >> 4) & 0xFFF);
  if (raw & 0x8000) tj12 -= 4096;
  *tj = (float)tj12 * 0.0625f;

  *fb = (uint8_t)(raw & 0x07);
  return !(raw & 0x10000);
}

void saveConfig(const ConfigState& cfg) { 
  preferences.putBytes("config", &cfg, sizeof(cfg));
}

void loadConfig(ConfigState& cfg) { 
  if (preferences.getBytesLength("config") == sizeof(cfg)) 
    preferences.getBytes("config", &cfg, sizeof(cfg));
}

void read_heater_input() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n'); 
    cmd.trim();
    if (cmd.startsWith("PWM")) { 
      pwm_override_enabled = true; 
      pwm_override_percent = cmd.substring(3).toFloat(); 
    }
    if (cmd == "AUTO") pwm_override_enabled = false;
  }
}