/*
  MAX31855 Manual Reader (ESP32)
  - Read multiple MAX31855 using shared SCK/DO and separate CS
  - Pure bit-bang (no external libraries)
  - Guard time between chips, interrupts disabled while shifting 32 bits

  Wiring (example):
    SCK (MAXCLK)  -> GPIO18
    DO  (MAXDO)   -> GPIO19
    CS1 (MAXCS1)  -> GPIO5
    CS2 (MAXCS2)  -> GPIO33
    VCC 3.3V, GND common
*/

#include <Arduino.h>

// ====== Pin config (EDIT to your board) ======
#define MAXCLK   18    // SCK shared
#define MAXDO    19    // DO / MISO shared
#define MAXCS1    5    // CS for sensor #0
#define MAXCS2   33    // CS for sensor #1

// Add more CS pins here if you have >2 sensors
static const int TC_CS_PINS[] = { MAXCS1, MAXCS2 };
const int NUM_THERMOCOUPLES = sizeof(TC_CS_PINS) / sizeof(TC_CS_PINS[0]);

// ====== Timing config ======
#ifndef MAX_GUARD_US
#define MAX_GUARD_US 50   // gap between CS changes; raise to 30–50 if edges look too tight
#endif

// ====== Storage ======
float tc_temp_display[8];       // thermocouple temperature (°C) — size >= NUM_THERMOCOUPLES
float cj_temp_display[8];       // cold-junction temperature (°C)
uint8_t tc_fault_bits[8];       // fault summary per channel (bit7: global fault flag, bit2..0: SCV/SCG/OC)

// ====== Helpers ======
static inline int32_t sign_extend(int32_t v, uint8_t nbits) {
  int32_t shift = 32 - nbits;
  return (v << shift) >> shift;
}

void max31855_init_pins() {
  pinMode(MAXCLK, OUTPUT);
  pinMode(MAXDO, INPUT_PULLUP);
  digitalWrite(MAXCLK, LOW);          // CPOL=0 idle LOW

  for (int i = 0; i < NUM_THERMOCOUPLES; ++i) {
    pinMode(TC_CS_PINS[i], OUTPUT);
    digitalWrite(TC_CS_PINS[i], HIGH); // idle = not selected
  }
}

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

  int32_t tj12 = (raw >> 4) & 0x0FFF;    // 12-bit
  tj12 = sign_extend(tj12, 12);
  float tj = (float)tj12 * 0.0625f;

  uint8_t fbits = (uint8_t)(raw & 0x07);
  if (fault) fbits |= 0x80; // mark global fault in bit7

  if (tc_c) *tc_c = tc;
  if (tj_c) *tj_c = tj;
  if (faults) *faults = fbits;

  return !fault;
}

// Read one channel by CS pin
bool max31855_read_celsius(int csPin, float* tc_c, float* tj_c, uint8_t* faults, uint32_t* raw_out=nullptr) {
  for (int attempt = 0; attempt < 3; ++attempt) {
    uint32_t raw = max31855_read_raw(csPin);
    if (raw_out) *raw_out = raw;

    // กรองเฟรมเพี้ยน (ลอย/อ่านพลาด)
    if (raw == 0x00000000UL || raw == 0xFFFFFFFFUL) {
      delayMicroseconds(MAX_GUARD_US);
      continue;  // ลองใหม่
    }

    return max31855_decode(raw, tc_c, tj_c, faults);
  }
  // ถ้าอ่าน 3 ครั้งยังไม่ได้ ให้ส่งค่าคงเดิม/NaN ตามต้องการ
  if (tc_c) *tc_c = NAN;
  if (tj_c) *tj_c = NAN;
  if (faults) *faults = 0xFF;
  return false;
}
// ====== Public API: read all sensors safely and store to arrays ======
void heater_read_manual() {
  for (int i = 0; i < NUM_THERMOCOUPLES; ++i) {
    float tc, tj;
    uint8_t fb;
    uint32_t raw;

    // Retry a couple times if you want to be extra robust
    bool ok = false;
    for (int attempt = 0; attempt < 2 && !ok; ++attempt) {
      ok = max31855_read_celsius(TC_CS_PINS[i], &tc, &tj, &fb, &raw);
      if (!ok) delayMicroseconds(MAX_GUARD_US);
    }

    tc_temp_display[i] = tc;
    cj_temp_display[i] = tj;
    tc_fault_bits[i]   = fb;

    // Debug print
    Serial.printf("TC[%d] RAW=0x%08lX  T=%.2f°C  CJ=%.2f°C  Fault=0x%02X%s\n",
                  i, raw, tc, tj, fb, ok ? "" : " (FAULT)");
  }
}

// ====== Arduino entry points ======
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nMAX31855 Manual Reader (no library)");
  max31855_init_pins();

  // Optional: quick sanity pulse to see pins alive
  all_cs_idle();
}

void loop() {
  heater_read_manual();
  delay(200);                 // set your polling interval here
}
