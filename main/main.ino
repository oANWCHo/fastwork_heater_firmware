/****************************************************
 *  Project: ESP32 + MAX31855 + Rotary Encoder (PCNT) + ILI9341
 *  Feature: แสดงค่าจาก encoder และ switch ตรงกลางจอ TFT
 *           พื้นหลังสีดำ ตัวอักษรสีส้ม แยกคนละบรรทัด
 ****************************************************/

// ========== [1) Include Libraries] ==========
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include "driver/pcnt.h"          // ESP32 Pulse Counter (PCNT) สำหรับอ่าน Rotary Encoder
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

// ========== [2) Pin Definitions] ==========
// --- MAX31855 Thermocouple ---
#define MAXDO     19   // (แชร์กับ TFT_MISO)
#define MAXCS     5
#define MAXCLK    18   // (แชร์กับ TFT_CLK)

// --- Relay Output ---
#define RELAY_PIN 15

// --- Rotary Encoder (ใช้ PCNT) ---
#define ENCODER_A 36   // Data (Pulse)
#define ENCODER_B 39   // CLK (Control)
#define ENCODER_SW 34  // Push Switch

// --- TFT ILI9341 ---
#define TFT_DC   27
#define TFT_CS   26
#define TFT_MOSI 13
#define TFT_MISO 12
#define TFT_CLK  14
#define TFT_RST  4

// ========== [3) Objects Initialization] ==========
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);
Adafruit_ILI9341  tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

// ========== [4) Global Variables] ==========
// สำหรับ Serial monitor/ลอจิก
long lastEncoderValue = 0;
int  lastSwitchState  = HIGH;

// สำหรับค่าที่จะ "แสดงผลบนจอ"
int16_t encoder_count_display = 0;  // mapping จากค่าที่อ่านได้จริง current_count
int     switch_state_display  = HIGH;

// สีส้มสำหรับตัวอักษรบนจอ
uint16_t COLOR_ORANGE = 0;

// ฟังก์ชันแสดงผลบนจอ (ประกาศล่วงหน้าเพื่อเรียกใช้ใน setup/loop ได้)
unsigned long testText();

// ========== [5) Setup] ==========
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(1); }

  // --- 5.1 ตั้งค่า Switch ของ Encoder ---
  pinMode(ENCODER_SW, INPUT_PULLUP);

  // --- 5.2 ตั้งค่า PCNT สำหรับ Encoder ---
  pcnt_config_t pcnt_config = {};
  pcnt_config.pulse_gpio_num = ENCODER_A;     // ช่องสัญญาณนับพัลส์
  pcnt_config.ctrl_gpio_num  = ENCODER_B;     // ช่องสัญญาณ control (กำหนดทิศ)
  pcnt_config.channel        = PCNT_CHANNEL_0;
  pcnt_config.unit           = PCNT_UNIT_0;
  pcnt_config.pos_mode       = PCNT_COUNT_DEC;   // ขอบขาขึ้นให้ DECREMENT
  pcnt_config.neg_mode       = PCNT_COUNT_INC;   // ขอบขาลงให้ INCREMENT
  pcnt_config.lctrl_mode     = PCNT_MODE_REVERSE;// เมื่อ control = LOW ให้ reverse
  pcnt_config.hctrl_mode     = PCNT_MODE_KEEP;   // เมื่อ control = HIGH ให้ keep
  pcnt_unit_config(&pcnt_config);

  // ฟิลเตอร์ฮาร์ดแวร์ (ดีบาวน์)
  pcnt_set_filter_value(PCNT_UNIT_0, 1000);
  pcnt_filter_enable(PCNT_UNIT_0);

  // เริ่มตัวนับ
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);

  // --- 5.3 ตั้งค่า Relay ---
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  Serial.println("MAX31855 & Hardware Encoder Test");
  Serial.println("Type 'ON' or 'OFF' to control the relay.");

  // --- 5.4 (ตัวอย่าง) MAX31855 เริ่มใช้งาน (คอมเมนต์ไว้หากยังไม่ได้ต่อจริง) ---
  delay(500);
  Serial.print("Initializing sensor...");
  if (!thermocouple.begin()) {
    Serial.println("ERROR.");
    while (1) delay(10);
  }
  Serial.println("DONE.");
  

  // --- 5.5 ตั้งค่า TFT ---
  tft.begin();
  tft.setRotation(0);  // หมุนจอ 0 องศา (240x320)
  tft.fillScreen(ILI9341_BLACK);

  // กำหนดค่าสีส้ม (RGB 255,165,0)
  COLOR_ORANGE = tft.color565(255, 165, 0);

  // วาดครั้งแรก
  testText();
}

// ========== [6) Main Loop] ==========
void loop() {
  read_heater_input();   // รับคำสั่ง ON/OFF จาก Serial -> คุมรีเลย์
  heater_read();         // อ่านอุณหภูมิจาก MAX31855 (ถ้าต่อใช้งาน)
  read_hardware_encoder();// อ่านค่าพัลส์ encoder ผ่าน PCNT
  check_encoder_switch(); // อ่านสถานะปุ่ม encoder

  testText();             // อัปเดตจอเมื่อค่ามีการเปลี่ยน
  delay(5);
}

// ========== [7) Heater / Thermocouple Section] ==========
void heater_read() {
  // อ่านอุณหภูมิแบบ Celsius (หากยังไม่ได้ต่อ IC จะได้ NAN)
  double c = thermocouple.readCelsius();
  Serial.print("C = ");
  Serial.println(c);
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
    } else {
      Serial.println("Unknown command. Please use 'ON' or 'OFF'.");
    }
  }
}

// ========== [8) Encoder (PCNT) Section] ==========
// อ่านค่าจาก PCNT แล้วเก็บลง encoder_count_display เพื่อไปโชว์บนจอ
void read_hardware_encoder() {
  int16_t current_count = 0;
  pcnt_get_counter_value(PCNT_UNIT_0, &current_count);

  // อัปเดตค่าที่จะแสดงบนจอ
  encoder_count_display = current_count;

  // แสดงใน Serial เมื่อมีการเปลี่ยน
  if (current_count != lastEncoderValue) {
    Serial.print("Encoder Value: ");
    Serial.println(current_count);
    lastEncoderValue = current_count;
  }
}

// อ่านสถานะปุ่มของเอนโค้ดเดอร์ และอัปเดต switch_state_display เพื่อไปโชว์บนจอ
void check_encoder_switch() {
  int currentSwitchState = digitalRead(ENCODER_SW);

  // อัปเดตค่าที่จะแสดงบนจอ
  switch_state_display = currentSwitchState;

  // ตรวจจับการกด (เปลี่ยนจาก HIGH -> LOW)
  if (lastSwitchState == HIGH && currentSwitchState == LOW) {
    Serial.println("Encoder Switch Pressed!");
    // ตย. การทำงานเมื่อกดปุ่ม (เช่น reset counter)
    // pcnt_counter_clear(PCNT_UNIT_0);

    delay(50); // debounce ง่าย ๆ
  }

  lastSwitchState = currentSwitchState;
}

// ========== [9) TFT Display Section] ==========
// แสดงผล 2 บรรทัด: ENC: <count> และ SW: PRESSED/RELEASED
// จัดวางให้อยู่กลางจอ พื้นดำ ตัวอักษรสีส้ม
// ใช้ข้อความความยาวคงที่ + ไม่ fillRect/fillScreen ระหว่างอัปเดต
unsigned long testText() {
  static bool ui_inited = false;
  static int yLine1, yLine2, xLine1, xLine2;
  static int16_t prev_count  = INT16_MIN;
  static int     prev_switch = -1;

  // ---- ตั้งค่าฟอนต์/ระยะห่าง ----
  const uint8_t TXT_SIZE = 3;
  const int     GAP      = 8;

  // ---- ใช้ "แม่แบบความยาวคงที่" เพื่อจัดกลางครั้งเดียว ----
  // แม่แบบบรรทัด 1: fix ความกว้างเผื่อเลข 6 หลัก (รวมเครื่องหมาย)
  const char* L1_TEMPLATE = "ENC: -123456";      // ยาวคงที่
  // แม่แบบบรรทัด 2: เลือกอันที่ยาวสุด แล้วทำให้อีกอันยาวเท่ากันด้วยช่องว่าง
  const char* L2_TEMPLATE = "SW: RELEASED";      // ยาวสุด

  if (!ui_inited) {
    tft.setTextWrap(false);
    tft.setTextSize(TXT_SIZE);
    tft.setTextColor(COLOR_ORANGE, ILI9341_BLACK); // วาดทับพร้อมพื้นหลังดำทุกตัวอักษร

    // จัดกึ่งกลางแนวตั้งจากความสูงฟอนต์
    const int CHAR_H = 8 * TXT_SIZE;
    int screenH = tft.height();
    int totalH  = CHAR_H + GAP + CHAR_H;
    int yStart  = (screenH - totalH) / 2;
    yLine1 = yStart;
    yLine2 = yStart + CHAR_H + GAP;

    // คำนวณ X กลาง "ครั้งเดียว" จากขนาดแม่แบบ (ความยาวคงที่)
    int16_t bx, by; uint16_t bw, bh;
    int screenW = tft.width();

    tft.getTextBounds(L1_TEMPLATE, 0, 0, &bx, &by, &bw, &bh);
    xLine1 = (screenW - (int)bw) / 2;

    tft.getTextBounds(L2_TEMPLATE, 0, 0, &bx, &by, &bw, &bh);
    xLine2 = (screenW - (int)bw) / 2;

    ui_inited = true;
  }

  // อัปเดตเฉพาะเมื่อค่าจริงเปลี่ยน
  if (encoder_count_display != prev_count || switch_state_display != prev_switch) {
    prev_count  = encoder_count_display;
    prev_switch = switch_state_display;

    // ---------- บรรทัด 1: ใช้ความกว้างคงที่ ----------
    // L1_TEMPLATE = "ENC: -123456" (ความยาวคงที่ 12 ตัว)
    // ใช้รูปแบบกว้างคงที่ 7 หลักหลัง "ENC: " (รวมเครื่องหมาย)
    char l1[16];
    snprintf(l1, sizeof(l1), "ENC: %7d", (int)encoder_count_display); // รวมช่องว่างให้กว้างคงที่

    // ---------- บรรทัด 2: ทำความยาวคงที่ให้เท่ากับ L2_TEMPLATE ----------
    // L2_TEMPLATE = "SW: RELEASED" (ยาว 12)
    char l2[16];
    if (switch_state_display == LOW) {
      // "SW: PRESSED" = 11 ตัว เพิ่มช่องว่างท้ายให้ครบ 12
      strcpy(l2, "SW: PRESSED ");
    } else {
      strcpy(l2, "SW: RELEASED");
    }

    // ---------- วาดทับโดยไม่มีการ fill ใด ๆ ----------
    tft.setCursor(xLine1, yLine1);
    tft.print(l1);  // ความยาวคงที่ → พิมพ์ทับเคส 1 หลัก/2 หลักได้พอดี ไม่มีซ้อน

    tft.setCursor(xLine2, yLine2);
    tft.print(l2);  // ความยาวคงที่เท่ากันเสมอ → ไม่ต้องเคลียร์ก่อน
  }
  return 0;
}