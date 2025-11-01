#pragma once

#include "OvenData.h"
#include <TFT_eSPI.h>

// ========== [UI State Variables] ==========
UI_State current_state = STATE_STANDBY;
int main_menu_cursor = 0;   // ตำแหน่งเมนูที่เลือก
int sub_menu_cursor = 0;    // ตำแหน่งเมนูย่อย
float temp_edit_value = 0;  // ค่าชั่วคราวขณะกำลังแก้ไข
uint32_t last_activity_time = 0;
const uint32_t IDLE_TIMEOUT_MS = 10000;  // 10 วินาที [cite: 17]
static int      enc_accumulator = 0;
static uint32_t enc_last_step_ms = 0;
static int enc_partial_ticks = 0;

// รายการเมนูหลัก (รวม Page 2 และ 3)
const char* main_menu_items[] = {
  "Heater 1",            // [cite: 18]
  "Heater 2",            // [cite: 19]
  "Heater 3",            // [cite: 20]
  "Max Temp Lock",       // [cite: 26]
  "Turn off When Idle",  // [cite: 27]
  "Light and Sound",     // [cite: 28]
  "Temp Unit",           // [cite: 29]
  "About"                // [cite: 30]
};
const int MAIN_MENU_COUNT = sizeof(main_menu_items) / sizeof(main_menu_items[0]);

// --- Encoder detent normalization ---
#ifndef ENC_PULSES_PER_DETENT
// Common encoders: 2 or 4 electrical pulses per physical detent
#define ENC_PULSES_PER_DETENT 4
#endif

// ========== [UI Helper Functions] ==========
// Convert raw ticks to whole detents; keeps remainder so 1 click is never lost
static int consume_encoder_detents(int raw_ticks) {
  enc_partial_ticks += raw_ticks;
  int det = enc_partial_ticks / ENC_PULSES_PER_DETENT;  // floor toward 0
  enc_partial_ticks -= det * ENC_PULSES_PER_DETENT;
  return det; // may be -N, 0, or +N
}

// Light/Sound helpers: 0=Off, 1=Light only, 2=Sound only, 3=Both
static inline int lightSoundToIndex(const OvenSettings& s) {
  if (!s.light_on && !s.sound_on) return 0;
  if ( s.light_on && !s.sound_on) return 1;
  if (!s.light_on &&  s.sound_on) return 2;
  return 3; // both
}

static inline void indexToLightSound(int idx, OvenSettings& s) {
  switch (idx) {
    case 0: s.light_on = false; s.sound_on = false; break;
    case 1: s.light_on = true;  s.sound_on = false; break;
    case 2: s.light_on = false; s.sound_on = true;  break;
    default:s.light_on = true;  s.sound_on = true;  break;
  }
}

// ฟังก์ชันช่วยวาดรายการเมนู
void drawMenuItem(TFT_eSprite& spr, int y, const char* text, bool selected) {
  if (selected) {
    spr.fillRect(0, y - 2, spr.width(), 20, TFT_YELLOW);
    spr.setTextColor(TFT_BLACK);
    spr.drawString(text, 10, y);
  } else {
    spr.fillRect(0, y - 2, spr.width(), 20, TFT_BLACK);
    spr.setTextColor(TFT_WHITE);
    spr.drawString(text, 10, y);
  }
}

// ฟังก์ชันช่วยวาดหน้าจอแก้ไขค่า (แบบทั่วไป)
void drawValueEditScreen(TFT_eSprite& spr, const char* title, float value, const char* unit) {
  spr.fillSprite(TFT_BLACK);
  spr.setTextSize(1);
  spr.setTextColor(TFT_WHITE);
  spr.drawString(title, 10, 20);

  spr.setTextSize(2);
  spr.setTextColor(TFT_YELLOW);
  spr.drawFloat(value, 1, 60, 80);
  spr.drawString(unit, 160, 80);

  spr.setTextSize(1);
  spr.setTextColor(TFT_SILVER);
  spr.drawString("Knob: Adjust", 10, 200);
  spr.drawString("Click: Confirm", 10, 215);
  spr.drawString("Dbl-Click: Back", 10, 230);
}

// ========== [Screen Drawing Functions] ==========
// Put this near your UI helpers:
static inline float toDisplayTemp(float c, bool useC) {
  if (isnan(c)) return c;
  return useC ? c : (c * 9.0f/5.0f + 32.0f);
}
static inline const char* unitStr(bool useC) { return useC ? "C" : "F"; }

// Replace your draw_standby(...) with this improved layout:
void draw_standby(TFT_eSprite& spr, SensorData& sensors, OvenSettings& settings) {
  spr.fillSprite(TFT_BLACK);
  spr.setTextSize(1);

  int y = 10;
  const int labelX = 10;
  const int valueX = 180;   // move numbers to a fixed right column

  auto drawRow = [&](const char* label, float val, bool highlight, bool haveSensor) {
    uint16_t color = highlight ? TFT_YELLOW : TFT_WHITE;
    spr.setTextColor(color);
    spr.drawString(label, labelX, y);
    spr.setTextColor(TFT_WHITE);

    spr.setCursor(valueX, y);
    if (haveSensor && !isnan(val)) {
      float v = toDisplayTemp(val, settings.unit_is_celsius);
      spr.printf("%.2f %s", v, unitStr(settings.unit_is_celsius));
    } else {
      spr.print("--- ");
      spr.print(unitStr(settings.unit_is_celsius));
    }
    y += 30;
  };

  // H1/H2/H3/IR
  drawRow("Heater 1 Temp :", sensors.TC1, settings.heater_running[0], true);
  drawRow("Heater 2 Temp :", sensors.TC2, settings.heater_running[1], true);
  drawRow("Heater 3 Temp :", NAN,          settings.heater_running[2], false);
  drawRow("IR Temp :",       sensors.IR_Avg, false,                    true);

  // Alert example
  spr.setTextSize(1);
  spr.setTextColor(TFT_ORANGE);
  if (!isnan(sensors.TC1) && sensors.TC1 > 270.0f) {
    spr.drawString("H1 > 270C Alert!", 10, y + 10);
  }
}

// 2. วาดหน้า Main Menu (Page 2 & 3)
void draw_main_menu(TFT_eSprite& spr) {
  spr.fillSprite(TFT_BLACK);
  spr.setTextSize(1);

  for (int i = 0; i < MAIN_MENU_COUNT; i++) {
    drawMenuItem(spr, 20 + i * 25, main_menu_items[i], i == main_menu_cursor);
  }
}

// 3. วาดหน้าตั้งค่า Idle (Page 7)
void draw_idle_menu(TFT_eSprite& spr, OvenSettings& settings) {
  spr.fillSprite(TFT_BLACK);
  spr.setTextSize(1);

  const char* items[] = { "15 min", "30 min", "60 min", "Always ON" };  // [cite: 50, 51, 52, 53]

  // หาว่าค่าปัจจุบันคือ index ไหน
  int current_idx = 3;  // Default to "Always ON"
  if (settings.idle_off_minutes == 15) current_idx = 0;
  else if (settings.idle_off_minutes == 30) current_idx = 1;
  else if (settings.idle_off_minutes == 60) current_idx = 2;

  for (int i = 0; i < 4; i++) {
    drawMenuItem(spr, 20 + i * 25, items[i], i == sub_menu_cursor);
    if (i == current_idx) {
      spr.drawString("*", spr.width() - 20, 20 + i * 25);  // มาร์คค่าที่เลือกไว้
    }
  }
}

// (ฟังก์ชันวาดหน้าจออื่นๆ เช่น Light/Sound, Unit, About จะใช้หลักการคล้ายๆ กัน)
// ... (เพื่อความกระชับ ขอข้ามไปก่อน แต่หลักการจะเหมือน draw_idle_menu) ...
// Helper: dynamic step based on spin speed

static float encoderStepWithAccel(int delta) {
  if (delta == 0) return 0.0f;

  uint32_t now = millis();
  uint32_t dt  = now - enc_last_step_ms;
  enc_last_step_ms = now;

  // Faster spins -> larger step
  float step = 0.5f;           // base step (was 0.5)
  if (dt < 12)  step = 5.0f;   // ~>80 Hz tick -> very fast spin
  else if (dt < 25) step = 2.0f;
  else if (dt < 50) step = 1.0f;

  return step * (float)delta;
}

// ========== [Main UI Drawing Function] ==========

void ui_draw(TFT_eSprite& spr, SensorData& sensors, OvenSettings& settings) {

  spr.setSwapBytes(true);  // [cite: 109]

  switch (current_state) {
    case STATE_STANDBY:
      draw_standby(spr, sensors, settings);
      break;

    case STATE_MENU_MAIN:
      draw_main_menu(spr);
      break;

    case STATE_EDIT_H1_START:
      drawValueEditScreen(spr, "Heater 1 Start Temp", temp_edit_value, "C/F");  // [cite: 35]
      break;
    case STATE_EDIT_H1_MAX:
      drawValueEditScreen(spr, "Heater 1 Max Temp", temp_edit_value, "C/F");  // [cite: 40]
      break;
      // (เพิ่มเคสสำหรับ H2, H3)

    case STATE_EDIT_MAX_LOCK:
      drawValueEditScreen(spr, "Max Temp Lock", temp_edit_value, "C/F");  // [cite: 45]
      break;

    case STATE_EDIT_IDLE_OFF:
      draw_idle_menu(spr, settings);  // [cite: 49]
      break;

      // (เพิ่มเคสสำหรับหน้าจอที่เหลือ)

    default:
      spr.fillSprite(TFT_BLACK);
      spr.setTextSize(1);
      spr.setTextColor(TFT_RED);
      spr.drawString("Invalid UI State!", 20, 20);
  }

  spr.pushSprite(0, 0);  // [cite: 172]
}

// ========== [UI Input Handling Function] ==========

// Normalize encoder pulses -> physical detents and handle all UI input in one place.
void ui_handle_input(int encoder_delta_raw, ClickType click, OvenSettings& settings) {
  // ===== Helpers (local statics) =====
  #ifndef ENC_PULSES_PER_DETENT
  // Set to 2 or 4 depending on your decoder; 2 is common.
  #define ENC_PULSES_PER_DETENT 2
  #endif

  static int      enc_partial_ticks = 0;
  static uint32_t enc_last_step_ms  = 0;

  auto consume_encoder_detents = [&](int raw_ticks)->int {
    enc_partial_ticks += raw_ticks;
    int det = 0;
    if (enc_partial_ticks >= 0) {
      det = enc_partial_ticks / ENC_PULSES_PER_DETENT;
      enc_partial_ticks -= det * ENC_PULSES_PER_DETENT;
    } else {
      det = - ((-enc_partial_ticks) / ENC_PULSES_PER_DETENT);
      enc_partial_ticks -= det * ENC_PULSES_PER_DETENT;
    }
    return det; // may be negative, zero, positive
  };

  auto encoderStepWithAccel = [&](int detents)->float {
    if (detents == 0) return 0.0f;
    const uint32_t now = millis();
    const uint32_t dt  = now - enc_last_step_ms;
    enc_last_step_ms = now;

    // Base step for a single detent
    float step = 0.5f;

    // Simple acceleration based on time between detents
    if (dt < 12)       step = 5.0f;  // very fast spin
    else if (dt < 25)  step = 2.0f;
    else if (dt < 50)  step = 1.0f;
    // else keep base 0.5f

    return step * (float)detents;
  };

  auto clampf = [](float v, float lo, float hi)->float {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
  };

  // ===== 1) Convert raw encoder pulses → detents =====
  const int detents = consume_encoder_detents(encoder_delta_raw);

  // Any activity? keep the UI alive
  if (detents != 0 || click != CLICK_NONE) {
    last_activity_time = millis();
  }

  // ===== 2) Idle bounce-back to Standby =====
  if (current_state != STATE_STANDBY) {
    const uint32_t now = millis();
    if (now - last_activity_time > IDLE_TIMEOUT_MS) {
      current_state = STATE_STANDBY;
      return;
    }
  }

  // ===== 3) State machine =====
  switch (current_state) {

    // ---------- Standby (Page 1) ----------
    case STATE_STANDBY: {
      // Do NOT toggle anything on rotation; this lets you see single-detent movement in debug if needed.
      if (click == CLICK_SINGLE) {
        current_state    = STATE_MENU_MAIN;
        main_menu_cursor = 0;
        sub_menu_cursor  = 0;
      }
      // Optional quick action: double click to toggle Heater1 (commented; enable if you want)
      // else if (click == CLICK_DOUBLE) {
      //   settings.heater_running[0] = !settings.heater_running[0];
      // }
    } break;

    // ---------- Main Menu (Pages 2–3) ----------
    case STATE_MENU_MAIN: {
      // Move cursor with detents (wrap around)
      if (detents != 0) {
        // MAIN_MENU_COUNT must match your items array
        main_menu_cursor += detents;
        if (main_menu_cursor < 0) main_menu_cursor = MAIN_MENU_COUNT - 1;
        if (main_menu_cursor >= MAIN_MENU_COUNT) main_menu_cursor = 0;
      }

      if (click == CLICK_DOUBLE) {
        // Double click: back to Standby
        current_state = STATE_STANDBY;
      } else if (click == CLICK_SINGLE) {
        // Enter selected item. Map your menu indices to edit states.
        // Adjust indices to match your main_menu_items[]
        switch (main_menu_cursor) {
          case 0: // "Heater 1"
            sub_menu_cursor = 0;            // 0 = Start Temp, 1 = Max Temp
            temp_edit_value = settings.heater_start_temp[0];
            current_state   = STATE_EDIT_H1_START;
            break;

          case 1: // "Heater 2"
            sub_menu_cursor = 0;
            temp_edit_value = settings.heater_start_temp[1];
            current_state   = STATE_EDIT_H2_START;
            break;

          case 2: // "Heater 3"
            sub_menu_cursor = 0;
            temp_edit_value = settings.heater_start_temp[2];
            current_state   = STATE_EDIT_H3_START;
            break;

          case 3: // "Max Temp Lock"
            temp_edit_value = settings.max_temp_lock;
            current_state   = STATE_EDIT_MAX_LOCK;
            break;

          case 4: // "Turn off When Idle"
            // sub_menu_cursor selects 15/30/60/Always; initialize from current setting
            if      (settings.idle_off_minutes == 15) sub_menu_cursor = 0;
            else if (settings.idle_off_minutes == 30) sub_menu_cursor = 1;
            else if (settings.idle_off_minutes == 60) sub_menu_cursor = 2;
            else                                      sub_menu_cursor = 3; // Always ON
            current_state = STATE_EDIT_IDLE_OFF;
            break;

          case 5: // "Light and Sound"
            sub_menu_cursor = lightSoundToIndex(settings);
            current_state = STATE_EDIT_LIGHT_SOUND;
            break;

          // Add more cases if your menu has more items (Units, About, etc.)
          default:
            // Fallback: go back
            current_state = STATE_STANDBY;
            break;
        }
      }
    } break;

    // ---------- Heater 1: Start Temp ----------
    case STATE_EDIT_H1_START: {
      if (detents != 0) {
        temp_edit_value += encoderStepWithAccel(detents);
        temp_edit_value  = clampf(temp_edit_value, 0.0f, 450.0f); // safety range
      }
      if (click == CLICK_DOUBLE) {
        // cancel → main menu
        current_state = STATE_MENU_MAIN;
      } else if (click == CLICK_SINGLE) {
        // save
        settings.heater_start_temp[0] = temp_edit_value;
        // move to Max edit next for convenience
        temp_edit_value = settings.heater_max_temp[0];
        current_state   = STATE_EDIT_H1_MAX;
      }
    } break;

    // ---------- Heater 1: Max Temp ----------
    case STATE_EDIT_H1_MAX: {
      if (detents != 0) {
        temp_edit_value += encoderStepWithAccel(detents);
        temp_edit_value  = clampf(temp_edit_value, 0.0f, 450.0f);
      }
      if (click == CLICK_DOUBLE) {
        current_state = STATE_MENU_MAIN;
      } else if (click == CLICK_SINGLE) {
        settings.heater_max_temp[0] = temp_edit_value;
        current_state = STATE_MENU_MAIN;
      }
    } break;

    // ---------- Heater 2: Start Temp ----------
    case STATE_EDIT_H2_START: {
      if (detents != 0) {
        temp_edit_value += encoderStepWithAccel(detents);
        temp_edit_value  = clampf(temp_edit_value, 0.0f, 450.0f);
      }
      if (click == CLICK_DOUBLE) {
        current_state = STATE_MENU_MAIN;
      } else if (click == CLICK_SINGLE) {
        settings.heater_start_temp[1] = temp_edit_value;
        temp_edit_value = settings.heater_max_temp[1];
        current_state   = STATE_EDIT_H2_MAX;
      }
    } break;

    // ---------- Heater 2: Max Temp ----------
    case STATE_EDIT_H2_MAX: {
      if (detents != 0) {
        temp_edit_value += encoderStepWithAccel(detents);
        temp_edit_value  = clampf(temp_edit_value, 0.0f, 450.0f);
      }
      if (click == CLICK_DOUBLE) {
        current_state = STATE_MENU_MAIN;
      } else if (click == CLICK_SINGLE) {
        settings.heater_max_temp[1] = temp_edit_value;
        current_state = STATE_MENU_MAIN;
      }
    } break;

    // ---------- Heater 3: Start Temp ----------
    case STATE_EDIT_H3_START: {
      if (detents != 0) {
        temp_edit_value += encoderStepWithAccel(detents);
        temp_edit_value  = clampf(temp_edit_value, 0.0f, 450.0f);
      }
      if (click == CLICK_DOUBLE) {
        current_state = STATE_MENU_MAIN;
      } else if (click == CLICK_SINGLE) {
        settings.heater_start_temp[2] = temp_edit_value;
        temp_edit_value = settings.heater_max_temp[2];
        current_state   = STATE_EDIT_H3_MAX;
      }
    } break;

    // ---------- Heater 3: Max Temp ----------
    case STATE_EDIT_H3_MAX: {
      if (detents != 0) {
        temp_edit_value += encoderStepWithAccel(detents);
        temp_edit_value  = clampf(temp_edit_value, 0.0f, 450.0f);
      }
      if (click == CLICK_DOUBLE) {
        current_state = STATE_MENU_MAIN;
      } else if (click == CLICK_SINGLE) {
        settings.heater_max_temp[2] = temp_edit_value;
        current_state = STATE_MENU_MAIN;
      }
    } break;

    // ---------- Max Temp Lock ----------
    case STATE_EDIT_MAX_LOCK: {
      if (detents != 0) {
        temp_edit_value += encoderStepWithAccel(detents);
        temp_edit_value  = clampf(temp_edit_value, 0.0f, 500.0f);
      }
      if (click == CLICK_DOUBLE) {
        current_state = STATE_MENU_MAIN;
      } else if (click == CLICK_SINGLE) {
        settings.max_temp_lock = temp_edit_value;
        current_state = STATE_MENU_MAIN;
      }
    } break;

    // ---------- Turn off When Idle ----------
    case STATE_EDIT_IDLE_OFF: {
      // 0=15min, 1=30min, 2=60min, 3=Always ON
      if (detents != 0) {
        sub_menu_cursor += detents;
        if (sub_menu_cursor < 0) sub_menu_cursor = 3;
        if (sub_menu_cursor > 3) sub_menu_cursor = 0;
      }
      if (click == CLICK_DOUBLE) {
        current_state = STATE_MENU_MAIN;
      } else if (click == CLICK_SINGLE) {
        if      (sub_menu_cursor == 0) settings.idle_off_minutes = 15;
        else if (sub_menu_cursor == 1) settings.idle_off_minutes = 30;
        else if (sub_menu_cursor == 2) settings.idle_off_minutes = 60;
        else                           settings.idle_off_minutes = 0; // 0 = Always ON
        current_state = STATE_MENU_MAIN;
      }
    } break;

    // ---------- Light & Sound ----------
    case STATE_EDIT_LIGHT_SOUND: {
      // Example: 0=Off, 1=Light only, 2=Sound only, 3=Both
      if (detents != 0) {
        sub_menu_cursor += detents;
        if (sub_menu_cursor < 0) sub_menu_cursor = 3;
        if (sub_menu_cursor > 3) sub_menu_cursor = 0;
      }
      if (click == CLICK_DOUBLE) {
        current_state = STATE_MENU_MAIN;
      } else if (click == CLICK_SINGLE) {
        indexToLightSound(sub_menu_cursor, settings); // ensure this field exists in OvenSettings
        current_state = STATE_MENU_MAIN;
      }
    } break;

    default:
      // Safety fallback
      current_state = STATE_STANDBY;
      break;
  }
}

