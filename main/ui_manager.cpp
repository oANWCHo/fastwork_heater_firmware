#include "ui_manager.h"
// 1. Include uploaded font files
#include "Arial18.h" 
#include "Arial12.h"

// --- Theme Color Definitions ---
#define COLOR_IDLE   0x39E7 // Dark Grey
#define COLOR_ACTIVE 0x05E0 // Dark Green
#define COLOR_DONE   TFT_BLUE
#define COLOR_WARN   TFT_ORANGE

// UI Selection Colors (Modern Theme)
#define C_SELECT_BG  0x18E3 // Dark Blue-ish Grey
#define C_SELECT_TXT TFT_CYAN
#define C_NORMAL_TXT TFT_WHITE
#define C_NORMAL_BG  TFT_BLACK

#define C_WHITE     0xFFFF 
#define C_GREY_BG   0xACB9 
#define C_PINK_BG   0xF71F 
#define C_LIME_BG   0xE7F8 
#define C_ACTIVE_CYCLE_BG 0xC6D8
#define C_RED_TXT   0xF986 
#define C_GREEN_TXT 0x4D6A 
#define C_BLACK     0x0000
#define C_GREY_TXT  0x7BEF 

const uint32_t INACTIVITY_TIMEOUT_MS = 10000;

// --- Menu Labels ---
const char* menu_item_labels_page_1[MENU_PAGE1_ITEM_COUNT] = {
  "Max Temp Lock",
  "Heater Calibration", 
  "IR Emissivity",
  "TC Probe Cal",   
  "Next Page >"
};

const char* menu_item_labels_page_2[MENU_PAGE2_ITEM_COUNT] = {
  "Turn off When Idle", 
  "Startup Mode", 
  "Sound",
  "Temp Unit",
  "About", 
  "< Prev Page"
};

const char* startup_mode_labels[STARTUP_MODE_COUNT] = {
  "OFF", 
  "Auto Run"
};

const char* idle_off_labels[IDLE_OFF_ITEM_COUNT] = {
  "10 sec. (Debug)", "15 min.", "30 min.", "60 min.", "Always ON"
};
const char* temp_unit_labels[2] = {
  "Celsius (C)", "Fahrenheit (F)"
};

const char* standby_button_labels[6] = {
  "Heater1", "Heater2", "Heater3", "start", "stop", "Settings"
};

void UIManager::switchToAutoMode() {
    _previous_screen = SCREEN_AUTO_MODE;  // บันทึกว่าอยู่ Auto Mode
    _current_screen = SCREEN_AUTO_MODE;
    resetInactivityTimer();
}

void UIManager::switchToStandby() {
    _previous_screen = SCREEN_STANDBY;
    _current_screen = SCREEN_STANDBY;
    resetInactivityTimer();
}
// --- Constructor & Core ---
UIManager::UIManager(TFT_eSPI* tft, ConfigSaveCallback save_callback) 
  : _tft(tft), _spr(_tft), _save_callback(save_callback) {
  _current_screen = SCREEN_STANDBY;
  _previous_screen = SCREEN_STANDBY;  // เริ่มต้นที่ Standby
  _quick_edit_step = Q_EDIT_TARGET;
  _blink_state = false;
  _is_editing_calibration = false; 
  _selected_menu_item = 0;
  _selected_menu_item_page_2 = 0; 
  _menu_step_accumulator = 0.0f;
  _standby_selection = 0;
  _auto_selection = 0;
}

void UIManager::begin() {
  _spr.createSprite(_tft->width(), _tft->height());
  _spr.setSwapBytes(true);
  _last_activity_time = millis();
}

uint16_t UIManager::color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void UIManager::openSettings() {
    // บันทึกหน้าปัจจุบันก่อนเข้า Settings
    if (_current_screen != SCREEN_SETTINGS_PAGE_1 && 
        _current_screen != SCREEN_SETTINGS_PAGE_2) {
        _previous_screen = _current_screen;
    }
    _current_screen = SCREEN_SETTINGS_PAGE_1;
    _selected_menu_item = 0;
    resetInactivityTimer();
}

void UIManager::exitSettings() {
    // กลับไปหน้าที่บันทึกไว้ (Auto Mode หรือ Manual Mode)
    _current_screen = _previous_screen;
    resetInactivityTimer();
}

void UIManager::enterQuickEdit() {
    if (_standby_selection >= 0 && _standby_selection <= 2) {
        _current_screen = SCREEN_QUICK_EDIT;
        _quick_edit_step = Q_EDIT_TARGET;
        _menu_step_accumulator = 0.0f;
        resetInactivityTimer();
    }
}

void UIManager::enterQuickEditAuto() {
    if (_auto_selection >= 0 && _auto_selection <= 2) {
        _current_screen = SCREEN_QUICK_EDIT_AUTO;
        _quick_edit_step = Q_EDIT_TARGET; // เริ่มแก้ที่ Target ก่อน
        _menu_step_accumulator = 0.0f;
        resetInactivityTimer();
    }
}

void UIManager::resetInactivityTimer() { 
  _last_activity_time = millis();
}

bool UIManager::checkInactivity(ConfigState& config, bool& has_go_to, float& go_to) { 
  uint32_t elapsed = millis() - _last_activity_time;

  if (_current_screen == SCREEN_SLEEP) return false;

  uint32_t sleep_timeout = 0;
  switch(config.idle_off_mode) {
    case 0: sleep_timeout = 10000;    break; 
    case 1: sleep_timeout = 900000;   break; 
    case 2: sleep_timeout = 1800000;  break; 
    case 3: sleep_timeout = 3600000;  break; 
    case 4: return false; 
  }

  if (elapsed > sleep_timeout) {
    _current_screen = SCREEN_SLEEP;
    has_go_to = false;
    go_to = NAN;
    return true;
  }
  return false;
}

// --- Main Draw Loop ---
void UIManager::draw(const AppState& state, const ConfigState& config) {
  static uint32_t last_blink_time = 0;
  if (millis() - last_blink_time > 500) {
    _blink_state = !_blink_state;
    last_blink_time = millis();
  }

  _spr.fillSprite(C_BLACK);

  switch (_current_screen) {
    case SCREEN_SLEEP:
      _spr.setTextColor(TFT_WHITE, TFT_BLACK);
      _spr.setTextDatum(MC_DATUM);
      _spr.loadFont(Arial18);
      _spr.drawString("ZZZ Sleeping...", _spr.width() / 2, _spr.height() / 2);
      _spr.unloadFont(); 
      _spr.pushSprite(0, 0);
      return; 

    case SCREEN_STANDBY:                  drawStandbyScreen(state, config); break;
    case SCREEN_AUTO_MODE:        drawAutoModeScreen(state, config); break;
    case SCREEN_QUICK_EDIT:               drawStandbyScreen(state, config); break;
    case SCREEN_QUICK_EDIT_AUTO:  drawAutoModeScreen(state, config); break;
    case SCREEN_SETTINGS_PAGE_1:          drawSettingsPage1(state, config); break; 
    case SCREEN_SETTINGS_PAGE_2:          drawSettingsPage2(state, config); break; 
    case SCREEN_SETTINGS_HEATER_TARGET_TEMP: drawSettingsHeaterTargetTemp(state); break;
    case SCREEN_SETTINGS_HEATER_MAX_TEMP: drawSettingsHeaterMaxTemp(state); break;
    case SCREEN_SETTINGS_MAX_TEMP_LOCK:   drawSettingsMaxTempLock(state); break;
    case SCREEN_SETTINGS_HEATER_CALIBRATE:  drawSettingsHeaterCalibrate(state); break;
    case SCREEN_SETTINGS_CALIBRATION_SELECT: drawSettingsCalibrationSelect(state, config); break; 
    case SCREEN_SETTINGS_IDLE_OFF:        drawSettingsIdleOff(state, config); break;
    case SCREEN_SETTINGS_STARTUP:  drawSettingsStartup(state, config); break;
    case SCREEN_SETTINGS_SOUND:           drawSettingsSound(state, config); break; 
    case SCREEN_SETTINGS_TC_PROBE_CAL:    drawSettingsTCProbeCal(state, config); break;
    case SCREEN_SETTINGS_TEMP_UNIT:       drawSettingsTempUnit(state, config); break;
    case SCREEN_SETTINGS_ABOUT:           drawSettingsAbout(state); break;
    case SCREEN_SETTINGS_EMISSIVITY: drawSettingsEmissivity(state, config); break;
  }
  _spr.pushSprite(0, 0);
}

// --- Helper UI Components ---

void UIManager::drawTaskBar() {
    int w = _spr.width();
    int h = 24;
    _spr.fillRect(0, 0, w, h, C_BLACK);
    _spr.drawFastHLine(0, h-1, w, TFT_DARKGREY);

    int icon_x = w - 25;
    int icon_y = h / 2;
    
    _spr.drawCircle(icon_x, icon_y, 8, TFT_DARKGREY);
    _spr.drawLine(icon_x, icon_y-3, icon_x, icon_y+2, TFT_DARKGREY);
    _spr.drawPixel(icon_x, icon_y+5, TFT_DARKGREY);
    
    icon_x -= 25;
    _spr.drawString("BT", icon_x, icon_y, 1);

    icon_x -= 25;
    _spr.drawCircle(icon_x, icon_y+4, 2, TFT_DARKGREY);
    _spr.drawArc(icon_x, icon_y+4, 6, 8, 220, 320, TFT_DARKGREY, C_BLACK);
    
    _spr.setTextColor(TFT_LIGHTGREY, C_BLACK);
    _spr.setTextDatum(ML_DATUM);
    _spr.drawString("Smart Heater", 5, icon_y);
}

void UIManager::drawHeader(const char* title) {
    drawTaskBar(); 

    int y_header = 24;
    int h_header = 32;
    
    _spr.fillRect(0, y_header, _spr.width(), h_header, TFT_WHITE);
    
    _spr.setTextColor(TFT_BLACK, TFT_WHITE);
    _spr.setTextDatum(MC_DATUM);
    _spr.loadFont(Arial18);
    _spr.drawString(title, _spr.width() / 2, y_header + (h_header/2));
    _spr.unloadFont();
}

// --- Screen Implementations ---

void UIManager::drawSettingsPage1(const AppState& state, const ConfigState& config) {
  drawHeader("Heater Setup");

  const int ITEM_HEIGHT = 24; 
  const int ITEM_SPACING = 27; 
  const int X_MARGIN = 10;
  const int Y_START = 65; 
  int w = _spr.width() - (2 * X_MARGIN);

  for (int i = 0; i < MENU_PAGE1_ITEM_COUNT; ++i) {
    int y = Y_START + i * ITEM_SPACING;
    uint16_t bg_color, text_color;

    if (i == _selected_menu_item) {
      bg_color = C_SELECT_BG;   
      text_color = C_SELECT_TXT; 
    } else {
      bg_color = TFT_BLACK;     
      text_color = TFT_WHITE;   
    }
    
    _spr.fillRect(X_MARGIN, y, w, ITEM_HEIGHT, bg_color);
    if (i == _selected_menu_item) {
      _spr.drawRect(X_MARGIN, y, w, ITEM_HEIGHT, TFT_DARKGREY);
    }
    _spr.setTextColor(text_color, bg_color);
    _spr.setTextDatum(MC_DATUM);
    _spr.loadFont(Arial18);
    _spr.drawString(menu_item_labels_page_1[i], _spr.width() / 2, y + ITEM_HEIGHT / 2);
    _spr.unloadFont();
  }
  
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Select | Press: OK | Dbl: Back", _spr.width() / 2, _spr.height() - 2);
}

void UIManager::drawSettingsPage2(const AppState& state, const ConfigState& config) {
  drawHeader("System Setup");

  const int ITEM_HEIGHT = 24;
  const int ITEM_SPACING = 27;
  const int X_MARGIN = 10;
  const int Y_START = 65;
  int w = _spr.width() - (2 * X_MARGIN);

  for (int i = 0; i < MENU_PAGE2_ITEM_COUNT; ++i) {
    int y = Y_START + i * ITEM_SPACING;
    uint16_t bg_color, text_color;

    if (i == _selected_menu_item_page_2) {
      bg_color = C_SELECT_BG;    
      text_color = C_SELECT_TXT; 
    } else {
      bg_color = TFT_BLACK;      
      text_color = TFT_WHITE;    
    }
    
    _spr.fillRect(X_MARGIN, y, w, ITEM_HEIGHT, bg_color);
    if (i == _selected_menu_item_page_2) {
      _spr.drawRect(X_MARGIN, y, w, ITEM_HEIGHT, TFT_DARKGREY);
    }
    _spr.setTextColor(text_color, bg_color);
    _spr.setTextDatum(MC_DATUM);
    _spr.loadFont(Arial18);
    _spr.drawString(menu_item_labels_page_2[i], _spr.width() / 2, y + ITEM_HEIGHT / 2);
    _spr.unloadFont();
  }
  
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Select | Press: OK | Dbl: Back", _spr.width() / 2, _spr.height() - 2);
}

void UIManager::drawSettingsStartup(const AppState& state, const ConfigState& config) {
  drawHeader("Startup Mode");

  const int ITEM_HEIGHT = 30;
  const int ITEM_SPACING = 35;
  const int X_MARGIN = 20;
  const int Y_START = 75;
  int w = _spr.width() - (2 * X_MARGIN);

  for (int i = 0; i < STARTUP_MODE_COUNT; ++i) {
    int y = Y_START + i * ITEM_SPACING;
    uint16_t bg_color, text_color;

    if (i == _selected_menu_item) {
        bg_color = C_SELECT_BG;
        text_color = C_SELECT_TXT;
    } else {
        bg_color = TFT_BLACK;
        text_color = TFT_WHITE;
    }
    
    _spr.fillRect(X_MARGIN, y, w, ITEM_HEIGHT, bg_color);
    if (i == _selected_menu_item) _spr.drawRect(X_MARGIN, y, w, ITEM_HEIGHT, TFT_DARKGREY);
    
    _spr.setTextColor(text_color, bg_color);
    _spr.setTextDatum(MC_DATUM);
    _spr.loadFont(Arial18);
    _spr.drawString(startup_mode_labels[i], _spr.width() / 2, y + ITEM_HEIGHT / 2);
    _spr.unloadFont();
  }
  
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Select | Press: Save", _spr.width() / 2, _spr.height() - 10);
}

void UIManager::drawSettingsIdleOff(const AppState& state, const ConfigState& config) {
  drawHeader("Turn off When Idle");

  const int ITEM_HEIGHT = 22;
  const int ITEM_SPACING = 25;
  const int X_MARGIN = 20;
  const int Y_START = 65;
  int w = _spr.width() - (2 * X_MARGIN);

  for (int i = 0; i < IDLE_OFF_ITEM_COUNT; ++i) {
    int y = Y_START + i * ITEM_SPACING;
    uint16_t bg_color, text_color;

    if (i == _selected_menu_item) {
        bg_color = C_SELECT_BG;
        text_color = C_SELECT_TXT;
    } else {
        bg_color = TFT_BLACK;
        text_color = TFT_WHITE;
    }

    _spr.fillRect(X_MARGIN, y, w, ITEM_HEIGHT, bg_color);
    if (i == _selected_menu_item) _spr.drawRect(X_MARGIN, y, w, ITEM_HEIGHT, TFT_DARKGREY);

    _spr.setTextColor(text_color, bg_color);
    _spr.setTextDatum(MC_DATUM);
    _spr.loadFont(Arial18);
    _spr.drawString(idle_off_labels[i], _spr.width() / 2, y + ITEM_HEIGHT / 2);
    _spr.unloadFont();
  }
}

void UIManager::drawSettingsTempUnit(const AppState& state, const ConfigState& config) {
    drawHeader("Temperature Unit");

    const int ITEM_HEIGHT = 35;
    const int ITEM_SPACING = 40; 
    const int X_MARGIN = 20;
    const int Y_START = 80;
    int w = _spr.width() - (2 * X_MARGIN);

    for (int i = 0; i < 2; ++i) {
        int y = Y_START + i * ITEM_SPACING;
        uint16_t bg_color, text_color;

        if (i == _selected_menu_item) {
            bg_color = C_SELECT_BG;
            text_color = C_SELECT_TXT;
        } else {
            bg_color = TFT_BLACK;
            text_color = TFT_WHITE;
        }
        
        _spr.fillRect(X_MARGIN, y, w, ITEM_HEIGHT, bg_color);
        if (i == _selected_menu_item) _spr.drawRect(X_MARGIN, y, w, ITEM_HEIGHT, TFT_DARKGREY);

        _spr.setTextColor(text_color, bg_color);
        _spr.setTextDatum(MC_DATUM);
        _spr.loadFont(Arial18);
        _spr.drawString(temp_unit_labels[i], _spr.width() / 2, y + ITEM_HEIGHT / 2);
        _spr.unloadFont();
    }
}

void UIManager::drawSettingsSound(const AppState& state, const ConfigState& config) { 
    drawHeader("Sound Settings");

    int w = _spr.width() - 60;
    int h = 40;
    int x = 30;
    int y = 90;

    _spr.fillRect(x, y, w, h, C_SELECT_BG);
    _spr.drawRect(x, y, w, h, TFT_DARKGREY);
    
    _spr.setTextDatum(MC_DATUM);
    _spr.setTextColor(C_SELECT_TXT, C_SELECT_BG);
    
    _spr.loadFont(Arial18);
    char buf[30];
    snprintf(buf, 30, "Sound: %s", config.sound_on ? "ON" : "OFF");
    _spr.drawString(buf, _spr.width() / 2, y + h/2);
    _spr.unloadFont();
    
    _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
    _spr.setTextDatum(BC_DATUM);
    _spr.setTextSize(1);
    _spr.drawString("Rot: Toggle | Dbl: Back", _spr.width() / 2, _spr.height() - 10); 
}

// --- UPDATED: Emissivity with Units ---
void UIManager::drawSettingsEmissivity(const AppState& state, const ConfigState& config) {
  drawHeader("IR Emissivity");

  const int ITEM_HEIGHT = 35;
  const int ITEM_SPACING = 38;
  const int X_MARGIN = 20;
  const int Y_START = 80;
  int w = _spr.width() - (2 * X_MARGIN);

  _spr.loadFont(Arial18);

  // --- Item 1: IR 1 ---
  {
      int y = Y_START;
      uint16_t bg_color = (0 == _selected_menu_item) ? C_SELECT_BG : TFT_BLACK;
      uint16_t txt_color = (0 == _selected_menu_item) ? C_SELECT_TXT : TFT_WHITE;
      
      _spr.fillRect(X_MARGIN, y, w, ITEM_HEIGHT, bg_color);
      if (0 == _selected_menu_item) _spr.drawRect(X_MARGIN, y, w, ITEM_HEIGHT, TFT_DARKGREY);
      
      _spr.setTextColor(txt_color, bg_color);
      _spr.setTextDatum(MC_DATUM);
      
      char buf[40];
      // Convert Temp for Display
      float disp_val = convertTemp(state.ir_temps[0], state.temp_unit);
      
      if (isnan(state.ir_temps[0])) snprintf(buf, 40, "IR1 (---): %.2f", config.ir_emissivity[0]);
      else snprintf(buf, 40, "IR1 (%.1f%c): %.2f", disp_val, state.temp_unit, config.ir_emissivity[0]);
      
      _spr.drawString(buf, _spr.width() / 2, y + ITEM_HEIGHT / 2);
  }

  // --- Item 2: IR 2 ---
  {
      int y = Y_START + ITEM_SPACING;
      uint16_t bg_color = (1 == _selected_menu_item) ? C_SELECT_BG : TFT_BLACK;
      uint16_t txt_color = (1 == _selected_menu_item) ? C_SELECT_TXT : TFT_WHITE;
      
      bool connected = !isnan(state.ir_temps[1]);
      if (!connected && 1 != _selected_menu_item) txt_color = TFT_DARKGREY;

      _spr.fillRect(X_MARGIN, y, w, ITEM_HEIGHT, bg_color);
      if (1 == _selected_menu_item) _spr.drawRect(X_MARGIN, y, w, ITEM_HEIGHT, TFT_DARKGREY);
      
      _spr.setTextColor(txt_color, bg_color);
      _spr.setTextDatum(MC_DATUM);
      
      char buf[40];
      // Convert Temp for Display
      float disp_val = convertTemp(state.ir_temps[1], state.temp_unit);
      
      if (connected) snprintf(buf, 40, "IR2 (%.1f%c): %.2f", disp_val, state.temp_unit, config.ir_emissivity[1]);
      else snprintf(buf, 40, "IR2: Not Connected");
      
      _spr.drawString(buf, _spr.width() / 2, y + ITEM_HEIGHT / 2);
  }

  _spr.unloadFont();
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Adjust | Press: Switch | Dbl: Back", _spr.width() / 2, _spr.height() - 10);
}

// --- UPDATED: TC Probe Cal with Units ---
void UIManager::drawSettingsTCProbeCal(const AppState& state, const ConfigState& config) {
  _temp_edit_value = state.tc_probe_temp;
  
  drawHeader("TC Probe Cal");

  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.setTextDatum(MC_DATUM);
  _spr.loadFont(Arial18);
  
  char buf[32];
  int info_y = 80;
  
  if (isnan(state.tc_probe_temp)) {
    _spr.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    _spr.drawString("Raw: ---", _spr.width() / 2, info_y);
    _spr.drawString("Offset: ---", _spr.width() / 2, info_y + 25);
    _spr.setTextColor(TFT_DARKGREY, TFT_BLACK);
    _spr.drawString("Val: ---", _spr.width() / 2, info_y + 50);
  } else {
    // 1. Calculate Raw in Absolute Temp
    float raw_c = state.tc_probe_temp - config.tc_probe_offset;
    float raw_disp = convertTemp(raw_c, state.temp_unit);
    
    // 2. Calculate Offset in Delta Temp (No +32 shift)
    float off_disp = convertDelta(config.tc_probe_offset, state.temp_unit);

    // 3. Calculate Final in Absolute Temp
    float val_disp = convertTemp(state.tc_probe_temp, state.temp_unit);

    _spr.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    snprintf(buf, 32, "Raw: %.2f %c", raw_disp, state.temp_unit);
    _spr.drawString(buf, _spr.width() / 2, info_y);
    
    snprintf(buf, 32, "Offset: %.2f %c", off_disp, state.temp_unit);
    _spr.drawString(buf, _spr.width() / 2, info_y + 25);
    
    _spr.setTextColor(TFT_GREEN, TFT_BLACK);
    snprintf(buf, 32, "Val: %.2f %c", val_disp, state.temp_unit);
    _spr.drawString(buf, _spr.width() / 2, info_y + 50);
  }

  int start_y = 150;
  const char* options[] = {"Tare (Set 0)", "Reset (0.0)"};
  
  int w = _spr.width() - 40;
  int h = 26;
  
  for (int i = 0; i < 2; i++) {
     int y = start_y + (i * 30); 
     uint16_t bg = (i == _selected_menu_item) ? C_SELECT_BG : TFT_BLACK;
     uint16_t txt = (i == _selected_menu_item) ? C_SELECT_TXT : TFT_WHITE;
     
     _spr.fillRect(20, y, w, h, bg);
     if(i == _selected_menu_item) _spr.drawRect(20, y, w, h, TFT_DARKGREY);
     
     _spr.setTextColor(txt, bg);
     _spr.drawString(options[i], _spr.width() / 2, y + h/2);
  }
  
  _spr.unloadFont();
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Press: Action | Dbl: Back", _spr.width() / 2, _spr.height() - 10);
}

// --- UPDATED: Calibration Select with Units ---
void UIManager::drawSettingsCalibrationSelect(const AppState& state, const ConfigState& config) {
  drawHeader("Heater Calibration");

  char item_labels[3][30];

  int start_y = 75;
  int w = _spr.width() - 20;
  int h = 30;
  
  _spr.loadFont(Arial18);

  for (int i = 0; i < 3; ++i) {
    int y = start_y + i * 35; 
    uint16_t bg = TFT_BLACK;
    uint16_t txt = TFT_WHITE;
    
    // Convert Offset for Display (Delta)
    float disp_off = convertDelta(config.tc_offsets[i], state.temp_unit);

    if (i == _selected_menu_item) {
        bg = C_SELECT_BG; 
        if (_is_editing_calibration) {
            txt = C_RED_TXT; 
            snprintf(item_labels[i], 30, "<< %.1f %c >>", disp_off, state.temp_unit);
        } else {
            txt = C_SELECT_TXT;
            snprintf(item_labels[i], 30, "Heater %d Offset: %.1f%c", i+1, disp_off, state.temp_unit);
        }
    } else {
        snprintf(item_labels[i], 30, "Heater %d Offset: %.1f%c", i+1, disp_off, state.temp_unit);
    }

    _spr.fillRect(10, y, w, h, bg);
    if(i == _selected_menu_item) _spr.drawRect(10, y, w, h, TFT_DARKGREY);

    _spr.setTextDatum(MC_DATUM);
    _spr.setTextColor(txt, bg);
    _spr.drawString(item_labels[i], _spr.width() / 2, y + h/2);
  }
  
  _spr.unloadFont();
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Select/Edit | Press: Toggle", _spr.width() / 2, _spr.height() - 10);
}

void UIManager::drawSettingsHeaterTargetTemp(const AppState& state) {
  char title_buffer[20];
  snprintf(title_buffer, sizeof(title_buffer), "Heater (%d)", _selected_menu_item + 1);
  drawHeader(title_buffer);

  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.setTextDatum(MC_DATUM);
  _spr.loadFont(Arial18);

  char temp_buffer[30];
  float display_value = convertTemp(_temp_edit_value, state.temp_unit);
  snprintf(temp_buffer, sizeof(temp_buffer), "Target Temp: %.1f %c", display_value, state.temp_unit);
  _spr.drawString(temp_buffer, _spr.width() / 2, _spr.height() / 2);
  
  _spr.unloadFont();
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Adjust | Press: Confirm", _spr.width() / 2, _spr.height() - 10);
}

void UIManager::drawSettingsHeaterMaxTemp(const AppState& state) {
  char title_buffer[20];
  snprintf(title_buffer, sizeof(title_buffer), "Heater (%d)", _selected_menu_item + 1);
  drawHeader(title_buffer);

  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.setTextDatum(MC_DATUM);
  _spr.loadFont(Arial18);

  char temp_buffer[30];
  float display_value = convertTemp(_temp_edit_value, state.temp_unit);
  snprintf(temp_buffer, sizeof(temp_buffer), "Max Temp: %.1f %c", display_value, state.temp_unit);
  _spr.drawString(temp_buffer, _spr.width() / 2, _spr.height() / 2);
  
  _spr.unloadFont();
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Adjust | Press: Confirm", _spr.width() / 2, _spr.height() - 10);
}

void UIManager::drawSettingsHeaterCalibrate(const AppState& state) {
  drawHeader("Heater Cal (Unused)");
}

void UIManager::drawSettingsMaxTempLock(const AppState& state) {
  drawHeader("Max Temp Lock");
  
  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.setTextDatum(MC_DATUM);
  _spr.loadFont(Arial18);

  char temp_buffer[30];
  float display_value = convertTemp(_temp_edit_value, state.temp_unit);
  snprintf(temp_buffer, sizeof(temp_buffer), "Max Temp: %.1f %c", display_value, state.temp_unit);
  _spr.drawString(temp_buffer, _spr.width() / 2, _spr.height() / 2);
  
  _spr.unloadFont();
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Adjust | Press: Confirm", _spr.width() / 2, _spr.height() - 10);
}

void UIManager::drawSettingsAbout(const AppState& state) {
    drawHeader("About Device");

    _spr.setTextColor(TFT_WHITE, TFT_BLACK);
    _spr.setTextDatum(MC_DATUM);
    _spr.loadFont(Arial18);
    _spr.setTextSize(1);

    int line_height = 12;
    int y_center = _spr.height() / 2 + 10;
    int y_start = y_center - (line_height * 3);

    _spr.drawString("Heater Control System", _spr.width() / 2, y_start);
    _spr.drawString("Firmware v1.0", _spr.width() / 2, y_start + (line_height * 2));
    _spr.drawString("(c) 2025 K'Tor", _spr.width() / 2, y_start + (line_height * 4));
    _spr.drawString("Contact: 08x-xxx-xxxx", _spr.width() / 2, y_start + (line_height * 6));
    
    _spr.unloadFont();

    _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
    _spr.setTextDatum(BC_DATUM);
    _spr.setTextSize(1);
    _spr.drawString("Press or Dbl: Back", _spr.width() / 2, _spr.height() - 10);
}

// --- Logic Implementations (Button/Encoder) ---

bool UIManager::handleButtonSingleClick(ConfigState& config, float& go_to, bool& has_go_to) {
  if (_current_screen == SCREEN_SLEEP) {
    _current_screen = SCREEN_STANDBY;
    resetInactivityTimer();
    return true; 
  }
  resetInactivityTimer();
  switch (_current_screen) {
    case SCREEN_STANDBY:
        if (_standby_selection >= 0 && _standby_selection <= 2) {
            // Toggle heater on/off
            config.heater_active[_standby_selection] = !config.heater_active[_standby_selection];
            if (_save_callback) _save_callback(config);
        } else if (_standby_selection == 5) {
            // Settings button pressed - open settings immediately
            openSettings();
        }
        return true;
    case SCREEN_QUICK_EDIT:
        if (_quick_edit_step == Q_EDIT_TARGET) {
            _quick_edit_step = Q_EDIT_MAX;
        } else {
            if (_save_callback) _save_callback(config);
            _current_screen = SCREEN_STANDBY;
        }
        return true;
    case SCREEN_QUICK_EDIT_AUTO:
        if (_quick_edit_step == Q_EDIT_TARGET) {
            _quick_edit_step = Q_EDIT_MAX; // ไปแก้ Max ต่อ
        } else {
            // บันทึกและออก
            if (_save_callback) _save_callback(config);
            _current_screen = SCREEN_AUTO_MODE;
        }
        return true;
    case SCREEN_SETTINGS_PAGE_1: 
      switch (_selected_menu_item) {
        case MENU_PAGE1_MAX_TEMP_LOCK: 
          _current_screen = SCREEN_SETTINGS_MAX_TEMP_LOCK;
          _temp_edit_value = config.max_temp_lock;
          break;
        case MENU_PAGE1_CALIBRATION: 
          _current_screen = SCREEN_SETTINGS_CALIBRATION_SELECT;
          _selected_menu_item = 0; 
          _is_editing_calibration = false; // Reset state
          break;
        case MENU_PAGE1_EMISSIVITY: 
          _current_screen = SCREEN_SETTINGS_EMISSIVITY;
          _selected_menu_item = 0; 
          break;
        case MENU_PAGE1_TC_PROBE_CAL: // Moved here
           _current_screen = SCREEN_SETTINGS_TC_PROBE_CAL; 
           _selected_menu_item = 0; 
           break;
        case MENU_PAGE1_NEXT_PAGE: 
          _current_screen = SCREEN_SETTINGS_PAGE_2;
          _selected_menu_item_page_2 = 0; 
          break;
      }
      _menu_step_accumulator = 0.0f;
      return true;
    case SCREEN_SETTINGS_PAGE_2: 
      switch (_selected_menu_item_page_2) {
        case MENU_PAGE2_IDLE_OFF: _current_screen = SCREEN_SETTINGS_IDLE_OFF; _selected_menu_item = config.idle_off_mode; break;
        case MENU_PAGE2_STARTUP: _current_screen = SCREEN_SETTINGS_STARTUP; _selected_menu_item = (int)config.startup_mode; if (_selected_menu_item >= STARTUP_MODE_COUNT) _selected_menu_item = 0; break;
        case MENU_PAGE2_SOUND: _current_screen = SCREEN_SETTINGS_SOUND; _selected_menu_item = 0; break;
        case MENU_PAGE2_TEMP_UNIT: _current_screen = SCREEN_SETTINGS_TEMP_UNIT; _selected_menu_item = (config.temp_unit == 'C') ? 0 : 1; break;
        case MENU_PAGE2_ABOUT: _current_screen = SCREEN_SETTINGS_ABOUT; break;
        case MENU_PAGE2_PREV_PAGE: _current_screen = SCREEN_SETTINGS_PAGE_1; _selected_menu_item = 0; break;
      }
      _menu_step_accumulator = 0.0f;
      return true;
    case SCREEN_SETTINGS_TC_PROBE_CAL:
      if (!isnan(_temp_edit_value)) {
        if (_selected_menu_item == 0) config.tc_probe_offset = config.tc_probe_offset - _temp_edit_value;
        else config.tc_probe_offset = 0.0f;
        if (_save_callback) _save_callback(config);
      }
      return true;
    case SCREEN_SETTINGS_HEATER_TARGET_TEMP:
      config.target_temps[_selected_menu_item] = _temp_edit_value;
      if (_save_callback) _save_callback(config);
      _current_screen = SCREEN_SETTINGS_HEATER_MAX_TEMP;
      _temp_edit_value = config.max_temps[_selected_menu_item];
      _menu_step_accumulator = 0.0f;
      return true;
    case SCREEN_SETTINGS_HEATER_MAX_TEMP:
      config.max_temps[_selected_menu_item] = _temp_edit_value;
      if (_save_callback) _save_callback(config);
      _current_screen = SCREEN_SETTINGS_PAGE_1; 
      _menu_step_accumulator = 0.0f;
      return true;
    // --- MODIFIED LOGIC FOR CALIBRATION SELECT ---
    case SCREEN_SETTINGS_CALIBRATION_SELECT:
      // Toggle Editing Mode
      _is_editing_calibration = !_is_editing_calibration;
      if (!_is_editing_calibration) {
          // If exiting edit mode, save config
          if (_save_callback) _save_callback(config);
      }
      _menu_step_accumulator = 0.0f;
      return true;
    // ---------------------------------------------
    case SCREEN_SETTINGS_HEATER_CALIBRATE:
      // Logic handled in SELECT screen now
      return true;
    case SCREEN_SETTINGS_MAX_TEMP_LOCK:
      config.max_temp_lock = _temp_edit_value;
      if (_save_callback) _save_callback(config);
      _current_screen = SCREEN_SETTINGS_PAGE_1; 
      _menu_step_accumulator = 0.0f;
      return true;
    case SCREEN_SETTINGS_IDLE_OFF:
      config.idle_off_mode = (IdleOffMode)_selected_menu_item;
      if (_save_callback) _save_callback(config);
      _current_screen = SCREEN_SETTINGS_PAGE_2; 
      _menu_step_accumulator = 0.0f;
      return true;
    case SCREEN_SETTINGS_SOUND: 
      config.sound_on = !config.sound_on;
      if (_save_callback) _save_callback(config);
      return true;
    case SCREEN_SETTINGS_STARTUP: 
      config.startup_mode = (StartupMode)_selected_menu_item;
      if (_save_callback) _save_callback(config);
      _current_screen = SCREEN_SETTINGS_PAGE_2; 
      _menu_step_accumulator = 0.0f;
      return true;
    case SCREEN_SETTINGS_EMISSIVITY:
      if (_selected_menu_item == 0) _selected_menu_item = 1; else _selected_menu_item = 0;
      return true;
    case SCREEN_SETTINGS_TEMP_UNIT:
      config.temp_unit = (_selected_menu_item == 0) ? 'C' : 'F';
      _current_screen = SCREEN_SETTINGS_PAGE_2; 
      if (_save_callback) _save_callback(config);
      _menu_step_accumulator = 0.0f;
      return true;
    case SCREEN_SETTINGS_ABOUT: _current_screen = SCREEN_SETTINGS_PAGE_2; return true;
    default: return false;
  }
}

bool UIManager::handleButtonDoubleClick(ConfigState& config) {
  if (_current_screen == SCREEN_SLEEP) {
    _current_screen = SCREEN_STANDBY;
    resetInactivityTimer();
    return true; 
  }
  resetInactivityTimer();
  switch (_current_screen) {
    case SCREEN_SETTINGS_PAGE_1: case SCREEN_SETTINGS_PAGE_2: _current_screen = SCREEN_STANDBY; break;
    case SCREEN_SETTINGS_HEATER_TARGET_TEMP: case SCREEN_SETTINGS_MAX_TEMP_LOCK: _current_screen = SCREEN_SETTINGS_PAGE_1; break;
    case SCREEN_SETTINGS_HEATER_MAX_TEMP: _current_screen = SCREEN_SETTINGS_HEATER_TARGET_TEMP; _temp_edit_value = config.target_temps[_selected_menu_item]; break;
    case SCREEN_SETTINGS_CALIBRATION_SELECT: 
        if (_is_editing_calibration) {
            _is_editing_calibration = false; // Cancel edit
        } else {
            _current_screen = SCREEN_SETTINGS_PAGE_1; 
        }
        break;
    case SCREEN_SETTINGS_HEATER_CALIBRATE: _current_screen = SCREEN_SETTINGS_CALIBRATION_SELECT; break;
    case SCREEN_SETTINGS_EMISSIVITY: _current_screen = SCREEN_SETTINGS_PAGE_1; break;
    case SCREEN_SETTINGS_IDLE_OFF: case SCREEN_SETTINGS_STARTUP: case SCREEN_SETTINGS_TEMP_UNIT: case SCREEN_SETTINGS_SOUND: case SCREEN_SETTINGS_TC_PROBE_CAL: case SCREEN_SETTINGS_ABOUT: _current_screen = SCREEN_SETTINGS_PAGE_2; break;
    default: return true;
  }
  _menu_step_accumulator = 0.0f;
  return true;
}

bool UIManager::handleEncoderRotation(float steps, ConfigState& config) {
  if (_current_screen == SCREEN_SLEEP) {
    _current_screen = SCREEN_STANDBY;
    resetInactivityTimer();
    return true; 
  }
  resetInactivityTimer();
  _menu_step_accumulator += steps;
  int change = (int)_menu_step_accumulator;
  if (change == 0) return true;
  _menu_step_accumulator -= (float)change;

  switch (_current_screen) {
    case SCREEN_STANDBY: {
      const int num_items = 3;
      int new_pos = _standby_selection + change;
      if (new_pos < 0) new_pos = 0;
      if (new_pos >= num_items) new_pos = num_items - 1;
      _standby_selection = new_pos;
      break;
    }
    case SCREEN_AUTO_MODE: {
      int new_pos = _auto_selection + change;
      if (new_pos < 0) new_pos = 0;
      if (new_pos > 2) new_pos = 2; 
      _auto_selection = new_pos;
      break;
    }
    case SCREEN_QUICK_EDIT: {
        int heaterIdx = _standby_selection;
        if (_quick_edit_step == Q_EDIT_TARGET) {
            config.target_temps[heaterIdx] += (float)change * 0.5f;
            if (config.target_temps[heaterIdx] < 0) config.target_temps[heaterIdx] = 0;
            if (config.target_temps[heaterIdx] > config.max_temp_lock) 
                config.target_temps[heaterIdx] = config.max_temp_lock;
        } else {
            config.max_temps[heaterIdx] += (float)change * 0.5f;
            if (config.max_temps[heaterIdx] < config.target_temps[heaterIdx]) 
                config.max_temps[heaterIdx] = config.target_temps[heaterIdx];
            if (config.max_temps[heaterIdx] > config.max_temp_lock) 
                config.max_temps[heaterIdx] = config.max_temp_lock;
        }
        break;
    }
    case SCREEN_QUICK_EDIT_AUTO: {
        // หมุนเพื่อปรับค่า (Edit Value)
        int cycleIdx = _auto_selection;
        if (_quick_edit_step == Q_EDIT_TARGET) {
            config.auto_target_temps[cycleIdx] += (float)change * 0.5f;
            // Limit Checks
            if (config.auto_target_temps[cycleIdx] < 0) config.auto_target_temps[cycleIdx] = 0;
            if (config.auto_target_temps[cycleIdx] > config.max_temp_lock) 
                config.auto_target_temps[cycleIdx] = config.max_temp_lock;
        } else {
            config.auto_max_temps[cycleIdx] += (float)change * 0.5f;
            // Limit Checks
            if (config.auto_max_temps[cycleIdx] < config.auto_target_temps[cycleIdx]) 
                config.auto_max_temps[cycleIdx] = config.auto_target_temps[cycleIdx];
            if (config.auto_max_temps[cycleIdx] > config.max_temp_lock) 
                config.auto_max_temps[cycleIdx] = config.max_temp_lock;
        }
        break;
    }
    case SCREEN_SETTINGS_PAGE_1: { 
      const int num_items = MENU_PAGE1_ITEM_COUNT;
      int new_pos = _selected_menu_item + change;
      if (new_pos < 0) new_pos = 0;
      if (new_pos >= num_items) new_pos = num_items - 1;
      _selected_menu_item = new_pos;
      break;
    }
    case SCREEN_SETTINGS_PAGE_2: { 
      const int num_items = MENU_PAGE2_ITEM_COUNT;
      int new_pos = _selected_menu_item_page_2 + change;
      if (new_pos < 0) new_pos = 0;
      if (new_pos >= num_items) new_pos = num_items - 1;
      _selected_menu_item_page_2 = new_pos;
      break;
    }
    case SCREEN_SETTINGS_TC_PROBE_CAL: {
      int new_pos = _selected_menu_item + change;
      if (new_pos < 0) new_pos = 0;
      if (new_pos > 1) new_pos = 1;
      _selected_menu_item = new_pos;
      break;
    }
    // --- MODIFIED ENCODER LOGIC FOR CALIBRATION SELECT ---
    case SCREEN_SETTINGS_CALIBRATION_SELECT: {
      if (_is_editing_calibration) {
          // EDIT MODE: Change Value
          float val = config.tc_offsets[_selected_menu_item];
          val += (float)change * 0.1f;
          // Clamp values
          if (val < -20.0f) val = -20.0f;
          if (val > 20.0f) val = 20.0f;
          config.tc_offsets[_selected_menu_item] = val;
      } else {
          // SELECTION MODE: Navigate Menu
          const int num_items = 3; 
          int new_pos = _selected_menu_item + change;
          if (new_pos < 0) new_pos = 0;
          if (new_pos >= num_items) new_pos = num_items - 1;
          _selected_menu_item = new_pos;
      }
      break;
    }
    // -----------------------------------------------------
    case SCREEN_SETTINGS_HEATER_TARGET_TEMP: {
      _temp_edit_value += (float)change * 0.5f;
      if (_temp_edit_value < 0) _temp_edit_value = 0;
      if (_temp_edit_value > config.max_temps[_selected_menu_item]) _temp_edit_value = config.max_temps[_selected_menu_item];
      break;
    }
    case SCREEN_SETTINGS_HEATER_MAX_TEMP: {
      _temp_edit_value += (float)change * 0.5f;
      if (_temp_edit_value < config.target_temps[_selected_menu_item]) _temp_edit_value = config.target_temps[_selected_menu_item];
      if (_temp_edit_value > config.max_temp_lock) _temp_edit_value = config.max_temp_lock;
      break;
    }
    case SCREEN_SETTINGS_HEATER_CALIBRATE: {
      // Unused but kept safe
      _temp_edit_value += (float)change * 0.1f;
      break;
    }
    case SCREEN_SETTINGS_MAX_TEMP_LOCK: {
      _temp_edit_value += (float)change * 0.5f;
      if (_temp_edit_value < 0) _temp_edit_value = 0;
      if (_temp_edit_value > 800) _temp_edit_value = 800; 
      break;
    }
    case SCREEN_SETTINGS_EMISSIVITY: {
      float* target = &config.ir_emissivity[_selected_menu_item];
      *target += (float)change * 0.01f;
      if (*target < 0.1f) *target = 0.1f;
      if (*target > 1.0f) *target = 1.0f;
      break;
    }
    case SCREEN_SETTINGS_STARTUP: { 
      const int num_items = STARTUP_MODE_COUNT;
      int new_pos = _selected_menu_item + change;
      if (new_pos < 0) new_pos = 0;
      if (new_pos >= num_items) new_pos = num_items - 1;
      _selected_menu_item = new_pos;
      break;
    }
    case SCREEN_SETTINGS_IDLE_OFF: {
      const int num_items = IDLE_OFF_ITEM_COUNT;
      int new_pos = _selected_menu_item + change;
      if (new_pos < 0) new_pos = 0;
      if (new_pos >= num_items) new_pos = num_items - 1;
      _selected_menu_item = new_pos;
      break;
    }
    case SCREEN_SETTINGS_SOUND: { 
      if (change != 0) { 
        config.sound_on = !config.sound_on;
        if (_save_callback) _save_callback(config);
      }
      break;
    }
    case SCREEN_SETTINGS_TEMP_UNIT: {
      int new_pos = _selected_menu_item + change;
      if (new_pos < 0) new_pos = 0;
      if (new_pos > 1) new_pos = 1;
      _selected_menu_item = new_pos;
      break;
    }
    default: return UIManager::handleEncoderRotation(steps, config);;
  }
  return true;
}

float UIManager::convertTemp(float temp_c, char unit) {
  if (unit == 'F' && !isnan(temp_c)) {
    return temp_c * 1.8f + 32.0f;
  }
  return temp_c;
}

// NEW Helper: Converts offsets/deltas without adding 32 degrees
float UIManager::convertDelta(float temp_c, char unit) {
  if (unit == 'F') {
    return temp_c * 1.8f; 
  }
  return temp_c;
}

uint16_t UIManager::getStatusColor(bool is_active, float current_temp, float target_temp) {
  if (!is_active) return COLOR_IDLE;
  if (fabsf(current_temp - target_temp) < 1.0f) return COLOR_DONE;
  return COLOR_ACTIVE;
}

void UIManager::drawStandbyScreen(const AppState& state, const ConfigState& config) {
  
  drawTaskBar();

  int top_offset = 24; 
  int screen_w = _spr.width();
  int screen_h = _spr.height();
  int gap = 6;
  
  int heater_h = 158; 
  int heater_w = (screen_w - (4 * gap)) / 3;
  
  int sensor_y = top_offset + gap + heater_h + gap;
  int sensor_h = screen_h - sensor_y - gap;
  int sensor_w = (screen_w - (3 * gap)) / 2;

  for (int i = 0; i < 3; i++) {
      int x = gap + (i * (heater_w + gap));
      int y = top_offset + gap; 
      
      bool isActive = config.heater_active[i];
      bool isLocked = state.heater_cutoff_state[i];
      bool globalRun = state.is_heating_active;
      
      // Per Diagram: Only grey out Main Heater (index 1) when Auto is actually running in background
      bool isMainHeaterInAutoRunning = (i == 1) && state.auto_running_background;

      // Per Diagram: If Auto is running in background, grey out main_index heater
      uint16_t bg_color;
      if (isMainHeaterInAutoRunning) {
          bg_color = C_GREY_BG;  // Grey out - Auto is using this heater
      } else {
          bg_color = (isActive || isLocked) ? C_WHITE : C_GREY_BG;
      }

      bool isEditingThis = (_current_screen == SCREEN_QUICK_EDIT && _standby_selection == i);
      if (isEditingThis) {
          bg_color = _blink_state ? C_WHITE : C_GREY_BG; 
      }

      _spr.fillRect(x, y, heater_w, heater_h, bg_color);
      int center_x = x + (heater_w / 2);
      
      uint16_t header_color = C_BLACK;
      if (isLocked && _blink_state) header_color = C_RED_TXT;

      _spr.loadFont(Arial18); 
      _spr.setTextColor(header_color, bg_color);
      _spr.setTextDatum(TC_DATUM);
      
      char title[15]; snprintf(title, 15, "Heater %d", i+1);
      _spr.drawString(title, center_x, y + 8); 
      _spr.drawFastHLine(x+5, y+34, heater_w-10, C_BLACK);

      int start_y = y + 36;       
      int val_offset = 14;
      int section_gap = 42;

      if (isEditingThis) {
          _spr.unloadFont(); _spr.loadFont(Arial12); 
          _spr.setTextColor(C_BLACK, bg_color);
          _spr.drawString("SET", center_x, start_y);

          _spr.unloadFont(); _spr.loadFont(Arial18);
          float t_set = convertTemp(config.target_temps[i], state.temp_unit);
          char buf[20];
          snprintf(buf, 20, "%.1f%c", t_set, state.temp_unit);
          
          uint16_t val_col = (_quick_edit_step == Q_EDIT_TARGET) ? C_RED_TXT : C_BLACK;
          _spr.setTextColor(val_col, bg_color);
          _spr.drawString(buf, center_x, start_y + val_offset);

          _spr.unloadFont(); _spr.loadFont(Arial12);
          _spr.setTextColor(C_BLACK, bg_color);
          int max_y = start_y + section_gap;
          _spr.drawString("MAX", center_x, max_y);

          _spr.unloadFont(); _spr.loadFont(Arial18);
          float t_max = convertTemp(config.max_temps[i], state.temp_unit);
          snprintf(buf, 20, "%.1f%c", t_max, state.temp_unit);
          
          val_col = (_quick_edit_step == Q_EDIT_MAX) ? C_RED_TXT : C_BLACK;
          _spr.setTextColor(val_col, bg_color);
          _spr.drawString(buf, center_x, max_y + val_offset);
      }
      else {
          _spr.unloadFont(); _spr.loadFont(Arial12);
          _spr.setTextColor(C_BLACK, bg_color);
          _spr.drawString("NOW", center_x, start_y);
          
          _spr.unloadFont(); _spr.loadFont(Arial18);
          uint16_t val_color = (isLocked && _blink_state) ? C_RED_TXT : C_BLACK;
          char buf[20];
          
          if (isnan(state.tc_temps[i])) {
            snprintf(buf, 20, "---%c", state.temp_unit);
            _spr.setTextColor(C_GREY_TXT, bg_color);
          } else {
            float t_now = convertTemp(state.tc_temps[i], state.temp_unit);
            snprintf(buf, 20, "%.1f%c", t_now, state.temp_unit); 
            _spr.setTextColor(val_color, bg_color);
          }
          _spr.drawString(buf, center_x, start_y + val_offset);

          int set_y = start_y + section_gap;
          _spr.unloadFont(); _spr.loadFont(Arial12);
          _spr.setTextColor(C_BLACK, bg_color);
          _spr.drawString("SET", center_x, set_y);
          
          _spr.unloadFont(); _spr.loadFont(Arial18);
          float t_set = convertTemp(config.target_temps[i], state.temp_unit);
          snprintf(buf, 20, "%.1f%c", t_set, state.temp_unit);
          _spr.setTextColor(C_BLACK, bg_color);
          _spr.drawString(buf, center_x, set_y + val_offset);

          int status_y = set_y + section_gap;
          _spr.unloadFont(); _spr.loadFont(Arial12);
          _spr.setTextColor(C_BLACK, bg_color);
          _spr.drawString("Status", center_x, status_y);

          _spr.unloadFont(); _spr.loadFont(Arial18);
          
          // Per Diagram: Check if this is Main Heater (index 1) and Auto is running in background
          bool isMainHeaterInAuto = (i == 1) && state.auto_running_background;
          
          if (isLocked) {
              uint16_t lock_color = _blink_state ? C_RED_TXT : bg_color; 
              _spr.setTextColor(lock_color, bg_color);
              _spr.drawString("Lock", center_x, status_y + val_offset);
          } 
          else if (isMainHeaterInAuto) {
              // Per Diagram: Main Heater when Auto is running in background
              // Grey out and show "In-use"
              _spr.setTextColor(C_GREY_TXT, bg_color);
              _spr.drawString("In-use", center_x, status_y + val_offset);
          }
          else if (!isActive) {
              _spr.setTextColor(C_GREY_TXT, bg_color); 
              _spr.drawString("OFF", center_x, status_y + val_offset);
          }
          else {
              // เช็คว่าควรแสดง Standby หรือไม่
              // 1. ถ้า Global Run ไม่ทำงาน (ปกติ)
              // 2. หรือถ้า Auto ทำงานอยู่ (background) -> Heater 1,3 ต้องเป็น Standby (เพราะ Auto คุมอยู่)
              bool force_standby = false;
              if (i != 1 && state.auto_running_background) {
                  force_standby = true;
              }

              if (!globalRun || force_standby) {
                  _spr.setTextColor(TFT_BLUE, bg_color); 
                  _spr.drawString("Standby", center_x, status_y + val_offset);
              } 
              else {
                  // แสดง Heating/Ready เมื่อ:
                  // 1. Global Run ทำงาน
                  // 2. และไม่ใช่เงื่อนไข force_standby ข้างบน
                  if (state.heater_ready[i]) {
                      _spr.setTextColor(C_GREEN_TXT, bg_color); 
                      _spr.drawString("Ready", center_x, status_y + val_offset);
                  } else {
                      _spr.setTextColor(TFT_ORANGE, bg_color); 
                      _spr.drawString("Heating", center_x, status_y + val_offset);
                  }
              }
          }
      } 
      _spr.unloadFont();

      if (_standby_selection == i) {
          _spr.drawRect(x, y, heater_w, heater_h, TFT_RED);
          _spr.drawRect(x+1, y+1, heater_w-2, heater_h-2, TFT_RED);
      } else {
          _spr.drawRect(x, y, heater_w, heater_h, C_BLACK);
      }
  }

  _spr.fillRect(gap, sensor_y, sensor_w, sensor_h, C_PINK_BG);
  _spr.loadFont(Arial18);
  _spr.setTextColor(C_BLACK, C_PINK_BG);
  _spr.setTextDatum(ML_DATUM);
  
  char ir_buf[30];
  
  if (isnan(state.ir_temps[0])) {
    snprintf(ir_buf, 30, "IR1 : ---%c", state.temp_unit);
  } else {
    float ir1 = convertTemp(state.ir_temps[0], state.temp_unit);
    snprintf(ir_buf, 30, "IR1 : %.1f%c", ir1, state.temp_unit);
  }
  _spr.drawString(ir_buf, gap + 10, sensor_y + (sensor_h/4) + 2);

  if (isnan(state.ir_temps[1])) {
    snprintf(ir_buf, 30, "IR2 : ---%c", state.temp_unit);
  } else {
    float ir2 = convertTemp(state.ir_temps[1], state.temp_unit);
    snprintf(ir_buf, 30, "IR2 : %.1f%c", ir2, state.temp_unit);
  }
  _spr.drawString(ir_buf, gap + 10, sensor_y + (sensor_h*3/4) - 1);
  _spr.unloadFont();

  int tc_x = gap + sensor_w + gap;
  _spr.fillRect(tc_x, sensor_y, sensor_w, sensor_h, C_LIME_BG);
  
  _spr.loadFont(Arial18);
  _spr.setTextColor(C_BLACK, C_LIME_BG);
  _spr.setTextDatum(MC_DATUM);
  
  if (isnan(state.tc_probe_temp)) {
    snprintf(ir_buf, 30, "TC Wire : ---%c", state.temp_unit);
  } else {
    float wire_t = convertTemp(state.tc_probe_temp, state.temp_unit);
    snprintf(ir_buf, 30, "TC Wire : %.1f%c", wire_t, state.temp_unit);
  }
  _spr.drawString(ir_buf, tc_x + (sensor_w/2), sensor_y + (sensor_h/2));
  _spr.unloadFont();
}

void UIManager::drawAutoModeScreen(const AppState& state, const ConfigState& config) {
    // ---------------------------------------------------------
    // 1. Taskbar + Header Section
    // ---------------------------------------------------------
    drawTaskBar(); // Add taskbar at the top
    
    int w = _spr.width();
    int header_y = 24; // Start below taskbar
    int header_h = 30;
    
    _spr.fillRect(0, header_y, w, header_h, C_BLACK); // Header BG
    
    _spr.loadFont(Arial18);
    _spr.setTextColor(TFT_WHITE, C_BLACK);
    _spr.setTextDatum(ML_DATUM);
    _spr.drawString("Auto mode", 5, header_y + 15);

    _spr.setTextDatum(MR_DATUM);
    char buf[40];
    // แสดงค่า IR1 แทน TC
    float ir1_temp = state.ir_temps[0];
    if (isnan(ir1_temp)) {
        snprintf(buf, 40, "Obj. Temp : ---%c", state.temp_unit);
    } else {
        float current_temp_display = convertTemp(ir1_temp, state.temp_unit);
        snprintf(buf, 40, "Obj. Temp : %.1f%c", current_temp_display, state.temp_unit);
    }
    _spr.drawString(buf, w - 5, header_y + 15);
    _spr.unloadFont();

    // ---------------------------------------------------------
    // 2. Cycle Boxes (Cycle 1, 2, 3)
    // ---------------------------------------------------------
    int gap = 4;
    int box_w = (w - (4 * gap)) / 3;
    int box_h = 120;
    int start_y = header_y + header_h + 5; // Start below header

    for (int i = 0; i < 3; i++) { // Loop 0-2 (Cycle 1-3)
        int x = gap + (i * (box_w + gap));
        
        // --- Determine Cycle State ---
        // Per Diagram: Check if Manual/Standby is running in background
        bool isManualRunningInBackground = state.manual_running_background;
        
        bool isThisCycleActive = (state.auto_step == (i + 1));
        bool globalRun = state.is_heating_active;
        bool isLocked = state.heater_cutoff_state[1]; // Main heater (index 1)
        bool isReady = state.heater_ready[1];          // Main heater (index 1)
        
        // --- Selection & Background Logic ---
        bool isSelected = (_auto_selection == i);
        uint16_t bg_color = TFT_WHITE;
        
        // Per Diagram: If Manual is running in background -> Grey out ALL cycles
        if (isManualRunningInBackground) {
            bg_color = C_GREY_BG;  // Grey out
        }
        // ถ้ากำลัง Edit ช่องนี้อยู่ ให้กระพริบพื้นหลัง (Blink)
        else {
            bool isEditingThis = (_current_screen == SCREEN_QUICK_EDIT_AUTO && isSelected);
            if (isEditingThis) {
                bg_color = _blink_state ? TFT_WHITE : C_GREY_BG; 
            } 
            // Priority 2: Active Cycle (Running) -> ใช้สีเขียวเข้ม (Darker Lime) เป็นพื้นหลัง
            else if (isThisCycleActive && globalRun) {
                 bg_color = C_ACTIVE_CYCLE_BG; 
            }
            // Priority 3: Normal
            else {
                 bg_color = TFT_WHITE; 
            }
        }

        _spr.fillRect(x, start_y, box_w, box_h, bg_color);

        // --- Border Logic ---
        // Active Cycle gets RED Border
        if (isSelected) {
             _spr.drawRect(x, start_y, box_w, box_h, TFT_RED);
             _spr.drawRect(x+1, start_y+1, box_w-2, box_h-2, TFT_RED);
        } else {
             _spr.drawRect(x, start_y, box_w, box_h, TFT_BLACK);
        }

        // --- Cycle Title ---
        _spr.loadFont(Arial18);
        _spr.setTextColor(C_BLACK, bg_color);
        _spr.setTextDatum(TC_DATUM);
        snprintf(buf, 20, "Cycle %d", i+1);
        _spr.drawString(buf, x + (box_w/2), start_y + 5); 
        _spr.drawFastHLine(x+5, start_y + 28, box_w-10, C_BLACK);

        // --- Display Values (SET & MAX) ---
        float t_set = convertTemp(config.auto_target_temps[i], state.temp_unit);
        float t_max = convertTemp(config.auto_max_temps[i], state.temp_unit);

        // -- SET Line --
        _spr.loadFont(Arial12);
        _spr.setTextColor(C_BLACK, bg_color);
        _spr.drawString("SET", x + (box_w/2), start_y + 32);

        _spr.loadFont(Arial18);
        bool isEditingThis = (_current_screen == SCREEN_QUICK_EDIT_AUTO && isSelected);
        // ถ้ากำลังแก้ค่า SET ให้ตัวเลขเป็นสีแดง
        uint16_t val_col = (isEditingThis && _quick_edit_step == Q_EDIT_TARGET) ? C_RED_TXT : C_BLACK;
        _spr.setTextColor(val_col, bg_color);
        snprintf(buf, 20, "%.1f%c", t_set, state.temp_unit);  // เปลี่ยนเป็น .1f
        _spr.drawString(buf, x + (box_w/2), start_y + 45);

        // -- MAX Line (Show in QuickEdit mode, Status in normal mode) --
        _spr.loadFont(Arial12);
        _spr.setTextColor(C_BLACK, bg_color);
        _spr.setTextDatum(TC_DATUM);
        
        if (isEditingThis) {
            // In QuickEdit mode: Show MAX label and value
            _spr.drawString("MAX", x + (box_w/2), start_y + 65);
            
            _spr.loadFont(Arial18);
            // ถ้ากำลังแก้ค่า MAX ให้ตัวเลขเป็นสีแดง
            val_col = (_quick_edit_step == Q_EDIT_MAX) ? C_RED_TXT : C_BLACK;
            _spr.setTextColor(val_col, bg_color);
            snprintf(buf, 20, "%.1f%c", t_max, state.temp_unit);  // เปลี่ยนเป็น .1f
            _spr.drawString(buf, x + (box_w/2), start_y + 85);
        } else {
            // Normal mode: Show Status label and state
            _spr.drawString("Status", x + (box_w/2), start_y + 65);
            
            _spr.loadFont(Arial18);
            
            // Per Diagram: If Manual running in background -> show "In-use"
            if (isManualRunningInBackground) {
                _spr.setTextColor(C_GREY_TXT, bg_color);
                _spr.drawString("In-use", x + (box_w/2), start_y + 85);
            }
            // State Logic (same as standby mode)
            else if (isLocked) {
                // Lock state - blink red
                uint16_t lock_color = _blink_state ? C_RED_TXT : bg_color; 
                _spr.setTextColor(lock_color, bg_color);
                _spr.drawString("Lock", x + (box_w/2), start_y + 85);
            } 
            else if (!isThisCycleActive) {
                // Not the active cycle - show Standby
                _spr.setTextColor(TFT_BLUE, bg_color); 
                _spr.drawString("Standby", x + (box_w/2), start_y + 85);
            }
            else if (!globalRun) {
                // Active cycle but not running - show Standby
                _spr.setTextColor(TFT_BLUE, bg_color); 
                _spr.drawString("Standby", x + (box_w/2), start_y + 85);
            } 
            else {
                // Active cycle and running - check if heating or ready
                if (isReady) {
                    _spr.setTextColor(C_GREEN_TXT, bg_color); 
                    _spr.drawString("Ready", x + (box_w/2), start_y + 85);
                } else {
                    int badge_w = 80;   // ความกว้างกล่อง
                    int badge_h = 26;   // ความสูงกล่อง
                    int center_x = x + (box_w / 2);
                    int pos_y = start_y + 85; // ตำแหน่ง Y เดิม

                    // 1. วาดกล่องสี่เหลี่ยมสีขาว (จัดกึ่งกลางแนวนอน)
                    _spr.fillRect(center_x - (badge_w / 2), pos_y, badge_w, badge_h, TFT_WHITE);
                    
                    // 2. วาดตัวหนังสือ "Heating" ตรงกลางกล่อง
                    _spr.setTextDatum(MC_DATUM); // เปลี่ยนจุดอ้างอิงเป็น Middle-Center ชั่วคราว
                    _spr.setTextColor(TFT_ORANGE, bg_color); 
                    _spr.drawString("Heating", center_x, pos_y + (badge_h / 2));
                    
                    _spr.setTextDatum(TC_DATUM);
                }
            }
        }
        _spr.unloadFont();
    }

    // ---------------------------------------------------------
    // 3. Bottom Sensors (IR1, IR2, TC Wire)
    // ---------------------------------------------------------
    int bot_y = start_y + box_h + 5;
    int bot_h = _spr.height() - bot_y - 5;
    int sens_w = (w - (3 * gap)) / 2;

    // Left Box (IR) - Pink Background
    _spr.fillRect(gap, bot_y, sens_w, bot_h, C_PINK_BG);
    _spr.loadFont(Arial18);
    _spr.setTextColor(C_BLACK, C_PINK_BG);
    _spr.setTextDatum(ML_DATUM);
    
    // IR1
    float ir1 = convertTemp(state.ir_temps[0], state.temp_unit);
    if(isnan(state.ir_temps[0])) snprintf(buf, 40, "IR1 : ---");
    else snprintf(buf, 40, "IR1 : %.1f%c", ir1, state.temp_unit);
    _spr.drawString(buf, gap + 10, bot_y + (bot_h/4));

    // IR2
    float ir2 = convertTemp(state.ir_temps[1], state.temp_unit);
    if(isnan(state.ir_temps[1])) snprintf(buf, 40, "IR2 : ---");
    else snprintf(buf, 40, "IR2 : %.1f%c", ir2, state.temp_unit);
    _spr.drawString(buf, gap + 10, bot_y + (bot_h*3/4));

    // Right Box (TC Wire) - Lime Background
    int tc_x = gap + sens_w + gap;
    _spr.fillRect(tc_x, bot_y, sens_w, bot_h, C_LIME_BG);
    _spr.setTextColor(C_BLACK, C_LIME_BG);
    _spr.setTextDatum(MC_DATUM);
    
    float wire = convertTemp(state.tc_probe_temp, state.temp_unit);
    if(isnan(state.tc_probe_temp)) snprintf(buf, 40, "TC Wire : ---");
    else snprintf(buf, 40, "TC Wire : %.1f%c", wire, state.temp_unit);
    _spr.drawString(buf, tc_x + (sens_w/2), bot_y + (bot_h/2));
    
    _spr.unloadFont();
}