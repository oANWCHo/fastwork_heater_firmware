#include "ui_manager.h"
// 1. Include ไฟล์ฟอนต์ที่อัปโหลดมา
#include "Arial18.h" 
#include "Arial12.h"

#define COLOR_IDLE   0x39E7 // Dark Grey
#define COLOR_ACTIVE 0x05E0 // Dark Green
#define COLOR_DONE   TFT_BLUE
#define COLOR_WARN   TFT_ORANGE

#define COLOR_START   0x07E0 // Green
#define COLOR_STOP    0xF800 // Red

#define C_WHITE     0xFFFF // #ffffff
#define C_GREY_BG   0xACB9 // #acb8c0
#define C_PINK_BG   0xF71F // #f9e4ff
#define C_LIME_BG   0xE7F8 // #e4ffc3
#define C_RED_TXT   0xF986 // #ff3131
#define C_GREEN_TXT 0x4D6A // #4cd964 (Approx Green)
#define C_BLACK     0x0000
#define C_GREY_TXT  0x7BEF // Dark Grey for OFF text

const uint32_t INACTIVITY_TIMEOUT_MS = 10000;

const char* menu_item_labels_page_1[MENU_PAGE1_ITEM_COUNT] = {
  "Max Temp Lock",
  "Heater Calibration", 
  "IR Emissivity",
  "Next Page >"
};

const char* menu_item_labels_page_2[MENU_PAGE2_ITEM_COUNT] = {
  "Turn off When Idle", "Startup Mode", "Sound","TC Probe Cal", "Temp Unit", "About", 
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

UIManager::UIManager(TFT_eSPI* tft, ConfigSaveCallback save_callback) 
  : _tft(tft), _spr(_tft), _save_callback(save_callback) {
  _current_screen = SCREEN_STANDBY;
  _quick_edit_step = Q_EDIT_TARGET;
  _blink_state = false;
  _selected_menu_item = 0;
  _selected_menu_item_page_2 = 0; 
  _menu_step_accumulator = 0.0f;
  _standby_selection = 0; 
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
    _current_screen = SCREEN_SETTINGS_PAGE_1;
    _selected_menu_item = 0;
    resetInactivityTimer();
}

void UIManager::exitSettings() {
    _current_screen = SCREEN_STANDBY;
    resetInactivityTimer();
}

void UIManager::enterQuickEdit() {
    // Only enter if we are highlighting a valid heater (0-2)
    if (_standby_selection >= 0 && _standby_selection <= 2) {
        _current_screen = SCREEN_QUICK_EDIT;
        _quick_edit_step = Q_EDIT_TARGET; // Start at SET
        _menu_step_accumulator = 0.0f;
        resetInactivityTimer();
    }
}

void UIManager::draw(const AppState& state, const ConfigState& config) {
  static uint32_t last_blink_time = 0;
  if (millis() - last_blink_time > 500) { // Blink speed 500ms
    _blink_state = !_blink_state;
    last_blink_time = millis();
  }

  _spr.fillSprite(C_BLACK); // Clear with Black

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
    case SCREEN_QUICK_EDIT:               drawStandbyScreen(state, config); break;
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

void UIManager::drawTaskBar() {
    int w = _spr.width();
    int h = 24; // Task bar height
    
    // Draw Background
    _spr.fillRect(0, 0, w, h, C_BLACK);
    _spr.drawFastHLine(0, h-1, w, TFT_DARKGREY);

    // Draw Greyed-out Icons (Right Aligned)
    int icon_x = w - 25;
    int icon_y = h / 2;
    
    // Warning Icon (!)
    _spr.drawCircle(icon_x, icon_y, 8, TFT_DARKGREY);
    _spr.drawLine(icon_x, icon_y-3, icon_x, icon_y+2, TFT_DARKGREY);
    _spr.drawPixel(icon_x, icon_y+5, TFT_DARKGREY);
    
    // Bluetooth (Simple B)
    icon_x -= 25;
    _spr.drawString("BT", icon_x, icon_y, 1); // Font 1 (Small)

    // Wifi (Arc)
    icon_x -= 25;
    _spr.drawCircle(icon_x, icon_y+4, 2, TFT_DARKGREY);
    _spr.drawArc(icon_x, icon_y+4, 6, 8, 220, 320, TFT_DARKGREY, C_BLACK);
    
    // Title (Left)
    _spr.setTextColor(TFT_LIGHTGREY, C_BLACK);
    _spr.setTextDatum(ML_DATUM);
    _spr.drawString("Smart Heater", 5, icon_y);
}

void UIManager::drawSettingsTCProbeCal(const AppState& state, const ConfigState& config) {
  _temp_edit_value = state.tc_probe_temp;
  
  _spr.fillSprite(TFT_BLACK);
  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.setTextDatum(TC_DATUM);
  
  _spr.loadFont(Arial18); // Load Font
  _spr.drawString("TC Probe Cal", _spr.width() / 2, 10);

  _spr.setTextDatum(MC_DATUM);

  char buf[32];
  float raw = state.tc_probe_temp - config.tc_probe_offset;
  
  _spr.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  snprintf(buf, 32, "Raw: %.2f C", raw);
  _spr.drawString(buf, _spr.width() / 2, 50);

  _spr.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  snprintf(buf, 32, "Offset: %.2f", config.tc_probe_offset);
  _spr.drawString(buf, _spr.width() / 2, 75);

  _spr.setTextColor(TFT_GREEN, TFT_BLACK);
  snprintf(buf, 32, "Val: %.2f C", state.tc_probe_temp);
  _spr.drawString(buf, _spr.width() / 2, 100);

  int start_y = 140;
  const char* options[] = {"Tare (Set 0)", "Reset (0.0)"};
  for (int i = 0; i < 2; i++) {
     if (i == _selected_menu_item) {
        _spr.setTextColor(TFT_BLACK, TFT_YELLOW);
     } else {
        _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
     }
     _spr.drawString(options[i], _spr.width() / 2, start_y + (i * 30));
  }
  
  _spr.unloadFont(); // Unload Font for footer
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Press: Action | Dbl: Back", _spr.width() / 2, _spr.height() - 10);
}

void UIManager::drawSettingsStartup(const AppState& state, const ConfigState& config) {
  _spr.fillSprite(TFT_BLACK);
  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.setTextDatum(TC_DATUM);
  
  _spr.loadFont(Arial18);
  _spr.drawString("Startup Mode", _spr.width() / 2, 20);

  for (int i = 0; i < STARTUP_MODE_COUNT; ++i) {
    _spr.setTextDatum(MC_DATUM);
    if (i == _selected_menu_item) {
        _spr.setTextColor(TFT_BLACK, TFT_YELLOW);
    } else {
        _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
    }
    _spr.drawString(startup_mode_labels[i], _spr.width() / 2, 80 + i * 40);
  }
  
  _spr.unloadFont();
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Select | Press: Save", _spr.width() / 2, _spr.height() - 10);
}

bool UIManager::handleButtonSingleClick(ConfigState& config, float& go_to, bool& has_go_to) {
  if (_current_screen == SCREEN_SLEEP) {
    _current_screen = SCREEN_STANDBY;
    resetInactivityTimer();
    return true; 
  }
  resetInactivityTimer();
  switch (_current_screen) {
    case SCREEN_STANDBY:
        // Toggle Heater Active State directly
        if (_standby_selection >= 0 && _standby_selection <= 2) {
            config.heater_active[_standby_selection] = !config.heater_active[_standby_selection];
            if (_save_callback) _save_callback(config);
        }
        return true;
    case SCREEN_QUICK_EDIT:
        if (_quick_edit_step == Q_EDIT_TARGET) {
            // Clicked while on SET -> Move to MAX
            _quick_edit_step = Q_EDIT_MAX;
        } else {
            // Clicked while on MAX -> Save and Exit
            if (_save_callback) _save_callback(config);
            _current_screen = SCREEN_STANDBY;
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
          break;
        case MENU_PAGE1_EMISSIVITY: 
          _current_screen = SCREEN_SETTINGS_EMISSIVITY;
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
        case MENU_PAGE2_TC_PROBE_CAL: _current_screen = SCREEN_SETTINGS_TC_PROBE_CAL; _selected_menu_item = 0; break;
        case MENU_PAGE2_TEMP_UNIT: _current_screen = SCREEN_SETTINGS_TEMP_UNIT; _selected_menu_item = (config.temp_unit == 'C') ? 0 : 1; break;
        case MENU_PAGE2_ABOUT: _current_screen = SCREEN_SETTINGS_ABOUT; break;
        case MENU_PAGE2_PREV_PAGE: _current_screen = SCREEN_SETTINGS_PAGE_1; _selected_menu_item = 0; break;
      }
      _menu_step_accumulator = 0.0f;
      return true;
    case SCREEN_SETTINGS_TC_PROBE_CAL:
      if (_selected_menu_item == 0) config.tc_probe_offset = config.tc_probe_offset - _temp_edit_value;
      else config.tc_probe_offset = 0.0f;
      if (_save_callback) _save_callback(config);
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
    case SCREEN_SETTINGS_CALIBRATION_SELECT:
      _current_screen = SCREEN_SETTINGS_HEATER_CALIBRATE;
      _temp_edit_value = config.tc_offsets[_selected_menu_item];
      _menu_step_accumulator = 0.0f;
      return true;
    case SCREEN_SETTINGS_HEATER_CALIBRATE:
      config.tc_offsets[_selected_menu_item] = _temp_edit_value; 
      if (_save_callback) _save_callback(config);
      _current_screen = SCREEN_SETTINGS_CALIBRATION_SELECT; 
      _menu_step_accumulator = 0.0f;
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
    case SCREEN_SETTINGS_SOUND: return true;
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
    case SCREEN_SETTINGS_CALIBRATION_SELECT: _current_screen = SCREEN_SETTINGS_PAGE_1; break;
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
    case SCREEN_QUICK_EDIT: {
        int heaterIdx = _standby_selection;
        if (_quick_edit_step == Q_EDIT_TARGET) {
            // Edit Target Temp
            config.target_temps[heaterIdx] += (float)change * 0.5f; // 0.5 step
            // Constrain
            if (config.target_temps[heaterIdx] < 0) config.target_temps[heaterIdx] = 0;
            if (config.target_temps[heaterIdx] > config.max_temp_lock) 
                config.target_temps[heaterIdx] = config.max_temp_lock;
        } else {
            // Edit Max Temp
            config.max_temps[heaterIdx] += (float)change * 0.5f;
            // Constrain
            if (config.max_temps[heaterIdx] < config.target_temps[heaterIdx]) 
                config.max_temps[heaterIdx] = config.target_temps[heaterIdx];
            if (config.max_temps[heaterIdx] > config.max_temp_lock) 
                config.max_temps[heaterIdx] = config.max_temp_lock;
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
    case SCREEN_SETTINGS_CALIBRATION_SELECT: {
      const int num_items = 3; 
      int new_pos = _selected_menu_item + change;
      if (new_pos < 0) new_pos = 0;
      if (new_pos >= num_items) new_pos = num_items - 1;
      _selected_menu_item = new_pos;
      break;
    }
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
      _temp_edit_value += (float)change * 0.1f;
      if (_temp_edit_value < -20.0f) _temp_edit_value = -20.0f;
      if (_temp_edit_value > 20.0f) _temp_edit_value = 20.0f;
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
  
  // --- LAYOUT ADJUSTMENTS ---
  int heater_h = 158; 
  int heater_w = (screen_w - (4 * gap)) / 3;
  
  int sensor_y = top_offset + gap + heater_h + gap;
  int sensor_h = screen_h - sensor_y - gap;
  int sensor_w = (screen_w - (3 * gap)) / 2;

  // --- DRAW 3 HEATERS ---
  for (int i = 0; i < 3; i++) {
      int x = gap + (i * (heater_w + gap));
      int y = top_offset + gap; 
      
      bool isActive = config.heater_active[i];
      bool isLocked = state.heater_cutoff_state[i];
      bool globalRun = state.is_heating_active;

      // Background Color
      uint16_t bg_color = (isActive || isLocked) ? C_WHITE : C_GREY_BG;

      // Quick Edit Highlight
      bool isEditingThis = (_current_screen == SCREEN_QUICK_EDIT && _standby_selection == i);
      if (isEditingThis) {
          bg_color = _blink_state ? C_WHITE : C_GREY_BG; 
      }

      _spr.fillRect(x, y, heater_w, heater_h, bg_color);
      int center_x = x + (heater_w / 2);
      
      // HEADER
      uint16_t header_color = C_BLACK;
      if (isLocked && _blink_state) header_color = C_RED_TXT;

      _spr.loadFont(Arial18); 
      _spr.setTextColor(header_color, bg_color);
      _spr.setTextDatum(TC_DATUM);
      
      char title[15]; snprintf(title, 15, "Heater %d", i+1);
      _spr.drawString(title, center_x, y + 8); 
      _spr.drawFastHLine(x+5, y+34, heater_w-10, C_BLACK);

      // --- TIGHTER CONTENT SPACING ---
      int start_y = y + 36;       
      int val_offset = 14;        // Gap between Label and Value
      int section_gap = 42;       // Gap between sections

      if (isEditingThis) {
          // === QUICK EDIT VIEW ===
          // [REQ 2] Use Arial12 for Labels (SET/MAX) to match Normal View
          _spr.unloadFont(); _spr.loadFont(Arial12); 
          _spr.setTextColor(C_BLACK, bg_color);
          _spr.drawString("SET", center_x, start_y);

          // Value uses Arial18
          _spr.unloadFont(); _spr.loadFont(Arial18);
          float t_set = convertTemp(config.target_temps[i], state.temp_unit);
          char buf[20];
          snprintf(buf, 20, "%.1f%c", t_set, state.temp_unit);
          
          uint16_t val_col = (_quick_edit_step == Q_EDIT_TARGET) ? C_RED_TXT : C_BLACK;
          _spr.setTextColor(val_col, bg_color);
          _spr.drawString(buf, center_x, start_y + val_offset);

          // MAX Label (Arial12)
          _spr.unloadFont(); _spr.loadFont(Arial12);
          _spr.setTextColor(C_BLACK, bg_color);
          int max_y = start_y + section_gap;
          _spr.drawString("MAX", center_x, max_y);

          // Value (Arial18)
          _spr.unloadFont(); _spr.loadFont(Arial18);
          float t_max = convertTemp(config.max_temps[i], state.temp_unit);
          snprintf(buf, 20, "%.1f%c", t_max, state.temp_unit);
          
          val_col = (_quick_edit_step == Q_EDIT_MAX) ? C_RED_TXT : C_BLACK;
          _spr.setTextColor(val_col, bg_color);
          _spr.drawString(buf, center_x, max_y + val_offset);
      }
      else {
          // === NORMAL VIEW ===
          
          // 1. NOW Section
          _spr.unloadFont(); _spr.loadFont(Arial12);
          _spr.setTextColor(C_BLACK, bg_color);
          _spr.drawString("NOW", center_x, start_y);
          
          _spr.unloadFont(); _spr.loadFont(Arial18);
          uint16_t val_color = (isLocked && _blink_state) ? C_RED_TXT : C_BLACK;
          char buf[20];
          float t_now = isnan(state.tc_temps[i]) ? 0.0 : convertTemp(state.tc_temps[i], state.temp_unit);
          snprintf(buf, 20, "%.1f%c", t_now, state.temp_unit); 
          _spr.setTextColor(val_color, bg_color); 
          _spr.drawString(buf, center_x, start_y + val_offset);

          // 2. SET Section
          int set_y = start_y + section_gap;
          _spr.unloadFont(); _spr.loadFont(Arial12);
          _spr.setTextColor(C_BLACK, bg_color);
          _spr.drawString("SET", center_x, set_y);
          
          _spr.unloadFont(); _spr.loadFont(Arial18);
          float t_set = convertTemp(config.target_temps[i], state.temp_unit);
          snprintf(buf, 20, "%.1f%c", t_set, state.temp_unit);
          _spr.setTextColor(C_BLACK, bg_color);
          _spr.drawString(buf, center_x, set_y + val_offset);

          // 3. STATUS Section
          int status_y = set_y + section_gap;
          _spr.unloadFont(); _spr.loadFont(Arial12);
          _spr.setTextColor(C_BLACK, bg_color);
          _spr.drawString("Status", center_x, status_y);

          _spr.unloadFont(); _spr.loadFont(Arial18);
          
          // [UPDATED PRIORITY LOGIC]
          if (isLocked) {
              // Priority 1: Lock (Overrides everything else)
              uint16_t lock_color = _blink_state ? C_RED_TXT : bg_color; 
              _spr.setTextColor(lock_color, bg_color);
              _spr.drawString("Lock", center_x, status_y + val_offset);
          } 
          else if (!isActive) {
              // Priority 2: OFF
              _spr.setTextColor(C_GREY_TXT, bg_color); 
              _spr.drawString("OFF", center_x, status_y + val_offset);
          }
          else if (!globalRun) {
              // Priority 3: Standby (Active but system stopped)
              _spr.setTextColor(TFT_BLUE, bg_color); 
              _spr.drawString("Standby", center_x, status_y + val_offset);
          } 
          else {
              // Priority 4: Running Status
              if (state.heater_ready[i]) {
                  _spr.setTextColor(C_GREEN_TXT, bg_color); 
                  _spr.drawString("Ready", center_x, status_y + val_offset);
              } else {
                  _spr.setTextColor(TFT_ORANGE, bg_color); 
                  _spr.drawString("Heating", center_x, status_y + val_offset);
              }
          }
      } 
      _spr.unloadFont();

      // Border Logic
      if (_standby_selection == i) {
          _spr.drawRect(x, y, heater_w, heater_h, C_BLACK);
          _spr.drawRect(x+1, y+1, heater_w-2, heater_h-2, C_BLACK);
      } else {
          _spr.drawRect(x, y, heater_w, heater_h, C_BLACK);
      }
  }

  // --- DRAW BOTTOM SENSORS ---
  _spr.fillRect(gap, sensor_y, sensor_w, sensor_h, C_PINK_BG);
  _spr.loadFont(Arial18);
  _spr.setTextColor(C_BLACK, C_PINK_BG);
  _spr.setTextDatum(ML_DATUM);
  
  char ir_buf[30];
  float ir1 = isnan(state.ir_temps[0]) ? 0.0 : convertTemp(state.ir_temps[0], state.temp_unit);
  float ir2 = isnan(state.ir_temps[1]) ? 0.0 : convertTemp(state.ir_temps[1], state.temp_unit);

  snprintf(ir_buf, 30, "IR1 : %.1f%c", ir1, state.temp_unit);
  _spr.drawString(ir_buf, gap + 10, sensor_y + (sensor_h/4) + 2);

  snprintf(ir_buf, 30, "IR2 : %.1f%c", ir2, state.temp_unit);
  _spr.drawString(ir_buf, gap + 10, sensor_y + (sensor_h*3/4) - 1);
  _spr.unloadFont();

  // TC Wire Panel
  int tc_x = gap + sensor_w + gap;
  _spr.fillRect(tc_x, sensor_y, sensor_w, sensor_h, C_LIME_BG);
  
  _spr.loadFont(Arial18);
  _spr.setTextColor(C_BLACK, C_LIME_BG);
  _spr.setTextDatum(MC_DATUM);
  
  float wire_t = isnan(state.tc_probe_temp) ? 0.0 : convertTemp(state.tc_probe_temp, state.temp_unit);
  snprintf(ir_buf, 30, "TC Wire : %.1f%c", wire_t, state.temp_unit);
  _spr.drawString(ir_buf, tc_x + (sensor_w/2), sensor_y + (sensor_h/2));
  _spr.unloadFont();
}

void UIManager::drawSettingsEmissivity(const AppState& state, const ConfigState& config) {
  _spr.fillSprite(TFT_BLACK);
  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.setTextDatum(TC_DATUM);
  
  _spr.loadFont(Arial18);
  _spr.drawString("IR Emissivity", _spr.width() / 2, 20);

  // IR 1
  _spr.setTextDatum(MC_DATUM);
  if (_selected_menu_item == 0) _spr.setTextColor(TFT_BLACK, TFT_YELLOW);
  else _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  
  char buf[32];
  snprintf(buf, 32, "IR1 (%.2f): %.2f", state.ir_temps[0], config.ir_emissivity[0]);
  _spr.drawString(buf, _spr.width() / 2, 80);

  // IR 2
  bool ir2_connected = !isnan(state.ir_temps[1]);
  if (_selected_menu_item == 1) {
    _spr.setTextColor(TFT_BLACK, ir2_connected ? TFT_YELLOW : TFT_LIGHTGREY);
  } else {
    _spr.setTextColor(ir2_connected ? TFT_YELLOW : TFT_DARKGREY, TFT_BLACK);
  }

  snprintf(buf, 32, "IR2 (%.2f): %.2f", state.ir_temps[1], config.ir_emissivity[1]);
  if (!ir2_connected) snprintf(buf, 32, "IR2: Not Connected");
  _spr.drawString(buf, _spr.width() / 2, 120);

  _spr.unloadFont();
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Adjust | Press: Switch | Dbl: Back", _spr.width() / 2, _spr.height() - 10);
}

void UIManager::drawSettingsPage1(const AppState& state, const ConfigState& config) {
  _spr.fillSprite(TFT_BLACK);
  _spr.setTextColor(TFT_WHITE, TFT_BLACK); 
  _spr.setTextDatum(TC_DATUM); 
  
  _spr.loadFont(Arial18);
  _spr.drawString("Settings - Page 1", _spr.width() / 2, 4); 

  const int ITEM_HEIGHT = 24;
  const int ITEM_SPACING = ITEM_HEIGHT + 4;
  const int X_MARGIN = 10;
  const int Y_START = 30; 
  int w = _spr.width() - (2 * X_MARGIN);

  for (int i = 0; i < MENU_PAGE1_ITEM_COUNT; ++i) {
    int y = Y_START + i * ITEM_SPACING;
    uint16_t bg_color;
    uint16_t text_color;

    if (i == _selected_menu_item) {
      bg_color = TFT_DARKGREY;
      text_color = TFT_YELLOW;
    } else {
      bg_color = TFT_YELLOW;
      text_color = TFT_BLACK;
    }
    
    _spr.fillRect(X_MARGIN, y, w, ITEM_HEIGHT, bg_color);
    if (i == _selected_menu_item) {
      _spr.drawRect(X_MARGIN, y, w, ITEM_HEIGHT, TFT_WHITE);
    }
    _spr.setTextColor(text_color, bg_color);
    _spr.setTextDatum(MC_DATUM);
    // Arial18 ถูก load อยู่แล้ว
    _spr.drawString(menu_item_labels_page_1[i], _spr.width() / 2, y + ITEM_HEIGHT / 2);
  }
  
  _spr.unloadFont();
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Select | Press: OK | Dbl-Press: Back", _spr.width() / 2, _spr.height() - 2);
}

void UIManager::drawSettingsPage2(const AppState& state, const ConfigState& config) {
  _spr.fillSprite(TFT_BLACK);
  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.setTextDatum(TC_DATUM);
  
  _spr.loadFont(Arial18);
  _spr.drawString("Settings - Page 2", _spr.width() / 2, 4);

  const int ITEM_HEIGHT = 24;
  const int ITEM_SPACING = ITEM_HEIGHT + 4;
  const int X_MARGIN = 10;
  const int Y_START = 30;
  int w = _spr.width() - (2 * X_MARGIN);

  for (int i = 0; i < MENU_PAGE2_ITEM_COUNT; ++i) {
    int y = Y_START + i * ITEM_SPACING;
    uint16_t bg_color;
    uint16_t text_color;

    if (i == _selected_menu_item_page_2) {
      bg_color = TFT_DARKGREY;
      text_color = TFT_YELLOW;
    } else {
      bg_color = TFT_YELLOW;
      text_color = TFT_BLACK;
    }
    
    _spr.fillRect(X_MARGIN, y, w, ITEM_HEIGHT, bg_color);
    if (i == _selected_menu_item_page_2) {
      _spr.drawRect(X_MARGIN, y, w, ITEM_HEIGHT, TFT_WHITE);
    }
    _spr.setTextColor(text_color, bg_color);
    _spr.setTextDatum(MC_DATUM);
    _spr.drawString(menu_item_labels_page_2[i], _spr.width() / 2, y + ITEM_HEIGHT / 2);
  }
  
  _spr.unloadFont();
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Select | Press: OK | Dbl-Press: Back", _spr.width() / 2, _spr.height() - 2);
}


void UIManager::drawSettingsHeaterTargetTemp(const AppState& state) {
  _spr.fillSprite(TFT_BLACK);
  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.setTextDatum(TC_DATUM);
  
  _spr.loadFont(Arial18);
  char title_buffer[20];
  snprintf(title_buffer, sizeof(title_buffer), "Heater (%d)", _selected_menu_item + 1);
  _spr.drawString(title_buffer, _spr.width() / 2, 20);

  _spr.setTextDatum(MC_DATUM);
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
  _spr.fillSprite(TFT_BLACK);
  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.setTextDatum(TC_DATUM);
  
  _spr.loadFont(Arial18);
  char title_buffer[20];
  snprintf(title_buffer, sizeof(title_buffer), "Heater (%d)", _selected_menu_item + 1);
  _spr.drawString(title_buffer, _spr.width() / 2, 20);

  _spr.setTextDatum(MC_DATUM);
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
  _spr.fillSprite(TFT_BLACK);
  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.setTextDatum(TC_DATUM);
  
  _spr.loadFont(Arial18);
  char title_buffer[20];
  snprintf(title_buffer, sizeof(title_buffer), "Heater (%d)", _selected_menu_item + 1);
  _spr.drawString(title_buffer, _spr.width() / 2, 20);
  
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.drawString("Calibration Offset", _spr.width() / 2, 60);

  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.setTextDatum(MC_DATUM);
  char temp_buffer[30];
  snprintf(temp_buffer, sizeof(temp_buffer), "Offset: %.1f C", _temp_edit_value);
  _spr.drawString(temp_buffer, _spr.width() / 2, _spr.height() / 2);
  
  _spr.unloadFont();
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Adjust | Press: Confirm", _spr.width() / 2, _spr.height() - 10);
}

void UIManager::drawSettingsCalibrationSelect(const AppState& state, const ConfigState& config) {
  _spr.fillSprite(TFT_BLACK);
  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.setTextDatum(TC_DATUM);
  
  _spr.loadFont(Arial18);
  _spr.drawString("Heater Calibration", _spr.width() / 2, 20);

  char item_labels[3][30];
  snprintf(item_labels[0], 30, "Heater 1 Offset: %.1fC", config.tc_offsets[0]);
  snprintf(item_labels[1], 30, "Heater 2 Offset: %.1fC", config.tc_offsets[1]);
  snprintf(item_labels[2], 30, "Heater 3 Offset: %.1fC", config.tc_offsets[2]);

  for (int i = 0; i < 3; ++i) {
    _spr.setTextDatum(MC_DATUM);
    if (i == _selected_menu_item) { 
        _spr.setTextColor(TFT_BLACK, TFT_YELLOW);
    } else {
        _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
    }
    _spr.drawString(item_labels[i], _spr.width() / 2, 80 + i * 40);
  }
  
  _spr.unloadFont();
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Select | Press: Edit | Dbl-Press: Back", _spr.width() / 2, _spr.height() - 10);
}

void UIManager::drawSettingsMaxTempLock(const AppState& state) {
  _spr.fillSprite(TFT_BLACK);
  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.setTextDatum(TC_DATUM);
  
  _spr.loadFont(Arial18);
  _spr.drawString("Max Temperature Lock", _spr.width() / 2, 20);
  
  _spr.setTextDatum(MC_DATUM);
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

void UIManager::drawSettingsIdleOff(const AppState& state, const ConfigState& config) {
  _spr.fillSprite(TFT_BLACK);
  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.setTextDatum(TC_DATUM);
  
  _spr.loadFont(Arial18);
  _spr.drawString("Turn off When Idle", _spr.width() / 2, 20);

  for (int i = 0; i < IDLE_OFF_ITEM_COUNT; ++i) {
    _spr.setTextDatum(MC_DATUM);
    if (i == _selected_menu_item) {
        _spr.setTextColor(TFT_BLACK, TFT_YELLOW);
    } else {
        _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
    }
    _spr.drawString(idle_off_labels[i], _spr.width() / 2, 80 + i * 30);
  }
}

void UIManager::drawSettingsSound(const AppState& state, const ConfigState& config) { 
    _spr.fillSprite(TFT_BLACK);
    _spr.setTextColor(TFT_WHITE, TFT_BLACK);
    _spr.setTextDatum(TC_DATUM);
    
    _spr.loadFont(Arial18);
    _spr.drawString("Sound", _spr.width() / 2, 20); 

    _spr.setTextDatum(MC_DATUM);
    _spr.drawRect(40, 100, _spr.width() - 80, 40, TFT_YELLOW); 
    _spr.setTextColor(TFT_WHITE);
    _spr.drawString(config.sound_on ? "Sound: ON" : "Sound: OFF", _spr.width() / 2, 120); 
    
    _spr.unloadFont();
    _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
    _spr.setTextDatum(BC_DATUM);
    _spr.setTextSize(1);
    _spr.drawString("Rot: Toggle | Dbl-Press: Back", _spr.width() / 2, _spr.height() - 10); 
}

void UIManager::drawSettingsTempUnit(const AppState& state, const ConfigState& config) {
    _spr.fillSprite(TFT_BLACK);
    _spr.setTextColor(TFT_WHITE, TFT_BLACK);
    _spr.setTextDatum(TC_DATUM);
    
    _spr.loadFont(Arial18);
    _spr.drawString("Temperature Unit", _spr.width() / 2, 20);

    for (int i = 0; i < 2; ++i) {
        _spr.setTextDatum(MC_DATUM);
        if (i == _selected_menu_item) {
            _spr.setTextColor(TFT_BLACK, TFT_YELLOW);
        } else {
            _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
        }
        _spr.drawString(temp_unit_labels[i], _spr.width() / 2, 100 + i * 60);
    }
}

void UIManager::drawSettingsAbout(const AppState& state) {
    _spr.fillSprite(TFT_BLACK);
    _spr.setTextColor(TFT_CYAN, TFT_BLACK);
    _spr.setTextDatum(TC_DATUM);
    
    _spr.loadFont(Arial18);
    _spr.drawString("About This Device", _spr.width() / 2, 20);

    _spr.setTextColor(TFT_WHITE, TFT_BLACK);
    _spr.setTextDatum(MC_DATUM);
    // About ใช้ Arial18 ได้ แต่อาจต้องลดขนาดบรรทัด
    // ถ้าต้องการตัวเล็ก กลับไปใช้ Default font ดีกว่า
    _spr.unloadFont(); // Use small font for details
    _spr.setTextSize(1);

    int line_height = 12;
    int y_center = _spr.height() / 2;
    int y_start = y_center - (line_height * 3);

    _spr.drawString("Oven Control System", _spr.width() / 2, y_start);
    _spr.drawString("Firmware v1.0", _spr.width() / 2, y_start + (line_height * 2));
    _spr.drawString("(c) 2025 K'Tor", _spr.width() / 2, y_start + (line_height * 4));
    _spr.drawString("Contact: 08x-xxx-xxxx", _spr.width() / 2, y_start + (line_height * 6));

    _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
    _spr.setTextDatum(BC_DATUM);
    _spr.setTextSize(1);
    _spr.drawString("Press or Dbl-Press to Go Back", _spr.width() / 2, _spr.height() - 10);
}