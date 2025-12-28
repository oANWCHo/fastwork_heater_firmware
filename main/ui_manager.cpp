#include "ui_manager.h"

#define COLOR_IDLE   0x39E7 // Dark Grey
#define COLOR_ACTIVE 0x05E0 // Dark Green
#define COLOR_DONE   TFT_BLUE
#define COLOR_WARN   TFT_ORANGE

#define COLOR_START   0x07E0 // Green
#define COLOR_STOP    0xF800 // Red

const uint32_t INACTIVITY_TIMEOUT_MS = 10000;

const char* menu_item_labels_page_1[MENU_PAGE1_ITEM_COUNT] = {
  "Heater 1", "Heater 2", "Heater 3", "Max Temp",
  "Heater Calibration", 
  "Next Page >"
};

const char* menu_item_labels_page_2[MENU_PAGE2_ITEM_COUNT] = {
  "Turn off When Idle", "Sound", "Temp Unit", "About", // <-- RENAMED
  "< Prev Page"
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

void UIManager::draw(const AppState& state, const ConfigState& config) {
  static uint32_t last_blink_time = 0;
  if (millis() - last_blink_time > 400) {
    _blink_state = !_blink_state;
    last_blink_time = millis();
  }

  switch (_current_screen) {
    case SCREEN_SLEEP:
      _spr.fillSprite(TFT_BLACK);
      _spr.setTextColor(TFT_WHITE, TFT_BLACK);
      _spr.setTextDatum(MC_DATUM);
      _spr.setTextSize(2);
      _spr.drawString("ZZZ Sleeping...", _spr.width() / 2, _spr.height() / 2);
      _spr.setTextSize(1);
      _spr.drawString("Press any button to wake", _spr.width() / 2, _spr.height() / 2 + 30);
      _spr.pushSprite(0, 0);
      return; // Stop drawing anything else

    case SCREEN_STANDBY:                  drawStandbyScreen(state, config); break;
    case SCREEN_SETTINGS_PAGE_1:          drawSettingsPage1(state, config); break; 
    case SCREEN_SETTINGS_PAGE_2:          drawSettingsPage2(state, config); break; 
    case SCREEN_SETTINGS_HEATER_TARGET_TEMP: drawSettingsHeaterTargetTemp(state); break;
    case SCREEN_SETTINGS_HEATER_MAX_TEMP: drawSettingsHeaterMaxTemp(state); break;
    case SCREEN_SETTINGS_MAX_TEMP_LOCK:   drawSettingsMaxTempLock(state); break;
    case SCREEN_SETTINGS_HEATER_CALIBRATE:  drawSettingsHeaterCalibrate(state); break;
    case SCREEN_SETTINGS_CALIBRATION_SELECT: drawSettingsCalibrationSelect(state, config); break; 
    case SCREEN_SETTINGS_IDLE_OFF:        drawSettingsIdleOff(state, config); break;
    case SCREEN_SETTINGS_SOUND:           drawSettingsSound(state, config); break; // <-- RENAMED
    case SCREEN_SETTINGS_TEMP_UNIT:       drawSettingsTempUnit(state, config); break;
    case SCREEN_SETTINGS_ABOUT:           drawSettingsAbout(state); break;
  }
  _spr.pushSprite(0, 0);
}

void UIManager::resetInactivityTimer() { 
  _last_activity_time = millis();
}

bool UIManager::checkInactivity(ConfigState& config, bool& has_go_to, float& go_to) { 
  uint32_t elapsed = millis() - _last_activity_time;

  // --- A. Existing Menu Timeout (Back to Standby after 10s) ---
  if (_current_screen != SCREEN_STANDBY && _current_screen != SCREEN_SLEEP) {
    if (elapsed > INACTIVITY_TIMEOUT_MS) {
      _current_screen = SCREEN_STANDBY;
      _menu_step_accumulator = 0.0f;
      _selected_menu_item = 0;
      _selected_menu_item_page_2 = 0; 
      _standby_selection = 0;
      // We don't reset activity time here, so Sleep can still trigger if user does nothing in Standby
    }
  } 

  // --- Sleep Logic ---
  // If we are already sleeping, do nothing
  if (_current_screen == SCREEN_SLEEP) return false;

  // Determine Timeout Duration based on config
  uint32_t sleep_timeout = 0;
  switch(config.idle_off_mode) {
    case 0: sleep_timeout = 10000;    break; // 10 sec. (Debug)
    case 1: sleep_timeout = 900000;   break; // 15 min
    case 2: sleep_timeout = 1800000;  break; // 30 min
    case 3: sleep_timeout = 3600000;  break; // 60 min
    case 4: return false; // Always ON (Never Sleep)
  }

  // Trigger Sleep
  if (elapsed > sleep_timeout) {
    _current_screen = SCREEN_SLEEP;
    
    // TURN OFF HEATERS
    has_go_to = false;
    go_to = NAN;
    return true;
  }
  return false;
}

bool UIManager::handleButtonSingleClick(ConfigState& config, float& go_to, bool& has_go_to) {
  
  if (_current_screen == SCREEN_SLEEP) {
    _current_screen = SCREEN_STANDBY;
    resetInactivityTimer();
    return true; // Consume event, do not trigger button action
  }
  resetInactivityTimer();
  switch (_current_screen) {
    case SCREEN_STANDBY:
      switch (_standby_selection) {
        case 0: config.heater_active[0] = !config.heater_active[0]; break;
        case 1: config.heater_active[1] = !config.heater_active[1]; break;
        case 2: config.heater_active[2] = !config.heater_active[2]; break;
	      case 3: // "start"
          { 
            bool allow_start = true;
            bool any_active = false;
            for (int i = 0; i < 3; i++) {
              if (config.heater_active[i]) {
                any_active = true;
                if (config.target_temps[i] > config.max_temp_lock) { allow_start = false; break; }
                if (config.target_temps[i] > config.max_temps[i]) { allow_start = false; break; }
              }
            }
            if (any_active && allow_start) { has_go_to = true; go_to = NAN; } 
            else { has_go_to = false; go_to = NAN; }
          }
          break;
        case 4: // "stop"
          has_go_to = false;
          go_to = NAN;
          break;
        case 5: // "Settings"
          _current_screen = SCREEN_SETTINGS_PAGE_1; 
          _selected_menu_item = 0;
          break;
      }
      if (_standby_selection >= 0 && _standby_selection <= 2) {
        if (_save_callback) _save_callback(config);
      }
      return true;

    case SCREEN_SETTINGS_PAGE_1: 
      switch (_selected_menu_item) {
        case MENU_PAGE1_HEATER_1: 
        case MENU_PAGE1_HEATER_2: 
        case MENU_PAGE1_HEATER_3: 
          _current_screen = SCREEN_SETTINGS_HEATER_TARGET_TEMP;
          _temp_edit_value = config.target_temps[_selected_menu_item];
          break;
        case MENU_PAGE1_MAX_TEMP_LOCK: 
          _current_screen = SCREEN_SETTINGS_MAX_TEMP_LOCK;
          _temp_edit_value = config.max_temp_lock;
          break;
        case MENU_PAGE1_CALIBRATION: 
          _current_screen = SCREEN_SETTINGS_CALIBRATION_SELECT;
          _selected_menu_item = 0; // Default to H1
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
        case MENU_PAGE2_IDLE_OFF:
          _current_screen = SCREEN_SETTINGS_IDLE_OFF;
          _selected_menu_item = config.idle_off_mode; 
          break;
        case MENU_PAGE2_SOUND: // <-- RENAMED
          _current_screen = SCREEN_SETTINGS_SOUND; // <-- RENAMED
          _selected_menu_item = 0;
          break;
        case MENU_PAGE2_TEMP_UNIT:
          _current_screen = SCREEN_SETTINGS_TEMP_UNIT;
          _selected_menu_item = (config.temp_unit == 'C') ? 0 : 1;
          break;
        case MENU_PAGE2_ABOUT:
          _current_screen = SCREEN_SETTINGS_ABOUT;
          break;
        case MENU_PAGE2_PREV_PAGE:
          _current_screen = SCREEN_SETTINGS_PAGE_1;
          _selected_menu_item = 0; 
          break;
      }
      _menu_step_accumulator = 0.0f;
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
      
    case SCREEN_SETTINGS_SOUND: // <-- RENAMED
      // This screen no longer uses _selected_menu_item to toggle
      // It's handled by rotation, but a click here has no effect
      // We return true to acknowledge the click, but do nothing
      return true;
      
    case SCREEN_SETTINGS_TEMP_UNIT:
      config.temp_unit = (_selected_menu_item == 0) ? 'C' : 'F';
      _current_screen = SCREEN_SETTINGS_PAGE_2; 
      if (_save_callback) _save_callback(config);
      _menu_step_accumulator = 0.0f;
      return true;
      
    case SCREEN_SETTINGS_ABOUT:
      _current_screen = SCREEN_SETTINGS_PAGE_2; 
      return true;

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
    case SCREEN_SETTINGS_PAGE_1: 
    case SCREEN_SETTINGS_PAGE_2: 
      _current_screen = SCREEN_STANDBY;
      break;
      
    // Page 1 sub-screens return to Page 1
    case SCREEN_SETTINGS_HEATER_TARGET_TEMP:
    case SCREEN_SETTINGS_MAX_TEMP_LOCK:
      _current_screen = SCREEN_SETTINGS_PAGE_1;
      break;
    case SCREEN_SETTINGS_HEATER_MAX_TEMP:
      _current_screen = SCREEN_SETTINGS_HEATER_TARGET_TEMP;
      _temp_edit_value = config.target_temps[_selected_menu_item];
      break;

    case SCREEN_SETTINGS_CALIBRATION_SELECT:
      _current_screen = SCREEN_SETTINGS_PAGE_1;
      break;
      
    case SCREEN_SETTINGS_HEATER_CALIBRATE:
      _current_screen = SCREEN_SETTINGS_CALIBRATION_SELECT; 
      break;

    case SCREEN_SETTINGS_IDLE_OFF:
    case SCREEN_SETTINGS_TEMP_UNIT:
    case SCREEN_SETTINGS_SOUND: 
    case SCREEN_SETTINGS_ABOUT:
      _current_screen = SCREEN_SETTINGS_PAGE_2;
      break;

    default: return false;
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
      const int num_items = 6;
      int new_pos = _standby_selection + change;
      
      // Clamp values (Stop at 0 and Max)
      if (new_pos < 0) new_pos = 0;
      if (new_pos >= num_items) new_pos = num_items - 1;
      
      _standby_selection = new_pos;
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
    
    case SCREEN_SETTINGS_CALIBRATION_SELECT: {
      const int num_items = 3; 
      int new_pos = _selected_menu_item + change;
      
      if (new_pos < 0) new_pos = 0;
      if (new_pos >= num_items) new_pos = num_items - 1;
      
      _selected_menu_item = new_pos;
      break;
    }

    // --- Numeric Adjustments (Already Clamped, kept same) ---
    case SCREEN_SETTINGS_HEATER_TARGET_TEMP: {
      _temp_edit_value += (float)change * 0.5f;
      if (_temp_edit_value < 0) _temp_edit_value = 0;
      if (_temp_edit_value > config.max_temps[_selected_menu_item]) {
        _temp_edit_value = config.max_temps[_selected_menu_item];
      }
      break;
    }
    case SCREEN_SETTINGS_HEATER_MAX_TEMP: {
      _temp_edit_value += (float)change * 0.5f;
      if (_temp_edit_value < config.target_temps[_selected_menu_item]) {
        _temp_edit_value = config.target_temps[_selected_menu_item];
      }
      if (_temp_edit_value > config.max_temp_lock) {
        _temp_edit_value = config.max_temp_lock;
      }
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
    // --------------------------------------------------------

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
      // Changed to clamp instead of toggle, so direction matters
      // 0 = Celsius, 1 = Fahrenheit
      int new_pos = _selected_menu_item + change;
      if (new_pos < 0) new_pos = 0;
      if (new_pos > 1) new_pos = 1;
      
      _selected_menu_item = new_pos;
      break;
    }
    default: return false;
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
  static float last_valid_standby_temps[6];
  static bool initialized = false;
  if (!initialized) {
    for (int i = 0; i < 6; i++) last_valid_standby_temps[i] = NAN;
    initialized = true;
  }
  
  _spr.fillSprite(TFT_BLACK);
  const int FONT_SIZE = 2;
  const int LINE_SPACING = 20;

  const char* labels[] = {
    "H1 Temp", "H2 Temp", "H3 Temp",
    "IR1 Temp", "IR2 Temp"
  };

  float temps_c[] = {
    state.tc_temps[0], 
    state.tc_temps[1], 
    state.tc_temps[2], 
    state.ir_temps[0], 
    state.ir_temps[1]
  };
  
  float settings[3][2] = {
    {config.target_temps[0], config.max_temps[0]},
    {config.target_temps[1], config.max_temps[1]},
    {config.target_temps[2], config.max_temps[2]},
  };

  for (int i = 0; i < 5; ++i) {
    int y = 4 + i * LINE_SPACING;
    uint16_t bg_color = COLOR_IDLE;
    
    if (i < 3) {
      if (config.heater_active[i]) {
        if (state.is_heating_active) {
          float current_target_temp = config.target_temps[i];
          bg_color = getStatusColor(true, temps_c[i], current_target_temp);
        } else {
          bg_color = COLOR_ACTIVE; 
        }
      }
    }
    
    if (temps_c[i] > 270.0f && _blink_state) {
      bg_color = COLOR_WARN;
    }

    _spr.fillRect(0, y, _spr.width(), LINE_SPACING, bg_color);
    
    if (i < 3 && config.heater_active[i] && state.is_heating_active) {
      float progress = (millis() % 1500) / 1500.0f;
      int bar_width = (int)((_spr.width() - 4) * progress);
      _spr.fillRect(2, y + LINE_SPACING - 4, bar_width, 3, TFT_CYAN);
    }
    
    _spr.setTextColor(TFT_WHITE, bg_color);
    _spr.setTextDatum(ML_DATUM); 
    _spr.setTextSize(FONT_SIZE); 
    _spr.drawString(labels[i], 5, y + (LINE_SPACING / 2));

    float display_temp;
    
    // If the CURRENT reading is NAN (Sensor Fault/Unplugged), we enforce NAN immediately
    if (isnan(temps_c[i])) {
       display_temp = NAN;
       // We also invalidate the 'last_valid' cache so it doesn't recover ghost data later
       last_valid_standby_temps[i] = NAN; 
    } else {
       // Only update history if we have a valid reading
       last_valid_standby_temps[i] = temps_c[i];
       display_temp = convertTemp(temps_c[i], state.temp_unit);
    }
    
    char temp_buffer[20];
    if (isnan(display_temp)) { 
      snprintf(temp_buffer, sizeof(temp_buffer), "---"); // Shows --- when unplugged
    } else {
      snprintf(temp_buffer, sizeof(temp_buffer), "%.2f %c", display_temp, state.temp_unit);
    }

    _spr.setTextDatum(MR_DATUM); 
    _spr.setTextSize(FONT_SIZE); 
    
    int temp_x_pos = 210; 
    _spr.drawString(temp_buffer, temp_x_pos, y + (LINE_SPACING / 2));

    if (i < 3) {
      char settings_buf[20];
      int settings_index = i;
      snprintf(settings_buf, sizeof(settings_buf), "(%.0f, %.0f)",
               convertTemp(settings[settings_index][0], state.temp_unit),
               convertTemp(settings[settings_index][1], state.temp_unit));
      _spr.setTextDatum(ML_DATUM); 
      _spr.setTextSize(1); 
      _spr.drawString(settings_buf, temp_x_pos + 5, y + (LINE_SPACING / 2)); 
    }
  }
  
  int button_h = 30;
  int x_padding = 10;
  int button_spacing = 10;
  int screen_w = _spr.width();
  int usable_w = screen_w - (2 * x_padding);

  int button_y1 = 130;
  int button_w1 = (usable_w - (2 * button_spacing)) / 3; 
  int total_w1 = (button_w1 * 3) + (button_spacing * 2);
  int x_start1 = (screen_w - total_w1) / 2;

  for (int i = 0; i < 3; i++) {
    int x = x_start1 + i * (button_w1 + button_spacing);
    uint16_t bg = TFT_WHITE;
    uint16_t fg = TFT_BLACK;
    if (state.heater_cutoff_state[i]) {
      bool blink_on = (millis() / 500) % 2;
      bg = blink_on ? TFT_RED : TFT_WHITE;
    } else if (config.heater_active[i]) {
      bg = COLOR_ACTIVE; 
    } else {
      bg = TFT_WHITE; 
    }
    _spr.fillRect(x, button_y1, button_w1, button_h, bg);
    _spr.setTextColor(fg, bg);
    _spr.setTextDatum(MC_DATUM);
    _spr.setTextSize(1);
    _spr.drawString(standby_button_labels[i], x + button_w1 / 2, button_y1 + button_h / 2);
    if (_standby_selection == i) {
      _spr.drawRect(x, button_y1, button_w1, button_h, TFT_SKYBLUE);
      _spr.drawRect(x+1, button_y1+1, button_w1-2, button_h-2, TFT_SKYBLUE); 
      _spr.drawRect(x+2, button_y1+2, button_w1-4, button_h-4, TFT_SKYBLUE); 
    }
  }

  int button_y2 = button_y1 + button_h + button_spacing;
  int button_w2 = (usable_w - (1 * button_spacing)) / 2; 
  int total_w2 = (button_w2 * 2) + (button_spacing * 1);
  int x_start2 = (screen_w - total_w2) / 2;
  uint16_t colors[] = {COLOR_START, COLOR_STOP};
  uint16_t fg_colors[] = {TFT_BLACK, TFT_WHITE};

  for (int i = 0; i < 2; i++) {
    int x = x_start2 + i * (button_w2 + button_spacing);
    uint16_t bg = colors[i];
    uint16_t fg = fg_colors[i];
    int selection_index = i + 3; 
    _spr.fillRect(x, button_y2, button_w2, button_h, bg);
    _spr.setTextColor(fg, bg);
    _spr.setTextDatum(MC_DATUM);
    _spr.setTextSize(1);
    _spr.drawString(standby_button_labels[selection_index], x + button_w2 / 2, button_y2 + button_h / 2);
    if (_standby_selection == selection_index) {
       _spr.drawRect(x, button_y2, button_w2, button_h, TFT_SKYBLUE);
       _spr.drawRect(x+1, button_y2+1, button_w2-2, button_h-2, TFT_SKYBLUE); 
       _spr.drawRect(x+2, button_y2+2, button_w2-4, button_h-4, TFT_SKYBLUE);
    }
  }

  int button_y3 = button_y2 + button_h + button_spacing;
  int button_w3 = usable_w; 
  int x_start3 = x_padding;
  int selection_index = 5;
  uint16_t bg = TFT_WHITE;
  uint16_t fg = TFT_BLACK;
  _spr.fillRect(x_start3, button_y3, button_w3, button_h, bg);
  _spr.setTextColor(fg, bg);
  _spr.setTextDatum(MC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString(standby_button_labels[selection_index], x_start3 + button_w3 / 2, button_y3 + button_h / 2);
  if (_standby_selection == selection_index) {
    _spr.drawRect(x_start3, button_y3, button_w3, button_h, TFT_SKYBLUE);
    _spr.drawRect(x_start3+1, button_y3+1, button_w3-2, button_h-2, TFT_SKYBLUE);
    _spr.drawRect(x_start3+1, button_y3+2, button_w3-4, button_h-4, TFT_SKYBLUE);
  }
}

void UIManager::drawSettingsPage1(const AppState& state, const ConfigState& config) {
  _spr.fillSprite(TFT_BLACK);
  _spr.setTextColor(TFT_WHITE, TFT_BLACK); 
  _spr.setTextDatum(TC_DATUM); 
  _spr.setTextSize(2); 
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

    bool out_of_bounds = false;
    if (i >= MENU_PAGE1_HEATER_1 && i <= MENU_PAGE1_HEATER_3) {
      if (config.max_temps[i] > config.max_temp_lock) {
        out_of_bounds = true;
      }
    }

    if (i == _selected_menu_item) {
      bg_color = TFT_DARKGREY;
      text_color = TFT_YELLOW;
    } else if (out_of_bounds) {
      bg_color = TFT_RED;
      text_color = TFT_WHITE;
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
    _spr.setTextSize(2);
    _spr.drawString(menu_item_labels_page_1[i], _spr.width() / 2, y + ITEM_HEIGHT / 2);
  }
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Select | Press: OK | Dbl-Press: Back", _spr.width() / 2, _spr.height() - 2);
}

void UIManager::drawSettingsPage2(const AppState& state, const ConfigState& config) {
  _spr.fillSprite(TFT_BLACK);
  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.setTextDatum(TC_DATUM);
  _spr.setTextSize(2);
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
    _spr.setTextSize(2);
    _spr.drawString(menu_item_labels_page_2[i], _spr.width() / 2, y + ITEM_HEIGHT / 2);
  }
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Select | Press: OK | Dbl-Press: Back", _spr.width() / 2, _spr.height() - 2);
}


void UIManager::drawSettingsHeaterTargetTemp(const AppState& state) {
  _spr.fillSprite(TFT_BLACK);
  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.setTextDatum(TC_DATUM);
  _spr.setTextSize(2);
  char title_buffer[20];
  snprintf(title_buffer, sizeof(title_buffer), "Heater (%d)", _selected_menu_item + 1);
  _spr.drawString(title_buffer, _spr.width() / 2, 20);
  _spr.setTextDatum(MC_DATUM);
  _spr.setTextSize(2);
  char temp_buffer[30];
  float display_value = convertTemp(_temp_edit_value, state.temp_unit);
  snprintf(temp_buffer, sizeof(temp_buffer), "Target Temp: %.1f %c", display_value, state.temp_unit);
  _spr.drawString(temp_buffer, _spr.width() / 2, _spr.height() / 2);
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Adjust | Press: Confirm", _spr.width() / 2, _spr.height() - 10);
}

void UIManager::drawSettingsHeaterMaxTemp(const AppState& state) {
  _spr.fillSprite(TFT_BLACK);
  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.setTextDatum(TC_DATUM);
  _spr.setTextSize(2);
  char title_buffer[20];

  snprintf(title_buffer, sizeof(title_buffer), "Heater (%d)", _selected_menu_item + 1);
  _spr.drawString(title_buffer, _spr.width() / 2, 20);
  _spr.setTextDatum(MC_DATUM);
  _spr.setTextSize(2);
  char temp_buffer[30];
  float display_value = convertTemp(_temp_edit_value, state.temp_unit);
  snprintf(temp_buffer, sizeof(temp_buffer), "Max Temp: %.1f %c", display_value, state.temp_unit);
  _spr.drawString(temp_buffer, _spr.width() / 2, _spr.height() / 2);
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Adjust | Press: Confirm", _spr.width() / 2, _spr.height() - 10);
}

void UIManager::drawSettingsHeaterCalibrate(const AppState& state) {
  _spr.fillSprite(TFT_BLACK);
  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.setTextDatum(TC_DATUM);
  _spr.setTextSize(2);
  char title_buffer[20];
  snprintf(title_buffer, sizeof(title_buffer), "Heater (%d)", _selected_menu_item + 1);
  _spr.drawString(title_buffer, _spr.width() / 2, 20);
  
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.drawString("Calibration Offset", _spr.width() / 2, 60);

  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.setTextDatum(MC_DATUM);
  _spr.setTextSize(2);
  char temp_buffer[30];
  snprintf(temp_buffer, sizeof(temp_buffer), "Offset: %.1f C", _temp_edit_value);
  _spr.drawString(temp_buffer, _spr.width() / 2, _spr.height() / 2);
  
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Adjust | Press: Confirm", _spr.width() / 2, _spr.height() - 10);
}

void UIManager::drawSettingsCalibrationSelect(const AppState& state, const ConfigState& config) {
  _spr.fillSprite(TFT_BLACK);
  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.setTextDatum(TC_DATUM);
  _spr.setTextSize(2);
  _spr.drawString("Heater Calibration", _spr.width() / 2, 20);

  char item_labels[3][30];
  snprintf(item_labels[0], 30, "Heater 1 Offset: %.1fC", config.tc_offsets[0]);
  snprintf(item_labels[1], 30, "Heater 2 Offset: %.1fC", config.tc_offsets[1]);
  snprintf(item_labels[2], 30, "Heater 3 Offset: %.1fC", config.tc_offsets[2]);

  for (int i = 0; i < 3; ++i) {
    _spr.setTextDatum(MC_DATUM);
    _spr.setTextSize(2);
    if (i == _selected_menu_item) { 
        _spr.setTextColor(TFT_BLACK, TFT_YELLOW);
    } else {
        _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
    }
    _spr.drawString(item_labels[i], _spr.width() / 2, 80 + i * 40);
  }
  
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Select | Press: Edit | Dbl-Press: Back", _spr.width() / 2, _spr.height() - 10);
}

void UIManager::drawSettingsMaxTempLock(const AppState& state) {
  _spr.fillSprite(TFT_BLACK);
  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.setTextDatum(TC_DATUM);
  _spr.setTextSize(2);
  _spr.drawString("Max Temperature Lock", _spr.width() / 2, 20);
  _spr.setTextDatum(MC_DATUM);
  _spr.setTextSize(2);
  char temp_buffer[30];
  float display_value = convertTemp(_temp_edit_value, state.temp_unit);
  snprintf(temp_buffer, sizeof(temp_buffer), "Max Temp: %.1f %c", display_value, state.temp_unit);
  _spr.drawString(temp_buffer, _spr.width() / 2, _spr.height() / 2);
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Adjust | Press: Confirm", _spr.width() / 2, _spr.height() - 10);
}

void UIManager::drawSettingsIdleOff(const AppState& state, const ConfigState& config) {
  _spr.fillSprite(TFT_BLACK);
  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.setTextDatum(TC_DATUM);
  _spr.setTextSize(2);
  _spr.drawString("Turn off When Idle", _spr.width() / 2, 20);

  for (int i = 0; i < IDLE_OFF_ITEM_COUNT; ++i) {
    _spr.setTextDatum(MC_DATUM);
    _spr.setTextSize(2);
    if (i == _selected_menu_item) {
        _spr.setTextColor(TFT_BLACK, TFT_YELLOW);
    } else {
        _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
    }
    _spr.drawString(idle_off_labels[i], _spr.width() / 2, 80 + i * 30);
  }
}

void UIManager::drawSettingsSound(const AppState& state, const ConfigState& config) { // <-- RENAMED
    _spr.fillSprite(TFT_BLACK);
    _spr.setTextColor(TFT_WHITE, TFT_BLACK);
    _spr.setTextDatum(TC_DATUM);
    _spr.setTextSize(2);
    _spr.drawString("Sound", _spr.width() / 2, 20); // <-- RENAMED

    _spr.setTextDatum(MC_DATUM);
    _spr.setTextSize(2);
    
    _spr.drawRect(40, 100, _spr.width() - 80, 40, TFT_YELLOW); // Centered
    _spr.setTextColor(TFT_WHITE);
    _spr.drawString(config.sound_on ? "Sound: ON" : "Sound: OFF", _spr.width() / 2, 120); // Centered
    
    _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
    _spr.setTextDatum(BC_DATUM);
    _spr.setTextSize(1);
    _spr.drawString("Rot: Toggle | Dbl-Press: Back", _spr.width() / 2, _spr.height() - 10); // <-- Updated help text
}

void UIManager::drawSettingsTempUnit(const AppState& state, const ConfigState& config) {
    _spr.fillSprite(TFT_BLACK);
    _spr.setTextColor(TFT_WHITE, TFT_BLACK);
    _spr.setTextDatum(TC_DATUM);
    _spr.setTextSize(2);
    _spr.drawString("Temperature Unit", _spr.width() / 2, 20);

    for (int i = 0; i < 2; ++i) {
        _spr.setTextDatum(MC_DATUM);
        _spr.setTextSize(2);
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
    _spr.setTextSize(2);
    _spr.drawString("About This Device", _spr.width() / 2, 20);

    _spr.setTextColor(TFT_WHITE, TFT_BLACK);
    _spr.setTextDatum(MC_DATUM);
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