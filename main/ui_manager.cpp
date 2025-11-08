#include "ui_manager.h"

#define COLOR_IDLE   0x39E7 // Dark Grey
#define COLOR_ACTIVE 0x05E0 // Dark Green
#define COLOR_DONE   TFT_BLUE
#define COLOR_WARN   TFT_ORANGE

#define COLOR_START   0x07E0 // Green
#define COLOR_STOP    0xF800 // Red

const uint32_t INACTIVITY_TIMEOUT_MS = 10000;

const char* menu_item_labels[MENU_ITEM_COUNT] = {
  "Heater 1", "Heater 2", "Heater 3", "Maximum Temperature Lock",
  "Turn off When Idle", "Light and Sound", "Temperature Unit", "About"
};
const char* idle_off_labels[IDLE_OFF_ITEM_COUNT] = {
  "15 min.", "30 min.", "60 min.", "Always ON"
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
  _menu_step_accumulator = 0.0f;
  _standby_selection = 0; // Default to "H1"
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
    case SCREEN_STANDBY:                  drawStandbyScreen(state, config); break;
    case SCREEN_SETTINGS_MAIN:            drawSettingsMain(state, config); break;
    case SCREEN_SETTINGS_HEATER_TARGET_TEMP: drawSettingsHeaterTargetTemp(state); break;
    case SCREEN_SETTINGS_HEATER_MAX_TEMP: drawSettingsHeaterMaxTemp(state); break;
    case SCREEN_SETTINGS_MAX_TEMP_LOCK:   drawSettingsMaxTempLock(state); break;
    case SCREEN_SETTINGS_IDLE_OFF:        drawSettingsIdleOff(state, config); break;
    case SCREEN_SETTINGS_LIGHT_SOUND:     drawSettingsLightSound(state, config); break;
    case SCREEN_SETTINGS_TEMP_UNIT:       drawSettingsTempUnit(state, config); break;
    case SCREEN_SETTINGS_ABOUT:           drawSettingsAbout(state); break;
  }
  _spr.pushSprite(0, 0);
}

void UIManager::resetInactivityTimer() { // <-- ADD THIS WHOLE FUNCTION
  _last_activity_time = millis();
}

void UIManager::checkInactivity() { // <-- ADD THIS WHOLE FUNCTION
  // Only check if we are NOT on the standby screen
  if (_current_screen != SCREEN_STANDBY) {
    if (millis() - _last_activity_time > INACTIVITY_TIMEOUT_MS) {
      // Inactivity detected, go back to standby
      _current_screen = SCREEN_STANDBY;
      // Reset menu states to default
      _menu_step_accumulator = 0.0f;
      _selected_menu_item = 0;
      _standby_selection = 0;
      _last_activity_time = millis(); // Reset timer
    }
  } else {
    // If we are on standby, just keep resetting the timer
    resetInactivityTimer();
  }
}
bool UIManager::handleButtonSingleClick(ConfigState& config, float& go_to, bool& has_go_to) {
  resetInactivityTimer();
  switch (_current_screen) {
    case SCREEN_STANDBY:
      switch (_standby_selection) {
        case 0: // "H1"
          config.heater_active[0] = !config.heater_active[0];
          break;
        case 1: // "H2"
          config.heater_active[1] = !config.heater_active[1];
          break;
        case 2: // "H3"
          config.heater_active[2] = !config.heater_active[2];
          break;
	      case 3: // "start"
          if (config.target_temps[0] <= config.max_temp_lock) {
            go_to = config.target_temps[0];
            has_go_to = true;
          }
          break;
        case 4: // "stop"
          has_go_to = false;
          go_to = NAN;
          break;
        case 5: // "Settings"
          _current_screen = SCREEN_SETTINGS_MAIN;
          _selected_menu_item = 0;
          break;
      }

      if (_standby_selection >= 0 && _standby_selection <= 2) {
        if (_save_callback) _save_callback(config);
      }
      return true;

    case SCREEN_SETTINGS_MAIN:
      switch (_selected_menu_item) {
        case MENU_HEATER_1: case MENU_HEATER_2: case MENU_HEATER_3:
          _current_screen = SCREEN_SETTINGS_HEATER_TARGET_TEMP;
          _temp_edit_value = config.target_temps[_selected_menu_item];
          break;
        case MENU_MAX_TEMP_LOCK:
          _current_screen = SCREEN_SETTINGS_MAX_TEMP_LOCK;
          _temp_edit_value = config.max_temp_lock;
          break;
        case MENU_IDLE_OFF:
          _current_screen = SCREEN_SETTINGS_IDLE_OFF;
          _selected_menu_item = config.idle_off_mode;
          break;
        case MENU_LIGHT_SOUND:
          _current_screen = SCREEN_SETTINGS_LIGHT_SOUND;
          _selected_menu_item = 0;
          break;
        case MENU_TEMP_UNIT:
          _current_screen = SCREEN_SETTINGS_TEMP_UNIT;
          _selected_menu_item = (config.temp_unit == 'C') ? 0 : 1;
          break;
        case MENU_ABOUT:
          _current_screen = SCREEN_SETTINGS_ABOUT;
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
      _current_screen = SCREEN_SETTINGS_MAIN;
      _menu_step_accumulator = 0.0f;
      return true;

    case SCREEN_SETTINGS_MAX_TEMP_LOCK:
      config.max_temp_lock = _temp_edit_value;
      if (_save_callback) _save_callback(config);
      _current_screen = SCREEN_SETTINGS_MAIN;
      _menu_step_accumulator = 0.0f;
      return true;

    case SCREEN_SETTINGS_IDLE_OFF:
      config.idle_off_mode = (IdleOffMode)_selected_menu_item;
      if (_save_callback) _save_callback(config);
      _current_screen = SCREEN_SETTINGS_MAIN;
      _menu_step_accumulator = 0.0f;
      return true;
      
    case SCREEN_SETTINGS_LIGHT_SOUND:
      _selected_menu_item = (_selected_menu_item == 0) ? 1 : 0;
      return true;
      
    case SCREEN_SETTINGS_TEMP_UNIT:
      config.temp_unit = (_selected_menu_item == 0) ? 'C' : 'F';
      _current_screen = SCREEN_SETTINGS_MAIN;
      if (_save_callback) _save_callback(config);
      _menu_step_accumulator = 0.0f;
      return true;
      
    case SCREEN_SETTINGS_ABOUT:
      _current_screen = SCREEN_SETTINGS_MAIN;
      return true;

    default: return false;
  }
}

bool UIManager::handleButtonDoubleClick(ConfigState& config) {
  resetInactivityTimer();
  switch (_current_screen) {
    case SCREEN_SETTINGS_MAIN:
    case SCREEN_SETTINGS_ABOUT:
      _current_screen = SCREEN_STANDBY;
      break;
    case SCREEN_SETTINGS_HEATER_TARGET_TEMP:
    case SCREEN_SETTINGS_MAX_TEMP_LOCK:
    case SCREEN_SETTINGS_IDLE_OFF:
    case SCREEN_SETTINGS_TEMP_UNIT:
      _current_screen = SCREEN_SETTINGS_MAIN;
      break;
    case SCREEN_SETTINGS_LIGHT_SOUND:
      _current_screen = SCREEN_SETTINGS_MAIN;
      
      break;
    case SCREEN_SETTINGS_HEATER_MAX_TEMP:
      _current_screen = SCREEN_SETTINGS_HEATER_TARGET_TEMP;
      _temp_edit_value = config.target_temps[_selected_menu_item];
      break;
    default: return false;
  }
  _menu_step_accumulator = 0.0f;
  return true;
}

bool UIManager::handleEncoderRotation(float steps, ConfigState& config) {
  resetInactivityTimer();
  _menu_step_accumulator += steps;
  int change = (int)_menu_step_accumulator;
  if (change == 0) return true;
  _menu_step_accumulator -= (float)change;

  switch (_current_screen) {
    case SCREEN_STANDBY: {
      const int num_items = 6;
      _standby_selection = (_standby_selection + change % num_items + num_items) % num_items;
      break;
    }
  
    case SCREEN_SETTINGS_MAIN: {
      const int num_items = MENU_ITEM_COUNT;
      _selected_menu_item = (_selected_menu_item + change % num_items + num_items) % num_items;
      break;
    }
    
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
    case SCREEN_SETTINGS_MAX_TEMP_LOCK: {
      _temp_edit_value += (float)change * 0.5f;
      if (_temp_edit_value < 0) _temp_edit_value = 0;
      if (_temp_edit_value > 800) _temp_edit_value = 800; // Hard limit
      break;
    }
    case SCREEN_SETTINGS_IDLE_OFF: {
      const int num_items = IDLE_OFF_ITEM_COUNT;
      _selected_menu_item = (_selected_menu_item + change % num_items + num_items) % num_items;
      break;
    }
    case SCREEN_SETTINGS_LIGHT_SOUND: {
      if (change != 0) { // Only save if there was actual change
        if (_selected_menu_item == 0) config.light_on = !config.light_on;
        else config.sound_on = !config.sound_on;
        if (_save_callback) _save_callback(config); // <-- ADD THIS
      }
      break;
    }
    case SCREEN_SETTINGS_TEMP_UNIT: {
      _selected_menu_item = (_selected_menu_item == 0) ? 1 : 0;
      break;
    }
    default: return false;
  }
  return true;
}

// All drawing functions and helpers below
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
  _spr.fillSprite(TFT_BLACK);
 // 1. MODIFIED: Font size is now 2
  const int FONT_SIZE = 2;
  
  // 2. MODIFIED: Kept at 20. 16px font + 4px spacing = 20px.
  // This makes 6 lines take 120px, fitting above the buttons at Y=130.
  const int LINE_SPACING = 20;
  
  // 3. MODIFIED: Labels are shortened to fit
  const char* labels[] = {
    "H1 Temp", "H2 Temp", "H3 Temp",
    "TC Temp", "IR1 Temp", "IR2 Temp"
  };

  float temps_c[] = {
    state.tc_temps[0], NAN, state.tc_temps[2],
    state.tc_temps[1], state.ir_temps[0], state.ir_temps[1]
  };
  
  float settings[3][2] = {
    {config.target_temps[0], config.max_temps[0]},
    {config.target_temps[1], config.max_temps[1]},
    {config.target_temps[2], config.max_temps[2]},
  };

  for (int i = 0; i < 6; ++i) {
    int y = 4 + i * LINE_SPACING;
    
    uint16_t bg_color = COLOR_IDLE;
    if (i == 0 && config.heater_active[0]) {
        bg_color = getStatusColor(state.is_heating_active, temps_c[i], state.target_temp);
    }

    if (temps_c[i] > 270.0f && _blink_state) {
      bg_color = COLOR_WARN;
    }
    _spr.fillRect(0, y, _spr.width(), LINE_SPACING, bg_color);

    if (i == 0 && state.is_heating_active) {
      float progress = (millis() % 1500) / 1500.0f;
      int bar_width = (int)((_spr.width() - 4) * progress);
      _spr.fillRect(2, y + LINE_SPACING - 4, bar_width, 3, TFT_CYAN);
    }
    
    _spr.setTextColor(TFT_WHITE, bg_color);
    _spr.setTextDatum(ML_DATUM);
    _spr.setTextSize(FONT_SIZE);
    
    char buffer[80];
    float display_temp = convertTemp(temps_c[i], state.temp_unit);
    
    if (i < 3) {
      char settings_buf[20];
      int settings_index = i;
      snprintf(settings_buf, sizeof(settings_buf), "(%.0f, %.0f)",
               convertTemp(settings[settings_index][0], state.temp_unit),
               convertTemp(settings[settings_index][1], state.temp_unit));
      
      if (isnan(display_temp)) {
        snprintf(buffer, sizeof(buffer), "%s --- %s", labels[i], settings_buf);
      } else {
        snprintf(buffer, sizeof(buffer), "%s %.2f %c %s",
                 labels[i], display_temp, state.temp_unit, settings_buf);
      }
    } else {
      if (isnan(display_temp)) {
        snprintf(buffer, sizeof(buffer), "%s ---", labels[i]);
      } else {
        snprintf(buffer, sizeof(buffer), "%s %.2f %c", labels[i], display_temp, state.temp_unit);
      }
    }
    _spr.drawString(buffer, 5, y + (LINE_SPACING / 2));
  }
  
  // --- NEW 3-ROW BUTTON LAYOUT ---
  
  int button_h = 30;
  int x_padding = 10;
  int button_spacing = 10;
  int screen_w = _spr.width();
  int usable_w = screen_w - (2 * x_padding);

  // Row 1: 3 Buttons (Heater1, Heater2, Heater3)
  int button_y1 = 130;
  int button_w1 = (usable_w - (2 * button_spacing)) / 3; // (300 - 20) / 3 = 93
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
      bg = COLOR_ACTIVE; // Green for enabled
    } else {
      bg = TFT_WHITE; // White for idle
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

  // Row 2: 2 Buttons (start, stop)
  int button_y2 = button_y1 + button_h + button_spacing;
  int button_w2 = (usable_w - (1 * button_spacing)) / 2; // (300 - 10) / 2 = 145
  int total_w2 = (button_w2 * 2) + (button_spacing * 1);
  int x_start2 = (screen_w - total_w2) / 2;
  
  uint16_t colors[] = {COLOR_START, COLOR_STOP};
  uint16_t fg_colors[] = {TFT_BLACK, TFT_WHITE};

  for (int i = 0; i < 2; i++) {
    int x = x_start2 + i * (button_w2 + button_spacing);
    uint16_t bg = colors[i];
    uint16_t fg = fg_colors[i];
    int selection_index = i + 3; // 3 and 4

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

  // Row 3: 1 Button (Settings)
  int button_y3 = button_y2 + button_h + button_spacing;
  int button_w3 = usable_w; // 300
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

void UIManager::drawSettingsMain(const AppState& state, const ConfigState& config) {
  _spr.fillSprite(TFT_BLACK);
  const int ITEM_HEIGHT = 24;
  const int ITEM_SPACING = ITEM_HEIGHT + 4;
  const int X_MARGIN = 10;
  int w = _spr.width() - (2 * X_MARGIN);

  for (int i = 0; i < MENU_ITEM_COUNT; ++i) {
    int y = 4 + i * ITEM_SPACING;
    uint16_t bg_color;
    uint16_t text_color;

    bool out_of_bounds = false;
    if (i >= MENU_HEATER_1 && i <= MENU_HEATER_3) {
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
    _spr.drawString(menu_item_labels[i], _spr.width() / 2, y + ITEM_HEIGHT / 2);
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

void UIManager::drawSettingsLightSound(const AppState& state, const ConfigState& config) {
    _spr.fillSprite(TFT_BLACK);
    _spr.setTextColor(TFT_WHITE, TFT_BLACK);
    _spr.setTextDatum(TC_DATUM);
    _spr.setTextSize(2);
    _spr.drawString("Light and Sound", _spr.width() / 2, 20);

    _spr.setTextDatum(MC_DATUM);
    _spr.setTextSize(2);
    if (_selected_menu_item == 0) _spr.drawRect(40, 80, _spr.width() - 80, 40, TFT_YELLOW);
    _spr.setTextColor(TFT_WHITE);
    _spr.drawString(config.light_on ? "Light: ON" : "Light: OFF", _spr.width() / 2, 100);

    if (_selected_menu_item == 1) _spr.drawRect(40, 140, _spr.width() - 80, 40, TFT_YELLOW);
    _spr.drawString(config.sound_on ? "Sound: ON" : "Sound: OFF", _spr.width() / 2, 160);
    
    _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
    _spr.setTextDatum(BC_DATUM);
    _spr.setTextSize(1);
    _spr.drawString("Press: Select | Rot: Toggle | Dbl-Press: Back", _spr.width() / 2, _spr.height() - 10);
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
