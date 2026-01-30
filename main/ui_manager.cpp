#include "ui_manager.h"
#include "Arial18.h"
#include "Arial12.h"
#include "taskbar_icons.h"

// WiFi Status enum (must match main.ino)
enum WiFiConnectionStatus {
  WIFI_STATUS_DISCONNECTED = 0,
  WIFI_STATUS_CONNECTING,
  WIFI_STATUS_CONNECTED
};

// --- Theme Color Definitions ---
#define COLOR_IDLE 0x39E7    // Dark Grey
#define COLOR_ACTIVE 0x05E0  // Dark Green
#define COLOR_DONE TFT_BLUE
#define COLOR_WARN TFT_ORANGE

// UI Selection Colors (Modern Theme)
#define C_SELECT_BG 0x18E3  // Dark Blue-ish Grey
#define C_SELECT_TXT TFT_CYAN
#define C_NORMAL_TXT TFT_WHITE
#define C_NORMAL_BG TFT_BLACK

#define C_WHITE 0xFFFF
#define C_GREY_BG 0xACB9
#define C_PINK_BG 0xF71F
#define C_LIME_BG 0xE7F8
#define C_ACTIVE_CYCLE_BG 0xC6D8
#define C_RED_TXT 0xF986
#define C_GREEN_TXT 0x4D6A
#define C_BLACK 0x0000
#define C_GREY_TXT 0x7BEF

extern WiFiConnectionStatus getWiFiStatus();
extern int getWiFiSignalStrength();

const uint32_t INACTIVITY_TIMEOUT_MS = 10000;

// ==========================================
// [ADDED] WiFi Charset & Menu Labels
// ==========================================
const char WIFI_CHARSET[] =
  " !\"#$%&'()*+,-./0123456789:;<=>?@"
  "ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`"
  "abcdefghijklmnopqrstuvwxyz{|}~";

const char* UIManager::getCharset() {
  return WIFI_CHARSET;
}
int UIManager::getCharsetLength() {
  return strlen(WIFI_CHARSET);
}

int UIManager::findCharInCharset(char c) {
  const char* charset = getCharset();
  for (int i = 0; i < getCharsetLength(); i++) {
    if (charset[i] == c) return i;
  }
  return 0;  // Default to space if not found
}

// --- Menu Labels ---
const char* menu_item_labels_page_1[MENU_PAGE1_ITEM_COUNT] = {
  "Max Temp Lock", "Heater Calibration", "IR Emissivity", "TC Probe Cal"
};

const char* menu_item_labels_page_2[MENU_PAGE2_ITEM_COUNT] = {
  "Turn off When Idle", "Startup Mode", "Sound", "Temp Unit", "Brightness", "About"
};

// [ADDED] Page 3 Labels
const char* menu_item_labels_page_3[MENU_PAGE3_ITEM_COUNT] = {
  "WiFi Settings"
};

// [ADDED] WiFi Menu Labels
const char* wifi_menu_labels[WIFI_MENU_ITEM_COUNT] = {
  "Edit SSID", "Edit Password", "Connection Status", "Reconnect WiFi", "< Back"
};

const char* startup_mode_labels[STARTUP_MODE_COUNT] = { "OFF", "Auto Run" };
const char* idle_off_labels[IDLE_OFF_ITEM_COUNT] = { "10 sec. (Debug)", "15 min.", "30 min.", "60 min.", "Always ON" };
const char* temp_unit_labels[2] = { "Celsius (C)", "Fahrenheit (F)" };
const char* standby_button_labels[6] = { "Heater1", "Heater2", "Heater3", "start", "stop", "Settings" };

// --- Screen Switching Helpers ---
void UIManager::switchToAutoMode() {
  _previous_screen = SCREEN_AUTO_MODE;
  _current_screen = SCREEN_AUTO_MODE;
  resetInactivityTimer();
}
void UIManager::switchToManualMode() {
  _previous_screen = SCREEN_MANUAL_MODE;
  _current_screen = SCREEN_MANUAL_MODE;
  _manual_selection = _manual_confirmed_preset;
  resetInactivityTimer();
}
void UIManager::switchToStandby() {
  _previous_screen = SCREEN_STANDBY;
  _current_screen = SCREEN_STANDBY;
  resetInactivityTimer();
}

// --- Constructor ---
UIManager::UIManager(TFT_eSPI* tft, ConfigSaveCallback save_callback)
  : _tft(tft), _spr(_tft), _save_callback(save_callback) {
  _current_screen = SCREEN_STANDBY;
  _previous_screen = SCREEN_STANDBY;
  _quick_edit_step = Q_EDIT_TARGET;
  _blink_state = false;
  _is_editing_calibration = false;
  _selected_menu_item = 0;
  _selected_menu_item_page_2 = 0;
  _menu_step_accumulator = 0.0f;
  _standby_selection = 0;
  _auto_selection = 0;
  _manual_selection = 0;
  _manual_confirmed_preset = 0;
  _show_warning = false;

  // [ADDED] Init WiFi UI variables
  _selected_menu_item_page_3 = 0;
  _selected_wifi_menu_item = 0;
  _char_entry_cursor = 0;
  _char_entry_char_index = 0;
  _char_entry_max_len = WIFI_SSID_MAX_LEN;
  _char_entry_editing = false;
  memset(_char_entry_buffer, 0, sizeof(_char_entry_buffer));
  _wifi_reconnect_callback = nullptr;
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
  if (_current_screen != SCREEN_SETTINGS_PAGE_1 && _current_screen != SCREEN_SETTINGS_PAGE_2 && _current_screen != SCREEN_SETTINGS_PAGE_3) {
    _previous_screen = _current_screen;
  }
  _current_screen = SCREEN_SETTINGS_PAGE_1;
  _selected_menu_item = 0;
  resetInactivityTimer();
}

void UIManager::exitSettings() {
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
    _quick_edit_step = Q_EDIT_TARGET;
    _menu_step_accumulator = 0.0f;
    resetInactivityTimer();
  }
}
void UIManager::enterQuickEditManual() {
  if (_manual_selection >= 0 && _manual_selection <= 3) {
    _current_screen = SCREEN_QUICK_EDIT_MANUAL;
    _quick_edit_step = Q_EDIT_TARGET;
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
  switch (config.idle_off_mode) {
    case 0: sleep_timeout = 10000; break;
    case 1: sleep_timeout = 900000; break;
    case 2: sleep_timeout = 1800000; break;
    case 3: sleep_timeout = 3600000; break;
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

  // Warning Logic
  _show_warning = false;
  for (int i = 0; i < 3; i++) {
    if (state.heater_cutoff_state[i]) {
      _show_warning = true;
      break;
    }
  }
  if (!_show_warning) {
    for (int i = 0; i < 2; i++) {
      if (!isnan(state.ir_ambient[i]) && state.ir_ambient[i] > 80.0f) {
        _show_warning = true;
        break;
      }
    }
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

    case SCREEN_STANDBY: drawStandbyScreen(state, config); break;
    case SCREEN_AUTO_MODE: drawAutoModeScreen(state, config); break;
    case SCREEN_MANUAL_MODE: drawManualModeScreen(state, config); break;
    case SCREEN_QUICK_EDIT_MANUAL: drawManualModeScreen(state, config); break;
    case SCREEN_QUICK_EDIT: drawStandbyScreen(state, config); break;
    case SCREEN_QUICK_EDIT_AUTO: drawAutoModeScreen(state, config); break;
    case SCREEN_SETTINGS_PAGE_1: drawSettingsPage1(state, config); break;
    case SCREEN_SETTINGS_PAGE_2: drawSettingsPage2(state, config); break;

    case SCREEN_SETTINGS_PAGE_3: drawSettingsPage3(state, config); break;
    case SCREEN_SETTINGS_WIFI_MENU: drawSettingsWiFiMenu(state, config); break;
    case SCREEN_SETTINGS_WIFI_SSID: drawSettingsWiFiSSID(state, config); break;
    case SCREEN_SETTINGS_WIFI_PASSWORD: drawSettingsWiFiPassword(state, config); break;
    case SCREEN_SETTINGS_WIFI_STATUS: drawSettingsWiFiStatus(state, config); break;

    case SCREEN_SETTINGS_HEATER_TARGET_TEMP: drawSettingsHeaterTargetTemp(state); break;
    case SCREEN_SETTINGS_HEATER_MAX_TEMP: drawSettingsHeaterMaxTemp(state); break;
    case SCREEN_SETTINGS_MAX_TEMP_LOCK: drawSettingsMaxTempLock(state); break;
    case SCREEN_SETTINGS_HEATER_CALIBRATE: drawSettingsHeaterCalibrate(state); break;
    case SCREEN_SETTINGS_CALIBRATION_SELECT: drawSettingsCalibrationSelect(state, config); break;
    case SCREEN_SETTINGS_IDLE_OFF: drawSettingsIdleOff(state, config); break;
    case SCREEN_SETTINGS_STARTUP: drawSettingsStartup(state, config); break;
    case SCREEN_SETTINGS_SOUND: drawSettingsSound(state, config); break;
    case SCREEN_SETTINGS_TC_PROBE_CAL: drawSettingsTCProbeCal(state, config); break;
    case SCREEN_SETTINGS_TEMP_UNIT: drawSettingsTempUnit(state, config); break;
    case SCREEN_SETTINGS_ABOUT: drawSettingsAbout(state); break;
    case SCREEN_SETTINGS_BRIGHTNESS: drawSettingsBrightness(state); break;
    case SCREEN_SETTINGS_EMISSIVITY: drawSettingsEmissivity(state, config); break;
  }
  _spr.pushSprite(0, 0);
}

// --- Helper UI Components ---
void UIManager::drawTaskBar() {
  int w = _spr.width();
  int h = 24;
  _spr.fillRect(0, 0, w, h, C_BLACK);
  _spr.drawFastHLine(0, h - 1, w, TFT_DARKGREY);
  int icon_spacing = 4, right_margin = 6;

  int wifi_x = w - right_margin - WIFI_ICON_WIDTH;
  int wifi_y = (h - WIFI_ICON_HEIGHT) / 2;
  WiFiConnectionStatus wifi_status = getWiFiStatus();
  int rssi = getWiFiSignalStrength();
  const unsigned char* wifi_icon = nullptr;
  uint16_t wifi_color = TFT_DARKGREY;

  switch (wifi_status) {
    case WIFI_STATUS_CONNECTED:
      if (rssi > -50) {
        wifi_icon = wifi_connected;
        wifi_color = TFT_GREEN;
      } else if (rssi > -70) {
        wifi_icon = wifi_signal_3;
        wifi_color = TFT_GREEN;
      } else if (rssi > -80) {
        wifi_icon = wifi_signal_2;
        wifi_color = TFT_YELLOW;
      } else {
        wifi_icon = wifi_signal_1;
        wifi_color = TFT_ORANGE;
      }
      break;
    case WIFI_STATUS_CONNECTING:
      wifi_icon = _blink_state ? wifi_connecting_1 : wifi_connecting_2;
      wifi_color = TFT_CYAN;
      break;
    default:
      wifi_icon = wifi_disconnected;
      wifi_color = TFT_RED;
      break;
  }
  if (wifi_icon) _spr.drawXBitmap(wifi_x, wifi_y, wifi_icon, WIFI_ICON_WIDTH, WIFI_ICON_HEIGHT, wifi_color);

  int bt_x = wifi_x - icon_spacing - BT_ICON_WIDTH;
  _spr.drawXBitmap(bt_x, (h - BT_ICON_HEIGHT) / 2, bt_icon, BT_ICON_WIDTH, BT_ICON_HEIGHT, TFT_DARKGREY);

  if (_show_warning) {
    int warn_x = bt_x - icon_spacing - WARNING_ICON_WIDTH;
    uint16_t warn_color = _blink_state ? TFT_YELLOW : TFT_ORANGE;
    _spr.drawXBitmap(warn_x, (h - WARNING_ICON_HEIGHT) / 2, warning_icon, WARNING_ICON_WIDTH, WARNING_ICON_HEIGHT, warn_color);
  }
  _spr.setTextColor(TFT_LIGHTGREY, C_BLACK);
  _spr.setTextDatum(ML_DATUM);
  _spr.drawString("", 5, h / 2);
}

void UIManager::drawHeader(const char* title) {
  drawTaskBar();
  int y_header = 24, h_header = 32;
  _spr.fillRect(0, y_header, _spr.width(), h_header, TFT_WHITE);
  _spr.setTextColor(TFT_BLACK, TFT_WHITE);
  _spr.setTextDatum(MC_DATUM);
  _spr.loadFont(Arial18);
  _spr.drawString(title, _spr.width() / 2, y_header + (h_header / 2));
  _spr.unloadFont();
}

// --- Screen Implementations ---

void UIManager::drawSettingsPage1(const AppState& state, const ConfigState& config) {
  drawHeader("Heater Setup");
  const int ITEM_HEIGHT = 24, ITEM_SPACING = 27, X_MARGIN = 10, Y_START = 65;
  int w = _spr.width() - (2 * X_MARGIN);
  for (int i = 0; i < MENU_PAGE1_ITEM_COUNT; ++i) {
    int y = Y_START + i * ITEM_SPACING;
    uint16_t bg_color = (i == _selected_menu_item) ? C_SELECT_BG : TFT_BLACK;
    uint16_t text_color = (i == _selected_menu_item) ? C_SELECT_TXT : TFT_WHITE;
    _spr.fillRect(X_MARGIN, y, w, ITEM_HEIGHT, bg_color);
    if (i == _selected_menu_item) _spr.drawRect(X_MARGIN, y, w, ITEM_HEIGHT, TFT_DARKGREY);
    _spr.setTextColor(text_color, bg_color);
    _spr.setTextDatum(MC_DATUM);
    _spr.loadFont(Arial18);
    _spr.drawString(menu_item_labels_page_1[i], _spr.width() / 2, y + ITEM_HEIGHT / 2);
    _spr.unloadFont();
  }
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Select | Press: OK | Hold: Back", _spr.width() / 2, _spr.height() - 2);
}

void UIManager::drawSettingsPage2(const AppState& state, const ConfigState& config) {
  drawHeader("System Setup");
  const int ITEM_HEIGHT = 24, ITEM_SPACING = 27, X_MARGIN = 10, Y_START = 65;
  int w = _spr.width() - (2 * X_MARGIN);
  for (int i = 0; i < MENU_PAGE2_ITEM_COUNT; ++i) {
    int y = Y_START + i * ITEM_SPACING;
    uint16_t bg_color = (i == _selected_menu_item_page_2) ? C_SELECT_BG : TFT_BLACK;
    uint16_t text_color = (i == _selected_menu_item_page_2) ? C_SELECT_TXT : TFT_WHITE;
    _spr.fillRect(X_MARGIN, y, w, ITEM_HEIGHT, bg_color);
    if (i == _selected_menu_item_page_2) _spr.drawRect(X_MARGIN, y, w, ITEM_HEIGHT, TFT_DARKGREY);
    _spr.setTextColor(text_color, bg_color);
    _spr.setTextDatum(MC_DATUM);
    _spr.loadFont(Arial18);
    _spr.drawString(menu_item_labels_page_2[i], _spr.width() / 2, y + ITEM_HEIGHT / 2);
    _spr.unloadFont();
  }
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Select | Press: OK | Hold: Back", _spr.width() / 2, _spr.height() - 2);
}

// [ADDED] Page 3 Drawing
void UIManager::drawSettingsPage3(const AppState& state, const ConfigState& config) {
  drawHeader("Network Setup");
  const int ITEM_HEIGHT = 24, ITEM_SPACING = 27, X_MARGIN = 10, Y_START = 65;
  int w = _spr.width() - (2 * X_MARGIN);
  for (int i = 0; i < MENU_PAGE3_ITEM_COUNT; ++i) {
    int y = Y_START + i * ITEM_SPACING;
    uint16_t bg_color = (i == _selected_menu_item_page_3) ? C_SELECT_BG : TFT_BLACK;
    uint16_t text_color = (i == _selected_menu_item_page_3) ? C_SELECT_TXT : TFT_WHITE;
    _spr.fillRect(X_MARGIN, y, w, ITEM_HEIGHT, bg_color);
    if (i == _selected_menu_item_page_3) _spr.drawRect(X_MARGIN, y, w, ITEM_HEIGHT, TFT_DARKGREY);
    _spr.setTextColor(text_color, bg_color);
    _spr.setTextDatum(MC_DATUM);
    _spr.loadFont(Arial18);
    _spr.drawString(menu_item_labels_page_3[i], _spr.width() / 2, y + ITEM_HEIGHT / 2);
    _spr.unloadFont();
  }
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Select | Press: OK | Hold: Back", _spr.width() / 2, _spr.height() - 2);
}

// [ADDED] WiFi Menu Drawing
void UIManager::drawSettingsWiFiMenu(const AppState& state, const ConfigState& config) {
  drawHeader("WiFi Settings");
  const int ITEM_HEIGHT = 22, ITEM_SPACING = 25, X_MARGIN = 10, Y_START = 60;
  int w = _spr.width() - (2 * X_MARGIN);

  for (int i = 0; i < WIFI_MENU_ITEM_COUNT; ++i) {
    int y = Y_START + i * ITEM_SPACING;
    uint16_t bg_color = (i == _selected_wifi_menu_item) ? C_SELECT_BG : TFT_BLACK;
    uint16_t text_color = (i == _selected_wifi_menu_item) ? C_SELECT_TXT : TFT_WHITE;
    _spr.fillRect(X_MARGIN, y, w, ITEM_HEIGHT, bg_color);
    if (i == _selected_wifi_menu_item) _spr.drawRect(X_MARGIN, y, w, ITEM_HEIGHT, TFT_DARKGREY);

    _spr.setTextColor(text_color, bg_color);
    _spr.setTextDatum(ML_DATUM);
    _spr.loadFont(Arial18);
    char buf[50];
    switch (i) {
      case WIFI_MENU_SSID:
        snprintf(buf, 50, "SSID: %.15s%s", (strlen(config.wifi_config.ssid) > 0) ? config.wifi_config.ssid : "(not set)", (strlen(config.wifi_config.ssid) > 15) ? "..." : "");
        _spr.drawString(buf, X_MARGIN + 5, y + ITEM_HEIGHT / 2);
        break;
      case WIFI_MENU_PASSWORD:
        snprintf(buf, 50, "Password: %s", (strlen(config.wifi_config.password) > 0) ? "****" : "(not set)");
        _spr.drawString(buf, X_MARGIN + 5, y + ITEM_HEIGHT / 2);
        break;
      default:
        _spr.setTextDatum(MC_DATUM);
        _spr.drawString(wifi_menu_labels[i], _spr.width() / 2, y + ITEM_HEIGHT / 2);
        break;
    }
    _spr.unloadFont();
  }
  _spr.setTextColor(TFT_DARKGREY, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  WiFiConnectionStatus wifi_status = getWiFiStatus();
  const char* status_str = (wifi_status == WIFI_STATUS_CONNECTED) ? "Connected" : (wifi_status == WIFI_STATUS_CONNECTING) ? "Connecting..."
                                                                                                                          : "Disconnected";
  char info_buf[60];
  snprintf(info_buf, 60, "Status: %s", status_str);
  _spr.drawString(info_buf, _spr.width() / 2, _spr.height() - 2);
}

// [ADDED] Character Entry Screen
void UIManager::drawCharEntryScreen(const char* title, bool is_password) {
  drawHeader(title);

  int w = _spr.width();
  int h = _spr.height();

  // Calculate visible characters
  const int CHAR_WIDTH = 14;
  const int CHAR_HEIGHT = 24;
  const int CHARS_VISIBLE = (w - 40) / CHAR_WIDTH;
  const int START_X = 20;
  const int TEXT_Y = 80;

  // Scroll Logic
  int scroll_offset = 0;
  if (_char_entry_cursor > CHARS_VISIBLE / 2) {
    scroll_offset = _char_entry_cursor - CHARS_VISIBLE / 2;
  }
  int str_len = strlen(_char_entry_buffer);
  if (scroll_offset + CHARS_VISIBLE > str_len + 1) {
    scroll_offset = max(0, str_len + 1 - CHARS_VISIBLE);
  }

  // Draw Text Box
  _spr.fillRect(START_X - 5, TEXT_Y - 5, w - 40 + 10, CHAR_HEIGHT + 10, TFT_DARKGREY);
  _spr.drawRect(START_X - 5, TEXT_Y - 5, w - 40 + 10, CHAR_HEIGHT + 10, TFT_WHITE);

  _spr.loadFont(Arial18);

  // Draw Buffer Characters
  for (int i = 0; i < CHARS_VISIBLE && (scroll_offset + i) <= str_len; i++) {
    int buf_idx = scroll_offset + i;
    int x = START_X + i * CHAR_WIDTH;

    bool is_cursor = (buf_idx == _char_entry_cursor);
    char display_char;

    if (buf_idx < str_len) {
      display_char = is_password ? '*' : _char_entry_buffer[buf_idx];
    } else {
      display_char = '_';
    }

    // Draw Cursor Highlight
    if (is_cursor) {
      if (_char_entry_editing) {
        _spr.fillRect(x - 1, TEXT_Y - 2, CHAR_WIDTH, CHAR_HEIGHT, _blink_state ? TFT_CYAN : TFT_BLUE);
        _spr.setTextColor(TFT_BLACK, _blink_state ? TFT_CYAN : TFT_BLUE);
      } else {
        _spr.fillRect(x - 1, TEXT_Y - 2, CHAR_WIDTH, CHAR_HEIGHT, TFT_YELLOW);
        _spr.setTextColor(TFT_BLACK, TFT_YELLOW);
      }
    } else {
      _spr.setTextColor(TFT_WHITE, TFT_DARKGREY);
    }

    char str[2] = { display_char, '\0' };
    _spr.setTextDatum(TL_DATUM);
    _spr.drawString(str, x, TEXT_Y);
  }

  _spr.unloadFont();

  // --- Character Selector (แก้ไขส่วนนี้เพื่อแสดง DEL) ---
  if (_char_entry_editing) {
    const char* charset = getCharset();
    int charset_len = getCharsetLength();
    int total_options = charset_len + 1;  // +1 for DEL

    int selector_y = TEXT_Y + 45;
    _spr.fillRect(10, selector_y, w - 20, 50, C_SELECT_BG);
    _spr.drawRect(10, selector_y, w - 20, 50, TFT_CYAN);

    _spr.loadFont(Arial18);
    _spr.setTextDatum(MC_DATUM);

    const int SHOW_CHARS = 9;
    int center_x = w / 2;
    int char_spacing = 24;

    for (int i = -SHOW_CHARS / 2; i <= SHOW_CHARS / 2; i++) {
      // Logic หมุนวนรวมตัวเลือก DEL
      int idx = (_char_entry_char_index + i + total_options) % total_options;
      int x = center_x + (i * char_spacing);

      bool is_del = (idx == charset_len);

      if (i == 0) {
        // Selected Item
        _spr.fillRect(x - 10, selector_y + 5, 20, 40, TFT_CYAN);
        if (is_del) {
          _spr.setTextColor(TFT_RED, TFT_CYAN);
          _spr.drawString("<", x, selector_y + 25);  // ใช้เครื่องหมาย < แทน Backspace
        } else {
          char str[2] = { charset[idx], '\0' };
          _spr.setTextColor(TFT_BLACK, TFT_CYAN);
          _spr.drawString(str, x, selector_y + 25);
        }
      } else {
        // Nearby Items
        uint16_t fade = (abs(i) == 1) ? TFT_WHITE : TFT_LIGHTGREY;
        _spr.setTextColor(fade, C_SELECT_BG);
        if (is_del) {
          _spr.drawString("<", x, selector_y + 25);
        } else {
          char str[2] = { charset[idx], '\0' };
          _spr.drawString(str, x, selector_y + 25);
        }
      }
    }

    _spr.unloadFont();

    // Show Name Helper
    _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
    _spr.setTextDatum(TC_DATUM);
    _spr.setTextSize(1);

    if (_char_entry_char_index == charset_len) {
      _spr.drawString("[BACKSPACE]", w / 2, selector_y + 52);
    } else {
      char current_char = charset[_char_entry_char_index];
      if (current_char == ' ') _spr.drawString("[SPACE]", w / 2, selector_y + 52);
    }
  }

  // Footer
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  if (_char_entry_editing) {
    _spr.drawString("Rot:Char | Press:Set | Hold:Cancel", w / 2, h - 12);
  } else {
    _spr.drawString("Rot:Move | Press:Edit | Hold:Done", w / 2, h - 12);
  }

  char len_buf[30];
  snprintf(len_buf, 30, "Length: %d/%d", (int)strlen(_char_entry_buffer), _char_entry_max_len);
  _spr.setTextDatum(BL_DATUM);
  _spr.setTextColor(TFT_DARKGREY, TFT_BLACK);
  _spr.drawString(len_buf, 5, h - 2);
}

void UIManager::drawSettingsWiFiSSID(const AppState& state, const ConfigState& config) {
  drawCharEntryScreen("Edit SSID", false);
}

void UIManager::drawSettingsWiFiPassword(const AppState& state, const ConfigState& config) {
  drawCharEntryScreen("Edit Password", true);
}

void UIManager::drawSettingsWiFiStatus(const AppState& state, const ConfigState& config) {
  drawHeader("WiFi Status");
  _spr.loadFont(Arial18);
  _spr.setTextSize(1);
  int y = 70, line_h = 25, x_label = 10, x_value = 120;

  WiFiConnectionStatus wifi_status = getWiFiStatus();
  _spr.setTextDatum(ML_DATUM);
  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.drawString("Status:", x_label, y);
  uint16_t status_color = (wifi_status == WIFI_STATUS_CONNECTED) ? TFT_GREEN : (wifi_status == WIFI_STATUS_CONNECTING) ? TFT_CYAN
                                                                                                                       : TFT_RED;
  _spr.setTextColor(status_color, TFT_BLACK);
  _spr.drawString((wifi_status == WIFI_STATUS_CONNECTED) ? "Connected" : (wifi_status == WIFI_STATUS_CONNECTING) ? "Connecting..."
                                                                                                                 : "Disconnected",
                  x_value, y);

  y += line_h;
  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.drawString("SSID:", x_label, y);
  _spr.setTextColor(TFT_CYAN, TFT_BLACK);
  _spr.drawString((strlen(config.wifi_config.ssid) > 0) ? config.wifi_config.ssid : "(default)", x_value, y);

  y += line_h;
  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.drawString("Signal:", x_label, y);
  int rssi = getWiFiSignalStrength();
  char rssi_buf[20];

  if (wifi_status == WIFI_STATUS_CONNECTED) {
    snprintf(rssi_buf, 20, "%d dBm", rssi);
    _spr.setTextColor((rssi > -50) ? TFT_GREEN : (rssi > -70) ? TFT_YELLOW
                                                              : TFT_ORANGE,
                      TFT_BLACK);
  } else {
    snprintf(rssi_buf, 20, "N/A");
    _spr.setTextColor(TFT_DARKGREY, TFT_BLACK);
  }
  _spr.drawString(rssi_buf, x_value, y);

  y += line_h;
  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.drawString("IP:", x_label, y);
  _spr.setTextColor(TFT_CYAN, TFT_BLACK);

  // ใช้ค่า IP จาก state ที่ส่งมาจาก main.ino แทน
  if (getWiFiStatus() == WIFI_STATUS_CONNECTED) {
    _spr.drawString(state.ip_address, x_value, y);  // แสดงเลข IP จริง
  } else {
    _spr.drawString("N/A", x_value, y);
  }
  y += line_h;
  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.drawString("Config:", x_label, y);
  _spr.setTextColor(config.wifi_config.use_custom ? TFT_GREEN : TFT_DARKGREY, TFT_BLACK);
  _spr.drawString(config.wifi_config.use_custom ? "Custom" : "Default", x_value, y);

  _spr.unloadFont();
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Press or Hold: Back", _spr.width() / 2, _spr.height() - 10);
}

void UIManager::drawSettingsStartup(const AppState& state, const ConfigState& config) {
  drawHeader("Startup Mode");
  const int ITEM_HEIGHT = 30, ITEM_SPACING = 35, X_MARGIN = 20, Y_START = 75;
  int w = _spr.width() - (2 * X_MARGIN);
  for (int i = 0; i < STARTUP_MODE_COUNT; ++i) {
    int y = Y_START + i * ITEM_SPACING;
    uint16_t bg_color = (i == _selected_menu_item) ? C_SELECT_BG : TFT_BLACK;
    uint16_t text_color = (i == _selected_menu_item) ? C_SELECT_TXT : TFT_WHITE;
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
  const int ITEM_HEIGHT = 22, ITEM_SPACING = 25, X_MARGIN = 20, Y_START = 65;
  int w = _spr.width() - (2 * X_MARGIN);
  for (int i = 0; i < IDLE_OFF_ITEM_COUNT; ++i) {
    int y = Y_START + i * ITEM_SPACING;
    uint16_t bg_color = (i == _selected_menu_item) ? C_SELECT_BG : TFT_BLACK;
    uint16_t text_color = (i == _selected_menu_item) ? C_SELECT_TXT : TFT_WHITE;
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
  const int ITEM_HEIGHT = 35, ITEM_SPACING = 40, X_MARGIN = 20, Y_START = 80;
  int w = _spr.width() - (2 * X_MARGIN);
  for (int i = 0; i < 2; ++i) {
    int y = Y_START + i * ITEM_SPACING;
    uint16_t bg_color = (i == _selected_menu_item) ? C_SELECT_BG : TFT_BLACK;
    uint16_t text_color = (i == _selected_menu_item) ? C_SELECT_TXT : TFT_WHITE;
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
  drawHeader("Sound Volume");
  _spr.setTextColor(TFT_WHITE, TFT_BLACK); _spr.setTextDatum(MC_DATUM); _spr.loadFont(Arial18);

  // วาด Bar Graph
  int bar_w = 200, bar_h = 20, x = (_spr.width() - bar_w) / 2, y = 90;
  _spr.drawRect(x, y, bar_w, bar_h, TFT_WHITE);
  
  // คำนวณความยาวแท่งสี
  int fill_w = (int)((_temp_edit_value / 100.0f) * bar_w);
  if (fill_w > 0) _spr.fillRect(x + 1, y + 1, fill_w - 2, bar_h - 2, TFT_CYAN); // สีฟ้า

  char buf[30]; 
  if (_temp_edit_value == 0) snprintf(buf, 30, "Mute (Off)");
  else snprintf(buf, 30, "Volume: %.0f%%", _temp_edit_value);
  
  _spr.drawString(buf, _spr.width() / 2, y + 40);

  _spr.unloadFont();
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK); _spr.setTextDatum(BC_DATUM); _spr.setTextSize(1);
  _spr.drawString("Rot: Adjust | Press: Save", _spr.width() / 2, _spr.height() - 10);
}

void UIManager::drawSettingsEmissivity(const AppState& state, const ConfigState& config) {
  drawHeader("IR Emissivity");
  const int ITEM_HEIGHT = 35, ITEM_SPACING = 38, X_MARGIN = 20, Y_START = 80;
  int w = _spr.width() - (2 * X_MARGIN);
  _spr.loadFont(Arial18);

  for (int i = 0; i < 2; i++) {
    int y = Y_START + (i * ITEM_SPACING);
    uint16_t bg_color = (i == _selected_menu_item) ? C_SELECT_BG : TFT_BLACK;
    uint16_t txt_color = (i == _selected_menu_item) ? C_SELECT_TXT : TFT_WHITE;
    if (i == 1 && isnan(state.ir_temps[1]) && i != _selected_menu_item) txt_color = TFT_DARKGREY;

    _spr.fillRect(X_MARGIN, y, w, ITEM_HEIGHT, bg_color);
    if (i == _selected_menu_item) _spr.drawRect(X_MARGIN, y, w, ITEM_HEIGHT, TFT_DARKGREY);

    _spr.setTextColor(txt_color, bg_color);
    _spr.setTextDatum(MC_DATUM);
    char buf[40];
    if (i == 0) {
      float disp_val = convertTemp(state.ir_temps[0], state.temp_unit);
      snprintf(buf, 40, isnan(state.ir_temps[0]) ? "IR1 (---): %.2f" : "IR1 (%.1f%c): %.2f", disp_val, isnan(state.ir_temps[0]) ? ' ' : state.temp_unit, config.ir_emissivity[0]);
    } else {
      float disp_val = convertTemp(state.ir_temps[1], state.temp_unit);
      if (!isnan(state.ir_temps[1])) snprintf(buf, 40, "IR2 (%.1f%c): %.2f", disp_val, state.temp_unit, config.ir_emissivity[1]);
      else snprintf(buf, 40, "IR2: Not Connected");
    }
    _spr.drawString(buf, _spr.width() / 2, y + ITEM_HEIGHT / 2);
  }
  _spr.unloadFont();
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Adjust | Press: Switch | Hold: Back", _spr.width() / 2, _spr.height() - 10);
}

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
    float raw_c = state.tc_probe_temp - config.tc_probe_offset;
    _spr.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    snprintf(buf, 32, "Raw: %.0f %c", convertTemp(raw_c, state.temp_unit), state.temp_unit);
    _spr.drawString(buf, _spr.width() / 2, info_y);
    snprintf(buf, 32, "Offset: %.0f %c", convertDelta(config.tc_probe_offset, state.temp_unit), state.temp_unit);
    _spr.drawString(buf, _spr.width() / 2, info_y + 25);
    _spr.setTextColor(TFT_GREEN, TFT_BLACK);
    snprintf(buf, 32, "Val: %.0f %c", convertTemp(state.tc_probe_temp, state.temp_unit), state.temp_unit);
    _spr.drawString(buf, _spr.width() / 2, info_y + 50);
  }

  int start_y = 150;
  const char* options[] = { "Tare (Set 0)", "Reset (0.0)" };
  int w = _spr.width() - 40;
  int h = 26;
  for (int i = 0; i < 2; i++) {
    int y = start_y + (i * 30);
    uint16_t bg = (i == _selected_menu_item) ? C_SELECT_BG : TFT_BLACK;
    uint16_t txt = (i == _selected_menu_item) ? C_SELECT_TXT : TFT_WHITE;
    _spr.fillRect(20, y, w, h, bg);
    if (i == _selected_menu_item) _spr.drawRect(20, y, w, h, TFT_DARKGREY);
    _spr.setTextColor(txt, bg);
    _spr.drawString(options[i], _spr.width() / 2, y + h / 2);
  }
  _spr.unloadFont();
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Press: Action | Hold: Back", _spr.width() / 2, _spr.height() - 10);
}

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
    float disp_off = convertDelta(config.tc_offsets[i], state.temp_unit);
    if (i == _selected_menu_item) {
      bg = C_SELECT_BG;
      if (_is_editing_calibration) {
        txt = C_RED_TXT;
        snprintf(item_labels[i], 30, "<< %.0f %c >>", disp_off, state.temp_unit);
      } else {
        txt = C_SELECT_TXT;
        snprintf(item_labels[i], 30, "Heater %d Offset: %.0f%c", i + 1, disp_off, state.temp_unit);
      }
    } else {
      snprintf(item_labels[i], 30, "Heater %d Offset: %.0f%c", i + 1, disp_off, state.temp_unit);
    }
    _spr.fillRect(10, y, w, h, bg);
    if (i == _selected_menu_item) _spr.drawRect(10, y, w, h, TFT_DARKGREY);
    _spr.setTextDatum(MC_DATUM);
    _spr.setTextColor(txt, bg);
    _spr.drawString(item_labels[i], _spr.width() / 2, y + h / 2);
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
  snprintf(temp_buffer, sizeof(temp_buffer), "Target Temp: %.0f %c", convertTemp(_temp_edit_value, state.temp_unit), state.temp_unit);
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
  snprintf(temp_buffer, sizeof(temp_buffer), "Max Temp: %.0f %c", convertTemp(_temp_edit_value, state.temp_unit), state.temp_unit);
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
  snprintf(temp_buffer, sizeof(temp_buffer), "Max Temp: %.0f %c", convertTemp(_temp_edit_value, state.temp_unit), state.temp_unit);
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
  _spr.drawString("Press or Hold: Back", _spr.width() / 2, _spr.height() - 10);
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
        config.heater_active[_standby_selection] = !config.heater_active[_standby_selection];
        if (_save_callback) _save_callback(config);
      } else if (_standby_selection == 5) {
        openSettings();
      }
      return true;
    case SCREEN_QUICK_EDIT:
      if (_quick_edit_step == Q_EDIT_TARGET) _quick_edit_step = Q_EDIT_MAX;
      else {
        if (_save_callback) _save_callback(config);
        _current_screen = SCREEN_STANDBY;
      }
      return true;
    case SCREEN_QUICK_EDIT_AUTO:
      if (_quick_edit_step == Q_EDIT_TARGET) _quick_edit_step = Q_EDIT_MAX;
      else {
        if (_save_callback) _save_callback(config);
        _current_screen = SCREEN_AUTO_MODE;
      }
      return true;
    case SCREEN_MANUAL_MODE:
      {
        int next_preset = _manual_confirmed_preset + 1;
        if (next_preset > 3) next_preset = 0;
        _manual_selection = next_preset;
        _manual_confirmed_preset = next_preset;
        return true;
      }
    case SCREEN_QUICK_EDIT_MANUAL:
      if (_quick_edit_step == Q_EDIT_TARGET) _quick_edit_step = Q_EDIT_MAX;
      else {
        if (_save_callback) _save_callback(config);
        _current_screen = SCREEN_MANUAL_MODE;
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
          _is_editing_calibration = false;
          break;
        case MENU_PAGE1_EMISSIVITY:
          _current_screen = SCREEN_SETTINGS_EMISSIVITY;
          _selected_menu_item = 0;
          break;
        case MENU_PAGE1_TC_PROBE_CAL:
          _current_screen = SCREEN_SETTINGS_TC_PROBE_CAL;
          _selected_menu_item = 0;
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
        case MENU_PAGE2_STARTUP:
          _current_screen = SCREEN_SETTINGS_STARTUP;
          _selected_menu_item = (int)config.startup_mode;
          if (_selected_menu_item >= STARTUP_MODE_COUNT) _selected_menu_item = 0;
          break;
        case MENU_PAGE2_SOUND:
          _current_screen = SCREEN_SETTINGS_SOUND;
          _temp_edit_value = (float)config.sound_volume; // โหลดค่า Volume ปัจจุบันมาแก้ไข
          break;
        case MENU_PAGE2_TEMP_UNIT:
          _current_screen = SCREEN_SETTINGS_TEMP_UNIT;
          _selected_menu_item = (config.temp_unit == 'C') ? 0 : 1;
          break;
        case MENU_PAGE2_BRIGHTNESS:
          _current_screen = SCREEN_SETTINGS_BRIGHTNESS;
          _temp_edit_value = (float)config.brightness;
          break;
        case MENU_PAGE2_ABOUT: _current_screen = SCREEN_SETTINGS_ABOUT; break;
      }
      _menu_step_accumulator = 0.0f;
      return true;

    // [ADDED] Page 3 Handler
    case SCREEN_SETTINGS_PAGE_3:
      switch (_selected_menu_item_page_3) {
        case MENU_PAGE3_WIFI_SETTINGS:
          _current_screen = SCREEN_SETTINGS_WIFI_MENU;
          _selected_wifi_menu_item = 0;
          break;
      }
      _menu_step_accumulator = 0.0f;
      return true;

    // [ADDED] WiFi Menu Handler
    case SCREEN_SETTINGS_WIFI_MENU:
      switch (_selected_wifi_menu_item) {
        case WIFI_MENU_SSID:
          strncpy(_char_entry_buffer, config.wifi_config.ssid, WIFI_SSID_MAX_LEN);
          _char_entry_buffer[WIFI_SSID_MAX_LEN] = '\0';
          _char_entry_cursor = strlen(_char_entry_buffer);
          _char_entry_char_index = 0;
          _char_entry_max_len = WIFI_SSID_MAX_LEN;
          _char_entry_editing = false;
          _current_screen = SCREEN_SETTINGS_WIFI_SSID;
          break;
        case WIFI_MENU_PASSWORD:
          strncpy(_char_entry_buffer, config.wifi_config.password, WIFI_PASS_MAX_LEN);
          _char_entry_buffer[WIFI_PASS_MAX_LEN] = '\0';
          _char_entry_cursor = strlen(_char_entry_buffer);
          _char_entry_char_index = 0;
          _char_entry_max_len = WIFI_PASS_MAX_LEN;
          _char_entry_editing = false;
          _current_screen = SCREEN_SETTINGS_WIFI_PASSWORD;
          break;
        case WIFI_MENU_STATUS: _current_screen = SCREEN_SETTINGS_WIFI_STATUS; break;
        case WIFI_MENU_CONNECT:
          if (_wifi_reconnect_callback) _wifi_reconnect_callback();
          break;
        case WIFI_MENU_BACK: _current_screen = SCREEN_SETTINGS_PAGE_3; break;
      }
      _menu_step_accumulator = 0.0f;
      return true;

    // [ADDED] Char Entry Logic
    case SCREEN_SETTINGS_WIFI_SSID:
      if (_char_entry_editing) {
        int charset_len = getCharsetLength();

        // เช็คว่าเป็นปุ่ม DEL หรือไม่
        if (_char_entry_char_index == charset_len) {
          // --- Backspace Logic ---
          if (_char_entry_cursor > 0) {
            int len = strlen(_char_entry_buffer);
            // ย้ายตัวอักษรตั้งเแต่ cursor มาทับตำแหน่งก่อนหน้า
            for (int k = _char_entry_cursor - 1; k < len; k++) {
              _char_entry_buffer[k] = _char_entry_buffer[k + 1];
            }
            _char_entry_cursor--;  // ถอย Cursor กลับ
          }
        } else {
          // --- Insert/Replace Logic (เดิม) ---
          int len = strlen(_char_entry_buffer);
          const char* charset = getCharset();

          if (_char_entry_cursor < _char_entry_max_len) {
            if (_char_entry_cursor >= len) {
              // Append
              _char_entry_buffer[_char_entry_cursor] = charset[_char_entry_char_index];
              _char_entry_buffer[_char_entry_cursor + 1] = '\0';
              _char_entry_cursor++;
            } else {
              // Replace
              _char_entry_buffer[_char_entry_cursor] = charset[_char_entry_char_index];
            }
          }
        }
        _char_entry_editing = false;
      } else {
        // Enter Edit Mode Logic (เหมือนเดิม)
        int len = strlen(_char_entry_buffer);
        if (_char_entry_cursor < len) {
          _char_entry_char_index = findCharInCharset(_char_entry_buffer[_char_entry_cursor]);
        } else {
          _char_entry_char_index = 0;
        }
        _char_entry_editing = true;
      }
      return true;
    case SCREEN_SETTINGS_BRIGHTNESS:
      config.brightness = (uint8_t)_temp_edit_value;
      if (_save_callback) _save_callback(config);
      _current_screen = SCREEN_SETTINGS_PAGE_2;
      _menu_step_accumulator = 0.0f;
      return true;

    case SCREEN_SETTINGS_WIFI_PASSWORD:
      if (_char_entry_editing) {
        int charset_len = getCharsetLength();

        if (_char_entry_char_index == charset_len) {  // DEL
          if (_char_entry_cursor > 0) {
            int len = strlen(_char_entry_buffer);
            for (int k = _char_entry_cursor - 1; k < len; k++) {
              _char_entry_buffer[k] = _char_entry_buffer[k + 1];
            }
            _char_entry_cursor--;
          }
        } else {
          // Insert/Replace Logic
          int len = strlen(_char_entry_buffer);
          const char* charset = getCharset();

          if (_char_entry_cursor < _char_entry_max_len) {
            if (_char_entry_cursor >= len) {
              _char_entry_buffer[_char_entry_cursor] = charset[_char_entry_char_index];
              _char_entry_buffer[_char_entry_cursor + 1] = '\0';
              _char_entry_cursor++;
            } else {
              _char_entry_buffer[_char_entry_cursor] = charset[_char_entry_char_index];
            }
          }
        }
        _char_entry_editing = false;
      } else {
        // Enter Edit Mode Logic
        int len = strlen(_char_entry_buffer);
        if (_char_entry_cursor < len) {
          _char_entry_char_index = findCharInCharset(_char_entry_buffer[_char_entry_cursor]);
        } else {
          _char_entry_char_index = 0;
        }
        _char_entry_editing = true;
      }
      return true;

    case SCREEN_SETTINGS_WIFI_STATUS: _current_screen = SCREEN_SETTINGS_WIFI_MENU; return true;

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
    case SCREEN_SETTINGS_CALIBRATION_SELECT:
      _is_editing_calibration = !_is_editing_calibration;
      if (!_is_editing_calibration) {
        if (_save_callback) _save_callback(config);
      }
      _menu_step_accumulator = 0.0f;
      return true;
    case SCREEN_SETTINGS_HEATER_CALIBRATE: return true;
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
      config.sound_volume = (uint8_t)_temp_edit_value; // บันทึกค่าความดัง
      if (_save_callback) _save_callback(config);
      _current_screen = SCREEN_SETTINGS_PAGE_2;
      _menu_step_accumulator = 0.0f; // Reset accumulator
      return true;
    case SCREEN_SETTINGS_STARTUP:
      config.startup_mode = (StartupMode)_selected_menu_item;
      if (_save_callback) _save_callback(config);
      _current_screen = SCREEN_SETTINGS_PAGE_2;
      _menu_step_accumulator = 0.0f;
      return true;
    case SCREEN_SETTINGS_EMISSIVITY:
      if (_selected_menu_item == 0) _selected_menu_item = 1;
      else _selected_menu_item = 0;
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

bool UIManager::handleButtonHold(ConfigState& config) {
  if (_current_screen == SCREEN_SLEEP) {
    _current_screen = SCREEN_STANDBY;
    resetInactivityTimer();
    return true;
  }
  if (_current_screen == SCREEN_MANUAL_MODE) {
    enterQuickEditManual();
    return true;
  }
  resetInactivityTimer();
  switch (_current_screen) {
    case SCREEN_SETTINGS_PAGE_1:
    case SCREEN_SETTINGS_PAGE_2:
    case SCREEN_SETTINGS_PAGE_3: _current_screen = SCREEN_STANDBY; break;
    case SCREEN_SETTINGS_HEATER_TARGET_TEMP:
    case SCREEN_SETTINGS_MAX_TEMP_LOCK: _current_screen = SCREEN_SETTINGS_PAGE_1; break;
    case SCREEN_SETTINGS_HEATER_MAX_TEMP:
      _current_screen = SCREEN_SETTINGS_HEATER_TARGET_TEMP;
      _temp_edit_value = config.target_temps[_selected_menu_item];
      break;
    case SCREEN_SETTINGS_CALIBRATION_SELECT:
      if (_is_editing_calibration) _is_editing_calibration = false;
      else _current_screen = SCREEN_SETTINGS_PAGE_1;
      break;
    case SCREEN_SETTINGS_HEATER_CALIBRATE: _current_screen = SCREEN_SETTINGS_CALIBRATION_SELECT; break;
    case SCREEN_SETTINGS_EMISSIVITY: _current_screen = SCREEN_SETTINGS_PAGE_1; break;
    case SCREEN_SETTINGS_IDLE_OFF:
    case SCREEN_SETTINGS_STARTUP:
    case SCREEN_SETTINGS_TEMP_UNIT:
    case SCREEN_SETTINGS_SOUND:
    case SCREEN_SETTINGS_TC_PROBE_CAL:
    case SCREEN_SETTINGS_ABOUT: _current_screen = SCREEN_SETTINGS_PAGE_2; break;
    case SCREEN_SETTINGS_BRIGHTNESS: _current_screen = SCREEN_SETTINGS_PAGE_2; break;
    case SCREEN_SETTINGS_WIFI_MENU: _current_screen = SCREEN_SETTINGS_PAGE_3; break;
    case SCREEN_SETTINGS_WIFI_STATUS: _current_screen = SCREEN_SETTINGS_WIFI_MENU; break;
    case SCREEN_SETTINGS_WIFI_SSID:
      if (_char_entry_editing) {
        _char_entry_editing = false;
      } else {
        strncpy(config.wifi_config.ssid, _char_entry_buffer, WIFI_SSID_MAX_LEN);
        config.wifi_config.ssid[WIFI_SSID_MAX_LEN] = '\0';
        config.wifi_config.use_custom = (strlen(config.wifi_config.ssid) > 0);
        if (_save_callback) _save_callback(config);
        _current_screen = SCREEN_SETTINGS_WIFI_MENU;
      }
      break;
    case SCREEN_SETTINGS_WIFI_PASSWORD:
      if (_char_entry_editing) {
        _char_entry_editing = false;
      } else {
        strncpy(config.wifi_config.password, _char_entry_buffer, WIFI_PASS_MAX_LEN);
        config.wifi_config.password[WIFI_PASS_MAX_LEN] = '\0';
        if (_save_callback) _save_callback(config);
        _current_screen = SCREEN_SETTINGS_WIFI_MENU;
      }
      break;

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
    case SCREEN_STANDBY:
      {
        int new_pos = _standby_selection + change;
        if (new_pos < 0) new_pos = 0;
        if (new_pos >= 3) new_pos = 2;
        _standby_selection = new_pos;
        break;
      }
    case SCREEN_AUTO_MODE:
      {
        int new_pos = _auto_selection + change;
        if (new_pos < 0) new_pos = 0;
        if (new_pos > 2) new_pos = 2;
        _auto_selection = new_pos;
        break;
      }
    case SCREEN_MANUAL_MODE:
      {
        int new_pos = _manual_selection + change;
        if (new_pos < 0) new_pos = 0;
        if (new_pos > 3) new_pos = 3;
        _manual_selection = new_pos;
        break;
      }
    case SCREEN_QUICK_EDIT:
      {
        int heaterIdx = _standby_selection;
        if (_quick_edit_step == Q_EDIT_TARGET) {
          config.target_temps[heaterIdx] += (float)change * 0.5f;
          if (config.target_temps[heaterIdx] < 0) config.target_temps[heaterIdx] = 0;
          if (config.target_temps[heaterIdx] > config.max_temp_lock) config.target_temps[heaterIdx] = config.max_temp_lock;
        } else {
          config.max_temps[heaterIdx] += (float)change * 0.5f;
          if (config.max_temps[heaterIdx] < config.target_temps[heaterIdx]) config.max_temps[heaterIdx] = config.target_temps[heaterIdx];
          if (config.max_temps[heaterIdx] > config.max_temp_lock) config.max_temps[heaterIdx] = config.max_temp_lock;
        }
        break;
      }
    case SCREEN_SETTINGS_BRIGHTNESS:
      {
        _temp_edit_value += (float)change * 5.0f;  // หมุนทีละ 5%
        if (_temp_edit_value < 0) _temp_edit_value = 0;
        if (_temp_edit_value > 100) _temp_edit_value = 100;
        break;
      }
    case SCREEN_QUICK_EDIT_AUTO:
      {
        int cycleIdx = _auto_selection;
        if (_quick_edit_step == Q_EDIT_TARGET) {
          config.auto_target_temps[cycleIdx] += (float)change * 0.5f;
          if (config.auto_target_temps[cycleIdx] < 0) config.auto_target_temps[cycleIdx] = 0;
          if (config.auto_target_temps[cycleIdx] > config.max_temp_lock) config.auto_target_temps[cycleIdx] = config.max_temp_lock;
        } else {
          config.auto_max_temps[cycleIdx] += (float)change * 0.5f;
          if (config.auto_max_temps[cycleIdx] < config.auto_target_temps[cycleIdx]) config.auto_max_temps[cycleIdx] = config.auto_target_temps[cycleIdx];
          if (config.auto_max_temps[cycleIdx] > config.max_temp_lock) config.auto_max_temps[cycleIdx] = config.max_temp_lock;
        }
        break;
      }
    case SCREEN_QUICK_EDIT_MANUAL:
      {
        int presetIdx = _manual_selection;
        if (_quick_edit_step == Q_EDIT_TARGET) {
          config.manual_target_temps[presetIdx] += (float)change * 0.5f;
          if (config.manual_target_temps[presetIdx] < 0) config.manual_target_temps[presetIdx] = 0;
          if (config.manual_target_temps[presetIdx] > config.max_temp_lock) config.manual_target_temps[presetIdx] = config.max_temp_lock;
        } else {
          config.manual_max_temps[presetIdx] += (float)change * 0.5f;
          if (config.manual_max_temps[presetIdx] < config.manual_target_temps[presetIdx]) config.manual_max_temps[presetIdx] = config.manual_target_temps[presetIdx];
          if (config.manual_max_temps[presetIdx] > config.max_temp_lock) config.manual_max_temps[presetIdx] = config.max_temp_lock;
        }
        break;
      }
    case SCREEN_SETTINGS_PAGE_1:
      {
        int new_pos = _selected_menu_item + change;

        if (new_pos >= MENU_PAGE1_ITEM_COUNT) {
          _current_screen = SCREEN_SETTINGS_PAGE_2;
          _selected_menu_item_page_2 = 0;
          return true;
        }
        if (new_pos < 0) new_pos = 0;

        _selected_menu_item = new_pos;
        break;
      }
    case SCREEN_SETTINGS_PAGE_2:
      {
        int new_pos = _selected_menu_item_page_2 + change;

        if (new_pos >= MENU_PAGE2_ITEM_COUNT) {
          _current_screen = SCREEN_SETTINGS_PAGE_3;
          _selected_menu_item_page_3 = 0;
          return true;
        }
        if (new_pos < 0) {
          _current_screen = SCREEN_SETTINGS_PAGE_1;
          _selected_menu_item = MENU_PAGE1_ITEM_COUNT - 1;
          return true;
        }
        _selected_menu_item_page_2 = new_pos;
        break;
      }
    case SCREEN_SETTINGS_PAGE_3:
      {
        int new_pos = _selected_menu_item_page_3 + change;

        // ถ้าหมุนเกินตัวสุดท้าย -> หยุด (เพราะหมดหน้าแล้ว)
        if (new_pos >= MENU_PAGE3_ITEM_COUNT) new_pos = MENU_PAGE3_ITEM_COUNT - 1;

        // ถ้าหมุนถอยหลังเกินตัวแรก -> กลับไปหน้า 2 บรรทัดสุดท้าย
        if (new_pos < 0) {
          _current_screen = SCREEN_SETTINGS_PAGE_2;
          _selected_menu_item_page_2 = MENU_PAGE2_ITEM_COUNT - 1;
          return true;
        }

        _selected_menu_item_page_3 = new_pos;
        break;
      }
    case SCREEN_SETTINGS_WIFI_MENU:
      {
        int new_pos = _selected_wifi_menu_item + change;
        if (new_pos < 0) new_pos = 0;
        if (new_pos >= WIFI_MENU_ITEM_COUNT) new_pos = WIFI_MENU_ITEM_COUNT - 1;
        _selected_wifi_menu_item = new_pos;
        break;
      }
    case SCREEN_SETTINGS_WIFI_SSID:
    case SCREEN_SETTINGS_WIFI_PASSWORD:
      {
        if (_char_entry_editing) {
          // Rotate through charset PLUS one extra slot for DEL
          int charset_len = getCharsetLength();
          int total_options = charset_len + 1;  // +1 สำหรับเมนู DEL
          _char_entry_char_index = (_char_entry_char_index + change + total_options) % total_options;
        } else {
          // Move cursor logic (เหมือนเดิม)
          int len = strlen(_char_entry_buffer);
          int max_cursor = min(len, _char_entry_max_len - 1);

          int new_pos = _char_entry_cursor + change;
          if (new_pos < 0) new_pos = 0;
          if (new_pos > max_cursor + 1) new_pos = max_cursor + 1;
          if (new_pos > len) new_pos = len;

          _char_entry_cursor = new_pos;
        }
        break;
      }
    case SCREEN_SETTINGS_WIFI_STATUS: break;

    case SCREEN_SETTINGS_TC_PROBE_CAL:
      {
        int new_pos = _selected_menu_item + change;
        if (new_pos < 0) new_pos = 0;
        if (new_pos > 1) new_pos = 1;
        _selected_menu_item = new_pos;
        break;
      }
    case SCREEN_SETTINGS_CALIBRATION_SELECT:
      {
        if (_is_editing_calibration) {
          float val = config.tc_offsets[_selected_menu_item];
          val += (float)change * 0.1f;
          if (val < -20.0f) val = -20.0f;
          if (val > 20.0f) val = 20.0f;
          config.tc_offsets[_selected_menu_item] = val;
        } else {
          int new_pos = _selected_menu_item + change;
          if (new_pos < 0) new_pos = 0;
          if (new_pos >= 3) new_pos = 2;
          _selected_menu_item = new_pos;
        }
        break;
      }
    case SCREEN_SETTINGS_HEATER_TARGET_TEMP:
      {
        _temp_edit_value += (float)change * 0.5f;
        if (_temp_edit_value < 0) _temp_edit_value = 0;
        if (_temp_edit_value > config.max_temps[_selected_menu_item]) _temp_edit_value = config.max_temps[_selected_menu_item];
        break;
      }
    case SCREEN_SETTINGS_HEATER_MAX_TEMP:
      {
        _temp_edit_value += (float)change * 0.5f;
        if (_temp_edit_value < config.target_temps[_selected_menu_item]) _temp_edit_value = config.target_temps[_selected_menu_item];
        if (_temp_edit_value > config.max_temp_lock) _temp_edit_value = config.max_temp_lock;
        break;
      }
    case SCREEN_SETTINGS_HEATER_CALIBRATE:
      {
        _temp_edit_value += (float)change * 0.1f;
        break;
      }
    case SCREEN_SETTINGS_MAX_TEMP_LOCK:
      {
        _temp_edit_value += (float)change * 0.5f;
        if (_temp_edit_value < 0) _temp_edit_value = 0;
        if (_temp_edit_value > 800) _temp_edit_value = 800;
        break;
      }
    case SCREEN_SETTINGS_EMISSIVITY:
      {
        float* target = &config.ir_emissivity[_selected_menu_item];
        *target += (float)change * 0.01f;
        if (*target < 0.1f) *target = 0.1f;
        if (*target > 1.0f) *target = 1.0f;
        break;
      }
    case SCREEN_SETTINGS_STARTUP:
      {
        int new_pos = _selected_menu_item + change;
        if (new_pos < 0) new_pos = 0;
        if (new_pos >= STARTUP_MODE_COUNT) new_pos = STARTUP_MODE_COUNT - 1;
        _selected_menu_item = new_pos;
        break;
      }
    case SCREEN_SETTINGS_IDLE_OFF:
      {
        int new_pos = _selected_menu_item + change;
        if (new_pos < 0) new_pos = 0;
        if (new_pos >= IDLE_OFF_ITEM_COUNT) new_pos = IDLE_OFF_ITEM_COUNT - 1;
        _selected_menu_item = new_pos;
        break;
      }
    case SCREEN_SETTINGS_SOUND: {
        _temp_edit_value += (float)change * 5.0f; // ปรับทีละ 5%
        if (_temp_edit_value < 0) _temp_edit_value = 0;
        if (_temp_edit_value > 100) _temp_edit_value = 100;
        break;
    }
    case SCREEN_SETTINGS_TEMP_UNIT:
      {
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
  if (unit == 'F' && !isnan(temp_c)) return temp_c * 1.8f + 32.0f;
  return temp_c;
}
float UIManager::convertDelta(float temp_c, char unit) {
  if (unit == 'F') return temp_c * 1.8f;
  return temp_c;
}

uint16_t UIManager::getStatusColor(bool is_active, float current_temp, float target_temp) {
  if (!is_active) return COLOR_IDLE;
  if (fabsf(current_temp - target_temp) < 1.0f) return COLOR_DONE;
  return COLOR_ACTIVE;
}

// Draw Standby, Auto, Manual Screens implementations remain largely the same,
// included below for completeness of the file structure.

void UIManager::drawStandbyScreen(const AppState& state, const ConfigState& config) {
  drawTaskBar();
  int top_offset = 24, screen_w = _spr.width(), screen_h = _spr.height(), gap = 6;
  int heater_h = 158, heater_w = (screen_w - (4 * gap)) / 3;
  int sensor_y = top_offset + gap + heater_h + gap, sensor_h = screen_h - sensor_y - gap, sensor_w = (screen_w - (3 * gap)) / 2;

  for (int i = 0; i < 3; i++) {
    int x = gap + (i * (heater_w + gap)), y = top_offset + gap;
    bool isActive = config.heater_active[i], isLocked = state.heater_cutoff_state[i], globalRun = state.is_heating_active;
    bool isControlledByOther = (i == 0) && (state.auto_running_background || state.manual_preset_running);
    uint16_t bg_color = isControlledByOther ? C_GREY_BG : ((isActive || isLocked) ? C_WHITE : C_GREY_BG);
    if (_current_screen == SCREEN_QUICK_EDIT && _standby_selection == i) bg_color = _blink_state ? C_WHITE : C_GREY_BG;

    _spr.fillRect(x, y, heater_w, heater_h, bg_color);
    int center_x = x + (heater_w / 2);
    _spr.loadFont(Arial18);
    _spr.setTextColor((isLocked && _blink_state) ? C_RED_TXT : C_BLACK, bg_color);
    _spr.setTextDatum(TC_DATUM);
    char title[15];
    snprintf(title, 15, "Heater %d", i + 1);
    _spr.drawString(title, center_x, y + 8);
    _spr.drawFastHLine(x + 5, y + 34, heater_w - 10, C_BLACK);

    int start_y = y + 36, val_offset = 14, section_gap = 42;
    if (_current_screen == SCREEN_QUICK_EDIT && _standby_selection == i) {
      _spr.unloadFont();
      _spr.loadFont(Arial12);
      _spr.setTextColor(C_BLACK, bg_color);
      _spr.drawString("SET", center_x, start_y);
      _spr.unloadFont();
      _spr.loadFont(Arial18);
      char buf[20];
      snprintf(buf, 20, "%.0f%c", convertTemp(config.target_temps[i], state.temp_unit), state.temp_unit);
      _spr.setTextColor((_quick_edit_step == Q_EDIT_TARGET) ? C_RED_TXT : C_BLACK, bg_color);
      _spr.drawString(buf, center_x, start_y + val_offset);
      _spr.unloadFont();
      _spr.loadFont(Arial12);
      _spr.setTextColor(C_BLACK, bg_color);
      int max_y = start_y + section_gap;
      _spr.drawString("MAX", center_x, max_y);
      _spr.unloadFont();
      _spr.loadFont(Arial18);
      snprintf(buf, 20, "%.0f%c", convertTemp(config.max_temps[i], state.temp_unit), state.temp_unit);
      _spr.setTextColor((_quick_edit_step == Q_EDIT_MAX) ? C_RED_TXT : C_BLACK, bg_color);
      _spr.drawString(buf, center_x, max_y + val_offset);
    } else {
      _spr.unloadFont();
      _spr.loadFont(Arial12);
      _spr.setTextColor(C_BLACK, bg_color);
      _spr.drawString("NOW", center_x, start_y);
      _spr.unloadFont();
      _spr.loadFont(Arial18);
      char buf[20];
      if (isnan(state.tc_temps[i])) {
        snprintf(buf, 20, "---%c", state.temp_unit);
        _spr.setTextColor(C_GREY_TXT, bg_color);
      } else {
        snprintf(buf, 20, "%.0f%c", convertTemp(state.tc_temps[i], state.temp_unit), state.temp_unit);
        _spr.setTextColor((isLocked && _blink_state) ? C_RED_TXT : C_BLACK, bg_color);
      }
      _spr.drawString(buf, center_x, start_y + val_offset);

      int set_y = start_y + section_gap;
      _spr.unloadFont();
      _spr.loadFont(Arial12);
      _spr.setTextColor(C_BLACK, bg_color);
      _spr.drawString("SET", center_x, set_y);
      _spr.unloadFont();
      _spr.loadFont(Arial18);
      snprintf(buf, 20, "%.0f%c", convertTemp(config.target_temps[i], state.temp_unit), state.temp_unit);
      _spr.setTextColor(C_BLACK, bg_color);
      _spr.drawString(buf, center_x, set_y + val_offset);

      int status_y = set_y + section_gap;
      _spr.unloadFont();
      _spr.loadFont(Arial12);
      _spr.setTextColor(C_BLACK, bg_color);
      _spr.drawString("Status", center_x, status_y);
      _spr.unloadFont();
      _spr.loadFont(Arial18);
      if (isLocked) {
        _spr.setTextColor(_blink_state ? C_RED_TXT : bg_color, bg_color);
        _spr.drawString("Lock", center_x, status_y + val_offset);
      } else if (isControlledByOther) {
        _spr.setTextColor(C_GREY_TXT, bg_color);
        _spr.drawString("In-use", center_x, status_y + val_offset);
      } else if (!isActive) {
        _spr.setTextColor(C_GREY_TXT, bg_color);
        _spr.drawString("OFF", center_x, status_y + val_offset);
      } else {
        bool isRunning = false;
        if (globalRun) {
          if (state.manual_was_started) isRunning = true;
        }
        if (isRunning) {
          if (state.heater_ready[i]) {
            _spr.setTextColor(C_GREEN_TXT, bg_color);
            _spr.drawString("Ready", center_x, status_y + val_offset);
          } else {
            _spr.setTextColor(TFT_ORANGE, bg_color);
            _spr.drawString("Heating", center_x, status_y + val_offset);
          }
        } else {
          _spr.setTextColor(TFT_BLUE, bg_color);
          _spr.drawString("Standby", center_x, status_y + val_offset);
        }
      }
    }
    _spr.unloadFont();
    if (_standby_selection == i) {
      _spr.drawRect(x, y, heater_w, heater_h, TFT_RED);
      _spr.drawRect(x + 1, y + 1, heater_w - 2, heater_h - 2, TFT_RED);
    } else {
      _spr.drawRect(x, y, heater_w, heater_h, C_BLACK);
    }
  }

  _spr.fillRect(gap, sensor_y, sensor_w, sensor_h, C_PINK_BG);
  _spr.loadFont(Arial18);
  _spr.setTextColor(C_BLACK, C_PINK_BG);
  _spr.setTextDatum(ML_DATUM);
  char ir_buf[30];
  if (isnan(state.ir_temps[0])) snprintf(ir_buf, 30, "IR1 : ---%c", state.temp_unit);
  else snprintf(ir_buf, 30, "IR1 : %.0f%c", convertTemp(state.ir_temps[0], state.temp_unit), state.temp_unit);
  _spr.drawString(ir_buf, gap + 10, sensor_y + (sensor_h / 4) + 2);
  if (isnan(state.ir_temps[1])) snprintf(ir_buf, 30, "IR2 : ---%c", state.temp_unit);
  else snprintf(ir_buf, 30, "IR2 : %.0f%c", convertTemp(state.ir_temps[1], state.temp_unit), state.temp_unit);
  _spr.drawString(ir_buf, gap + 10, sensor_y + (sensor_h * 3 / 4) - 1);
  _spr.unloadFont();
  int tc_x = gap + sensor_w + gap;
  _spr.fillRect(tc_x, sensor_y, sensor_w, sensor_h, C_LIME_BG);
  _spr.loadFont(Arial18);
  _spr.setTextColor(C_BLACK, C_LIME_BG);
  char wire_buf[40];
  int center_x = tc_x + (sensor_w / 2);
  int center_y = sensor_y + (sensor_h / 2);
  if (isnan(state.tc_probe_temp)) snprintf(wire_buf, 40, "TC Temp: ---%c", state.temp_unit);
  else snprintf(wire_buf, 40, "TC Temp: %.0f%c", convertTemp(state.tc_probe_temp, state.temp_unit), state.temp_unit);
  _spr.setTextDatum(BC_DATUM);
  _spr.drawString(wire_buf, center_x, center_y - 2);
  if (isnan(state.tc_probe_peak) || state.tc_probe_peak < -100) snprintf(wire_buf, 40, "Max(5s): ---");
  else snprintf(wire_buf, 40, "Max(5s): %.0f%c", convertTemp(state.tc_probe_peak, state.temp_unit), state.temp_unit);
  _spr.setTextDatum(TC_DATUM);
  _spr.drawString(wire_buf, center_x, center_y + 2);
  _spr.unloadFont();
}

void UIManager::drawAutoModeScreen(const AppState& state, const ConfigState& config) {
  drawTaskBar();
  int w = _spr.width(), h = _spr.height(), header_y = 24, header_h = 30;
  _spr.fillRect(0, header_y, w, header_h, C_BLACK);
  _spr.loadFont(Arial18);
  _spr.setTextColor(TFT_WHITE, C_BLACK);
  _spr.setTextDatum(ML_DATUM);
  _spr.drawString("Auto mode", 5, header_y + 15);
  _spr.setTextDatum(MR_DATUM);
  char buf[40];
  if (isnan(state.ir_temps[0])) snprintf(buf, 40, "Obj. Temp : ---%c", state.temp_unit);
  else snprintf(buf, 40, "Obj. Temp : %.0f%c", convertTemp(state.ir_temps[0], state.temp_unit), state.temp_unit);
  _spr.drawString(buf, w - 5, header_y + 15);
  _spr.unloadFont();

  int gap = 6, sensor_y = 24 + 6 + 158 + 6, sensor_h = h - sensor_y - gap, sensor_w = (w - (3 * gap)) / 2;
  int cycle_start_y = header_y + header_h + gap, cycle_box_h = sensor_y - gap - cycle_start_y, cycle_box_w = (w - (4 * gap)) / 3;

  for (int i = 0; i < 3; i++) {
    int x = gap + (i * (cycle_box_w + gap));
    bool standbyUsingMain = state.manual_running_background && config.heater_active[0];
    bool isOtherModeRunning = state.manual_preset_running || (standbyUsingMain && !state.auto_running_background);
    bool isThisCycleActive = (state.auto_step == (i + 1)), globalRun = state.is_heating_active, isLocked = state.heater_cutoff_state[0], isReady = state.heater_ready[0], isSelected = (_auto_selection == i);
    uint16_t bg_color = isOtherModeRunning ? C_GREY_BG : ((_current_screen == SCREEN_QUICK_EDIT_AUTO && isSelected) ? (_blink_state ? TFT_WHITE : C_GREY_BG) : ((isThisCycleActive && globalRun) ? C_ACTIVE_CYCLE_BG : TFT_WHITE));

    _spr.fillRect(x, cycle_start_y, cycle_box_w, cycle_box_h, bg_color);
    if (isSelected) {
      _spr.drawRect(x, cycle_start_y, cycle_box_w, cycle_box_h, TFT_RED);
      _spr.drawRect(x + 1, cycle_start_y + 1, cycle_box_w - 2, cycle_box_h - 2, TFT_RED);
    } else {
      _spr.drawRect(x, cycle_start_y, cycle_box_w, cycle_box_h, TFT_BLACK);
    }

    _spr.loadFont(Arial18);
    _spr.setTextColor(C_BLACK, bg_color);
    _spr.setTextDatum(TC_DATUM);
    snprintf(buf, 20, "Cycle %d", i + 1);
    _spr.drawString(buf, x + (cycle_box_w / 2), cycle_start_y + 8);
    _spr.drawFastHLine(x + 5, cycle_start_y + 32, cycle_box_w - 10, C_BLACK);

    int set_y_label = cycle_start_y + 38, set_y_val = set_y_label + 14, status_y_label = set_y_val + 24, status_y_val = status_y_label + 20;
    _spr.loadFont(Arial12);
    _spr.setTextColor(C_BLACK, bg_color);
    _spr.drawString("SET", x + (cycle_box_w / 2), set_y_label);
    _spr.loadFont(Arial18);
    bool isEditingThis = (_current_screen == SCREEN_QUICK_EDIT_AUTO && isSelected);
    _spr.setTextColor((isEditingThis && _quick_edit_step == Q_EDIT_TARGET) ? C_RED_TXT : C_BLACK, bg_color);
    snprintf(buf, 20, "%.0f%c", convertTemp(config.auto_target_temps[i], state.temp_unit), state.temp_unit);
    _spr.drawString(buf, x + (cycle_box_w / 2), set_y_val);

    _spr.loadFont(Arial12);
    _spr.setTextColor(C_BLACK, bg_color);
    _spr.setTextDatum(TC_DATUM);
    if (isEditingThis) {
      _spr.drawString("MAX", x + (cycle_box_w / 2), status_y_label);
      _spr.loadFont(Arial18);
      _spr.setTextColor((_quick_edit_step == Q_EDIT_MAX) ? C_RED_TXT : C_BLACK, bg_color);
      snprintf(buf, 20, "%.0f%c", convertTemp(config.auto_max_temps[i], state.temp_unit), state.temp_unit);
      _spr.drawString(buf, x + (cycle_box_w / 2), status_y_val);
    } else {
      _spr.drawString("Status", x + (cycle_box_w / 2), status_y_label);
      _spr.loadFont(Arial18);
      if (isOtherModeRunning) {
        _spr.setTextColor(C_GREY_TXT, bg_color);
        _spr.drawString("In-use", x + (cycle_box_w / 2), status_y_val);
      } else if (isLocked) {
        _spr.setTextColor(_blink_state ? C_RED_TXT : bg_color, bg_color);
        _spr.drawString("Lock", x + (cycle_box_w / 2), status_y_val);
      } else if (!isThisCycleActive || !globalRun) {
        _spr.setTextColor(TFT_BLUE, bg_color);
        _spr.drawString("Standby", x + (cycle_box_w / 2), status_y_val);
      } else {
        if (isReady) {
          _spr.setTextColor(C_GREEN_TXT, bg_color);
          _spr.drawString("Ready", x + (cycle_box_w / 2), status_y_val);
        } else {
          int badge_w = 80, badge_h = 26, center_x = x + (cycle_box_w / 2);
          _spr.fillRect(center_x - (badge_w / 2), status_y_val, badge_w, badge_h, TFT_WHITE);
          _spr.setTextDatum(MC_DATUM);
          _spr.setTextColor(TFT_ORANGE, bg_color);
          _spr.drawString("Heating", center_x, status_y_val + (badge_h / 2));
          _spr.setTextDatum(TC_DATUM);
        }
      }
    }
    _spr.unloadFont();
  }
  _spr.fillRect(gap, sensor_y, sensor_w, sensor_h, C_PINK_BG);
  _spr.loadFont(Arial18);
  _spr.setTextColor(C_BLACK, C_PINK_BG);
  _spr.setTextDatum(ML_DATUM);
  char ir_buf[30];
  if (isnan(state.ir_temps[0])) snprintf(ir_buf, 30, "IR1 : ---%c", state.temp_unit);
  else snprintf(ir_buf, 30, "IR1 : %.0f%c", convertTemp(state.ir_temps[0], state.temp_unit), state.temp_unit);
  _spr.drawString(ir_buf, gap + 10, sensor_y + (sensor_h / 4) + 2);
  if (isnan(state.ir_temps[1])) snprintf(ir_buf, 30, "IR2 : ---%c", state.temp_unit);
  else snprintf(ir_buf, 30, "IR2 : %.0f%c", convertTemp(state.ir_temps[1], state.temp_unit), state.temp_unit);
  _spr.drawString(ir_buf, gap + 10, sensor_y + (sensor_h * 3 / 4) - 1);
  _spr.unloadFont();
  int tc_x = gap + sensor_w + gap;
  _spr.fillRect(tc_x, sensor_y, sensor_w, sensor_h, C_LIME_BG);
  _spr.loadFont(Arial18);
  _spr.setTextColor(C_BLACK, C_LIME_BG);
  char wire_buf[40];
  int center_x = tc_x + (sensor_w / 2);
  int center_y = sensor_y + (sensor_h / 2);
  if (isnan(state.tc_probe_temp)) snprintf(wire_buf, 40, "TC Temp: ---%c", state.temp_unit);
  else snprintf(wire_buf, 40, "TC Temp: %.0f%c", convertTemp(state.tc_probe_temp, state.temp_unit), state.temp_unit);
  _spr.setTextDatum(BC_DATUM);
  _spr.drawString(wire_buf, center_x, center_y - 2);
  if (isnan(state.tc_probe_peak) || state.tc_probe_peak < -100) snprintf(wire_buf, 40, "Max(5s): ---");
  else snprintf(wire_buf, 40, "Max(5s): %.0f%c", convertTemp(state.tc_probe_peak, state.temp_unit), state.temp_unit);
  _spr.setTextDatum(TC_DATUM);
  _spr.drawString(wire_buf, center_x, center_y + 2);
  _spr.unloadFont();
}

void UIManager::drawManualModeScreen(const AppState& state, const ConfigState& config) {
  drawTaskBar();
  int w = _spr.width(), h = _spr.height(), header_y = 24, header_h = 30;
  _spr.fillRect(0, header_y, w, header_h, C_BLACK);
  _spr.loadFont(Arial18);
  _spr.setTextColor(TFT_WHITE, C_BLACK);
  _spr.setTextDatum(ML_DATUM);
  _spr.drawString("Presets", 5, header_y + 15);
  _spr.setTextDatum(MR_DATUM);
  char buf[40];
  if (isnan(state.tc_temps[0]))
    snprintf(buf, 40, "Heater Temp : ---%c", state.temp_unit);
  else
    snprintf(buf, 40, "Heater Temp : %.0f%c", convertTemp(state.tc_temps[0], state.temp_unit), state.temp_unit);
  _spr.drawString(buf, w - 5, header_y + 15);
  _spr.unloadFont();

  int gap = 6, sensor_y = h - 50, sensor_h = 50 - gap, sensor_w = (w - (3 * gap)) / 2;
  int status_line_h = 24, status_line_y = sensor_y - status_line_h - gap, preset_start_y = header_y + header_h + gap, preset_area_h = status_line_y - gap - preset_start_y;
  int preset_box_h = (preset_area_h - gap) / 2, preset_box_w = (w - (3 * gap)) / 2;
  bool standbyUsingMain = state.manual_running_background && config.heater_active[0];
  bool isOtherModeRunning = state.auto_running_background || (standbyUsingMain && !state.manual_preset_running);
  bool globalRun = state.is_heating_active, isLocked = state.heater_cutoff_state[0], isReady = state.heater_ready[0];
  int activePresetIdx = (state.manual_preset_running && globalRun) ? state.manual_preset_index : -1;

  for (int i = 0; i < 4; i++) {
    int col = i % 2, row = i / 2, x = gap + (col * (preset_box_w + gap)), y = preset_start_y + (row * (preset_box_h + gap));
    bool isThisPresetActive = (state.manual_preset_index == i && state.manual_preset_running), isCursor = (_manual_selection == i), isConfirmed = (_manual_confirmed_preset == i);
    bool isEditingThis = (_current_screen == SCREEN_QUICK_EDIT_MANUAL && isCursor);
    uint16_t bg_color = isOtherModeRunning ? C_GREY_BG : (isEditingThis ? (_blink_state ? TFT_WHITE : C_GREY_BG) : ((isThisPresetActive && globalRun) ? C_ACTIVE_CYCLE_BG : (isConfirmed ? TFT_WHITE : C_GREY_BG)));

    _spr.fillRect(x, y, preset_box_w, preset_box_h, bg_color);
    if (isCursor) {
      _spr.drawRect(x, y, preset_box_w, preset_box_h, TFT_RED);
      _spr.drawRect(x + 1, y + 1, preset_box_w - 2, preset_box_h - 2, TFT_RED);
    } else {
      _spr.drawRect(x, y, preset_box_w, preset_box_h, TFT_BLACK);
    }

    int split_w = preset_box_w * 0.30;
    _spr.loadFont(Arial18);
    _spr.setTextColor(C_BLACK, bg_color);
    _spr.setTextDatum(MC_DATUM);
    snprintf(buf, 20, "P%d", i + 1);
    _spr.drawString(buf, x + (split_w / 2), y + (preset_box_h / 2));
    _spr.drawFastVLine(x + split_w, y + 5, preset_box_h - 10, C_BLACK);

    int text_start_x = x + split_w + 6;
    _spr.loadFont(Arial12);
    _spr.setTextDatum(TL_DATUM);
    if (isEditingThis) {
      _spr.setTextColor((_quick_edit_step == Q_EDIT_TARGET) ? C_BLACK : C_RED_TXT, bg_color);
      _spr.drawString((_quick_edit_step == Q_EDIT_TARGET) ? "SET" : "MAX", text_start_x, y + 6);
    } else {
      _spr.setTextColor(C_BLACK, bg_color);
      _spr.drawString("SET", text_start_x, y + 6);
    }

    _spr.loadFont(Arial18);
    _spr.setTextDatum(TL_DATUM);
    if (isEditingThis) {
      uint16_t val_color = (_quick_edit_step == Q_EDIT_TARGET) ? C_RED_TXT : C_BLACK;
      if (_quick_edit_step == Q_EDIT_MAX) val_color = C_RED_TXT;
      _spr.setTextColor(val_color, bg_color);
      snprintf(buf, 30, "%.0f%c", convertTemp((_quick_edit_step == Q_EDIT_TARGET) ? config.manual_target_temps[i] : config.manual_max_temps[i], state.temp_unit), state.temp_unit);
    } else {
      _spr.setTextColor(C_BLACK, bg_color);
      snprintf(buf, 30, "%.0f%c", convertTemp(config.manual_target_temps[i], state.temp_unit), state.temp_unit);
    }
    _spr.drawString(buf, text_start_x, y + 22);
    _spr.unloadFont();
  }

  _spr.setTextDatum(MC_DATUM);
  _spr.loadFont(Arial18);
  const char* status_text = "---";
  uint16_t status_color = C_GREY_TXT;
  if (isOtherModeRunning) {
    status_text = "In-use";
    status_color = C_GREY_TXT;
  } else if (activePresetIdx >= 0) {
    if (isLocked) {
      status_text = "Lock";
      status_color = _blink_state ? C_RED_TXT : C_BLACK;
    } else if (isReady) {
      status_text = "Ready";
      status_color = C_GREEN_TXT;
    } else {
      status_text = "Heating";
      status_color = TFT_ORANGE;
    }
  } else if (_manual_confirmed_preset >= 0) {
    status_text = "Standby";
    status_color = TFT_BLUE;
  }
  _spr.setTextColor(TFT_WHITE, C_BLACK);
  snprintf(buf, 40, "Status : ");
  _spr.setTextDatum(MR_DATUM);
  _spr.drawString(buf, w / 2, status_line_y + status_line_h / 2);
  _spr.setTextColor(status_color, C_BLACK);
  _spr.setTextDatum(ML_DATUM);
  _spr.drawString(status_text, w / 2, status_line_y + status_line_h / 2);
  _spr.unloadFont();

  _spr.fillRect(gap, sensor_y, sensor_w, sensor_h, C_PINK_BG);
  _spr.loadFont(Arial18);
  _spr.setTextColor(C_BLACK, C_PINK_BG);
  _spr.setTextDatum(ML_DATUM);
  if (isnan(state.ir_temps[0])) snprintf(buf, 30, "IR1: ---%c", state.temp_unit);
  else snprintf(buf, 30, "IR1: %.0f%c", convertTemp(state.ir_temps[0], state.temp_unit), state.temp_unit);
  _spr.drawString(buf, gap + 5, sensor_y + 12);
  if (isnan(state.ir_temps[1])) snprintf(buf, 30, "IR2: ---%c", state.temp_unit);
  else snprintf(buf, 30, "IR2: %.0f%c", convertTemp(state.ir_temps[1], state.temp_unit), state.temp_unit);
  _spr.drawString(buf, gap + 5, sensor_y + sensor_h - 12);
  _spr.unloadFont();
  int tc_x = gap + sensor_w + gap;
  _spr.fillRect(tc_x, sensor_y, sensor_w, sensor_h, C_LIME_BG);
  _spr.loadFont(Arial18);
  _spr.setTextColor(C_BLACK, C_LIME_BG);
  char wire_buf[40];
  int center_x = tc_x + (sensor_w / 2);
  int center_y = sensor_y + (sensor_h / 2);
  if (isnan(state.tc_probe_temp)) snprintf(wire_buf, 40, "TC Wire: ---%c", state.temp_unit);
  else snprintf(wire_buf, 40, "TC Wire: %.0f%c", convertTemp(state.tc_probe_temp, state.temp_unit), state.temp_unit);
  _spr.setTextDatum(BC_DATUM);
  _spr.drawString(wire_buf, center_x, center_y - 2);
  if (isnan(state.tc_probe_peak) || state.tc_probe_peak < -100) snprintf(wire_buf, 40, "Max(5s): ---");
  else snprintf(wire_buf, 40, "Max(5s): %.0f%c", convertTemp(state.tc_probe_peak, state.temp_unit), state.temp_unit);
  _spr.setTextDatum(TC_DATUM);
  _spr.drawString(wire_buf, center_x, center_y + 2);
  _spr.unloadFont();
}

void UIManager::drawSettingsBrightness(const AppState& state) {
  drawHeader("Screen Brightness");
  _spr.setTextColor(TFT_WHITE, TFT_BLACK);
  _spr.setTextDatum(MC_DATUM);
  _spr.loadFont(Arial18);

  int bar_w = 200, bar_h = 20, x = (_spr.width() - bar_w) / 2, y = 90;
  _spr.drawRect(x, y, bar_w, bar_h, TFT_WHITE);
  int fill_w = (int)((_temp_edit_value / 100.0f) * bar_w);
  if (fill_w > 0) _spr.fillRect(x + 1, y + 1, fill_w - 2, bar_h - 2, TFT_GREEN);

  char buf[30];
  snprintf(buf, 30, "Value: %.0f%%", _temp_edit_value);
  _spr.drawString(buf, _spr.width() / 2, y + 40);

  _spr.unloadFont();
  _spr.setTextColor(TFT_YELLOW, TFT_BLACK);
  _spr.setTextDatum(BC_DATUM);
  _spr.setTextSize(1);
  _spr.drawString("Rot: Adjust | Press: Save", _spr.width() / 2, _spr.height() - 10);
}
