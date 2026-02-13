#include "ui_manager.h"
#include "Arial30.h"
#include "Arial24.h"
#include "Arial20.h"
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

// UI Selection Colors (Modern Theme - legacy, kept for WiFi char entry)
#define C_SELECT_BG 0x18E3  // Dark Blue-ish Grey
#define C_SELECT_TXT TFT_CYAN
#define C_NORMAL_TXT TFT_WHITE
#define C_NORMAL_BG TFT_BLACK

// Settings Theme (matches Auto/Manual/Standby light theme)
#define C_SET_BG       0xE7FB  // #f5faf0 - Light green-white background
#define C_SET_CARD_BG   0x9E51  // #d9ead1 - Light green menu item bg (unselected)
#define C_SET_CARD_BRD  0xDF77  // same as card bg (subtle)
#define C_SET_SEL_BG    0x3289  // #345127 - Dark green (selected item bg)
#define C_SET_SEL_TXT   0xFFFF  // White text on selected
#define C_SET_NORM_TXT  0x2104  // Dark text on normal items
#define C_SET_TITLE     0x0000  // #000000 Black title
#define C_SET_FOOTER    0x4B29  // Muted green footer text
#define C_SET_SCROLLBAR 0xDF77  // #d9ead1 scrollbar track
#define C_SET_SCROLL_TH 0x3289  // #345127 scrollbar thumb (dark green)
#define C_SET_CHEVRON   0x7BCF  // Grey chevron color
#define C_SET_VALUE_TXT 0x7BCF  // Grey value/info text
#define C_SET_ACCENT    0x3289  // #345127 accent (active indicators)

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

// Unified settings menu labels (all pages combined)
const int SETTINGS_TOTAL_ITEMS = MENU_PAGE1_ITEM_COUNT + MENU_PAGE2_ITEM_COUNT + MENU_PAGE3_ITEM_COUNT;
const char* settings_all_labels[] = {
  "Max Temp Lock", "Heater Calibration", "IR Emissivity", "TC Probe Cal",
  "Turn off When Idle", "Startup Mode", "Sound", "Temp Unit", "Brightness", "About",
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

void drawPWMBar(TFT_eSprite* spr, int x, int y, int w, int h, float percent) {
  int num_bars = 18; // เพิ่มจาก 15 เป็น 18 ขีด
  int bar_gap = 2;
  // คำนวณความกว้างของแต่ละขีดใหม่ตามจำนวนที่เพิ่มขึ้น
  int bar_w = (w - ((num_bars - 1) * bar_gap)) / num_bars; 
  int active_bars = (int)((percent / 100.0f) * num_bars);

  for (int i = 0; i < num_bars; i++) {
    int bx = x + (i * (bar_w + bar_gap));
    uint16_t color = TFT_LIGHTGREY; 
    
    if (i < active_bars) {
      if (i < num_bars * 0.4) color = 0x07E0;      // Green
      else if (i < num_bars * 0.7) color = 0xFFE0; // Yellow
      else color = 0xF800;                         // Red/Orange
    }
    spr->fillRect(bx, y, bar_w, h, color);
  }
}

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
  _current_screen = SCREEN_BOOT;
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
  _boot_start_time = 0;
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
  _last_cursor_move_time = 0;
  _show_cursor = false;
}

void UIManager::begin() {
  _spr.createSprite(_tft->width(), _tft->height());
  _spr.setSwapBytes(true);
  _last_activity_time = millis();
  _boot_start_time = millis();
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
  if (_current_screen == SCREEN_SLEEP || _current_screen == SCREEN_BOOT) return false;
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
  if (millis() - _last_cursor_move_time > 5000) {
    _show_cursor = false;
  }

  if (_current_screen == SCREEN_SETTINGS_WIFI_SSID || 
      _current_screen == SCREEN_SETTINGS_WIFI_PASSWORD) {
    _show_cursor = true; 
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

  if (!_show_warning) {
    if (state.is_warning) {
      _show_warning = true;
    }
  }

  _spr.fillSprite(C_BLACK);

  switch (_current_screen) {
    case SCREEN_BOOT: 
      drawBootScreen();
      if (millis() - _boot_start_time > 3000) {
        _current_screen = SCREEN_STANDBY; 
        resetInactivityTimer();           
      }
      break;
    case SCREEN_SLEEP:
      _spr.fillSprite(C_SET_BG);
      _spr.setTextColor(C_SET_VALUE_TXT, C_SET_BG);
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
  // Light background below taskbar (matches auto/manual/standby)
  int top_y = 24;
  _spr.fillRect(0, top_y, _spr.width(), _spr.height() - top_y, C_SET_BG);
  
  // Bold dark green title, left-aligned like MODE headers
  _spr.loadFont(Arial18);
  _spr.setTextColor(C_SET_TITLE, C_SET_BG);
  _spr.setTextDatum(TL_DATUM);
  _spr.drawString(title, 10, top_y + 8);
  _spr.unloadFont();
}

// --- Themed Settings Footer ---
void drawSettingsFooter(TFT_eSprite* spr, const char* left, const char* center, const char* right) {
  int y = spr->height() - 16;
  spr->fillRect(0, y, spr->width(), 16, C_SET_BG);
  spr->setTextSize(1);
  spr->setTextColor(C_SET_FOOTER, C_SET_BG);
  if (left) {   spr->setTextDatum(BL_DATUM); spr->drawString(left, 10, spr->height() - 3); }
  if (center) { spr->setTextDatum(BC_DATUM); spr->drawString(center, spr->width()/2, spr->height() - 3); }
  if (right) {  spr->setTextDatum(BR_DATUM); spr->drawString(right, spr->width() - 10, spr->height() - 3); }
}

// Separator line with optional pipe chars
void drawSettingsFooter3(TFT_eSprite* spr, const char* l, const char* c, const char* r) {
  int y = spr->height() - 16;
  spr->fillRect(0, y, spr->width(), 16, C_SET_BG);
  spr->setTextSize(1);
  spr->setTextColor(C_SET_FOOTER, C_SET_BG);
  spr->setTextDatum(BL_DATUM); spr->drawString(l, 10, spr->height() - 3);
  // Draw separator pipes
  int pipe_x1 = 10 + spr->textWidth(l) + 8;
  spr->drawString("|", pipe_x1, spr->height() - 3);
  spr->setTextDatum(BC_DATUM); spr->drawString(c, spr->width()/2, spr->height() - 3);
  int cw = spr->textWidth(c);
  int pipe_x2 = spr->width()/2 + cw/2 + 8;
  spr->drawString("|", pipe_x2, spr->height() - 3);
  spr->setTextDatum(BR_DATUM); spr->drawString(r, spr->width() - 10, spr->height() - 3);
}

// --- Themed Scrollbar ---
void drawSettingsScrollbar(TFT_eSprite* spr, int current_item, int total_items, int y_start, int y_end) {
  if (total_items <= 1) return;
  int x = spr->width() - 8;
  int track_h = y_end - y_start;
  int thumb_h = max(12, track_h / total_items);
  int travel = track_h - thumb_h;
  int thumb_y = y_start + (travel * current_item) / (total_items - 1);
  
  spr->fillRoundRect(x, y_start, 4, track_h, 2, C_SET_SCROLLBAR);
  spr->fillRoundRect(x, thumb_y, 4, thumb_h, 2, C_SET_SCROLL_TH);
}

// --- Screen Implementations ---

void UIManager::drawSettingsPage1(const AppState& state, const ConfigState& config) {
  drawHeader("SETTING");
  const int ITEM_HEIGHT = 36, ITEM_SPACING = 40, X_MARGIN = 8, Y_START = 58;
  int w = _spr.width() - (2 * X_MARGIN) - 12; // leave room for scrollbar
  int footer_y = _spr.height() - 18;
  int max_visible = (footer_y - Y_START) / ITEM_SPACING;

  // _selected_menu_item is now global index (0..SETTINGS_TOTAL_ITEMS-1)
  int scroll_offset = 0;
  if (_selected_menu_item >= max_visible) {
    scroll_offset = _selected_menu_item - max_visible + 1;
  }

  for (int i = 0; i < SETTINGS_TOTAL_ITEMS; ++i) {
    int vi = i - scroll_offset;
    if (vi < 0 || vi >= max_visible) continue;
    int y = Y_START + vi * ITEM_SPACING;

    bool sel = (i == _selected_menu_item);
    uint16_t bg = sel ? C_SET_SEL_BG : C_SET_CARD_BG;
    uint16_t txt = sel ? C_SET_SEL_TXT : C_SET_NORM_TXT;

    _spr.fillRoundRect(X_MARGIN, y, w, ITEM_HEIGHT, 6, bg);
    if (!sel) _spr.drawRoundRect(X_MARGIN, y, w, ITEM_HEIGHT, 6, C_SET_CARD_BRD);

    _spr.setTextColor(txt, bg);
    _spr.setTextDatum(ML_DATUM);
    _spr.loadFont(Arial18);
    _spr.drawString(settings_all_labels[i], X_MARGIN + 14, y + ITEM_HEIGHT / 2);
    // Chevron
    _spr.setTextDatum(MR_DATUM);
    _spr.setTextColor(sel ? C_SET_SEL_TXT : C_SET_CHEVRON, bg);
    _spr.drawString(">", X_MARGIN + w - 10, y + ITEM_HEIGHT / 2);
    _spr.unloadFont();
  }

  drawSettingsScrollbar(&_spr, _selected_menu_item, SETTINGS_TOTAL_ITEMS, Y_START, Y_START + max_visible * ITEM_SPACING);
  drawSettingsFooter3(&_spr, "Rot: Select", "Press: OK", "Hold: Back");
}

void UIManager::drawSettingsPage2(const AppState& state, const ConfigState& config) {
  // Unified: redirect to page 1 drawing (all items in one list)
  drawSettingsPage1(state, config);
}

// [ADDED] Page 3 Drawing - unified redirect
void UIManager::drawSettingsPage3(const AppState& state, const ConfigState& config) {
  drawSettingsPage1(state, config);
}

// [ADDED] WiFi Menu Drawing
void UIManager::drawSettingsWiFiMenu(const AppState& state, const ConfigState& config) {
  drawHeader("WiFi Settings");
  const int ITEM_HEIGHT = 34, ITEM_SPACING = 38, X_MARGIN = 8, Y_START = 56;
  int w = _spr.width() - (2 * X_MARGIN) - 12;
  int footer_y = _spr.height() - 18;
  int max_visible = (footer_y - Y_START) / ITEM_SPACING;

  int scroll_offset = 0;
  if (_selected_wifi_menu_item >= max_visible) {
    scroll_offset = _selected_wifi_menu_item - max_visible + 1;
  }

  for (int i = 0; i < WIFI_MENU_ITEM_COUNT; ++i) {
    int vi = i - scroll_offset;
    if (vi < 0 || vi >= max_visible) continue;
    int y = Y_START + vi * ITEM_SPACING;

    bool sel = (i == _selected_wifi_menu_item);
    uint16_t bg = sel ? C_SET_SEL_BG : C_SET_CARD_BG;
    uint16_t txt = sel ? C_SET_SEL_TXT : C_SET_NORM_TXT;

    _spr.fillRoundRect(X_MARGIN, y, w, ITEM_HEIGHT, 6, bg);
    if (!sel) _spr.drawRoundRect(X_MARGIN, y, w, ITEM_HEIGHT, 6, C_SET_CARD_BRD);

    _spr.setTextColor(txt, bg);
    _spr.setTextDatum(ML_DATUM);
    _spr.loadFont(Arial18);

    char buf[50];
    switch (i) {
      case WIFI_MENU_SSID:
        snprintf(buf, 50, "SSID: %.12s%s", (strlen(config.wifi_config.ssid) > 0) ? config.wifi_config.ssid : "(none)", (strlen(config.wifi_config.ssid) > 12) ? ".." : "");
        _spr.drawString(buf, X_MARGIN + 10, y + ITEM_HEIGHT / 2);
        break;
      case WIFI_MENU_PASSWORD:
        snprintf(buf, 50, "Pass: %.16s%s", 
          (strlen(config.wifi_config.password) > 0) ? config.wifi_config.password : "(none)",
          (strlen(config.wifi_config.password) > 16) ? ".." : "");
        _spr.drawString(buf, X_MARGIN + 10, y + ITEM_HEIGHT / 2);
        break;
      default:
        _spr.drawString(wifi_menu_labels[i], X_MARGIN + 14, y + ITEM_HEIGHT / 2);
        break;
    }

    // Chevron for sub-pages
    if (i != WIFI_MENU_CONNECT && i != WIFI_MENU_BACK) {
      _spr.setTextDatum(MR_DATUM);
      _spr.setTextColor(sel ? C_SET_SEL_TXT : C_SET_CHEVRON, bg);
      _spr.drawString(">", X_MARGIN + w - 10, y + ITEM_HEIGHT / 2);
    }
    _spr.unloadFont();
  }

  drawSettingsScrollbar(&_spr, _selected_wifi_menu_item, WIFI_MENU_ITEM_COUNT, Y_START, Y_START + max_visible * ITEM_SPACING);

  // Status indicator at bottom
  WiFiConnectionStatus wifi_status = getWiFiStatus();
  uint16_t st_col = (wifi_status == WIFI_STATUS_CONNECTED) ? 0x07E0 : (wifi_status == WIFI_STATUS_CONNECTING) ? TFT_CYAN : TFT_RED;
  const char* st_str = (wifi_status == WIFI_STATUS_CONNECTED) ? "Connected" : (wifi_status == WIFI_STATUS_CONNECTING) ? "Connecting..." : "Disconnected";
  
  _spr.setTextSize(1);
  _spr.setTextColor(st_col, C_SET_BG);
  _spr.setTextDatum(BC_DATUM);
  _spr.drawString(st_str, _spr.width() / 2, _spr.height() - 3);
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
  _spr.fillRect(START_X - 5, TEXT_Y - 5, w - 40 + 10, CHAR_HEIGHT + 10, C_SET_CARD_BG);
  _spr.drawRect(START_X - 5, TEXT_Y - 5, w - 40 + 10, CHAR_HEIGHT + 10, C_SET_CARD_BRD);

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
        _spr.setTextColor(TFT_WHITE, C_SET_SEL_BG);
      }
    } else {
      _spr.setTextColor(C_SET_NORM_TXT, C_SET_CARD_BG);
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
    _spr.fillRect(10, selector_y, w - 20, 50, C_SET_SEL_BG);
    _spr.drawRect(10, selector_y, w - 20, 50, C_SET_ACCENT);

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
        _spr.fillRect(x - 10, selector_y + 5, 20, 40, TFT_YELLOW);
        if (is_del) {
          _spr.setTextColor(TFT_RED, TFT_YELLOW);
          _spr.drawString("<", x, selector_y + 25);  // ใช้เครื่องหมาย < แทน Backspace
        } else {
          char str[2] = { charset[idx], '\0' };
          _spr.setTextColor(TFT_BLACK, TFT_YELLOW);
          _spr.drawString(str, x, selector_y + 25);
        }
      } else {
        // Nearby Items
        uint16_t fade = (abs(i) == 1) ? TFT_WHITE : TFT_LIGHTGREY;
        _spr.setTextColor(fade, C_SET_SEL_BG);
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
    _spr.setTextColor(C_SET_FOOTER, C_SET_BG);
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
  _spr.setTextSize(1);
  if (_char_entry_editing) {
    drawSettingsFooter(&_spr, NULL, "Rot:Char | Press:Set | Hold:Cancel", NULL);
  } else {
    drawSettingsFooter(&_spr, NULL, "Rot:Move | Press:Edit | Hold:Done", NULL);
  }

  char len_buf[30];
  snprintf(len_buf, 30, "Length: %d/%d", (int)strlen(_char_entry_buffer), _char_entry_max_len);
  _spr.setTextDatum(BL_DATUM);
  _spr.setTextColor(C_SET_VALUE_TXT, C_SET_BG);
  _spr.drawString(len_buf, 5, h - 2);
}

void UIManager::drawSettingsWiFiSSID(const AppState& state, const ConfigState& config) {
  drawCharEntryScreen("Edit SSID", false);
}

void UIManager::drawSettingsWiFiPassword(const AppState& state, const ConfigState& config) {
  drawCharEntryScreen("Edit Password", false);
}

void UIManager::drawSettingsWiFiStatus(const AppState& state, const ConfigState& config) {
  drawHeader("WiFi Status");
  _spr.loadFont(Arial18);
  int y = 60, line_h = 28, x_label = 14, x_value = 110;

  WiFiConnectionStatus wifi_status = getWiFiStatus();

  // Status row
  _spr.setTextDatum(ML_DATUM);
  _spr.setTextColor(C_SET_NORM_TXT, C_SET_BG);
  _spr.drawString("Status:", x_label, y);
  uint16_t status_color = (wifi_status == WIFI_STATUS_CONNECTED) ? 0x07E0 : (wifi_status == WIFI_STATUS_CONNECTING) ? TFT_CYAN : TFT_RED;
  _spr.setTextColor(status_color, C_SET_BG);
  _spr.drawString((wifi_status == WIFI_STATUS_CONNECTED) ? "Connected" : (wifi_status == WIFI_STATUS_CONNECTING) ? "Connecting..." : "Disconnected", x_value, y);

  y += line_h;
  _spr.setTextColor(C_SET_NORM_TXT, C_SET_BG);
  _spr.drawString("SSID:", x_label, y);
  _spr.setTextColor(C_SET_ACCENT, C_SET_BG);
  _spr.drawString((strlen(config.wifi_config.ssid) > 0) ? config.wifi_config.ssid : "(default)", x_value, y);

  y += line_h;
  _spr.setTextColor(C_SET_NORM_TXT, C_SET_BG);
  _spr.drawString("Signal:", x_label, y);
  int rssi = getWiFiSignalStrength();
  char rssi_buf[20];
  if (wifi_status == WIFI_STATUS_CONNECTED) {
    snprintf(rssi_buf, 20, "%d dBm", rssi);
    _spr.setTextColor((rssi > -50) ? 0x07E0 : (rssi > -70) ? TFT_YELLOW : TFT_ORANGE, C_SET_BG);
  } else {
    snprintf(rssi_buf, 20, "N/A");
    _spr.setTextColor(C_SET_VALUE_TXT, C_SET_BG);
  }
  _spr.drawString(rssi_buf, x_value, y);

  y += line_h;
  _spr.setTextColor(C_SET_NORM_TXT, C_SET_BG);
  _spr.drawString("IP:", x_label, y);
  _spr.setTextColor(C_SET_ACCENT, C_SET_BG);
  if (getWiFiStatus() == WIFI_STATUS_CONNECTED) _spr.drawString(state.ip_address, x_value, y);
  else _spr.drawString("N/A", x_value, y);

  y += line_h;
  _spr.setTextColor(C_SET_NORM_TXT, C_SET_BG);
  _spr.drawString("Config:", x_label, y);
  _spr.setTextColor(config.wifi_config.use_custom ? 0x07E0 : C_SET_VALUE_TXT, C_SET_BG);
  _spr.drawString(config.wifi_config.use_custom ? "Custom" : "Default", x_value, y);

  _spr.unloadFont();
  drawSettingsFooter(&_spr, NULL, "Press or Hold: Back", NULL);
}

void UIManager::drawSettingsStartup(const AppState& state, const ConfigState& config) {
  drawHeader("Startup Mode");
  const int ITEM_HEIGHT = 36, ITEM_SPACING = 42, X_MARGIN = 14, Y_START = 70;
  int w = _spr.width() - (2 * X_MARGIN);
  for (int i = 0; i < STARTUP_MODE_COUNT; ++i) {
    int y = Y_START + i * ITEM_SPACING;
    bool sel = (i == _selected_menu_item);
    uint16_t bg = sel ? C_SET_SEL_BG : C_SET_CARD_BG;
    uint16_t txt = sel ? C_SET_SEL_TXT : C_SET_NORM_TXT;
    _spr.fillRoundRect(X_MARGIN, y, w, ITEM_HEIGHT, 6, bg);
    if (!sel) _spr.drawRoundRect(X_MARGIN, y, w, ITEM_HEIGHT, 6, C_SET_CARD_BRD);
    _spr.setTextColor(txt, bg);
    _spr.setTextDatum(ML_DATUM);
    _spr.loadFont(Arial18);
    _spr.drawString(startup_mode_labels[i], X_MARGIN + 14, y + ITEM_HEIGHT / 2);
    _spr.unloadFont();
  }
  drawSettingsFooter(&_spr, NULL, "Rot: Select | Press: Save", NULL);
}

void UIManager::drawSettingsIdleOff(const AppState& state, const ConfigState& config) {
  drawHeader("Turn off When Idle");
  const int ITEM_HEIGHT = 34, ITEM_SPACING = 38, X_MARGIN = 10, Y_START = 56;
  int w = _spr.width() - (2 * X_MARGIN);
  int footer_y = _spr.height() - 18;
  int max_visible = (footer_y - Y_START) / ITEM_SPACING;

  int scroll_offset = 0;
  if (_selected_menu_item >= max_visible) {
    scroll_offset = _selected_menu_item - max_visible + 1;
  }

  for (int i = 0; i < IDLE_OFF_ITEM_COUNT; ++i) {
    int vi = i - scroll_offset;
    if (vi < 0 || vi >= max_visible) continue;
    int y = Y_START + vi * ITEM_SPACING;

    bool sel = (i == _selected_menu_item);
    uint16_t bg = sel ? C_SET_SEL_BG : C_SET_CARD_BG;
    uint16_t txt = sel ? C_SET_SEL_TXT : C_SET_NORM_TXT;
    _spr.fillRoundRect(X_MARGIN, y, w, ITEM_HEIGHT, 6, bg);
    if (!sel) _spr.drawRoundRect(X_MARGIN, y, w, ITEM_HEIGHT, 6, C_SET_CARD_BRD);
    _spr.setTextColor(txt, bg);
    _spr.setTextDatum(ML_DATUM);
    _spr.loadFont(Arial18);
    _spr.drawString(idle_off_labels[i], X_MARGIN + 14, y + ITEM_HEIGHT / 2);
    _spr.unloadFont();
  }

  if (IDLE_OFF_ITEM_COUNT > max_visible) {
    drawSettingsScrollbar(&_spr, _selected_menu_item, IDLE_OFF_ITEM_COUNT, Y_START, Y_START + max_visible * ITEM_SPACING);
  }
}

void UIManager::drawSettingsTempUnit(const AppState& state, const ConfigState& config) {
  drawHeader("Temperature Unit");
  const int ITEM_HEIGHT = 40, ITEM_SPACING = 46, X_MARGIN = 14, Y_START = 75;
  int w = _spr.width() - (2 * X_MARGIN);
  for (int i = 0; i < 2; ++i) {
    int y = Y_START + i * ITEM_SPACING;
    bool sel = (i == _selected_menu_item);
    uint16_t bg = sel ? C_SET_SEL_BG : C_SET_CARD_BG;
    uint16_t txt = sel ? C_SET_SEL_TXT : C_SET_NORM_TXT;
    _spr.fillRoundRect(X_MARGIN, y, w, ITEM_HEIGHT, 6, bg);
    if (!sel) _spr.drawRoundRect(X_MARGIN, y, w, ITEM_HEIGHT, 6, C_SET_CARD_BRD);
    _spr.setTextColor(txt, bg);
    _spr.setTextDatum(ML_DATUM);
    _spr.loadFont(Arial18);
    _spr.drawString(temp_unit_labels[i], X_MARGIN + 14, y + ITEM_HEIGHT / 2);
    _spr.unloadFont();
  }
}

void UIManager::drawSettingsSound(const AppState& state, const ConfigState& config) {
  drawHeader("Sound Volume");
  _spr.setTextDatum(MC_DATUM);
  _spr.loadFont(Arial18);

  int bar_w = 200, bar_h = 22, x = (_spr.width() - bar_w) / 2, y = 90;
  // Track
  _spr.fillRoundRect(x, y, bar_w, bar_h, 4, C_SET_CARD_BG);
  _spr.drawRoundRect(x, y, bar_w, bar_h, 4, C_SET_CARD_BRD);
  // Fill
  int fill_w = (int)((_temp_edit_value / 100.0f) * (bar_w - 4));
  if (fill_w > 0) _spr.fillRoundRect(x + 2, y + 2, fill_w, bar_h - 4, 3, C_SET_ACCENT);

  char buf[30];
  if (_temp_edit_value == 0) snprintf(buf, 30, "Mute (Off)");
  else snprintf(buf, 30, "Volume: %.0f%%", _temp_edit_value);

  _spr.setTextColor(C_SET_NORM_TXT, C_SET_BG);
  _spr.drawString(buf, _spr.width() / 2, y + 42);

  _spr.unloadFont();
  drawSettingsFooter(&_spr, NULL, "Rot: Adjust | Press: Save", NULL);
}

void UIManager::drawSettingsEmissivity(const AppState& state, const ConfigState& config) {
  drawHeader("IR Emissivity");
  const int ITEM_HEIGHT = 40, ITEM_SPACING = 46, X_MARGIN = 14, Y_START = 70;
  int w = _spr.width() - (2 * X_MARGIN);
  _spr.loadFont(Arial18);

  for (int i = 0; i < 2; i++) {
    int y = Y_START + (i * ITEM_SPACING);
    bool sel = (i == _selected_menu_item);
    uint16_t bg = sel ? C_SET_SEL_BG : C_SET_CARD_BG;
    uint16_t txt = sel ? C_SET_SEL_TXT : C_SET_NORM_TXT;
    if (i == 1 && isnan(state.ir_temps[1]) && !sel) txt = C_SET_VALUE_TXT;

    _spr.fillRoundRect(X_MARGIN, y, w, ITEM_HEIGHT, 6, bg);
    if (!sel) _spr.drawRoundRect(X_MARGIN, y, w, ITEM_HEIGHT, 6, C_SET_CARD_BRD);

    _spr.setTextColor(txt, bg);
    _spr.setTextDatum(MC_DATUM);
    char buf[40];
    if (i == 0) {
      if (!isnan(state.ir_temps[0])) {
        float disp_val = convertTemp(state.ir_temps[0], state.temp_unit);
        snprintf(buf, 40, "IR1 (%.1f%c): %.2f", disp_val, state.temp_unit, config.ir_emissivity[0]);
      } else {
        snprintf(buf, 40, "IR1: Not Connected");
      }
    } else {
      float disp_val = convertTemp(state.ir_temps[1], state.temp_unit);
      if (!isnan(state.ir_temps[1])) snprintf(buf, 40, "IR2 (%.1f%c): %.2f", disp_val, state.temp_unit, config.ir_emissivity[1]);
      else snprintf(buf, 40, "IR2: Not Connected");
    }
    _spr.drawString(buf, _spr.width() / 2, y + ITEM_HEIGHT / 2);
  }
  _spr.unloadFont();
  drawSettingsFooter(&_spr, NULL, "Rot: Adjust | Press: Switch | Hold: Back", NULL);
}

void UIManager::drawSettingsTCProbeCal(const AppState& state, const ConfigState& config) {
  _temp_edit_value = state.tc_probe_temp;
  drawHeader("TC Probe Cal");
  _spr.setTextDatum(MC_DATUM);
  _spr.loadFont(Arial18);
  char buf[32];
  int info_y = 70;

  if (isnan(state.tc_probe_temp)) {
    _spr.setTextColor(C_SET_VALUE_TXT, C_SET_BG);
    _spr.drawString("Raw: ---", _spr.width() / 2, info_y);
    _spr.drawString("Offset: ---", _spr.width() / 2, info_y + 28);
    _spr.setTextColor(C_SET_VALUE_TXT, C_SET_BG);
    _spr.drawString("Val: ---", _spr.width() / 2, info_y + 56);
  } else {
    float raw_c = state.tc_probe_temp - config.tc_probe_offset;
    _spr.setTextColor(C_SET_NORM_TXT, C_SET_BG);
    snprintf(buf, 32, "Raw: %.0f %c", convertTemp(raw_c, state.temp_unit), state.temp_unit);
    _spr.drawString(buf, _spr.width() / 2, info_y);
    snprintf(buf, 32, "Offset: %.0f %c", convertDelta(config.tc_probe_offset, state.temp_unit), state.temp_unit);
    _spr.drawString(buf, _spr.width() / 2, info_y + 28);
    _spr.setTextColor(C_SET_ACCENT, C_SET_BG);
    snprintf(buf, 32, "Val: %.0f %c", convertTemp(state.tc_probe_temp, state.temp_unit), state.temp_unit);
    _spr.drawString(buf, _spr.width() / 2, info_y + 56);
  }

  int start_y = 140;
  const char* options[] = { "Tare (Set 0)", "Reset (0.0)" };
  int w = _spr.width() - 40;
  int h = 34;
  for (int i = 0; i < 2; i++) {
    int y = start_y + (i * 40);
    bool sel = (i == _selected_menu_item);
    uint16_t bg = sel ? C_SET_SEL_BG : C_SET_CARD_BG;
    uint16_t txt = sel ? C_SET_SEL_TXT : C_SET_NORM_TXT;
    _spr.fillRoundRect(20, y, w, h, 6, bg);
    if (!sel) _spr.drawRoundRect(20, y, w, h, 6, C_SET_CARD_BRD);
    _spr.setTextColor(txt, bg);
    _spr.drawString(options[i], _spr.width() / 2, y + h / 2);
  }
  _spr.unloadFont();
  drawSettingsFooter(&_spr, NULL, "Press: Action | Hold: Back", NULL);
}

void UIManager::drawSettingsCalibrationSelect(const AppState& state, const ConfigState& config) {
  drawHeader("Heater Calibration");
  char item_labels[3][30];
  int start_y = 65;
  int w = _spr.width() - 24;
  int h = 36;
  _spr.loadFont(Arial18);

  for (int i = 0; i < 3; ++i) {
    int y = start_y + i * 42;
    bool sel = (i == _selected_menu_item);
    uint16_t bg = sel ? C_SET_SEL_BG : C_SET_CARD_BG;
    uint16_t txt = C_SET_NORM_TXT;
    float disp_off = convertDelta(config.tc_offsets[i], state.temp_unit);
    snprintf(item_labels[i], 30, "Heater %d Offset: %.0f%c", i + 1, disp_off, state.temp_unit);

    if (sel) {
      if (_is_editing_calibration) {
        txt = TFT_RED;
      } else {
        txt = C_SET_SEL_TXT;
      }
    } 
    _spr.fillRoundRect(12, y, w, h, 6, bg);
    if (!sel) _spr.drawRoundRect(12, y, w, h, 6, C_SET_CARD_BRD);
    _spr.setTextDatum(MC_DATUM);
    _spr.setTextColor(txt, bg);
    _spr.drawString(item_labels[i], _spr.width() / 2, y + h / 2);
  }
  _spr.unloadFont();
  drawSettingsFooter(&_spr, NULL, "Rot: Select/Edit | Press: Toggle", NULL);
}

void UIManager::drawSettingsHeaterTargetTemp(const AppState& state) {
  char title_buffer[20];
  snprintf(title_buffer, sizeof(title_buffer), "Heater (%d)", _selected_menu_item + 1);
  drawHeader(title_buffer);
  _spr.setTextDatum(MC_DATUM);
  _spr.loadFont(Arial18);

  // Value card
  int card_w = 220, card_h = 50;
  int cx = _spr.width() / 2, cy = _spr.height() / 2 - 5;
  _spr.fillRoundRect(cx - card_w/2, cy - card_h/2, card_w, card_h, 8, C_SET_CARD_BG);
  _spr.drawRoundRect(cx - card_w/2, cy - card_h/2, card_w, card_h, 8, C_SET_ACCENT);

  char temp_buffer[30];
  snprintf(temp_buffer, sizeof(temp_buffer), "Target: %.0f %c", convertTemp(_temp_edit_value, state.temp_unit), state.temp_unit);
  _spr.setTextColor(C_SET_NORM_TXT, C_SET_CARD_BG);
  _spr.drawString(temp_buffer, cx, cy);

  _spr.unloadFont();
  drawSettingsFooter(&_spr, NULL, "Rot: Adjust | Press: Confirm", NULL);
}

void UIManager::drawSettingsHeaterMaxTemp(const AppState& state) {
  char title_buffer[20];
  snprintf(title_buffer, sizeof(title_buffer), "Heater (%d)", _selected_menu_item + 1);
  drawHeader(title_buffer);
  _spr.setTextDatum(MC_DATUM);
  _spr.loadFont(Arial18);

  int card_w = 220, card_h = 50;
  int cx = _spr.width() / 2, cy = _spr.height() / 2 - 5;
  _spr.fillRoundRect(cx - card_w/2, cy - card_h/2, card_w, card_h, 8, C_SET_CARD_BG);
  _spr.drawRoundRect(cx - card_w/2, cy - card_h/2, card_w, card_h, 8, TFT_ORANGE);

  char temp_buffer[30];
  snprintf(temp_buffer, sizeof(temp_buffer), "Max: %.0f %c", convertTemp(_temp_edit_value, state.temp_unit), state.temp_unit);
  _spr.setTextColor(C_SET_NORM_TXT, C_SET_CARD_BG);
  _spr.drawString(temp_buffer, cx, cy);

  _spr.unloadFont();
  drawSettingsFooter(&_spr, NULL, "Rot: Adjust | Press: Confirm", NULL);
}

void UIManager::drawSettingsHeaterCalibrate(const AppState& state) {
  drawHeader("Heater Cal (Unused)");
}

void UIManager::drawSettingsMaxTempLock(const AppState& state) {
  drawHeader("Max Temp Lock");
  _spr.setTextDatum(MC_DATUM);
  _spr.loadFont(Arial18);

  int card_w = 220, card_h = 50;
  int cx = _spr.width() / 2, cy = _spr.height() / 2 - 5;
  _spr.fillRoundRect(cx - card_w/2, cy - card_h/2, card_w, card_h, 8, C_SET_CARD_BG);
  _spr.drawRoundRect(cx - card_w/2, cy - card_h/2, card_w, card_h, 8, TFT_RED);

  char temp_buffer[30];
  snprintf(temp_buffer, sizeof(temp_buffer), "Max: %.0f %c", convertTemp(_temp_edit_value, state.temp_unit), state.temp_unit);
  _spr.setTextColor(C_SET_NORM_TXT, C_SET_CARD_BG);
  _spr.drawString(temp_buffer, cx, cy);

  _spr.unloadFont();
  drawSettingsFooter(&_spr, NULL, "Rot: Adjust | Press: Confirm", NULL);
}

void UIManager::drawSettingsAbout(const AppState& state) {
  drawHeader("About Device");
  _spr.setTextDatum(MC_DATUM);
  _spr.loadFont(Arial18);
  _spr.setTextSize(1);

  // Info card
  int card_x = 12, card_y = 60, card_w = _spr.width() - 24, card_h = 120;
  _spr.fillRoundRect(card_x, card_y, card_w, card_h, 8, C_SET_CARD_BG);
  _spr.drawRoundRect(card_x, card_y, card_w, card_h, 8, C_SET_CARD_BRD);

  int line_h = 24;
  int y_start = card_y + 18;
  _spr.setTextColor(C_SET_TITLE, C_SET_CARD_BG);
  _spr.drawString("Heater Control System", _spr.width() / 2, y_start);
  _spr.setTextColor(C_SET_NORM_TXT, C_SET_CARD_BG);
  _spr.drawString("Firmware v1.0", _spr.width() / 2, y_start + line_h);
  _spr.drawString("(c) 2025 K'Tor", _spr.width() / 2, y_start + (line_h * 2));
  _spr.setTextColor(C_SET_VALUE_TXT, C_SET_CARD_BG);
  _spr.drawString("Contact: 08x-xxx-xxxx", _spr.width() / 2, y_start + (line_h * 3));

  _spr.unloadFont();
  drawSettingsFooter(&_spr, NULL, "Press or Hold: Back", NULL);
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
    case SCREEN_SETTINGS_PAGE_2:
    case SCREEN_SETTINGS_PAGE_3:
    {
      // Unified: _selected_menu_item is global index
      int idx = _selected_menu_item;
      
      // Page 1 items: 0..3
      if (idx < MENU_PAGE1_ITEM_COUNT) {
        switch (idx) {
          case MENU_PAGE1_MAX_TEMP_LOCK:
            _current_screen = SCREEN_SETTINGS_MAX_TEMP_LOCK;
            _temp_edit_value = config.max_temp_lock;
            break;
          case MENU_PAGE1_CALIBRATION:
            _current_screen = SCREEN_SETTINGS_CALIBRATION_SELECT;
            _is_editing_calibration = false;
            break;
          case MENU_PAGE1_EMISSIVITY:
            _current_screen = SCREEN_SETTINGS_EMISSIVITY;
            break;
          case MENU_PAGE1_TC_PROBE_CAL:
            _current_screen = SCREEN_SETTINGS_TC_PROBE_CAL;
            break;
        }
      }
      // Page 2 items: 4..9
      else if (idx < MENU_PAGE1_ITEM_COUNT + MENU_PAGE2_ITEM_COUNT) {
        int p2_idx = idx - MENU_PAGE1_ITEM_COUNT;
        switch (p2_idx) {
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
            _temp_edit_value = (float)config.sound_volume;
            break;
          case MENU_PAGE2_TEMP_UNIT:
            _current_screen = SCREEN_SETTINGS_TEMP_UNIT;
            _selected_menu_item = (config.temp_unit == 'C') ? 0 : 1;
            break;
          case MENU_PAGE2_BRIGHTNESS:
            _current_screen = SCREEN_SETTINGS_BRIGHTNESS;
            _temp_edit_value = (float)config.brightness;
            break;
          case MENU_PAGE2_ABOUT:
            _current_screen = SCREEN_SETTINGS_ABOUT;
            break;
        }
      }
      // Page 3 items: 10+
      else {
        int p3_idx = idx - MENU_PAGE1_ITEM_COUNT - MENU_PAGE2_ITEM_COUNT;
        switch (p3_idx) {
          case MENU_PAGE3_WIFI_SETTINGS:
            _current_screen = SCREEN_SETTINGS_WIFI_MENU;
            _selected_wifi_menu_item = 0;
            break;
        }
      }
      _menu_step_accumulator = 0.0f;
      return true;
    }

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
        case WIFI_MENU_BACK:
          _current_screen = SCREEN_SETTINGS_PAGE_1;
          _selected_menu_item = MENU_PAGE1_ITEM_COUNT + MENU_PAGE2_ITEM_COUNT + MENU_PAGE3_WIFI_SETTINGS;
          break;
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
      _current_screen = SCREEN_SETTINGS_PAGE_1;
      _selected_menu_item = MENU_PAGE1_ITEM_COUNT + MENU_PAGE2_BRIGHTNESS;
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
      _selected_menu_item = MENU_PAGE1_CALIBRATION;
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
      _selected_menu_item = MENU_PAGE1_MAX_TEMP_LOCK;
      _menu_step_accumulator = 0.0f;
      return true;
    case SCREEN_SETTINGS_IDLE_OFF:
      config.idle_off_mode = (IdleOffMode)_selected_menu_item;
      if (_save_callback) _save_callback(config);
      _current_screen = SCREEN_SETTINGS_PAGE_1;
      _selected_menu_item = MENU_PAGE1_ITEM_COUNT + MENU_PAGE2_IDLE_OFF;
      _menu_step_accumulator = 0.0f;
      return true;
    case SCREEN_SETTINGS_SOUND:
      config.sound_volume = (uint8_t)_temp_edit_value;
      if (_save_callback) _save_callback(config);
      _current_screen = SCREEN_SETTINGS_PAGE_1;
      _selected_menu_item = MENU_PAGE1_ITEM_COUNT + MENU_PAGE2_SOUND;
      _menu_step_accumulator = 0.0f;
      return true;
    case SCREEN_SETTINGS_STARTUP:
      config.startup_mode = (StartupMode)_selected_menu_item;
      if (_save_callback) _save_callback(config);
      _current_screen = SCREEN_SETTINGS_PAGE_1;
      _selected_menu_item = MENU_PAGE1_ITEM_COUNT + MENU_PAGE2_STARTUP;
      _menu_step_accumulator = 0.0f;
      return true;
    case SCREEN_SETTINGS_EMISSIVITY:
      if (_selected_menu_item == 0) _selected_menu_item = 1;
      else _selected_menu_item = 0;
      return true;
    case SCREEN_SETTINGS_TEMP_UNIT:
      config.temp_unit = (_selected_menu_item == 0) ? 'C' : 'F';
      _current_screen = SCREEN_SETTINGS_PAGE_1;
      _selected_menu_item = MENU_PAGE1_ITEM_COUNT + MENU_PAGE2_TEMP_UNIT;
      if (_save_callback) _save_callback(config);
      _menu_step_accumulator = 0.0f;
      return true;
    case SCREEN_SETTINGS_ABOUT:
      _current_screen = SCREEN_SETTINGS_PAGE_1;
      _selected_menu_item = MENU_PAGE1_ITEM_COUNT + MENU_PAGE2_ABOUT;
      return true;
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
    case SCREEN_SETTINGS_MAX_TEMP_LOCK:
      _current_screen = SCREEN_SETTINGS_PAGE_1;
      _selected_menu_item = MENU_PAGE1_MAX_TEMP_LOCK;
      break;
    case SCREEN_SETTINGS_HEATER_MAX_TEMP:
      _current_screen = SCREEN_SETTINGS_HEATER_TARGET_TEMP;
      _temp_edit_value = config.target_temps[_selected_menu_item];
      break;
    case SCREEN_SETTINGS_CALIBRATION_SELECT:
      if (_is_editing_calibration) _is_editing_calibration = false;
      else { _current_screen = SCREEN_SETTINGS_PAGE_1; _selected_menu_item = MENU_PAGE1_CALIBRATION; }
      break;
    case SCREEN_SETTINGS_HEATER_CALIBRATE: _current_screen = SCREEN_SETTINGS_CALIBRATION_SELECT; break;
    case SCREEN_SETTINGS_EMISSIVITY:
      _current_screen = SCREEN_SETTINGS_PAGE_1;
      _selected_menu_item = MENU_PAGE1_EMISSIVITY;
      break;
    case SCREEN_SETTINGS_IDLE_OFF:
      _current_screen = SCREEN_SETTINGS_PAGE_1;
      _selected_menu_item = MENU_PAGE1_ITEM_COUNT + MENU_PAGE2_IDLE_OFF;
      break;
    case SCREEN_SETTINGS_STARTUP:
      _current_screen = SCREEN_SETTINGS_PAGE_1;
      _selected_menu_item = MENU_PAGE1_ITEM_COUNT + MENU_PAGE2_STARTUP;
      break;
    case SCREEN_SETTINGS_TEMP_UNIT:
      _current_screen = SCREEN_SETTINGS_PAGE_1;
      _selected_menu_item = MENU_PAGE1_ITEM_COUNT + MENU_PAGE2_TEMP_UNIT;
      break;
    case SCREEN_SETTINGS_SOUND:
      _current_screen = SCREEN_SETTINGS_PAGE_1;
      _selected_menu_item = MENU_PAGE1_ITEM_COUNT + MENU_PAGE2_SOUND;
      break;
    case SCREEN_SETTINGS_TC_PROBE_CAL:
      _current_screen = SCREEN_SETTINGS_PAGE_1;
      _selected_menu_item = MENU_PAGE1_TC_PROBE_CAL;
      break;
    case SCREEN_SETTINGS_ABOUT:
      _current_screen = SCREEN_SETTINGS_PAGE_1;
      _selected_menu_item = MENU_PAGE1_ITEM_COUNT + MENU_PAGE2_ABOUT;
      break;
    case SCREEN_SETTINGS_BRIGHTNESS:
      _current_screen = SCREEN_SETTINGS_PAGE_1;
      _selected_menu_item = MENU_PAGE1_ITEM_COUNT + MENU_PAGE2_BRIGHTNESS;
      break;
    case SCREEN_SETTINGS_WIFI_MENU:
      _current_screen = SCREEN_SETTINGS_PAGE_1;
      _selected_menu_item = MENU_PAGE1_ITEM_COUNT + MENU_PAGE2_ITEM_COUNT + MENU_PAGE3_WIFI_SETTINGS;
      break;
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
  if (change != 0) {
      _last_cursor_move_time = millis(); // บันทึกเวลาที่หมุน
      _show_cursor = true;
  }
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
    case SCREEN_SETTINGS_PAGE_2:
    case SCREEN_SETTINGS_PAGE_3:
      {
        // Unified scrolling: _selected_menu_item is global index 0..SETTINGS_TOTAL_ITEMS-1
        int new_pos = _selected_menu_item + change;
        if (new_pos < 0) new_pos = 0;
        if (new_pos >= SETTINGS_TOTAL_ITEMS) new_pos = SETTINGS_TOTAL_ITEMS - 1;
        _selected_menu_item = new_pos;
        // Keep screen on PAGE_1 (unified drawing)
        _current_screen = SCREEN_SETTINGS_PAGE_1;
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
          val += (float)change * 1.0f;
          if (val < -20.0f) val = -20.0f;
          if (val > 20.0f) val = 20.0f;
          config.tc_offsets[_selected_menu_item] = val;
          _menu_step_accumulator = 0;
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
        if (*target < 0.01f) *target = 0.01f;
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
    case SCREEN_SETTINGS_SOUND:
      {
        _temp_edit_value += (float)change * 5.0f;  // ปรับทีละ 5%
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

  // --- สีใหม่สำหรับ Manual Mode ---
  // #09904B => color565(0x09, 0x90, 0x4B) = green accent for title/cursor
  uint16_t C_MANUAL_GREEN  = color565(0x09, 0x90, 0x4B);  // #09904B
  // #f2ffee => color565(0xF2, 0xFF, 0xEE) = light green card bg (OFF state)
  uint16_t C_CARD_HIGHLIGHT = color565(0xF2, 0xFF, 0xEE);  // #f2ffee
  // #C7E7B9 => เข้มขึ้น
  uint16_t C_BG_BOTTOM     = color565(0x4A, 0x9A, 0x40);  // #4A9A40 (เข้มขึ้นอีก)

  int top_y = 24;
  int screen_h = _spr.height() - top_y;
  int screen_w = _spr.width();

  // --- Background gradient: white (top) -> #4A9A40 (bottom, darker green) ---
  for (int row = 0; row < screen_h; row++) {
    float t = (float)row / (float)(screen_h - 1);
    uint8_t r = 255 - (uint8_t)(t * (255 - 0x4A));
    uint8_t g = 255 - (uint8_t)(t * (255 - 0x9A));
    uint8_t b = 255 - (uint8_t)(t * (255 - 0x40));
    _spr.drawFastHLine(0, top_y + row, screen_w, color565(r, g, b));
  }

  // --- Title Section ---
  int title_x = 10;
  int title_y = top_y + 5;

  // สี gradient ที่ตำแหน่ง title_y สำหรับใช้เป็น bg text
  float t_title = (float)(title_y - top_y) / (float)(screen_h - 1);
  uint8_t bg_r = 255 - (uint8_t)(t_title * (255 - 0x4A));
  uint8_t bg_g = 255 - (uint8_t)(t_title * (255 - 0x9A));
  uint8_t bg_b = 255 - (uint8_t)(t_title * (255 - 0x40));
  uint16_t title_bg = color565(bg_r, bg_g, bg_b);

  _spr.loadFont(Arial12);
  _spr.setTextColor(C_MANUAL_GREEN, title_bg);
  _spr.setTextDatum(TL_DATUM);
  _spr.drawString("MODE", title_x, title_y);
  _spr.unloadFont();
  
  _spr.loadFont(Arial18);
  _spr.setTextColor(C_MANUAL_GREEN, title_bg);  // #09904B สำหรับคำว่า MANUAL
  _spr.drawString("MANUAL", title_x, title_y + 15); 
  _spr.unloadFont();

  // [จุดแก้ไข] เตรียมตัวแปรหน่วยเป็น String เพื่อใช้วัดความกว้าง
  char unit_char = (state.temp_unit == 'c' || state.temp_unit == 'C') ? 'C' : 'F';
  char unit_str[2] = {unit_char, '\0'};

  // --- Heater Cards ---
  int card_y = title_y + 40;
  int card_h = 115; 
  int gap = 6;
  int card_w = (_spr.width() - (4 * gap)) / 3;

  for (int i = 0; i < 3; i++) {
    int x = gap + (i * (card_w + gap));
    bool is_selected = (i == _standby_selection && _show_cursor);
    bool is_active = config.heater_active[i];
    bool is_editing_this = (_current_screen == SCREEN_QUICK_EDIT && is_selected);
    
    bool system_engaged = state.auto_running_background || state.manual_preset_running;
    bool is_in_use = is_active && system_engaged;

    // สีพื้นกล่อง: active = ขาว, ไม่ active = #f2ffee (เขียวอ่อน)
    uint16_t bg_color = (is_active || is_in_use) ? TFT_WHITE : C_CARD_HIGHLIGHT;
    uint16_t border_col = color565(0xD0, 0xE8, 0xD0);  // เขียวอ่อนสำหรับ default border

    // Cursor/selection border ใช้ #09904B แทน TFT_RED
    if (is_editing_this) border_col = _blink_state ? C_MANUAL_GREEN : TFT_DARKGREY;
    else if (is_selected) border_col = C_MANUAL_GREEN;        
    else if (is_active || is_in_use) border_col = C_MANUAL_GREEN;         
    
    _spr.fillRoundRect(x + 2, card_y + 2, card_w, card_h, 5, 0xBDF7); 
    _spr.fillRoundRect(x, card_y, card_w, card_h, 5, bg_color);
    _spr.drawRoundRect(x, card_y, card_w, card_h, 5, border_col);
    if (is_selected) {
      _spr.drawRoundRect(x+1, card_y+1, card_w-2, card_h-2, 5, border_col);
      _spr.drawRoundRect(x+2, card_y+2, card_w-4, card_h-4, 4, border_col);
    }

    int cx = x + (card_w / 2);
    char val_buf[32];
    
    // Header
    _spr.loadFont(Arial12);
    _spr.setTextColor(TFT_BLACK, bg_color);
    _spr.setTextDatum(TC_DATUM);
    snprintf(val_buf, 20, "HEATER %d", i + 1);
    _spr.drawString(val_buf, cx, card_y + 8);
    
    // --- [จุดแก้ไขใหญ่] Logic การจัดวางอุณหภูมิ + หน่วย แบบ Dynamic ---
    if (isnan(state.tc_temps[i])) {
      strcpy(val_buf, "---");
    } else {
      snprintf(val_buf, 20, "%.0f", convertTemp(state.tc_temps[i], state.temp_unit));
    }

    // 1. วัดความกว้างตัวเลข (Font ใหญ่)
    _spr.loadFont(Arial30);
    int val_width = _spr.textWidth(val_buf);
    _spr.loadFont(Arial30);// do not change
    int unit_width = _spr.textWidth(unit_str);

    // 2. คำนวณความกว้างรวม (ตัวเลข + ช่องว่างเล็กน้อย + หน่วย) เพื่อหาจุดเริ่มวาดให้กึ่งกลางกล่องพอดี
    int total_block_width = val_width + 4 + unit_width;
    int start_x = cx - (total_block_width / 2);

    // 3. วาดตัวเลข - ไล่สีตามอุณหภูมิอิงจาก max_temp ของ heater นั้นๆ
    //    เขียว(เย็น) -> เหลือง(กลาง) -> ส้ม -> แดง(ร้อนเกิน max)
    _spr.loadFont(Arial30);
    _spr.setTextDatum(TL_DATUM);
    
    uint16_t temp_color;
    if (!is_active && !is_in_use) {
      temp_color = TFT_DARKGREY;
    } else if (state.tc_faults[i]) {
      temp_color = TFT_RED;
    } else if (state.heater_cutoff_state[i]) {
      temp_color = TFT_RED;
    } else if (isnan(state.tc_temps[i])) {
      temp_color = TFT_DARKGREY;
    } else {
      // ไล่สีอิงจาก max temp: 0% = เขียว, 50% = เหลือง, 80% = ส้ม, 100%+ = แดง
      float max_t = config.max_temps[i];
      float cur_t = state.tc_temps[i];
      float ratio = (max_t > 0) ? (cur_t / max_t) : 0;
      if (ratio < 0) ratio = 0;
      
      uint8_t tr, tg, tb;
      if (ratio <= 0.5f) {
        // เขียว -> เหลือง (0% -> 50%)
        float f = ratio / 0.5f;
        tr = (uint8_t)(0x09 + f * (0xE0 - 0x09));  // 0x09 -> 0xE0
        tg = (uint8_t)(0x90 + f * (0xC0 - 0x90));  // 0x90 -> 0xC0
        tb = (uint8_t)(0x00);
      } else if (ratio <= 0.85f) {
        // เหลือง -> ส้ม (50% -> 85%)
        float f = (ratio - 0.5f) / 0.35f;
        tr = (uint8_t)(0xE0 + f * (0xFF - 0xE0));  // 0xE0 -> 0xFF
        tg = (uint8_t)(0xC0 - f * (0xC0 - 0x80));  // 0xC0 -> 0x80
        tb = 0x00;
      } else {
        // ส้ม -> แดง (85% -> 100%+)
        float f = (ratio - 0.85f) / 0.15f;
        if (f > 1.0f) f = 1.0f;
        tr = 0xFF;
        tg = (uint8_t)(0x80 - f * 0x80);  // 0x80 -> 0x00
        tb = 0x00;
      }
      temp_color = color565(tr, tg, tb);
    }
    
    _spr.setTextColor(temp_color, bg_color);
    _spr.drawString(val_buf, start_x, card_y + 35);

    // 4. วาดหน่วย (Font ใหญ่) ต่อท้ายตัวเลขทันที - ใช้สีเดียวกับตัวเลข
    _spr.loadFont(Arial30); // do not change
    _spr.setTextColor(temp_color, bg_color); 
    _spr.drawString(unit_str, start_x + val_width + 2, card_y + 35); 
    // --- [จบส่วนแก้ไข Dynamic Alignment] ---

    // --- SET Row ---
    _spr.loadFont(Arial12);
    String label_txt = "SET:";
    float val_to_show = config.target_temps[i];
    uint16_t edit_color = TFT_BLACK;

    if (is_editing_this) {
        edit_color = C_MANUAL_GREEN; // ใช้สีเขียวแทน TFT_RED ตอน edit
        if (_quick_edit_step == Q_EDIT_TARGET) { label_txt = "SET >"; val_to_show = config.target_temps[i]; }
        else { label_txt = "MAX >"; val_to_show = config.max_temps[i]; }
    }

    _spr.setTextColor(edit_color, bg_color);
    _spr.setTextDatum(TL_DATUM);
    _spr.drawString(label_txt, x + 8, card_y + 65);
    _spr.setTextDatum(TR_DATUM);
    snprintf(val_buf, 20, "%.0f %c", convertTemp(val_to_show, state.temp_unit), unit_char);
    _spr.drawString(val_buf, x + card_w - 8, card_y + 65);

    // --- Status Row ---
    const char* status_txt = "OFF";
    uint16_t status_col = TFT_DARKGREY;

    if (!is_active) { status_txt = "OFF"; status_col = TFT_DARKGREY; }
    else if (state.tc_faults[i]) { status_txt = "FAULT"; status_col = TFT_RED; }
    else if (state.heater_cutoff_state[i]) { status_txt = "CUTOFF"; status_col = TFT_ORANGE; }
    else if (is_in_use) { status_txt = "IN-USE"; status_col = 0x07E0; }  // เขียวสำหรับ IN-USE
    else if (state.is_heating_active) { 
        if (state.heater_ready[i]) { status_txt = "READY"; status_col = 0x07E0; }
        else { status_txt = "HEATING"; status_col = TFT_ORANGE; }
    } else { status_txt = "STANDBY"; status_col = TFT_BLUE; }

    _spr.loadFont(Arial12); 
    int status_w = 6 + 5 + _spr.textWidth(status_txt); 
    int s_start_x = cx - (status_w / 2);
    _spr.fillCircle(s_start_x + 3, card_y + 95, 3, status_col);
    _spr.setTextDatum(ML_DATUM);
    _spr.setTextColor(status_col, bg_color);
    _spr.drawString(status_txt, s_start_x + 11, card_y + 96);
    _spr.unloadFont();
  }

  // --- Bottom Sensors (White Box) ---
  int btm_y = card_y + card_h + 8;
  _spr.fillRoundRect(6, btm_y, _spr.width()-12, _spr.height()-btm_y-4, 5, TFT_WHITE);
  const char* labels[] = {"IR1", "IR2", "TC Temp", "MAX(5s)"};
  for(int i=0; i<4; i++) {
     int bx = 6 + (i * (_spr.width()-12)/4) + ((_spr.width()-12)/8);
     _spr.loadFont(Arial12); 
     _spr.setTextColor(TFT_BLACK, TFT_WHITE);
     _spr.setTextDatum(TC_DATUM);
     _spr.drawString(labels[i], bx, btm_y + 4);
     
     _spr.loadFont(Arial18); 
     float val = (i==0)?state.ir_temps[0] : (i==1)?state.ir_temps[1] : (i==2)?state.tc_probe_temp : state.tc_probe_peak;
     char vbuf[20];
     if(isnan(val)) {
         strcpy(vbuf, "---"); 
     } else {
         snprintf(vbuf, 20, "%.0f %c", convertTemp(val, state.temp_unit), unit_char);
     }
     _spr.drawString(vbuf, bx, btm_y + 22);
     _spr.unloadFont();
  }
}

void UIManager::drawAutoModeScreen(const AppState& state, const ConfigState& config) {
  drawTaskBar();
  int top_y = 24;
  int screen_h = _spr.height() - top_y;
  int screen_w = _spr.width();

  // --- สีใหม่สำหรับ Auto Mode ---
  uint16_t C_AUTO_CYAN    = color565(0x00, 0xB3, 0xB9);  // #00B3B9
  uint16_t C_CARD_ACTIVE  = color565(0xEC, 0xFD, 0xFD);  // #ecfdfd
  // gradient bottom: #9C99CA
  
  // --- Background gradient: white (top) -> #9C99CA (bottom) ---
  for (int row = 0; row < screen_h; row++) {
    float t = (float)row / (float)(screen_h - 1);
    uint8_t r = 255 - (uint8_t)(t * (255 - 0x9C));
    uint8_t g = 255 - (uint8_t)(t * (255 - 0x99));
    uint8_t b = 255 - (uint8_t)(t * (255 - 0xCA));
    _spr.drawFastHLine(0, top_y + row, screen_w, color565(r, g, b));
  }

  // เตรียมหน่วยอุณหภูมิ
  char unit_char = (state.temp_unit == 'c' || state.temp_unit == 'C') ? 'C' : 'F';

  // --- 1. Header Section ---
  int title_y = top_y + 5;
  float t_title = (float)(title_y - top_y) / (float)(screen_h - 1);
  uint8_t bg_r = 255 - (uint8_t)(t_title * (255 - 0x9C));
  uint8_t bg_g = 255 - (uint8_t)(t_title * (255 - 0x99));
  uint8_t bg_b = 255 - (uint8_t)(t_title * (255 - 0xCA));
  uint16_t title_bg = color565(bg_r, bg_g, bg_b);

  _spr.loadFont(Arial12);
  _spr.setTextColor(C_AUTO_CYAN, title_bg); 
  _spr.setTextDatum(TL_DATUM);
  _spr.drawString("MODE", 10, title_y);
  _spr.loadFont(Arial18);
  _spr.setTextColor(C_AUTO_CYAN, title_bg);  // #00B3B9 สำหรับคำว่า AUTO
  _spr.drawString("AUTO RUN", 10, title_y + 12);

  // Obj Temp
  _spr.loadFont(Arial12);
  _spr.setTextColor(TFT_BLACK, title_bg);
  _spr.setTextDatum(TR_DATUM);
  _spr.drawString("OBJ TEMP", _spr.width() - 10, title_y + 2);
  _spr.loadFont(Arial18);
  char buf[32];
  if (isnan(state.ir_temps[0])) strcpy(buf, "---");
  else snprintf(buf, 32, "%.0f %c", convertTemp(state.ir_temps[0], state.temp_unit), unit_char);
  _spr.drawString(buf, _spr.width() - 10, title_y + 16);
  _spr.unloadFont();

  // --- 2. Cycle Cards ---
  int card_y = title_y + 42;
  int card_h = 112; 
  int gap = 6;
  int card_w = (_spr.width() - (4 * gap)) / 3;

  bool standbyUsingMain = state.manual_running_background && config.heater_active[0];
  bool isOtherModeRunning = state.manual_preset_running || (standbyUsingMain && !state.auto_running_background);
  bool isLocked = state.heater_cutoff_state[0];
  bool globalRun = state.is_heating_active;

  for (int i = 0; i < 3; i++) {
    int x = gap + (i * (card_w + gap));
    bool is_active_step = (state.auto_step == (i + 1));
    bool is_selected = (_auto_selection == i && _show_cursor);
    bool is_editing = (_current_screen == SCREEN_QUICK_EDIT_AUTO && is_selected);
    
    // สีพื้นกล่อง: active/heating = #ecfdfd, ไม่ active = ขาว
    uint16_t bg = (is_active_step && globalRun) ? C_CARD_ACTIVE : TFT_WHITE;
    uint16_t border = color565(0xD0, 0xE0, 0xE0);  // default border อ่อน

    // Cursor/selection border ใช้ #00B3B9
    if (is_editing) border = _blink_state ? C_AUTO_CYAN : TFT_DARKGREY;
    else if (is_selected) border = C_AUTO_CYAN;
    else if (is_active_step && globalRun) border = C_AUTO_CYAN;

    _spr.fillRoundRect(x, card_y, card_w, card_h, 5, bg);
    _spr.drawRoundRect(x, card_y, card_w, card_h, 5, border);
    if (is_selected) {
      _spr.drawRoundRect(x+1, card_y+1, card_w-2, card_h-2, 5, border);
      _spr.drawRoundRect(x+2, card_y+2, card_w-4, card_h-4, 4, border);
    }

    int cx = x + (card_w / 2);
    _spr.loadFont(Arial12);
    _spr.setTextColor(TFT_BLACK, bg);
    _spr.setTextDatum(TC_DATUM);
    snprintf(buf, 20, "CYCLE %d", i + 1);
    _spr.drawString(buf, cx, card_y + 8);
    
    const char* label_txt = "SET";
    float val_to_show = config.auto_target_temps[i];

    if (is_editing) {
       if (_quick_edit_step == Q_EDIT_MAX) {
          label_txt = "MAX";
          val_to_show = config.auto_max_temps[i];
       }
    }
    
    // "SET" / "MAX" label — ใช้ built-in font เล็กลง 1 step
    _spr.unloadFont();
    _spr.setTextSize(1);
    _spr.setTextColor(TFT_BLACK, bg);
    _spr.setTextDatum(TC_DATUM);
    _spr.drawString(label_txt, cx, card_y + 30);

    // --- ไล่สีอุณหภูมิอิงจาก auto_max_temps ---
    uint16_t val_color;
    if (is_editing) {
      val_color = C_AUTO_CYAN;  // สี cyan ตอน edit
    } else {
      float max_t = config.auto_max_temps[i];
      float cur_set = config.auto_target_temps[i];
      float ratio = (max_t > 0) ? (cur_set / max_t) : 0;
      if (ratio < 0) ratio = 0;
      
      uint8_t tr, tg, tb;
      if (ratio <= 0.5f) {
        float f = ratio / 0.5f;
        tr = (uint8_t)(0x00 + f * (0xE0 - 0x00));
        tg = (uint8_t)(0xB3 + f * (0xC0 - 0xB3));
        tb = (uint8_t)(0xB9 - f * (0xB9 - 0x00));
      } else if (ratio <= 0.85f) {
        float f = (ratio - 0.5f) / 0.35f;
        tr = (uint8_t)(0xE0 + f * (0xFF - 0xE0));
        tg = (uint8_t)(0xC0 - f * (0xC0 - 0x80));
        tb = 0x00;
      } else {
        float f = (ratio - 0.85f) / 0.15f;
        if (f > 1.0f) f = 1.0f;
        tr = 0xFF;
        tg = (uint8_t)(0x80 - f * 0x80);
        tb = 0x00;
      }
      val_color = color565(tr, tg, tb);
    }

    _spr.loadFont(Arial30);
    _spr.setTextColor(val_color, bg);
    _spr.setTextDatum(TC_DATUM);
    snprintf(buf, 20, "%.0f %c", convertTemp(val_to_show, state.temp_unit), unit_char);
    _spr.drawString(buf, cx, card_y + 48);
    _spr.unloadFont();

    // Status Row
    const char* status_txt = "STANDBY";
    uint16_t status_col = TFT_BLUE;
    if (state.tc_faults[0]) { status_txt = "H1 OFF"; status_col = TFT_RED; }
    else if (isOtherModeRunning) { status_txt = "IN-USE"; status_col = 0x7BEF; }
    else if (isLocked) { status_txt = "LOCK"; status_col = _blink_state ? TFT_RED : bg; }
    else if (!is_active_step || !globalRun) { status_txt = "STANDBY"; status_col = TFT_BLUE; }
    else {
      if (state.heater_ready[0]) { status_txt = "READY"; status_col = 0x07E0; }
      else { status_txt = "HEATING"; status_col = TFT_ORANGE; }
    }

    _spr.loadFont(Arial12); 
    int total_w = 6 + 4 + _spr.textWidth(status_txt); 
    int start_x = cx - (total_w / 2);
    _spr.fillCircle(start_x + 3, card_y + 90, 3, status_col);
    _spr.setTextDatum(ML_DATUM);
    _spr.setTextColor(status_col, bg);
    _spr.drawString(status_txt, start_x + 10, card_y + 91);
    _spr.unloadFont();
  }

  // --- 3. Bottom Sensors Row ---
  int btm_y = card_y + card_h + 8;
  int btm_w = _spr.width() - 12;
  int btm_h = _spr.height() - btm_y - 4; 
  
  _spr.fillRoundRect(6, btm_y, btm_w, btm_h, 5, TFT_WHITE);
  const char* labels[] = {"IR1", "IR2", "TC Temp", "MAX(5s)"};
  
  for(int i=0; i<4; i++) {
     int bx = 6 + (i * btm_w/4) + (btm_w/8);
     _spr.loadFont(Arial12); 
     _spr.setTextColor(TFT_BLACK, TFT_WHITE);
     _spr.setTextDatum(TC_DATUM);
     _spr.drawString(labels[i], bx, btm_y + 4);
     _spr.loadFont(Arial18); 
     float val = (i==0)?state.ir_temps[0] : (i==1)?state.ir_temps[1] : (i==2)?state.tc_probe_temp : state.tc_probe_peak;
     if(isnan(val)) strcpy(buf, "---"); 
     else snprintf(buf, 10, "%.0f %c", convertTemp(val, state.temp_unit), unit_char);
     
     _spr.drawString(buf, bx, btm_y + 22);
     _spr.unloadFont();
  }
}

void UIManager::drawManualModeScreen(const AppState& state, const ConfigState& config) {
  drawTaskBar();
  int top_y = 24;
  int screen_h = _spr.height() - top_y;
  int screen_w = _spr.width();

  // --- สีใหม่สำหรับ Preset Mode ---
  uint16_t C_PRESET_OLIVE  = color565(0x91, 0x91, 0x0F);  // #91910f
  uint16_t C_CARD_ACTIVE   = color565(0xEC, 0xFD, 0xFD);  // #ecfdfd
  // gradient bottom: #CDCD06

  // --- Background gradient: white (top) -> #CDCD06 (bottom) ---
  for (int row = 0; row < screen_h; row++) {
    float t = (float)row / (float)(screen_h - 1);
    uint8_t r = 255 - (uint8_t)(t * (255 - 0xCD));
    uint8_t g = 255 - (uint8_t)(t * (255 - 0xCD));
    uint8_t b = 255 - (uint8_t)(t * (255 - 0x06));
    _spr.drawFastHLine(0, top_y + row, screen_w, color565(r, g, b));
  }

  char unit_char = (state.temp_unit == 'c' || state.temp_unit == 'C') ? 'C' : 'F';

  // --- 1. Header Section ---
  int title_y = top_y + 5;
  float t_title = (float)(title_y - top_y) / (float)(screen_h - 1);
  uint8_t bg_r = 255 - (uint8_t)(t_title * (255 - 0xCD));
  uint8_t bg_g = 255 - (uint8_t)(t_title * (255 - 0xCD));
  uint8_t bg_b = 255 - (uint8_t)(t_title * (255 - 0x06));
  uint16_t title_bg = color565(bg_r, bg_g, bg_b);

  _spr.loadFont(Arial12);
  _spr.setTextColor(C_PRESET_OLIVE, title_bg);
  _spr.setTextDatum(TL_DATUM);
  _spr.drawString("MODE", 10, title_y);
  _spr.loadFont(Arial18);
  _spr.setTextColor(C_PRESET_OLIVE, title_bg);  // #91910f
  _spr.drawString("PRESET", 10, title_y + 15);
  _spr.unloadFont();

  // Heater Temp (Right Side)
  _spr.loadFont(Arial12);
  _spr.setTextColor(TFT_BLACK, title_bg);
  _spr.setTextDatum(TR_DATUM);
  _spr.drawString("OBJ TEMP", _spr.width() - 10, title_y + 2);
  
  _spr.loadFont(Arial18);
  char buf[40];
  if (isnan(state.ir_temps[0])) snprintf(buf, 40, "--- %c", unit_char);
  else snprintf(buf, 40, "%.0f %c", convertTemp(state.ir_temps[0], state.temp_unit), unit_char);
  
  _spr.setTextColor(TFT_BLACK, title_bg);
  _spr.drawString(buf, _spr.width() - 10, title_y + 18);
  _spr.unloadFont();

  // --- 2. Preset Grid (2x2) ---
  int gap = 6;
  int grid_y = top_y + 45;
  int card_w = (_spr.width() - (3 * gap)) / 2;
  int card_h = 55; 

  bool standbyUsingMain = state.manual_running_background && config.heater_active[0];
  bool isOtherModeRunning = state.auto_running_background || (standbyUsingMain && !state.manual_preset_running);
  bool globalRun = state.is_heating_active;

  for (int i = 0; i < 4; i++) {
    int col = i % 2; int row = i / 2;
    int x = gap + (col * (card_w + gap)); int y = grid_y + (row * (card_h + gap));
    bool isCursor = (_manual_selection == i && _show_cursor);
    bool isConfirmed = (_manual_confirmed_preset == i);
    bool isEditingThis = (_current_screen == SCREEN_QUICK_EDIT_MANUAL && isCursor);
    bool isThisPresetActive = (state.manual_preset_index == i && state.manual_preset_running);

    // สีพื้นกล่อง: active/confirmed = #ecfdfd, อื่นๆ = ขาว
    uint16_t bg = (isConfirmed || isThisPresetActive) ? C_CARD_ACTIVE : TFT_WHITE;
    if (isOtherModeRunning) bg = color565(0xE7, 0xE7, 0xE0);  // เทาอ่อนอมเหลือง

    uint16_t border = color565(0xD0, 0xD8, 0xD0);  // default border
    if (isCursor) border = C_PRESET_OLIVE;           // #91910f
    else if (isConfirmed) border = C_PRESET_OLIVE;

    _spr.fillRoundRect(x + 2, y + 2, card_w, card_h, 5, 0xBDF7);
    _spr.fillRoundRect(x, y, card_w, card_h, 5, bg);
    _spr.drawRoundRect(x, y, card_w, card_h, 5, border);
    if (isCursor) {
      _spr.drawRoundRect(x+1, y+1, card_w-2, card_h-2, 5, border);
      _spr.drawRoundRect(x+2, y+2, card_w-4, card_h-4, 4, border);
    }

    _spr.loadFont(Arial18); _spr.setTextColor(TFT_BLACK, bg); _spr.setTextDatum(ML_DATUM);
    snprintf(buf, 10, "P%d", i + 1);
    _spr.drawString(buf, x + 8, y + (card_h/2) - 8);
    _spr.drawFastVLine(x + 35, y + 5, card_h - 10, 0xAAAAAA);

    // "SET" / "MAX" label — ใช้ built-in font เล็กลง 1 step
    const char* label_txt = "SET";
    float val = (isEditingThis && _quick_edit_step == Q_EDIT_MAX) ? config.manual_max_temps[i] : config.manual_target_temps[i];
    if (isEditingThis && _quick_edit_step == Q_EDIT_MAX) label_txt = "MAX";

    _spr.unloadFont();
    _spr.setTextSize(1);
    _spr.setTextDatum(TL_DATUM);
    _spr.setTextColor(isEditingThis ? C_PRESET_OLIVE : TFT_DARKGREY, bg);
    _spr.drawString(label_txt, x + 42, y + 5);

    // --- ไล่สีอุณหภูมิอิงจาก manual_max_temps ---
    uint16_t val_color;
    if (isEditingThis) {
      val_color = C_PRESET_OLIVE;
    } else {
      float max_t = config.manual_max_temps[i];
      float cur_set = config.manual_target_temps[i];
      float ratio = (max_t > 0) ? (cur_set / max_t) : 0;
      if (ratio < 0) ratio = 0;
      
      uint8_t tr, tg, tb;
      if (ratio <= 0.5f) {
        float f = ratio / 0.5f;
        tr = (uint8_t)(0x09 + f * (0xE0 - 0x09));
        tg = (uint8_t)(0x90 + f * (0xC0 - 0x90));
        tb = 0x00;
      } else if (ratio <= 0.85f) {
        float f = (ratio - 0.5f) / 0.35f;
        tr = (uint8_t)(0xE0 + f * (0xFF - 0xE0));
        tg = (uint8_t)(0xC0 - f * (0xC0 - 0x80));
        tb = 0x00;
      } else {
        float f = (ratio - 0.85f) / 0.15f;
        if (f > 1.0f) f = 1.0f;
        tr = 0xFF;
        tg = (uint8_t)(0x80 - f * 0x80);
        tb = 0x00;
      }
      val_color = color565(tr, tg, tb);
    }

    _spr.loadFont(Arial18); _spr.setTextColor(val_color, bg);
    _spr.setTextDatum(TL_DATUM);
    snprintf(buf, 20, "%.0f %c", convertTemp(val, state.temp_unit), unit_char);
    _spr.drawString(buf, x + 42, y + 18);

    // --- สถานะ (Dot + Text) ---
    const char* p_status = "OFF"; uint16_t p_col = TFT_DARKGREY;
    if (state.tc_faults[0]) { p_status = "FAULT"; p_col = TFT_RED; }
    else if (isOtherModeRunning) { p_status = "IN-USE"; p_col = 0x7BEF; }
    else if (isThisPresetActive && globalRun) {
        if (state.heater_ready[0]) { p_status = "READY"; p_col = 0x07E0; }
        else { p_status = "HEATING"; p_col = TFT_ORANGE; }
    } else if (state.manual_preset_running && globalRun && !isThisPresetActive) {
        p_status = "IN-USE"; p_col = 0x7BEF;
    } else if (isConfirmed) { p_status = "STANDBY"; p_col = TFT_BLUE; }

    _spr.loadFont(Arial12); _spr.fillCircle(x + 45, y + 42, 3, p_col);
    _spr.setTextColor(p_col, bg); _spr.drawString(p_status, x + 53, y + 42);
    _spr.unloadFont();
  }

  // --- 3. Global Status Bar ---
  int status_y = grid_y + (2 * (card_h + gap)) + 6;
  // คำนวณสี gradient ที่ตำแหน่ง status_y
  float t_status = (float)(status_y - top_y) / (float)(screen_h - 1);
  uint8_t sr = 255 - (uint8_t)(t_status * (255 - 0xCD));
  uint8_t sg = 255 - (uint8_t)(t_status * (255 - 0xCD));
  uint8_t sb = 255 - (uint8_t)(t_status * (255 - 0x06));
  uint16_t status_bg = color565(sr, sg, sb);

  const char* status_text = "---";
  uint16_t status_color = TFT_DARKGREY;
  
  int activePresetIdx = (state.manual_preset_running && globalRun) ? state.manual_preset_index : -1;
  bool isLocked = state.heater_cutoff_state[0];
  bool isReady = state.heater_ready[0];

  if (state.tc_faults[0]) { status_text = "H1 FAULT"; status_color = TFT_RED; }
  else if (isOtherModeRunning) { status_text = "System In-Use"; status_color = TFT_ORANGE; }
  else if (activePresetIdx >= 0) {
    if (isLocked) { status_text = "Locked"; status_color = _blink_state ? TFT_RED : TFT_BLACK; }
    else if (isReady) { status_text = "Ready"; status_color = 0x07E0; }
    else { status_text = "Heating..."; status_color = TFT_ORANGE; }
  } else if (_manual_confirmed_preset >= 0) { status_text = "Standby"; status_color = TFT_BLUE; }

  _spr.loadFont(Arial12);
  int txt_w = _spr.textWidth(status_text);
  int dot_x = (_spr.width() / 2) - (txt_w / 2) - 8;
  _spr.fillCircle(dot_x, status_y + 6, 4, status_color);
  _spr.setTextColor(status_color, status_bg);
  _spr.setTextDatum(TC_DATUM);
  _spr.drawString(status_text, _spr.width()/2 + 5, status_y);
  _spr.unloadFont();

  // --- 4. Bottom Sensors ---
  int btm_y = _spr.height() - 48; 
  _spr.fillRoundRect(6, btm_y, _spr.width()-12, 42, 5, TFT_WHITE);
  const char* b_labels[] = {"IR1", "IR2", "TC Temp", "MAX(5s)"};
  for(int i=0; i<4; i++) {
     int bx = 6 + (i * (_spr.width()-12)/4) + ((_spr.width()-12)/8);
     _spr.loadFont(Arial12); _spr.setTextColor(TFT_BLACK, TFT_WHITE); _spr.setTextDatum(TC_DATUM);
     _spr.drawString(b_labels[i], bx, btm_y + 4);
     _spr.loadFont(Arial18); 
     float val = (i==0)?state.ir_temps[0] : (i==1)?state.ir_temps[1] : (i==2)?state.tc_probe_temp : state.tc_probe_peak;
     if(isnan(val)) strcpy(buf, "---"); else snprintf(buf, 20, "%.0f %c", convertTemp(val, state.temp_unit), unit_char);
     _spr.drawString(buf, bx, btm_y + 22);
  }
}
void UIManager::drawSettingsBrightness(const AppState& state) {
  drawHeader("Screen Brightness");
  _spr.setTextDatum(MC_DATUM);
  _spr.loadFont(Arial18);

  int bar_w = 200, bar_h = 22, x = (_spr.width() - bar_w) / 2, y = 90;
  _spr.fillRoundRect(x, y, bar_w, bar_h, 4, C_SET_CARD_BG);
  _spr.drawRoundRect(x, y, bar_w, bar_h, 4, C_SET_CARD_BRD);
  int fill_w = (int)((constrain(_temp_edit_value, 0.0f, 100.0f) / 100.0f) * (bar_w - 4));
  if (fill_w > 0) _spr.fillRoundRect(x + 2, y + 2, fill_w, bar_h - 4, 3, C_SET_ACCENT);

  char buf[30];
  snprintf(buf, 30, "Value: %.0f%%", _temp_edit_value);
  _spr.setTextColor(C_SET_NORM_TXT, C_SET_BG);
  _spr.drawString(buf, _spr.width() / 2, y + 42);

  _spr.unloadFont();
  drawSettingsFooter(&_spr, NULL, "Rot: Adjust | Press: Save", NULL);
}

void UIManager::drawBootScreen() {
  // ใช้สีเขียวเข้ม (ประมาณค่าจากรูปภาพ) 
  // RGB(53, 75, 45) -> 565 Hex: 0x3245
  #define C_BOOT_BG 0x3245 
  
  _spr.fillSprite(C_BOOT_BG);
  
  _spr.setTextColor(TFT_WHITE, C_BOOT_BG);
  _spr.setTextDatum(MC_DATUM);
  
  // ใช้ Font ขนาดใหญ่ (Arial18 เป็น Font ใหญ่สุดที่มีในโค้ดตอนนี้)
  _spr.loadFont(Arial18); 
  
  // จัดกึ่งกลางหน้าจอ
  int cx = _spr.width() / 2;
  int cy = _spr.height() / 2;
  
  // วาดข้อความ "SMART HEATER"
  _spr.drawString("SMART", cx, cy - 20);
  _spr.drawString("HEATER", cx, cy + 20);
  
  _spr.unloadFont();
}