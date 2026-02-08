#ifndef UI_MANAGER_H
#define UI_MANAGER_H

#include <TFT_eSPI.h>

enum UIScreen {
  SCREEN_BOOT,
  SCREEN_STANDBY,
  SCREEN_AUTO_MODE,
  SCREEN_MANUAL_MODE,
  SCREEN_QUICK_EDIT,
  SCREEN_QUICK_EDIT_AUTO,
  SCREEN_QUICK_EDIT_MANUAL,
  SCREEN_SLEEP,
  SCREEN_SETTINGS_PAGE_1, 
  SCREEN_SETTINGS_PAGE_2, 
  SCREEN_SETTINGS_PAGE_3,           // NEW: Page 3 for WiFi Settings
  SCREEN_SETTINGS_HEATER_TARGET_TEMP,
  SCREEN_SETTINGS_HEATER_MAX_TEMP,
  SCREEN_SETTINGS_HEATER_CALIBRATE, 
  SCREEN_SETTINGS_CALIBRATION_SELECT, 
  SCREEN_SETTINGS_MAX_TEMP_LOCK,
  SCREEN_SETTINGS_IDLE_OFF,
  SCREEN_SETTINGS_STARTUP,
  SCREEN_SETTINGS_EMISSIVITY,
  SCREEN_SETTINGS_SOUND,
  SCREEN_SETTINGS_TEMP_UNIT,
  SCREEN_SETTINGS_TC_PROBE_CAL,
  SCREEN_SETTINGS_ABOUT,
  SCREEN_SETTINGS_BRIGHTNESS,
  SCREEN_SETTINGS_WIFI_MENU,        // NEW: WiFi sub-menu
  SCREEN_SETTINGS_WIFI_SSID,        // NEW: SSID entry screen
  SCREEN_SETTINGS_WIFI_PASSWORD,    // NEW: Password entry screen
  SCREEN_SETTINGS_WIFI_STATUS       // NEW: WiFi status/info screen
};

enum MenuItemPage1 {
  MENU_PAGE1_MAX_TEMP_LOCK,
  MENU_PAGE1_CALIBRATION, 
  MENU_PAGE1_EMISSIVITY,
  MENU_PAGE1_TC_PROBE_CAL, 
  MENU_PAGE1_ITEM_COUNT 
};

enum MenuItemPage2 {
  MENU_PAGE2_IDLE_OFF,
  MENU_PAGE2_STARTUP,  
  MENU_PAGE2_SOUND,
  MENU_PAGE2_TEMP_UNIT,
  MENU_PAGE2_BRIGHTNESS,
  MENU_PAGE2_ABOUT,
  MENU_PAGE2_ITEM_COUNT
};

// NEW: Page 3 menu items
enum MenuItemPage3 {
  MENU_PAGE3_WIFI_SETTINGS,
  MENU_PAGE3_ITEM_COUNT
};

// NEW: WiFi menu items
enum WiFiMenuItem {
  WIFI_MENU_SSID,
  WIFI_MENU_PASSWORD,
  WIFI_MENU_STATUS,
  WIFI_MENU_CONNECT,
  WIFI_MENU_BACK,
  WIFI_MENU_ITEM_COUNT
};

enum IdleOffMode {
  IDLE_OFF_10_SEC,
  IDLE_OFF_15_MIN,
  IDLE_OFF_30_MIN,
  IDLE_OFF_60_MIN,
  IDLE_OFF_ALWAYS_ON,
  IDLE_OFF_ITEM_COUNT
};

enum StartupMode {
  STARTUP_OFF,
  STARTUP_AUTORUN,  
  STARTUP_MODE_COUNT
};

// NEW: WiFi configuration structure
#define WIFI_SSID_MAX_LEN 32
#define WIFI_PASS_MAX_LEN 64

struct WiFiConfig {
  char ssid[WIFI_SSID_MAX_LEN + 1];
  char password[WIFI_PASS_MAX_LEN + 1];
  bool use_custom;  // true = use custom settings, false = use hardcoded
};

struct ConfigState {
  float target_temps[3];
  float max_temps[3];
  float max_temp_lock;
  float auto_target_temps[3]; 
  float auto_max_temps[3];
  float manual_target_temps[4];  // 4 presets for Manual Mode
  float manual_max_temps[4];     // 4 presets for Manual Mode
  char temp_unit;
  IdleOffMode idle_off_mode;
  bool light_on;
  bool sound_on;
  uint8_t sound_volume;
  bool heater_active[3];
  float tc_offsets[3];
  float tc_probe_offset;
  StartupMode startup_mode;
  float ir_emissivity[2];
  uint8_t brightness;
  WiFiConfig wifi_config;  // NEW: WiFi settings
};

struct AppState {
  float tc_temps[3];
  float tc_probe_temp;
  float tc_probe_peak;
  float ir_temps[2];
  float ir_ambient[2];
  float heater_power[3];
  bool is_heating_active;
  float target_temp;
  char temp_unit;
  uint8_t tc_faults[3];
  bool heater_cutoff_state[3];
  bool heater_ready[3];
  uint8_t auto_step;
  bool auto_mode_enabled;        
  bool auto_running_background;  
  bool manual_running_background; 
  bool manual_mode_enabled;       
  bool manual_preset_running;     
  bool manual_was_started;
  uint8_t manual_preset_index;
  char ip_address[20];
  bool is_warning;
};

enum QuickEditStep {
  Q_EDIT_TARGET,
  Q_EDIT_MAX
};

typedef void (*ConfigSaveCallback)(const ConfigState& config);
typedef void (*WiFiReconnectCallback)();  // NEW: Callback to trigger WiFi reconnection

class UIManager {
public:
  UIManager(TFT_eSPI* tft, ConfigSaveCallback save_callback = nullptr);
  void begin();
  
  void draw(const AppState& state, const ConfigState& config);
  
  bool handleButtonSingleClick(ConfigState& config, float& go_to, bool& has_go_to);
  bool handleButtonHold(ConfigState& config);
  bool handleEncoderRotation(float steps, ConfigState& config);

  bool checkInactivity(ConfigState& config, bool& has_go_to, float& go_to);
  void resetInactivityTimer();

  void openSettings();
  void exitSettings();           
  void enterQuickEdit();
  void enterQuickEditAuto();
  void enterQuickEditManual();
  void switchToAutoMode();
  void switchToManualMode();
  void switchToStandby();
  UIScreen getScreen() const { return _current_screen; } 

  int getManualSelection() const { return _manual_selection; }
  int getManualConfirmedPreset() const { return _manual_confirmed_preset; }
  void setManualConfirmedPreset(int preset) { _manual_confirmed_preset = preset; }
  void setWiFiReconnectCallback(WiFiReconnectCallback callback) { _wifi_reconnect_callback = callback; }

private:
  TFT_eSPI* _tft;
  TFT_eSprite _spr;
  UIScreen _current_screen;
  UIScreen _previous_screen;
  QuickEditStep _quick_edit_step;
  
  bool _blink_state;
  bool _is_editing_calibration;
  int _selected_menu_item; 
  int _selected_menu_item_page_2; 
  int _selected_menu_item_page_3;  
  int _selected_wifi_menu_item;      
  float _menu_step_accumulator;
  float _temp_edit_value;
  
  int _standby_selection;
  int _auto_selection;
  int _manual_selection;
  int _manual_confirmed_preset;
  bool _show_warning;
  
  uint32_t _boot_start_time;
  uint32_t _last_activity_time;

  ConfigSaveCallback _save_callback;
  WiFiReconnectCallback _wifi_reconnect_callback;  // NEW

  // NEW: Character entry variables
  char _char_entry_buffer[WIFI_PASS_MAX_LEN + 1];  // Temporary buffer for entry
  int _char_entry_cursor;                           // Current cursor position
  int _char_entry_char_index;                       // Current character index in charset
  int _char_entry_max_len;                          // Maximum length for current field
  bool _char_entry_editing;                         // true = editing character, false = moving cursor

  // Character set for WiFi entry (printable ASCII)
  static const char* getCharset();
  static int getCharsetLength();
  int findCharInCharset(char c);

  void drawStandbyScreen(const AppState& state, const ConfigState& config);
  void drawSettingsPage1(const AppState& state, const ConfigState& config); 
  void drawSettingsPage2(const AppState& state, const ConfigState& config); 
  void drawSettingsPage3(const AppState& state, const ConfigState& config);   // NEW
  void drawSettingsHeaterTargetTemp(const AppState& state);
  void drawSettingsHeaterMaxTemp(const AppState& state);
  void drawSettingsHeaterCalibrate(const AppState& state);
  void drawSettingsCalibrationSelect(const AppState& state, const ConfigState& config); 
  void drawSettingsMaxTempLock(const AppState& state);
  void drawSettingsIdleOff(const AppState& state, const ConfigState& config);
  void drawSettingsStartup(const AppState& state, const ConfigState& config);
  void drawSettingsSound(const AppState& state, const ConfigState& config); 
  void drawSettingsTCProbeCal(const AppState& state, const ConfigState& config);
  void drawSettingsTempUnit(const AppState& state, const ConfigState& config);
  void drawSettingsAbout(const AppState& state);
  void drawSettingsBrightness(const AppState& state);
  void drawSettingsEmissivity(const AppState& state, const ConfigState& config);
  void drawAutoModeScreen(const AppState& state, const ConfigState& config);
  void drawManualModeScreen(const AppState& state, const ConfigState& config);

  void drawSettingsWiFiMenu(const AppState& state, const ConfigState& config);
  void drawSettingsWiFiSSID(const AppState& state, const ConfigState& config);
  void drawSettingsWiFiPassword(const AppState& state, const ConfigState& config);
  void drawSettingsWiFiStatus(const AppState& state, const ConfigState& config);
  void drawCharEntryScreen(const char* title, bool is_password);
  void drawBootScreen();
  void drawTaskBar();
  void drawHeader(const char* title); 

  uint16_t getStatusColor(bool is_active, float current_temp, float target_temp);
  float convertTemp(float temp_c, char unit);
  float convertDelta(float temp_c, char unit);
  uint32_t _last_cursor_move_time;
  bool _show_cursor;
  uint16_t color565(uint8_t r, uint8_t g, uint8_t b);
};

#endif // UI_MANAGER_H