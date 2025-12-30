#ifndef UI_MANAGER_H
#define UI_MANAGER_H

#include <TFT_eSPI.h>

enum UIScreen {
  SCREEN_STANDBY,
  SCREEN_SLEEP,
  SCREEN_SETTINGS_PAGE_1, 
  SCREEN_SETTINGS_PAGE_2, 
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
  SCREEN_SETTINGS_ABOUT
};

enum MenuItemPage1 {
  MENU_PAGE1_HEATER_1,
  MENU_PAGE1_HEATER_2,
  MENU_PAGE1_HEATER_3,
  MENU_PAGE1_MAX_TEMP_LOCK,
  MENU_PAGE1_CALIBRATION, 
  MENU_PAGE1_EMISSIVITY,
  MENU_PAGE1_NEXT_PAGE,
  MENU_PAGE1_ITEM_COUNT 
};

enum MenuItemPage2 {
  MENU_PAGE2_IDLE_OFF,
  MENU_PAGE2_STARTUP,  
  MENU_PAGE2_SOUND,
  MENU_PAGE2_TC_PROBE_CAL,
  MENU_PAGE2_TEMP_UNIT,
  MENU_PAGE2_ABOUT,
  MENU_PAGE2_PREV_PAGE,
  MENU_PAGE2_ITEM_COUNT
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

struct ConfigState {
  float target_temps[3];
  float max_temps[3];
  float max_temp_lock;
  char temp_unit;
  IdleOffMode idle_off_mode;
  bool light_on;
  bool sound_on;
  bool heater_active[3];
  float tc_offsets[3];
  float tc_probe_offset;
  StartupMode startup_mode;
  float ir_emissivity[2];
};

struct AppState {
  float tc_temps[3];
  float tc_probe_temp;
  float ir_temps[2];
  float ir_ambient[2];
  bool is_heating_active;
  float target_temp;
  char temp_unit;
  uint8_t tc_faults[3];
  bool heater_cutoff_state[3];
};

typedef void (*ConfigSaveCallback)(const ConfigState& config);

class UIManager {
public:
  UIManager(TFT_eSPI* tft, ConfigSaveCallback save_callback = nullptr);
  void begin();
  
  void draw(const AppState& state, const ConfigState& config);
  
  bool handleButtonSingleClick(ConfigState& config, float& go_to, bool& has_go_to);
  
  bool handleButtonDoubleClick(ConfigState& config);
  bool handleEncoderRotation(float steps, ConfigState& config);

  bool checkInactivity(ConfigState& config, bool& has_go_to, float& go_to);
  void resetInactivityTimer();

private:
  TFT_eSPI* _tft;
  TFT_eSprite _spr;
  UIScreen _current_screen;
  bool _blink_state;
  int _selected_menu_item; 
  int _selected_menu_item_page_2; 
  float _menu_step_accumulator;
  float _temp_edit_value;
  
  int _standby_selection;
  uint32_t _last_activity_time;

  ConfigSaveCallback _save_callback;

  void drawStandbyScreen(const AppState& state, const ConfigState& config);
  void drawSettingsPage1(const AppState& state, const ConfigState& config); 
  void drawSettingsPage2(const AppState& state, const ConfigState& config); 
  void drawSettingsHeaterTargetTemp(const AppState& state);
  void drawSettingsHeaterMaxTemp(const AppState& state);
  void drawSettingsHeaterCalibrate(const AppState& state);
  void drawSettingsCalibrationSelect(const AppState& state, const ConfigState& config); // <-- ADDED
  void drawSettingsMaxTempLock(const AppState& state);
  void drawSettingsIdleOff(const AppState& state, const ConfigState& config);
  void drawSettingsStartup(const AppState& state, const ConfigState& config);
  void drawSettingsSound(const AppState& state, const ConfigState& config); 
  void drawSettingsTCProbeCal(const AppState& state, const ConfigState& config);
  void drawSettingsTempUnit(const AppState& state, const ConfigState& config);
  void drawSettingsAbout(const AppState& state);
  void drawSettingsEmissivity(const AppState& state, const ConfigState& config);

  uint16_t getStatusColor(bool is_active, float current_temp, float target_temp);
  float convertTemp(float temp_c, char unit);
};

#endif // UI_MANAGER_H