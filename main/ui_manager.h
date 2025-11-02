#ifndef UI_MANAGER_H
#define UI_MANAGER_H

#include <TFT_eSPI.h>

enum UIScreen {
  SCREEN_STANDBY,
  SCREEN_SETTINGS_MAIN,
  SCREEN_SETTINGS_HEATER_START_TEMP,
  SCREEN_SETTINGS_HEATER_MAX_TEMP,
  SCREEN_SETTINGS_MAX_TEMP_LOCK,
  SCREEN_SETTINGS_IDLE_OFF,
  SCREEN_SETTINGS_LIGHT_SOUND,
  SCREEN_SETTINGS_TEMP_UNIT,
  SCREEN_SETTINGS_ABOUT
};

enum MenuItem {
  MENU_HEATER_1,
  MENU_HEATER_2,
  MENU_HEATER_3,
  MENU_MAX_TEMP_LOCK,
  MENU_IDLE_OFF,
  MENU_LIGHT_SOUND,
  MENU_TEMP_UNIT,
  MENU_ABOUT,
  MENU_ITEM_COUNT
};

enum IdleOffMode {
  IDLE_OFF_15_MIN,
  IDLE_OFF_30_MIN,
  IDLE_OFF_60_MIN,
  IDLE_OFF_ALWAYS_ON,
  IDLE_OFF_ITEM_COUNT
};

struct ConfigState {
  float start_temps[3];
  float max_temps[3];
  float max_temp_lock;
  char temp_unit;
  IdleOffMode idle_off_mode;
  bool light_on;
  bool sound_on;
  bool heater_active[3]; // [NEW] To store ON/OFF state for H1, H2, H3
};

struct AppState {
  float tc_temps[3];
  float ir_temps[2];
  bool is_heating_active;
  float target_temp;
  char temp_unit;
  uint8_t tc_faults[3];
};

class UIManager {
public:
  UIManager(TFT_eSPI* tft);
  void begin();
  
  // [MODIFIED] Draw function signature
  void draw(const AppState& state, const ConfigState& config);
  
  // [MODIFIED] Single click needs to set go_to vars
  bool handleButtonSingleClick(ConfigState& config, float& go_to, bool& has_go_to);
  
  bool handleButtonDoubleClick(ConfigState& config);
  bool handleEncoderRotation(float steps, ConfigState& config);

private:
  TFT_eSPI* _tft;
  TFT_eSprite _spr;
  UIScreen _current_screen;
  bool _blink_state;
  int _selected_menu_item;
  float _menu_step_accumulator;
  float _temp_edit_value;
  
  int _standby_selection;

  void drawStandbyScreen(const AppState& state, const ConfigState& config);
  void drawSettingsMain(const AppState& state, const ConfigState& config);    void drawSettingsHeaterStartTemp(const AppState& state);
  void drawSettingsHeaterMaxTemp(const AppState& state);
  void drawSettingsMaxTempLock(const AppState& state);
  void drawSettingsIdleOff(const AppState& state, const ConfigState& config);
  void drawSettingsLightSound(const AppState& state, const ConfigState& config);
  void drawSettingsTempUnit(const AppState& state, const ConfigState& config);
  void drawSettingsAbout(const AppState& state);

  // Helper functions
  uint16_t getStatusColor(bool is_active, float current_temp, float target_temp);
  float convertTemp(float temp_c, char unit);
};

#endif // UI_MANAGER_H