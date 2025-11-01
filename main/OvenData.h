#pragma once // ป้องกันการ include ซ้ำ

// ----- 1. สถานะของหน้าจอ (UI States) -----
// แมปตามหน้าต่างๆ ใน PDF
enum UI_State {
  STATE_STANDBY,        // หน้า Standby (Page 1)
  STATE_MENU_MAIN,      // หน้า Setting หลัก (Page 2 & 3)
  STATE_EDIT_H1_START,  // ตั้งค่า Heater 1 Start Temp (Page 4)
  STATE_EDIT_H1_MAX,    // ตั้งค่า Heater 1 Max Temp (Page 5)
  STATE_EDIT_H2_START,
  STATE_EDIT_H2_MAX,
  STATE_EDIT_H3_START,
  STATE_EDIT_H3_MAX,
  STATE_EDIT_MAX_LOCK,  // ตั้งค่า Max Temp Lock (Page 6)
  STATE_EDIT_IDLE_OFF,  // ตั้งค่า Turn off When Idle (Page 7)
  STATE_EDIT_LIGHT_SOUND, // ตั้งค่า Light and Sound (Page 8)
  STATE_EDIT_TEMP_UNIT, // ตั้งค่า Temperature Unit (Page 9)
  STATE_PAGE_ABOUT      // หน้า About (Page 10)
};

// ----- 2. ประเภทการกดปุ่ม -----
enum ClickType {
  CLICK_NONE,
  CLICK_SINGLE,
  CLICK_DOUBLE
};

// ----- 3. โครงสร้างข้อมูลสำหรับเก็บค่า Settings -----
struct OvenSettings {
  // ค่าจาก PDF
  float heater_start_temp[3] = { 50.0, 50.0, 50.0 };
  float heater_max_temp[3]   = { 250.0, 250.0, 250.0 };
  float max_temp_lock        = 300.0;
  int   idle_off_minutes     = 30; // 0=Always ON, 15, 30, 60
  bool  light_on             = true;
  bool  sound_on             = true;
  bool  unit_is_celsius      = true; // true=C, false=F

  // ค่าสำหรับควบคุมการทำงาน (เพิ่มจาก .ino เดิม)
  bool heater_running[3]     = { false, false, false };
  // bool heater_finished[3] = { false, false, false }; // (ยังไม่ implement)
};

// ----- 4. โครงสร้างข้อมูลสำหรับเก็บค่าเซ็นเซอร์ -----
struct SensorData {
  float TC1 = NAN;
  float TC2 = NAN;
  float TC3 = NAN; // ยังไม่มีฮาร์ดแวร์
  float IR_Avg = NAN;
  float TC1_CJ = NAN; // Cold Junction (เผื่อ Debug)
  float TC2_CJ = NAN;
};