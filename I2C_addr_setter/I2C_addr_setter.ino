#include <Wire.h>

// กำหนดขา SDA และ SCL สำหรับ ESP32-S3
#define I2C_SDA 8
#define I2C_SCL 9

// Address เดิม (ถ้าไม่เคยเปลี่ยนคือ 0x5A)
uint8_t oldAddr = 0x10; 
// Address ใหม่ที่ต้องการ (0x10)
uint8_t newAddr = 0x11;

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(2000);

  Serial.println("--- MLX90614 Address Changer (Low Level) ---");
  
  // 1. ลบค่าเดิมใน EEPROM (ตำแหน่ง 0x2E) โดยการเขียน 0x0000
  Serial.println("Step 1: Clearing old address...");
  writeEEPROM(oldAddr, 0x2E, 0x00, 0x00);
  delay(100);

  // 2. เขียน Address ใหม่ลงไป (0x10)
  // หมายเหตุ: MLX90614 เก็บค่าเป็น 16-bit ใน EEPROM โดย Address อยู่ที่ byte ต่ำ
  Serial.print("Step 2: Writing new address 0x");
  Serial.println(newAddr, HEX);
  writeEEPROM(oldAddr, 0x2E, newAddr, 0x00);
  delay(100);

  Serial.println("------------------------------------------");
  Serial.println("DONE! Please POWER OFF and POWER ON the sensor.");
  Serial.println("Then run the I2C Scanner to check.");
}

void loop() {}

// ฟังก์ชันสำหรับเขียน EEPROM พร้อมคำนวณ PEC (CRC-8)
void writeEEPROM(uint8_t devAddr, uint8_t reg, uint8_t lowByte, uint8_t highByte) {
  // ขั้นตอนการเขียน EEPROM ของ MLX90614:
  // [Addr_W] [Reg] [Low_Data] [High_Data] [PEC]

  // 1. ลบค่าเดิมก่อน (ส่ง 0x00, 0x00)
  sendWriteCommand(devAddr, reg, 0x00, 0x00);
  delay(100); // รอให้ EEPROM ลบเสร็จ

  // 2. เขียนค่าใหม่
  sendWriteCommand(devAddr, reg, lowByte, highByte);
  delay(100);
}

void sendWriteCommand(uint8_t devAddr, uint8_t reg, uint8_t low, uint8_t high) {
  uint8_t addrW = devAddr << 1;
  uint8_t data[] = {addrW, reg, low, high};
  uint8_t pec = crc8(data, 4);

  Wire.beginTransmission(devAddr);
  Wire.write(reg);
  Wire.write(low);
  Wire.write(high);
  Wire.write(pec);
  Wire.endTransmission();
}

// CRC-8 สำหรับ MLX90614 (Polynomial 0x07)
uint8_t crc8(uint8_t *ptr, uint8_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc ^= *ptr++;
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x07;
      else crc <<= 1;
    }
  }
  return crc;
}