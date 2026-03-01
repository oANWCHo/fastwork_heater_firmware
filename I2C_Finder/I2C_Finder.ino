#include <Wire.h>

// กำหนดขา SDA และ SCL สำหรับ ESP32-S3 (ถ้าบอร์ดคุณใช้ขาอื่น สามารถเปลี่ยนเลขตรงนี้ได้)
#define I2C_SDA 8
#define I2C_SCL 9

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  // เริ่มต้น I2C ด้วยขาที่กำหนด
  bool status = Wire.begin(I2C_SDA, I2C_SCL);
  
  if (!status) {
    Serial.println("I2C initialization failed");
  } else {
    Serial.println("\nI2C Scanner for ESP32-S3");
  }
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning...");

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("Scan finished\n");

  delay(5000);
}