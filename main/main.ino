#include <SPI.h>
#include "Adafruit_MAX31855.h"

// Define pins for thermocouple
#define MAXDO   19
#define MAXCS   5
#define MAXCLK  18

// Define the pin for the relay
#define RELAY_PIN 15


// Initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc

  // --- New code for Relay ---
  pinMode(RELAY_PIN, OUTPUT);      // Set the relay pin as an output
  digitalWrite(RELAY_PIN, LOW);    // Ensure the relay is OFF by default
  // --------------------------

  Serial.println("MAX31855 test");
  Serial.println("Type 'ON' or 'OFF' to control the relay."); // Instructions for the user

  // wait for MAX chip to stabilize
  delay(500);
  Serial.print("Initializing sensor...");
  if (!thermocouple.begin()) {
    Serial.println("ERROR.");
    while (1) delay(10);
  }

  Serial.println("DONE.");
}

void loop() {
  // --- New code to check for Serial commands ---
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Read the incoming command
    command.trim(); // Remove any whitespace

    if (command.equalsIgnoreCase("ON")) {
      digitalWrite(RELAY_PIN, HIGH); // Turn the relay ON
      Serial.println("Relay turned ON");
    } else if (command.equalsIgnoreCase("OFF")) {
      digitalWrite(RELAY_PIN, LOW);  // Turn the relay OFF
      Serial.println("Relay turned OFF");
    } else {
      Serial.println("Unknown command. Please use 'ON' or 'OFF'.");
    }
  }
  // ---------------------------------------------


  // Original code for reading the temperature
//  Serial.print("Internal Temp = ");
//  Serial.println(thermocouple.readInternal());

  double c = thermocouple.readCelsius();
//  if (isnan(c)) {
//    Serial.println("Thermocouple fault(s) detected!");
//    uint8_t e = thermocouple.readError();
//    if (e & MAX31855_FAULT_OPEN) Serial.println("FAULT: Thermocouple is open - no connections.");
//    if (e & MAX31855_FAULT_SHORT_GND) Serial.println("FAULT: Thermocouple is short-circuited to GND.");
//    if (e & MAX31855_FAULT_SHORT_VCC) Serial.println("FAULT: Thermocouple is short-circuited to VCC.");
//  } else {
//    Serial.print("C = ");
//    Serial.println(c);
//  }
   Serial.print("C = ");
   Serial.println(c);
  delay(200);
}
