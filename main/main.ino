#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include "driver/pcnt.h" // Required for the ESP32 Pulse Counter

// Define pins for thermocouple
//#define MAXDO     19
//#define MAXCS     5
//#define MAXCLK    18

#define MAXDO     12
#define MAXCS     25
#define MAXCLK    14
// Define the pin for the relay
#define RELAY_PIN 15

// Define pins for the Rotary Encoder
//#define ENCODER_A 25
//#define ENCODER_B 26
//#define ENCODER_SW 27

#define ENCODER_A 36
#define ENCODER_B 39
#define ENCODER_SW 34

// Initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

// Variable to track the encoder's value for printing
long lastEncoderValue = 0;
int lastSwitchState = HIGH; 

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(1);

  // --- Encoder Setup using HARDWARE PULSE COUNTER ---
  pinMode(ENCODER_SW, INPUT_PULLUP);

  // 1. Configure the PCNT unit
  pcnt_config_t pcnt_config = {};
  pcnt_config.pulse_gpio_num = ENCODER_A;
  pcnt_config.ctrl_gpio_num = ENCODER_B;
  pcnt_config.channel = PCNT_CHANNEL_0;
  pcnt_config.unit = PCNT_UNIT_0;
  pcnt_config.pos_mode = PCNT_COUNT_DEC;
  pcnt_config.neg_mode = PCNT_COUNT_INC;
  pcnt_config.lctrl_mode = PCNT_MODE_REVERSE;
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;

  // 2. Initialize PCNT unit
  pcnt_unit_config(&pcnt_config);

  // 3. Optional: Set a hardware filter to debounce the signal
  pcnt_set_filter_value(PCNT_UNIT_0, 1000);
  pcnt_filter_enable(PCNT_UNIT_0);

  // 4. Initialize and start the counter
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);
  // --- End of Encoder Setup ---

  // --- Relay Setup ---
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  Serial.println("MAX31855 & Hardware Encoder Test");
  Serial.println("Type 'ON' or 'OFF' to control the relay.");

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
  read_heater_input();
  heater_read();
  read_hardware_encoder(); 
  check_encoder_switch();
  delay(50); // Main loop delay
}

void heater_read() {
  double c = thermocouple.readCelsius();
  Serial.print("C = ");
  Serial.println(c);
}

void read_heater_input() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.equalsIgnoreCase("ON")) {
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println("Relay turned ON");
    } else if (command.equalsIgnoreCase("OFF")) {
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("Relay turned OFF");
    } else {
      Serial.println("Unknown command. Please use 'ON' or 'OFF'.");
    }
  }
}

// This function now reads the value directly from the PCNT hardware
void read_hardware_encoder() {
  int16_t current_count = 0;
  pcnt_get_counter_value(PCNT_UNIT_0, &current_count);

  // Only print the value if it has changed
  if (current_count != lastEncoderValue) {
    Serial.print("Encoder Value: ");
    Serial.println(current_count);
    lastEncoderValue = current_count; // Update the last known value
  }
}

void check_encoder_switch() {
  int currentSwitchState = digitalRead(ENCODER_SW);

  // Check if the switch has just been pressed (gone from HIGH to LOW)
  if (lastSwitchState == HIGH && currentSwitchState == LOW) {
    Serial.println("Encoder Switch Pressed!");
    
    // --- You can add an action here! ---
    // For example, reset the encoder count to zero on press:
    
    delay(50); // Simple debounce to prevent multiple triggers
  }
  
  // Update the last known state of the switch
  lastSwitchState = currentSwitchState;
}
