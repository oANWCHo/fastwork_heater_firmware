#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include "driver/pcnt.h" // Required for the ESP32 Pulse Counter
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

// Define pins for thermocouple
//#define MAXDO     19
//#define MAXCS     5
//#define MAXCLK    18

#define MAXDO     12 //Same with TFT_MISO
#define MAXCS     25
#define MAXCLK    14 //Same with TFT_MISO
// Define the pin for the relay
#define RELAY_PIN 15

// Define pins for the Rotary Encoder
//#define ENCODER_A 25
//#define ENCODER_B 26
//#define ENCODER_SW 27

#define ENCODER_A 36 //Data
#define ENCODER_B 39 //CLK
#define ENCODER_SW 34 //SW

// For the Adafruit shield, these are the default.
#define TFT_DC 27
#define TFT_CS 26
#define TFT_MOSI 13
#define TFT_MISO 12
#define TFT_CLK 14
#define TFT_RST 4

// Initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

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
  // delay(500);
  // Serial.print("Initializing sensor...");
  // if (!thermocouple.begin()) {
  //   Serial.println("ERROR.");
  //   while (1) delay(10);
  // }
  // Serial.println("DONE.");
  
  tft.begin();
  tft.setRotation(0);
  testText();
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
unsigned long testText() {
  tft.fillScreen(ILI9341_BLACK);
  unsigned long start = micros();
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2);
  tft.println(1234.56);
  tft.setTextColor(ILI9341_RED);    tft.setTextSize(3);
  tft.println(0xDEADBEEF, HEX);
  tft.println();
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(5);
  tft.println("Groop");
  tft.setTextSize(2);
  tft.println("I implore thee,");
  tft.setTextSize(1);
  tft.println("my foonting turlingdromes.");
  tft.println("And hooptiously drangle me");
  tft.println("with crinkly bindlewurdles,");
  tft.println("Or I will rend thee");
  tft.println("in the gobberwarts");
  tft.println("with my blurglecruncheon,");
  tft.println("see if I don't!");
  return micros() - start;
}
