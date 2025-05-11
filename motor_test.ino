#include "ArduinoMotorShieldR3.h"
#include "AxisEncoderShield3.h"

// Create motor shield object
ArduinoMotorShieldR3 motorShield;

#define ENCODER_CHANNEL 2  // Second slot on the encoder shield

void setup() {
  Serial.begin(115200);
  motorShield.init();
  initEncoderShield();  // Initialize the encoder shield

  Serial.println("Testing Motor on Channel B");
}

void loop() {

  // Half speed forward
  Serial.println("Motor B 50% Forward");
  motorShield.setM2Speed(0);
  while(1);
}

// Function to print the current encoder counts
void printEncoderCounts() {
  long counts = getEncoderValue(ENCODER_CHANNEL);
  Serial.print("Encoder Slot ");
  Serial.print(ENCODER_CHANNEL);
  Serial.print(" = ");
  Serial.println(counts);
}
