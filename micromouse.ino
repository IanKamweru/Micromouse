/*
 * Project: Micromouse
 * Author: Ian Kamweru
 * Description: Main control file
 */

#include <Wire.h>
#include "TOFSensor.h"
#include "PCA9548A.h"
#include "ArduinoMotorShieldR3.h"
#include "AxisEncoderShield3.h"

// Create ToF sensor objects
TOFSensor leftToF(0, "Left TOF");
TOFSensor frontToF(1, "Front TOF");
TOFSensor rightToF(2, "Right TOF");

// Create motor shield object
ArduinoMotorShieldR3 motorShield;

#define ENCODER_CHANNEL_1 1  // First slot on the encoder shield
#define ENCODER_CHANNEL_2 2  // Second slot on the encoder shield

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize each ToF sensor
  if (!leftToF.init()) Serial.println("Failed to initialize Left ToF");
  if (!frontToF.init()) Serial.println("Failed to initialize Front ToF");
  if (!rightToF.init()) Serial.println("Failed to initialize Right ToF");

  motorShield.init();
  initEncoderShield();  // Initialize the encoder shield
}

void loop() {
  Get distances from each sensor
  int leftDistance = leftToF.getDistance();
  int frontDistance = frontToF.getDistance();
  int rightDistance = rightToF.getDistance();

  // Print the values to the Serial Monitor
  Serial.print("Left: ");
  Serial.print(leftDistance);
  Serial.print(" mm, Front: ");
  Serial.print(frontDistance);
  Serial.print(" mm, Right: ");
  Serial.print(rightDistance);
  Serial.println(" mm");

  motorShield.setM2Speed(0);
  delay(1000);
  motorShield.setM2Speed(150);
  delay(2000);
  printEncoderCounts();
  motorShield.setM2Speed(0);
  delay(1000);

  while(1); // Halt program
}

// Function to print the current encoder counts
void printEncoderCounts() {
  long counts = getEncoderValue(ENCODER_CHANNEL_2);
  Serial.print("Encoder Slot ");
  Serial.print(ENCODER_CHANNEL_2);
  Serial.print(" = ");
  Serial.println(counts);
}
