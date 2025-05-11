/*
 * Project: Micromouse
 * Author: Ian Kamweru
 * Description: Main control file
 */

#include <Wire.h>
#include "TOFSensor.h"
#include "PCA9548A.h"

// Create ToF sensor objects
TOFSensor leftToF(0, "Left TOF");
TOFSensor frontToF(1, "Front TOF");
TOFSensor rightToF(2, "Right TOF");

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize each ToF sensor
  if (!leftToF.init()) Serial.println("Failed to initialize Left ToF");
  if (!frontToF.init()) Serial.println("Failed to initialize Front ToF");
  if (!rightToF.init()) Serial.println("Failed to initialize Right ToF");

  Serial.println("All ToF sensors initialized!");
}

void loop() {
  // Get distances from each sensor
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

  delay(500);
}
