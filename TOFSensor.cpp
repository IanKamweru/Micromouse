/*
 * Author: Ian Kamweru
 * Description: Implementation of the TOFSensor class.
 */

#include "TOFSensor.h"

// Constructor: sets the I2C channel and label for the sensor
TOFSensor::TOFSensor(uint8_t channel, const char* label) {
  this->channel = channel;
  this->label = label;
}

// Initialize the sensor: selects the I2C channel and initializes the VL6180X
bool TOFSensor::init() {
  selectChannel(channel);  // Set the correct multiplexer channel
  delay(10);  // Allow time for the channel to switch
  return sensor.begin();  // Initialize the sensor and return success status
}

// Get the distance reading from the sensor
int TOFSensor::getDistance() {
  selectChannel(channel);  // Activate the specific channel
  delay(10);  // Allow channel to settle

  int distance = sensor.readRange();  // Read the distance value
  uint8_t status = sensor.readRangeStatus();  // Check the sensor status

  // If the status is valid, return the distance, otherwise return -1 as an error indicator
  if (status == 0) {
    return distance;
  } else {
    return -1;  // Return -1 if an error occurs
  }
}
