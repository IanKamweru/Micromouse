/*
 * Author: Ian Kamweru
 * Description: Header file for Time of Flight sensor management. Defines the TOFSensor class,
 * which handles initialization and distance retrieval from VL6180X sensors.
 */

#ifndef TOFSENSOR_H
#define TOFSENSOR_H

#include <Adafruit_VL6180X.h>
#include "PCA9548A.h"

class TOFSensor {
public:
  // Constructor: Initializes the ToF sensor with a channel and label
  TOFSensor(uint8_t channel, const char* label);

  // Initialize the ToF sensor, returns true if successful
  bool init();

  // Get the current distance measured by the sensor
  int getDistance();

private:
  uint8_t channel;        // I2C multiplexer channel for this sensor
  const char* label;      // Descriptive name for this sensor
  Adafruit_VL6180X sensor;  // ToF sensor object
};

#endif
