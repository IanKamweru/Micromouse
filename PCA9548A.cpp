/*
 * Author: Ian Kamweru
 * Description: Implementation of PCA9548A multiplexer functions. Allows selection
 * of individual I2C channels.
 */

#include "PCA9548A.h"

// Selects the specified I2C channel on the multiplexer
void selectChannel(uint8_t channel) {
  if (channel > 7) return;  // Ensure channel is within valid range (0-7)
  Wire.beginTransmission(PCA9548A_ADDR);  // Start I2C communication
  Wire.write(1 << channel);  // Send command to activate the desired channel
  Wire.endTransmission();  // End I2C communication
}
