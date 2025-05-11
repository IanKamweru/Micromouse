/*
 * Author: Ian Kamweru
 * Description: Header file for PCA9548A I2C multiplexer management. Provides
 * functions for channel selection.
 */

#ifndef PCA9548A_H
#define PCA9548A_H

#include <Wire.h>

// I2C address of the PCA9548A multiplexer
#define PCA9548A_ADDR 0x70

// Function to select a specific I2C channel on the multiplexer
void selectChannel(uint8_t channel);

#endif
