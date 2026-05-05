#pragma once

#ifndef _F_IMUUtils_H_
#define _F_IMUUtils_H_

#include <Wire.h>
#include <Arduino.h>

// Declare the functions
void writeByteI2C(TwoWire& wire, uint8_t address, uint8_t subAddress, uint8_t data);
uint8_t readByteI2C(TwoWire& wire, uint8_t address, uint8_t subAddress);
void readBytesI2C(TwoWire& wire, uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest);
void rmwByteI2C(TwoWire& wire, uint8_t address, uint8_t subAddress, uint8_t mask, uint8_t data);

// Geometry rotation table (8 orientations).
// Each entry encodes three output axes as signed 1-indexed source axes:
//   positive n → output = v[n-1],  negative n → output = -v[-n-1]
extern const int8_t GEO_MAP[8][3];

// Apply one entry from GEO_MAP to a 3-element float array.
float applyGeo(int8_t m, const float* v);

#endif