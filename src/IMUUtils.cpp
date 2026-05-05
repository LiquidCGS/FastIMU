#include "IMUUtils.hpp"

void writeByteI2C(TwoWire& wire, uint8_t address, uint8_t subAddress, uint8_t data) {
    wire.beginTransmission(address);  // Initialize the Tx buffer
    wire.write(subAddress);           // Put slave register address in Tx buffer
    wire.write(data);                 // Put data in Tx buffer
    wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByteI2C(TwoWire& wire, uint8_t address, uint8_t subAddress) {
    uint8_t data;                           // `data` will store the register data
    wire.beginTransmission(address);        // Initialize the Tx buffer
    wire.write(subAddress);                 // Put slave register address in Tx buffer
    wire.endTransmission(false);            // Send the Tx buffer, but send a restart to keep connection alive
    wire.requestFrom(address, (uint8_t)1);  // Read one byte from slave register address
    data = wire.read();                     // Fill Rx buffer with result
    return data;                            // Return data read from slave register
}

void readBytesI2C(TwoWire& wire, uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest) {
    wire.beginTransmission(address);  // Initialize the Tx buffer
    wire.write(subAddress);           // Put slave register address in Tx buffer
    wire.endTransmission(false);      // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    wire.requestFrom(address, count); // Read bytes from slave register address
    while (wire.available()) {
        dest[i++] = wire.read();      // Put read results in the Rx buffer
    }
}

void rmwByteI2C(TwoWire& wire, uint8_t address, uint8_t subAddress, uint8_t mask, uint8_t data) {
    uint8_t current = readByteI2C(wire, address, subAddress);
    uint8_t modified = (current & ~mask) | (data & mask);
    writeByteI2C(wire, address, subAddress, modified);
}

const int8_t GEO_MAP[8][3] = {
    { 1,  2,  3},  // 0: x=+a0, y=+a1, z=+a2
    {-2,  1,  3},  // 1: x=-a1, y=+a0, z=+a2
    {-1, -2,  3},  // 2: x=-a0, y=-a1, z=+a2
    { 2, -1,  3},  // 3: x=+a1, y=-a0, z=+a2
    {-3, -2, -1},  // 4: x=-a2, y=-a1, z=-a0
    {-3,  1, -2},  // 5: x=-a2, y=+a0, z=-a1
    {-3,  2,  1},  // 6: x=-a2, y=+a1, z=+a0
    {-3, -1,  2},  // 7: x=-a2, y=-a0, z=+a1
};

float applyGeo(int8_t m, const float* v) {
    return m > 0 ? v[m - 1] : -v[-m - 1];
}