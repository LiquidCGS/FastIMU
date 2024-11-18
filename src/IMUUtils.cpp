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
