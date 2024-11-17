#pragma once

#ifndef _F_HMC5883L_H_
#define _F_HMC5883L_H_

#include "IMUBase.hpp"
/*

	HMC5883L REGISTERS

*/

#define HMC5883L_CFGA		0x00
#define HMC5883L_CFGB		0x01
#define HMC5883L_MODE		0x02
#define HMC5883L_X_MSB		0x03
#define HMC5883L_X_LSB		0x04
#define HMC5883L_Z_MSB		0x05
#define HMC5883L_Z_LSB      0x06
#define HMC5883L_Y_MSB		0x07
#define HMC5883L_Y_LSB      0x08
#define HMC5883L_STATUS     0x09
#define HMC5883L_IDA		0x0A
#define HMC5883L_IDB		0x0B
#define HMC5883L_IDC		0x0C

#define HMC5883L_WHOAMI_VALUE	0x33

class HMC5883L : public IMUBase {
public:
	explicit HMC5883L(TwoWire& wire = Wire) : wire(wire) {};

	// Inherited via IMUBase
	int init(calData cal, uint8_t address) override;

	void update() override;
	void getAccel(AccelData* out) override {};
	void getGyro(GyroData* out) override {};
	void getMag(MagData* out) override;
	void getQuat(Quaternion* out) override {};
	float getTemp() override { return -1; };

	void calibrateAccelGyro(calData* cal) override {};
	void calibrateMag(calData* cal) override;
	int setGyroRange(int range) override { return -1; };
	int setAccelRange(int range) override { return -1; };
	int setIMUGeometry(int index) override { geometryIndex = index; return 0; };

	bool hasMagnetometer() override {
		return true;
	}
	bool hasTemperature() override {
		return false;
	}
	bool hasQuatOutput() override {
		return false;
	}

	String IMUName() override {
		return "HMC5883L";
	}
	String IMUType() override {
		return "HMC5883L";
	}
	String IMUManufacturer() override {
		return "Bosch";
	}

private:
	float mRes = 10. * 819.2f / 2048.f;				//mRes value for full range (+-819.2uT scaled * 10) readings (12 bit)
	int geometryIndex = 0;
	MagData mag = { 0 };

	calData calibration;
	uint8_t IMUAddress;

	TwoWire& wire;

	void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
	{
		wire.beginTransmission(address);  // Initialize the Tx buffer
		wire.write(subAddress);           // Put slave register address in Tx buffer
		wire.write(data);                 // Put data in Tx buffer
		wire.endTransmission();           // Send the Tx buffer
	}

	uint8_t readByte(uint8_t address, uint8_t subAddress)
	{
		uint8_t data; 						   // `data` will store the register data
		wire.beginTransmission(address);         // Initialize the Tx buffer
		wire.write(subAddress);                  // Put slave register address in Tx buffer
		wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
		wire.requestFrom(address, (uint8_t)1);  // Read one byte from slave register address
		data = wire.read();                      // Fill Rx buffer with result
		return data;                             // Return data read from slave register
	}

	void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest)
	{
		wire.beginTransmission(address);   // Initialize the Tx buffer
		wire.write(subAddress);            // Put slave register address in Tx buffer
		wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
		uint8_t i = 0;
		wire.requestFrom(address, count);  // Read bytes from slave register address
		while (wire.available()) {
			dest[i++] = wire.read();
		}         // Put read results in the Rx buffer
	}
};
#endif /* _F_HMC5883L_H_ */
