#pragma once

#ifndef _F_QMC5883L_H_
#define _F_QMC5883L_H_

#include "IMUBase.hpp"
/*

	QMC5883L REGISTERS

*/

#define QMC5883L_X_LSB		0x00
#define QMC5883L_X_MSB		0x01
#define QMC5883L_Y_LSB		0x02
#define QMC5883L_Y_MSB		0x03
#define QMC5883L_Z_LSB		0x04
#define QMC5883L_Z_MSB		0x05
#define QMC5883L_STATUS     0x06
#define QMC5883L_T_LSB		0x07
#define QMC5883L_T_MSB      0x08
#define QMC5883L_CTRL       0x09
#define QMC5883L_RESET1		0x0A
#define QMC5883L_RESET2		0x0B
#define QMC5883L_WHOAMI		0x0D

#define QMC5883L_WHOAMI_VALUE	0xFF

class QMC5883L : public IMUBase {
public:
	QMC5883L() {};

	// Inherited via IMUBase
	int init(calData cal, uint8_t address) override;

	void update() override;
	void getAccel(AccelData* out) override {};
	void getGyro(GyroData* out) override {};
	void getMag(MagData* out) override;
	void getQuat(Quaternion* out) override {};
	float getTemp() override { return temperature; };

	void calibrateAccelGyro(calData* cal) override {};
	void calibrateMag(calData* cal) override;
	int setGyroRange(int range) override { return -1; };
	int setAccelRange(int range) override { return -1; };
	int setIMUGeometry(int index) override { geometryIndex = index; return 0; };

	bool hasMagnetometer() override {
		return true;
	}
	bool hasTemperature() override {
		return true;
	}
	bool hasQuatOutput() override {
		return false;
	}

	String IMUName() override {
		return "QMC5883L";
	}
	String IMUType() override {
		return "QMC5883L";
	}
	String IMUManufacturer() override {
		return "QST";
	}

private:
	float mRes = 8.f / 32768.f;				//mRes value for full range (+-8 gauss) readings (16 bit)
	float tRes = 100.f / 32768.f;			//mRes value for full range (+-8 gauss) readings (16 bit)
	float temperature = 0.f;
	int geometryIndex = 0;
	
	AccelData accel = { 0 };
	GyroData gyro = { 0 };
	MagData mag = { 0 };

	calData calibration;
	uint8_t IMUAddress;

	void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
	{
		Wire.beginTransmission(address);  // Initialize the Tx buffer
		Wire.write(subAddress);           // Put slave register address in Tx buffer
		Wire.write(data);                 // Put data in Tx buffer
		Wire.endTransmission();           // Send the Tx buffer
	}

	uint8_t readByte(uint8_t address, uint8_t subAddress)
	{
		uint8_t data; 						   // `data` will store the register data
		Wire.beginTransmission(address);         // Initialize the Tx buffer
		Wire.write(subAddress);                  // Put slave register address in Tx buffer
		Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
		Wire.requestFrom(address, (uint8_t)1);  // Read one byte from slave register address
		data = Wire.read();                      // Fill Rx buffer with result
		return data;                             // Return data read from slave register
	}

	void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest)
	{
		Wire.beginTransmission(address);   // Initialize the Tx buffer
		Wire.write(subAddress);            // Put slave register address in Tx buffer
		Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
		uint8_t i = 0;
		Wire.requestFrom(address, count);  // Read bytes from slave register address
		while (Wire.available()) {
			dest[i++] = Wire.read();
		}         // Put read results in the Rx buffer
	}
};
#endif /* _F_QMC5883L_H_ */
