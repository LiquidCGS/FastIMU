#pragma once

#ifndef _F_QMI8658_H_
#define _F_QMI8658_H_

#include "IMUBase.hpp"
/*

	QMI8658 REGISTERS

*/
#define QMI8658_RESET 0x60
#define QMI8658_GZ_H 0x40
#define QMI8658_GZ_L 0x3F
#define QMI8658_GY_H 0x3E
#define QMI8658_GY_L 0x3D
#define QMI8658_GX_H 0x3C
#define QMI8658_GX_L 0x3B
#define QMI8658_AZ_H 0x3A
#define QMI8658_AZ_L 0x39
#define QMI8658_AY_H 0x38
#define QMI8658_AY_L 0x37
#define QMI8658_AX_H 0x36
#define QMI8658_AX_L 0x35
#define QMI8658_TEMP_H 0x34
#define QMI8658_TEMP_L 0x33
#define QMI8658_TIMESTAMP_HIGH 0x32
#define QMI8658_TIMESTAMP_MID 0x31
#define QMI8658_TIMESTAMP_LOW	0x30
#define QMI8658_STATUS1 0x2F
#define QMI8658_STATUS0 0x2E
#define QMI8658_STATUSINT 0x2D
#define QMI8658_I2CM_STATUS 0x2C
#define QMI8658_FIFO_DATA 0x17
#define QMI8658_FIFO_STATUS 0x16
#define QMI8658_FIFO_SMPL_CNT 0x15
#define QMI8658_FIFO_CTRL 0x14
#define QMI8658_FIFO_WTM_TH 0x13
#define QMI8658_CAL4_H 0x12
#define QMI8658_CAL4_L 0x11
#define QMI8658_CAL3_H 0x10
#define QMI8658_CAL3_L 0x0F
#define QMI8658_CAL2_H 0x0E
#define QMI8658_CAL2_L 0x0D
#define QMI8658_CAL1_H 0x0C
#define QMI8658_CAL1_L 0x0B
#define QMI8658_CTRL9 0x0A
#define QMI8658_CTRL8 0x09
#define QMI8658_CTRL7 0x08
#define QMI8658_CTRL6 0x07
#define QMI8658_CTRL5 0x06
#define QMI8658_CTRL4 0x05
#define QMI8658_CTRL3 0x04
#define QMI8658_CTRL2 0x03
#define QMI8658_CTRL1 0x02
#define QMI8658_REVISION_ID 0x01
#define QMI8658_WHO_AM_I 0x00
#define QMI8658_WHO_AM_I_DEFAULT_VALUE 0x05

class QMI8658 : public IMUBase {
public:
	QMI8658() {};

	// Inherited via IMUBase
	int init(calData cal, uint8_t address) override;

	void update() override;
	void getAccel(AccelData* out) override;
	void getGyro(GyroData* out) override;
	void getMag(MagData* out) override {};
	void getQuat(Quaternion* out) override {};
	float getTemp() override { return temperature; };

	int setGyroRange(int range) override;
	int setAccelRange(int range) override;
	int setIMUGeometry(int index) override { geometryIndex = index; return 0; };

	void calibrateAccelGyro(calData* cal) override;
	virtual void calibrateMag(calData* cal) override {};

	bool hasMagnetometer() override {
		return false;
	}
	bool hasTemperature() override {
		return true;
	}
	bool hasQuatOutput() override {
		return false;
	}

	String IMUName() override {
		return "QMI-8658";
	}
	String IMUType() override {
		return "QMI8658";
	}
	String IMUManufacturer() override {
		return "QST";
	}

private:
	float aRes = 16.0 / 32768.0;			//ares value for full range (16g) readings
	float gRes = 2000.0 / 32768.0;			//gres value for full range (2000dps) readings
	int geometryIndex = 0;

	float temperature = 0.f;
	AccelData accel = { 0 };
	GyroData gyro = { 0 };

	calData calibration;
	uint8_t IMUAddress;

	bool dataAvailable(){ return (readByte(IMUAddress, QMI8658_STATUS0) & 0x03);}

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
#endif /* _F_QMI8658_H_ */