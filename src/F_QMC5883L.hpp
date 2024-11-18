#pragma once

#ifndef _F_QMC5883L_H_
#define _F_QMC5883L_H_

#include "IMUBase.hpp"
#include "IMUUtils.hpp"
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
	explicit QMC5883L(TwoWire& wire = Wire) : wire(wire) {};

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
	float mRes = 10. * 819.2f / 32768.f;				//mRes value for full range (+-819.2 uT scaled * 10) readings (16 bit)
	float tRes = 100.f / 32768.f;			//tRes value for full range readings (16 bit)
	float temperature = 0.f;
	int geometryIndex = 0;
	
	AccelData accel = { 0 };
	GyroData gyro = { 0 };
	MagData mag = { 0 };

	calData calibration;
	uint8_t IMUAddress;

	TwoWire& wire;
};
#endif /* _F_QMC5883L_H_ */
