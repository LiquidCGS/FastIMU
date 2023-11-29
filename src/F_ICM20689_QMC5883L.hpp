#pragma once

#ifndef _F_ICM20689_QMC5883L_H_
#define _F_ICM20689_QMC5883L_H_

#include "F_ICM20689.hpp"
#include "F_QMC5883L.hpp"

class ICM20689_QMC5883L : public IMUBase {
public:
	ICM20689_QMC5883L() {};

	// Inherited via IMUBase
	int init(calData cal, uint8_t address) override {
		int e = IMU.init(cal, address);
		if (e) { return e; }
		e = MAG.init(cal, 0x0D);
		if (e) { return -2; }
	}

	void update() override {
		IMU.update();
		MAG.update();
	}
	void getAccel(AccelData* out) override {
		IMU.getAccel(out);
	}
	void getGyro(GyroData* out) override{
		IMU.getGyro(out);
	}
	void getMag(MagData* out) override {
		MAG.getMag(out);
	}
	void getQuat(Quaternion* out) override {};
	float getTemp() override { return IMU.getTemp();; };

	int setGyroRange(int range) override	{ return IMU.setGyroRange(range); }
	int setAccelRange(int range) override	{ return IMU.setAccelRange(range); }
	int setIMUGeometry(int index) override 	{ return IMU.setIMUGeometry(index); }

	void calibrateAccelGyro(calData* cal) override {
		IMU.calibrateAccelGyro(cal);
	}
	virtual void calibrateMag(calData* cal) override {
		MAG.calibrateMag(cal);
	}

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
		return "ICM20689 + QMC5883L";
	}
	String IMUType() override {
		return "ICM20689_QMC5883L";
	}
	String IMUManufacturer() override {
		return "InvenSense + QST";
	}

private:
	ICM20689 IMU;
	QMC5883L MAG;
	
	calData calibration;
	uint8_t IMUAddress;
};
#endif /* _F_ICM20689_QMC5883L_H_ */
