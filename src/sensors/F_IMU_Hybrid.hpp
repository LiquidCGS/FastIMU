#pragma once

#ifndef _F_IMU_HYBRID_H_
#define _F_IMU_HYBRID_H_

#include <Wire.h>

template <typename IMUType, typename MAGType>
class IMU_HYBRID {
public:
	explicit IMU_HYBRID(TwoWire& wire = Wire);
	~IMU_HYBRID();

	int init(calData cal, uint8_t address = 0x00);
	void update();
	void getAccel(AccelData* out);
	void getGyro(GyroData* out);
	void getMag(MagData* out);
	void getQuat(Quaternion* out);
	float getTemp();
	int setGyroRange(int range);
	int setAccelRange(int range);
	int setIMUGeometry(int index);
	void calibrateAccelGyro(calData* cal);
	void calibrateMag(calData* cal);
	bool hasMagnetometer();
	bool hasTemperature();
	bool hasQuatOutput();
	String IMUName();
	String getIMUType();
	String IMUManufacturer();

private:
	IMUType* IMU;
	MAGType* MAG;
	TwoWire& wire;
};

template <typename IMUType, typename MAGType>
IMU_HYBRID<IMUType, MAGType>::IMU_HYBRID(TwoWire& wire)
	: wire(wire) {
	IMU = new IMUType(wire);
	MAG = new MAGType(wire);
}

template <typename IMUType, typename MAGType>
IMU_HYBRID<IMUType, MAGType>::~IMU_HYBRID() {
	delete IMU;
	delete MAG;
}

template <typename IMUType, typename MAGType>
int IMU_HYBRID<IMUType, MAGType>::init(calData cal, uint8_t address) {
	int e = IMU->init(cal);
	if (e) return e;
	e = MAG->init(cal);
	if (e) return -2;
	return 0;
}

template <typename IMUType, typename MAGType>
void IMU_HYBRID<IMUType, MAGType>::update() {
	IMU->update();
	MAG->update();
}

template <typename IMUType, typename MAGType>
void IMU_HYBRID<IMUType, MAGType>::getAccel(AccelData* out) {
	IMU->getAccel(out);
}

template <typename IMUType, typename MAGType>
void IMU_HYBRID<IMUType, MAGType>::getGyro(GyroData* out) {
	IMU->getGyro(out);
}

template <typename IMUType, typename MAGType>
void IMU_HYBRID<IMUType, MAGType>::getMag(MagData* out) {
	MAG->getMag(out);
}

template <typename IMUType, typename MAGType>
void IMU_HYBRID<IMUType, MAGType>::getQuat(Quaternion* out) {
	out->qW = 1.0f; out->qX = 0.0f; out->qY = 0.0f; out->qZ = 0.0f;
}

template <typename IMUType, typename MAGType>
float IMU_HYBRID<IMUType, MAGType>::getTemp() {
	return IMU->getTemp();
}

template <typename IMUType, typename MAGType>
int IMU_HYBRID<IMUType, MAGType>::setGyroRange(int range) {
	return IMU->setGyroRange(range);
}

template <typename IMUType, typename MAGType>
int IMU_HYBRID<IMUType, MAGType>::setAccelRange(int range) {
	return IMU->setAccelRange(range);
}

template <typename IMUType, typename MAGType>
int IMU_HYBRID<IMUType, MAGType>::setIMUGeometry(int index) {
	return IMU->setIMUGeometry(index);
}

template <typename IMUType, typename MAGType>
void IMU_HYBRID<IMUType, MAGType>::calibrateAccelGyro(calData* cal) {
	IMU->calibrateAccelGyro(cal);
}

template <typename IMUType, typename MAGType>
void IMU_HYBRID<IMUType, MAGType>::calibrateMag(calData* cal) {
	MAG->calibrateMag(cal);
}

template <typename IMUType, typename MAGType>
bool IMU_HYBRID<IMUType, MAGType>::hasMagnetometer() {
	return MAG->hasMagnetometer();
}

template <typename IMUType, typename MAGType>
bool IMU_HYBRID<IMUType, MAGType>::hasTemperature() {
	return IMU->hasTemperature();
}

template <typename IMUType, typename MAGType>
bool IMU_HYBRID<IMUType, MAGType>::hasQuatOutput() {
	return false;
}

template <typename IMUType, typename MAGType>
String IMU_HYBRID<IMUType, MAGType>::IMUName() {
	return (IMU->IMUName() + " + " + MAG->IMUName());
}

template <typename IMUType, typename MAGType>
String IMU_HYBRID<IMUType, MAGType>::getIMUType() {
	return "IMU_HYBRID";
}

template <typename IMUType, typename MAGType>
String IMU_HYBRID<IMUType, MAGType>::IMUManufacturer() {
	return (IMU->IMUManufacturer() + " + " + MAG->IMUManufacturer());
}

#endif /* _F_IMU_HYBRID_H_ */
