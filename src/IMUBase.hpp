#ifndef _F_IMUBase_H_
#define _F_IMUBase_H_

#include <Wire.h>
#include "Arduino.h"

struct AccelData {
	float accelX;
	float accelY;
	float accelZ;
	uint32_t timestamp;
};

struct GyroData {
	float gyroX;
	float gyroY;
	float gyroZ;
	uint32_t timestamp;
};

struct MagData {
	float magX;
	float magY;
	float magZ;
	uint32_t timestamp;
};

struct Quaternion {
	float qW;
	float qX;
	float qY;
	float qZ;
	uint32_t timestamp;
};

struct calData {
	bool valid;
	float accelBias[3];
	float gyroBias[3];
	float magBias[3];
	float magScale[3];
};

class IMUBase {
public:
	virtual ~IMUBase() {}

	virtual int init(calData cal, uint8_t address = 0x00) = 0;

	virtual void update() = 0;

	virtual void getAccel(AccelData* out) = 0;
	virtual void getGyro(GyroData* out) = 0;
	virtual void getMag(MagData* out) = 0;
	virtual void getQuat(Quaternion* out) = 0;
	virtual float getTemp() = 0;

	virtual int setGyroRange(int range) = 0;
	virtual int setAccelRange(int range) = 0;
	virtual int setIMUGeometry(int index) = 0;

	virtual int setAccelODR(int odr_hz) { return -1; }
	virtual int setGyroODR(int odr_hz)  { return -1; }
	virtual int getAccelODR()           { return -1; }
	virtual int getGyroODR()            { return -1; }

	virtual int setMagODR(int odr_hz)   { return -1; }
	virtual int getMagODR()             { return -1; }

	virtual void calibrateAccelGyro(calData* cal) = 0;
	virtual void calibrateMag(calData* cal) = 0;

	virtual bool hasMagnetometer() {
		return false;
	}
	virtual bool hasTemperature() {
		return false;
	}
	virtual bool hasQuatOutput() {
		return false;
	}

	virtual String IMUName(){
		return "Unknown";
	}
	virtual String IMUType(){
		return "Unknown";
	}
	virtual String IMUManufacturer(){
		return "Unknown";
	}

	static int nearestHigherODR(const int* table, int count, int requested) {
		for (int i = 0; i < count; i++)
			if (table[i] >= requested) return table[i];
		return table[count - 1];
	}
};

#endif /* _F_IMUBase_H_ */