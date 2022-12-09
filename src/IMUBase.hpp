#ifndef _F_IMUBase_H_
#define _F_IMUBase_H_

#include <Wire.h>
#include "Arduino.h"

typedef struct AccelData {
	float accelX;
	float accelY;
	float accelZ;
};

typedef struct GyroData {
	float gyroX;
	float gyroY;
	float gyroZ;
};

typedef struct MagData {
	float magX;
	float magY;
	float magZ;
};

typedef struct Quaternion {
	float qW;
	float qX;
	float qY;
	float qZ;
};

typedef struct calData {
	bool valid;
	float accelBias[3];
	float gyroBias[3];
	float magBias[3];
	float magScale[3];
};

class IMUBase {
public:
	virtual ~IMUBase() {}

	virtual int init(calData cal, uint8_t address) = 0;
	virtual void update() = 0;

	virtual void getAccel(AccelData* out) = 0;
	virtual void getGyro(GyroData* out) = 0;
	virtual void getMag(MagData* out) = 0;
	virtual void getQuat(float* out) = 0;
	virtual float getTemp() = 0;

	virtual int setGyroRange(uint8_t range) = 0;
	virtual int setAccelRange(uint8_t range) = 0;

	virtual void calibrateAccelGyro(float* out_accelBias, float* out_gyroBias) = 0;
	virtual void calibrateMag(float* out_magBias, float* out_magScale) = 0;

	virtual bool hasMagnetometer() {
		return false;
	}
	virtual bool hasTemperature() {
		return false;
	}
	virtual bool hasQuatOutput() {
		return false;
	}
};

#endif /* _F_IMUBase_H_ */