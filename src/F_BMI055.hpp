#pragma once

#ifndef _F_BMI055_H_
#define _F_BMI055_H_

#include "IMUBase.hpp"
#include "IMUUtils.hpp"
/*

	BMI055 REGISTERS

*/

#define BMI055_ACCEL_ID       0xFA
#define BMI055_GYRO_ID        0x0F

#define BMI055_ACCD_CHIPID    0x00
#define BMI055_ACCD_X_LSB    0x02
#define BMI055_ACCD_X_MSB    0x03
#define BMI055_ACCD_Y_LSB    0x04
#define BMI055_ACCD_Y_MSB    0x05
#define BMI055_ACCD_Z_LSB    0x06
#define BMI055_ACCD_Z_MSB    0x07
#define BMI055_ACCD_TEMP     0x08
#define BMI055_PMU_RANGE     0x0F
#define BMI055_PMU_BW        0x10
#define BMI055_PMU_LPW       0x11
#define BMI055_BGW_SOFTRESET 0x14

#define BMI055_GYR_CHIP_ID       0x00
#define BMI055_GYR_RATE_X_LSB    0x02
#define BMI055_GYR_RATE_X_MSB    0x03
#define BMI055_GYR_RATE_Y_LSB    0x04
#define BMI055_GYR_RATE_Y_MSB    0x05
#define BMI055_GYR_RATE_Z_LSB    0x06
#define BMI055_GYR_RATE_Z_MSB    0x07
#define BMI055_GYR_RANGE         0x0F
#define BMI055_GYR_BW            0x10
#define BMI055_GYR_LPM1          0x11
#define BMI055_GYR_BGW_SOFTRESET 0x14


class BMI055 : public IMUBase {
public:
	explicit BMI055(TwoWire& wire = Wire) : wire(wire) {};

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
	void calibrateMag(calData* cal) override {};

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
		return "BMI-055";
	}
	String IMUType() override {
		return "BMI055";
	}
	String IMUManufacturer() override {
		return "Bosch";
	}

private:
	float aRes = 16.f / 2048.f;				//ares value for full range (16g) readings (12 bit)
	float gRes = 2000.f / 32768.f;			//gres value for full range (2000dps) readings
	int geometryIndex = 0;

	float temperature = 0.f;
	AccelData accel = { 0 };
	GyroData gyro = { 0 };

	calData calibration;

	uint8_t AccelAddress;
	uint8_t GyroAddress;

	TwoWire& wire;
};
#endif /* _F_BMI055_H_ */
