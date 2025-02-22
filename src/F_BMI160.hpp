#pragma once

#ifndef _F_BMI160_H_
#define _F_BMI160_H_

#include "IMUBase.hpp"
#include "IMUUtils.hpp"
/*

	BMI160 REGISTERS

*/
#define BMI160_CMD 0x7E
#define BMI160_STEP_CNT_1 0x79
#define BMI160_STEP_CNT_0 0x78
#define BMI160_OFFSET_6 0x77
#define BMI160_OFFSET_5 0x76
#define BMI160_OFFSET_4 0x75
#define BMI160_OFFSET_3 0x74
#define BMI160_OFFSET_2 0x73
#define BMI160_OFFSET_1 0x72
#define BMI160_OFFSET_0 0x71
#define BMI160_NV_CONF 0x70
#define BMI160_SELF_TEST 0x6D
#define BMI160_PMU_TRIGGER 0x6C
#define BMI160_IF_CONF 0x6B
#define BMI160_CONF 0x6A
#define BMI160_FOC_CONF 0x69
#define BMI160_INT_FLAT_1 0x68
#define BMI160_INT_FLAT_0 0x67
#define BMI160_INT_ORIENT_1 0x66
#define BMI160_INT_ORIENT_0 0x65
#define BMI160_INT_TAP_1 0x64
#define BMI160_INT_TAP_0 0x63
#define BMI160_INT_MOTION_3 0x62
#define BMI160_INT_MOTION_2 0x61
#define BMI160_INT_MOTION_1 0x60
#define BMI160_INT_MOTION_0 0x5F
#define BMI160_INT_LOWHIGH_4 0x5E
#define BMI160_INT_LOWHIGH_3 0x5D
#define BMI160_INT_LOWHIGH_2 0x5C
#define BMI160_INT_LOWHIGH_1 0x5B
#define BMI160_INT_LOWHIGH_0 0x5A
#define BMI160_INT_DATA_1 0x59
#define BMI160_INT_DATA_0 0x58
#define BMI160_INT_MAP_2 0x57
#define BMI160_INT_MAP_1 0x56
#define BMI160_INT_MAP_0 0x55
#define BMI160_INT_LATCH 0x54
#define BMI160_INT_OUT_CTRL 0x53
#define BMI160_INT_EN_2 0x52
#define BMI160_INT_EN_1 0x51
#define BMI160_INT_EN_0 0x50
#define BMI160_MAG_IF_4 0x4F
#define BMI160_MAG_IF_3 0x4E
#define BMI160_MAG_IF_2 0x4D
#define BMI160_MAG_IF_1 0x4C
#define BMI160_MAG_IF_0 0x4B
#define BMI160_FIFO_CONFIG_1 0x47
#define BMI160_FIFO_CONFIG_0 0x46
#define BMI160_FIFO_DOWNS 0x45
#define BMI160_MAG_CONF 0x44
#define BMI160_GYR_RANGE 0x43
#define BMI160_GYR_CONF 0x42
#define BMI160_ACC_RANGE 0x41
#define BMI160_ACC_CONF 0x40
#define BMI160_FIFO_DATA 0x24
#define BMI160_FIFO_LENGTH_1 0x23
#define BMI160_FIFO_LENGTH_0 0x22
#define BMI160_TEMPERATURE_1 0x21
#define BMI160_TEMPERATURE_0 0x20
#define BMI160_INT_STATUS_3 0x1F
#define BMI160_INT_STATUS_2 0x1E
#define BMI160_INT_STATUS_1 0x1D
#define BMI160_INT_STATUS_0	0x1C
#define BMI160_STATUS 0x1B
#define BMI160_SENSOR_TIME_H 0x1A
#define BMI160_SENSOR_TIME_M 0x19
#define BMI160_SENSOR_TIME_L 0x18
#define BMI160_ACC_Z_H 0x17
#define BMI160_ACC_Z_L 0x16
#define BMI160_ACC_Y_H 0x15
#define BMI160_ACC_Y_L 0x14
#define BMI160_ACC_X_H 0x13
#define BMI160_ACC_X_L 0x12
#define BMI160_GYR_Z_H 0x11
#define BMI160_GYR_Z_L 0x10
#define BMI160_GYR_Y_H 0x0F
#define BMI160_GYR_Y_L 0x0E
#define BMI160_GYR_X_H 0x0D
#define BMI160_GYR_X_L 0x0C
#define BMI160_RHALL_H 0x0B
#define BMI160_RHALL_L 0x0A
#define BMI160_MAG_Z_H 0x09
#define BMI160_MAG_Z_L 0x08
#define BMI160_MAG_Y_H 0x07
#define BMI160_MAG_Y_L 0x06
#define BMI160_MAG_X_H 0x05
#define BMI160_MAG_X_L 0x04
#define BMI160_PMU_STATUS 0x03
#define BMI160_ERR_REG 0x02
#define BMI160_CHIP_ID 0x00
#define BMI160_CHIP_ID_DEFAULT_VALUE 0xD1
#define BMI160_DEFAULT_ADDRESS 0x69

class BMI160 : public IMUBase {
public:
	explicit BMI160(TwoWire& wire = Wire) : wire(wire) {};

	// Inherited via IMUBase
	int init(calData cal, uint8_t address = BMI160_DEFAULT_ADDRESS) override;

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
		return "BMI-160";
	}
	String IMUType() override {
		return "BMI160";
	}
	String IMUManufacturer() override {
		return "Bosch";
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

	TwoWire& wire;
};
#endif /* _F_BMI160_H_ */
