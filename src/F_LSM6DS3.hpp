#pragma once

#ifndef _F_LSM6DS3_H_
#define _F_LSM6DS3_H_

#include "IMUBase.hpp"
#include "IMUUtils.hpp"
/*

	LSM6DS3 REGISTERS

*/
#define LSM6DS3_FUNC_CFG_ACCESS		0x01
#define LSM6DS3_SENSOR_SYNC_TIME_FRAME 0x04
#define LSM6DS3_FIFO_CTRL1			0x06
#define LSM6DS3_FIFO_CTRL2			0x07
#define LSM6DS3_FIFO_CTRL3			0x08
#define LSM6DS3_FIFO_CTRL4			0x09
#define LSM6DS3_FIFO_CTRL5			0x0A
#define LSM6DS3_ORIENT_CFG_G		0x0B
#define LSM6DS3_INT1_CTRL			0x0D
#define LSM6DS3_INT2_CTRL			0x0E
#define LSM6DS3_WHO_AM_I			0x0F
#define LSM6DS3_CTRL1_XL			0x10
#define LSM6DS3_CTRL2_G				0x11
#define LSM6DS3_CTRL3_C				0x12
#define LSM6DS3_CTRL4_C				0x13
#define LSM6DS3_CTRL5_C				0x14
#define LSM6DS3_CTRL6_C				0x15
#define LSM6DS3_CTRL7_G				0x16
#define LSM6DS3_CTRL8_XL			0x17
#define LSM6DS3_CTRL9_XL			0x18
#define LSM6DS3_CTRL10_C			0x19
#define LSM6DS3_MASTER_CONFIG		0x1A
#define LSM6DS3_WAKE_UP_SRC			0x1B
#define LSM6DS3_TAP_SRC				0x1C
#define LSM6DS3_D6D_SRC				0x1D
#define LSM6DS3_STATUS_REG			0x1E
#define LSM6DS3_OUT_TEMP_L			0x20
#define LSM6DS3_OUT_TEMP_H			0x21
#define LSM6DS3_OUTX_L_G			0x22
#define LSM6DS3_OUTX_H_G			0x23
#define LSM6DS3_OUTY_L_G			0x24
#define LSM6DS3_OUTY_H_G			0x25
#define LSM6DS3_OUTZ_L_G			0x26
#define LSM6DS3_OUTZ_H_G			0x27
#define LSM6DS3_OUTX_L_XL			0x28
#define LSM6DS3_OUTX_H_XL			0x29
#define LSM6DS3_OUTY_L_XL			0x2A
#define LSM6DS3_OUTY_H_XL			0x2B
#define LSM6DS3_OUTZ_L_XL			0x2C
#define LSM6DS3_OUTZ_H_XL			0x2D

#define LSM6DS3_WHOAMI_DEFAULT_VALUE	0x69
#define LSM6DS3TR_C_WHOAMI_DEFAULT_VALUE 0x6A

#define LSM6DS3_DEFAULT_ADDRESS 0x6B

class LSM6DS3 : public IMUBase {
public:
	explicit LSM6DS3(TwoWire& wire = Wire) : wire(wire) {};

	// Inherited via IMUBase
	int init(calData cal, uint8_t address = LSM6DS3_DEFAULT_ADDRESS) override;

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
		return "LSM6DS3";
	}
	String IMUType() override {
		return "LSM6DS3";
	}
	String IMUManufacturer() override {
		return "ST";
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
	
	uint8_t checkReady(uint8_t address, uint8_t timeout)
	{
		uint8_t IMUWhoAmI = 0;
		// Wait until a valid byte is returned, up until timeout value.
		for (uint8_t checkCount = timeout; checkCount > 0; checkCount--) {
			IMUWhoAmI = readByteI2C(wire, address, LSM6DS3_WHO_AM_I);
			if (IMUWhoAmI == 0xFF) { delay(1); } else { break; }
		}
		// Return IMU identifier if found.
		return IMUWhoAmI;
	}
};
#endif /* _F_LSM6DS3_H_ */
