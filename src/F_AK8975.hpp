#pragma once

#ifndef _F_AK8975_H_
#define _F_AK8975_H_

#include "IMUBase.hpp"
#include "IMUUtils.hpp"
/*

	AK8975 REGISTERS

*/

//Magnetometer Registers
#define AK8975_ADDRESS   0x0C
#define AK8975_WHO_AM_I  0x00 // should return 0x48
#define AK8975_WHOAMI_DEFAULT_VALUE 0x48
#define AK8975_INFO      0x01
#define AK8975_ST1       0x02  // data ready status bit 0
#define AK8975_XOUT_L	 0x03  // data
#define AK8975_XOUT_H	 0x04
#define AK8975_YOUT_L	 0x05
#define AK8975_YOUT_H	 0x06
#define AK8975_ZOUT_L	 0x07
#define AK8975_ZOUT_H	 0x08
#define AK8975_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8975_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8975_ASTC      0x0C  // Self test control
#define AK8975_I2CDIS    0x0F  // I2C disable
#define AK8975_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8975_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8975_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value
#define AK8975_DEFAULT_ADDRESS 0x0C

class AK8975 : public IMUBase {
public:
	explicit AK8975(TwoWire& wire = Wire) : wire(wire) {};

	// Inherited via IMUBase
	int init(calData cal, uint8_t address = AK8975_DEFAULT_ADDRESS) override;

	void update() override;
	void getAccel(AccelData* out) override;
	void getGyro(GyroData* out) override;
	void getMag(MagData* out) override;
	void getQuat(Quaternion* out) override {};
	float getTemp() override { return 0.f; };

	int setGyroRange(int range) override;
	int setAccelRange(int range) override;
	int setIMUGeometry(int index) override { geometryIndex = index; return 0; };

	void calibrateAccelGyro(calData* cal) override;
	void calibrateMag(calData* cal) override;

	bool hasMagnetometer() override {
		return true;
	}
	bool hasTemperature() override {
		return false;
	}
	bool hasQuatOutput() override {
		return false;
	}

	String IMUName() override {
		return "AK8975";
	}
	String IMUType() override {
		return "AK8975";
	}
	String IMUManufacturer() override {
		return "AsahiKASEI";
	}
private:
	float mRes = 10. * 1229. / 8196.0;		//mres value for full range (1229uT) readings

	int geometryIndex = 0;
	float temperature = 0.f;
	MagData mag = { 0 };

	calData calibration;
	uint8_t IMUAddress;

	TwoWire& wire;

	float factoryMagCal[3] = { 0 };

	bool dataAvailable()
	{ 
		return (readByteI2C(wire, AK8975_ADDRESS, AK8975_ST1) & 0x01);
	}
};
#endif /* _F_AK8975_H_ */
