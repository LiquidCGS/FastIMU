#pragma once

#ifndef _F_AK09918_H_
#define _F_AK09918_H_

#include "IMUBase.hpp"
#include "IMUUtils.hpp"
/*

	AK09918 REGISTERS

*/

//Magnetometer Registers
#define AK09918_ADDRESS   	0x0C
#define AK09918_WIA1        0x00    // Company ID
#define AK09918_WIA2        0x01    // Device ID
#define AK09918_RSV1        0x02    // Reserved 1
#define AK09918_RSV2        0x03    // Reserved 2
#define AK09918_ST1         0x10    // DataStatus 1
#define AK09918_HXL         0x11    // X-axis data 
#define AK09918_HXH         0x12
#define AK09918_HYL         0x13    // Y-axis data
#define AK09918_HYH         0x14
#define AK09918_HZL         0x15    // Z-axis data
#define AK09918_HZH         0x16
#define AK09918_TMPS        0x17    // Dummy
#define AK09918_ST2         0x18    // Datastatus 2
#define AK09918_CNTL1       0x30    // Dummy
#define AK09918_CNTL2       0x31    // Control settings
#define AK09918_CNTL3       0x32    // Control settings

// Byte values
#define AK09918_SRST_BIT    0x01    // Soft Reset
#define AK09918_HOFL_BIT    0x08    // Sensor Over Flow
#define AK09918_DOR_BIT     0x02    // Data Over Run
#define AK09918_DRDY_BIT    0x01    // Data Ready

enum AK09918_mode_type_t {
    AK09918_POWER_DOWN = 0x00,
    AK09918_NORMAL = 0x01,
    AK09918_CONTINUOUS_10HZ = 0x02,
    AK09918_CONTINUOUS_20HZ = 0x04,
    AK09918_CONTINUOUS_50HZ = 0x06,
    AK09918_CONTINUOUS_100HZ = 0x08,
    AK09918_SELF_TEST = 0x10, // ignored by switchMode() and initialize(), call selfTest() to use this mode
};

enum AK09918_err_type_t {
    AK09918_ERR_OK = 0,                 // ok
    AK09918_ERR_DOR = 1,                // data skipped
    AK09918_ERR_NOT_RDY = 2,            // not ready
    AK09918_ERR_TIMEOUT = 3,            // read/write timeout
    AK09918_ERR_SELFTEST_FAILED = 4,    // self test failed
    AK09918_ERR_OVERFLOW = 5,           // sensor overflow, means |x|+|y|+|z| >= 4912uT
    AK09918_ERR_WRITE_FAILED = 6,       // fail to write
    AK09918_ERR_READ_FAILED = 7,        // fail to read
};

class AK09918 : public IMUBase {
public:
	explicit AK09918(TwoWire& wire = Wire) : wire(wire) {};

	// Inherited via IMUBase
	int init(calData cal, uint8_t address) override;

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

	bool isDataReady();
	bool isDataSkip();

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
		return "AK09918";
	}
	String IMUType() override {
		return "AK09918";
	}
	String IMUManufacturer() override {
		return "AsahiKASEI";
	}
private:
	int geometryIndex = 0;
	float temperature = 0.f;
	MagData mag = { 0 };

	float mRes = 10. * 4912. / 32760.0;		//mres value for full range (4912uT) readings

	calData calibration;
	uint8_t IMUAddress;

	TwoWire& wire;

	bool dataAvailable()
	{ 
		return (readByteI2C(wire, AK09918_ADDRESS, AK09918_ST1) & 0x01);
	}
};
#endif /* _F_AK09918_H_ */
