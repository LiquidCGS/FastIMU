#pragma once

#ifndef _F_BMM150_H_
#define _F_BMM150_H_

#include "../IMUBase.hpp"
#include "../IMUUtils.hpp"

/*

	BMM150 REGISTERS

*/
#define BMM150_WHOAMI       0x40
#define BMM150_DATAX        0x42
#define BMM150_CTRL         0x4A
#define BMM150_PWR          0x4B
#define BMM150_CONFIG       0x4C
#define BMM150_INT2         0x4D
#define BMM150_INT1         0x4E
#define BMM150_LOWT         0x4F
#define BMM150_HIGHT        0x50
#define BMM150_REPXY        0x51
#define BMM150_REPZ         0x52
#define BMM150_TRIM_X1Y1    0x5D
#define BMM150_TRIM_Z4      0x62
#define BMM150_TRIM_Z2      0x68

#define BMM150_WHOAMI_VALUE 0x32

/* Overflow in raw ADC counts */
#define BMM150_OVF_XY       ((int16_t)-4096)
#define BMM150_OVF_Z        ((int16_t)-16384)

struct BMM150TrimData {
    int8_t   digX1,  digY1;
    int8_t   digX2,  digY2;
    int8_t   digXY2;
    uint8_t  digXY1;
    uint16_t digZ1,  digXYZ1;
    int16_t  digZ2,  digZ3,  digZ4;
};

class BMM150 : public IMUBase {
public:
	explicit BMM150(TwoWire& wire = Wire) : wire(wire) {};

	// Inherited via IMUBase
	int init(calData cal, uint8_t address) override;

	void update() override;
	void getAccel(AccelData* out) override {};
	void getGyro(GyroData* out) override {};
	void getMag(MagData* out) override;
	void getQuat(Quaternion* out) override {};
	float getTemp() override { return 0.f; };

	void calibrateAccelGyro(calData* cal) override {};
	void calibrateMag(calData* cal) override;
	int setGyroRange(int range) override { return -1; };
	int setAccelRange(int range) override { return -1; };
	int setIMUGeometry(int index) override { geometryIndex = index; return 0; };

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
		return "BMM-150";
	}
	String IMUType() override {
		return "BMM150";
	}
	String IMUManufacturer() override {
		return "Bosch";
	}

private:
	int geometryIndex = 0;
	MagData mag = { 0 };
	BMM150TrimData trim = {};

	calData calibration;
	uint8_t IMUAddress;
	TwoWire& wire;

	void loadTrim();
	float compensateX(int16_t raw, uint16_t rhall);
	float compensateY(int16_t raw, uint16_t rhall);
	float compensateZ(int16_t raw, uint16_t rhall);
};
#endif /* _F_BMM150_H_ */
