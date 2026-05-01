#pragma once

#ifndef _F_ICM20948_H_
#define _F_ICM20948_H_

#include "../IMUBase.hpp"
#include "../IMUUtils.hpp"

/*

	ICM20948 REGISTERS Bank 0

*/
#define ICM20948_WHO_AM_I               0x00
#define ICM20948_USER_CTRL              0x03
#define ICM20948_LP_CONFIG              0x05
#define ICM20948_PWR_MGMT_1             0x06
#define ICM20948_PWR_MGMT_2             0x07
#define ICM20948_INT_PIN_CFG            0x0F
#define ICM20948_INT_ENABLE             0x10
#define ICM20948_INT_ENABLE_1           0x11
#define ICM20948_INT_ENABLE_2           0x12
#define ICM20948_INT_ENABLE_3           0x13
#define ICM20948_INT_STATUS             0x19
#define ICM20948_INT_STATUS_1           0x1A
#define ICM20948_INT_STATUS_2           0x1B
#define ICM20948_INT_STATUS_3           0x1C
#define ICM20948_ACCEL_XOUT_H           0x2D
#define ICM20948_ACCEL_XOUT_L           0x2E
#define ICM20948_ACCEL_YOUT_H           0x2F
#define ICM20948_ACCEL_YOUT_L           0x30
#define ICM20948_ACCEL_ZOUT_H           0x31
#define ICM20948_ACCEL_ZOUT_L           0x32
#define ICM20948_GYRO_XOUT_H            0x33
#define ICM20948_GYRO_XOUT_L            0x34
#define ICM20948_GYRO_YOUT_H            0x35
#define ICM20948_GYRO_YOUT_L            0x36
#define ICM20948_GYRO_ZOUT_H            0x37
#define ICM20948_GYRO_ZOUT_L            0x38
#define ICM20948_TEMP_OUT_H             0x39
#define ICM20948_TEMP_OUT_L             0x3A
#define ICM20948_EXT_SLV_SENS_DATA_00   0x3B
#define ICM20948_FIFO_EN_1              0x66
#define ICM20948_FIFO_EN_2              0x67
#define ICM20948_FIFO_RST               0x68
#define ICM20948_FIFO_MODE              0x69
#define ICM20948_FIFO_COUNTH            0x70
#define ICM20948_FIFO_COUNTL            0x71
#define ICM20948_FIFO_R_W               0x72
#define ICM20948_DATA_RDY_STATUS        0x74
#define ICM20948_FIFO_CFG               0x76
#define ICM20948_REG_BANK_SEL           0x7F

/*

	ICM20948 REGISTERS Bank 2

*/
#define ICM20948_GYRO_SMPLRT_DIV    0x00
#define ICM20948_GYRO_CONFIG_1      0x01
#define ICM20948_GYRO_CONFIG_2      0x02
#define ICM20948_ACCEL_SMPLRT_DIV_1 0x10
#define ICM20948_ACCEL_SMPLRT_DIV_2 0x11
#define ICM20948_ACCEL_CONFIG       0x14
#define ICM20948_ACCEL_CONFIG_2     0x15

/*

	ICM20948 REGISTERS Bank 3

*/
#define ICM20948_I2C_MST_CTRL       0x01
#define ICM20948_I2C_SLV0_ADDR      0x03
#define ICM20948_I2C_SLV0_REG       0x04
#define ICM20948_I2C_SLV0_CTRL      0x05
#define ICM20948_I2C_SLV4_ADDR      0x13
#define ICM20948_I2C_SLV4_REG       0x14
#define ICM20948_I2C_SLV4_CTRL      0x15
#define ICM20948_I2C_SLV4_DO        0x16
#define ICM20948_I2C_SLV4_DI        0x17

// AK09916
#define AK09916_ADDRESS     0x0C
#define AK09916_WIA2        0x01
#define AK09916_HXL         0x11
#define AK09916_CNTL2       0x31
#define AK09916_CNTL3       0x32

#define ICM20948_WHOAMI_DEFAULT_VALUE   0xEA
#define ICM20948_DEFAULT_ADDRESS        0x68

class ICM20948 : public IMUBase {
public:
	explicit ICM20948(TwoWire& wire = Wire) : wire(wire) {};

	// Inherited via IMUBase
	int init(calData cal, uint8_t address = ICM20948_DEFAULT_ADDRESS) override;

	void update() override;
	void getAccel(AccelData* out) override;
	void getGyro(GyroData* out) override;
	void getMag(MagData* out) override;
	void getQuat(Quaternion* out) override {};
	float getTemp() override { return temperature; };

	int setGyroRange(int range) override;
	int setAccelRange(int range) override;
	int setIMUGeometry(int index) override {
		geometryIndex = index;
		return 0;
	};

	void calibrateAccelGyro(calData* cal) override;
	void calibrateMag(calData* cal) override;

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
		return "ICM-20948";
	}
	String IMUType() override {
		return "ICM20948";
	}
	String IMUManufacturer() override {
		return "InvenSense";
	}

private:
	float aRes = 16.0f / 32768.0f;
	float gRes = 2000.0f / 32768.0f;
	float mRes = 4912.f / 32760.0f;
	int geometryIndex = 0;

	float temperature = 0.f;
	AccelData accel = { 0 };
	GyroData gyro = { 0 };
	MagData mag = { 0 };
	bool magInitialized = false;

	calData calibration;
	uint8_t IMUAddress;

	TwoWire& wire;

	void selectBank(uint8_t bank) {
		writeByteI2C(wire, IMUAddress, ICM20948_REG_BANK_SEL, bank << 4);
	}

	// Write one byte to an AK09916 register via the ICM20948
	void writeAK(uint8_t reg, uint8_t val) {
		selectBank(3);
		writeByteI2C(wire, IMUAddress, ICM20948_I2C_SLV4_ADDR, AK09916_ADDRESS);
		writeByteI2C(wire, IMUAddress, ICM20948_I2C_SLV4_DO,   val);
		writeByteI2C(wire, IMUAddress, ICM20948_I2C_SLV4_REG,  reg);
		writeByteI2C(wire, IMUAddress, ICM20948_I2C_SLV4_CTRL, 0x80);
		uint32_t t = millis();
		while ((readByteI2C(wire, IMUAddress, ICM20948_I2C_SLV4_CTRL) & 0x80) && (millis() - t < 100)) {}
		selectBank(0);
	}

	// Read one byte from an AK09916 register via the ICM20948
	uint8_t readAK(uint8_t reg) {
		selectBank(3);
		writeByteI2C(wire, IMUAddress, ICM20948_I2C_SLV4_ADDR, 0x80 | AK09916_ADDRESS);
		writeByteI2C(wire, IMUAddress, ICM20948_I2C_SLV4_REG,  reg);
		writeByteI2C(wire, IMUAddress, ICM20948_I2C_SLV4_CTRL, 0x80);
		uint32_t t = millis();
		while ((readByteI2C(wire, IMUAddress, ICM20948_I2C_SLV4_CTRL) & 0x80) && (millis() - t < 100)) {}
		uint8_t result = readByteI2C(wire, IMUAddress, ICM20948_I2C_SLV4_DI);
		selectBank(0);
		return result;
	}

};
#endif /* _F_ICM20948_H_ */
