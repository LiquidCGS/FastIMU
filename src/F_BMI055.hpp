#pragma once

#ifndef _F_BMI055_H_
#define _F_BMI055_H_

#include "IMUBase.hpp"
/*

	BMI055 REGISTERS

*/

#define BMX055_ACCEL_ID       0xFA
#define BMX055_GYRO_ID        0x0F

#define BMI055_ACCD_CHIPID    0x00
#define BMI055_ACCD_X_LSB    0x02
#define BMI055_ACCD_X_MSB    0x03
#define BMI055_ACCD_Y_LSB    0x04
#define BMI055_ACCD_Y_MSB    0x05
#define BMI055_ACCD_Z_LSB    0x06
#define BMI055_ACCD_Z_MSB    0x07
#define BMI055_ACCD_TEMP     0x08
#define BMI055_INT_STATUS_0  0x09
#define BMI055_INT_STATUS_1  0x0A
#define BMI055_INT_STATUS_2  0x0B
#define BMI055_INT_STATUS_3  0x0C
#define BMI055_FIFO_STATUS   0x0E
#define BMI055_PMU_RANGE     0x0F
#define BMI055_PMU_BW        0x10
#define BMI055_PMU_LPW       0x11
#define BMI055_PMU_LOW_POWER 0x12
#define BMI055_ACCD_HBW      0x13
#define BMI055_BGW_SOFTRESET 0x14
#define BMI055_INT_EN_0      0x16
#define BMI055_INT_EN_1      0x17
#define BMI055_INT_EN_2      0x18
#define BMI055_INT_MAP_0     0x19
#define BMI055_INT_MAP_1     0x1A
#define BMI055_INT_MAP_2     0x1B
#define BMI055_INT_SRC       0x1E
#define BMI055_INT_OUT_CTRL  0x20
#define BMI055_INT_RST_LATCH 0x21
#define BMI055_INT_0         0x22
#define BMI055_INT_1         0x23
#define BMI055_FIFO_CONFIG_0 0x30
#define BMI055_PMU_SELF_TEST 0x32
#define BMI055_TRIM_NVM_CTRL 0x33
#define BMI055_BGW_SPI3_WDT  0x34
#define BMI055_OFC_CTRL      0x36
#define BMI055_OFC_SETTING   0x37
#define BMI055_OFC_OFFSET_X  0x38
#define BMI055_OFC_OFFSET_Y  0x39
#define BMI055_OFC_OFFSET_Z  0x3A
#define BMI055_TRIM_GP0      0x3B
#define BMI055_TRIM_GP1      0x3C
#define BMI055_FIFO_CONFIG_1 0x3E
#define BMI055_FIFO_DATA     0x3F

#define BMI055_GYR_CHIP_ID       0x00
#define BMI055_GYR_RATE_X_LSB    0x02
#define BMI055_GYR_RATE_X_MSB    0x03
#define BMI055_GYR_RATE_Y_LSB    0x04
#define BMI055_GYR_RATE_Y_MSB    0x05
#define BMI055_GYR_RATE_Z_LSB    0x06
#define BMI055_GYR_RATE_Z_MSB    0x07
#define BMI055_GYR_INT_STATUS_0  0x09
#define BMI055_GYR_INT_STATUS_1  0x0A
#define BMI055_GYR_INT_STATUS_2  0x0B
#define BMI055_GYR_INT_STATUS_3  0x0C
#define BMI055_GYR_FIFO_STATUS   0x0E
#define BMI055_GYR_RANGE         0x0F
#define BMI055_GYR_BW            0x10
#define BMI055_GYR_LPM1          0x11
#define BMI055_GYR_LPM2          0x12
#define BMI055_GYR_RATE_HBW      0x13
#define BMI055_GYR_BGW_SOFTRESET 0x14
#define BMI055_GYR_INT_EN_0      0x15
#define BMI055_GYR_INT_EN_1      0x16
#define BMI055_GYR_INT_MAP_0     0x17
#define BMI055_GYR_INT_MAP_1     0x18
#define BMI055_GYR_INT_MAP_2     0x19
#define BMI055_GYR_INT_DATA_SRC  0x1A
#define BMI055_GYR_FOC_DATA_SRC  0x1B
#define BMI055_GYR_ANY_MOTION    0x1C
#define BMI055_GYR_FIFO_WATER_MK 0x1E
#define BMI055_GYR_INT_RST_LATCH 0x21
#define BMI055_GYR_HIGH_TH_X     0x22
#define BMI055_GYR_HIGH_DUR_X    0x23
#define BMI055_GYR_HIGH_TH_Y     0x24
#define BMI055_GYR_HIGH_DUR_Y    0x25
#define BMI055_GYR_HIGH_TH_Z     0x26
#define BMI055_GYR_HIGH_DUR_Z    0x27
#define BMI055_GYR_SOC           0x31
#define BMI055_GYR_A_FOC         0x32
#define BMI055_GYR_TRIM_NVM_CTRL 0x33
#define BMI055_GYR_BGW_SPI3_WDT  0x34
#define BMI055_GYR_OFC1          0x36
#define BMI055_GYR_OFC2          0x37
#define BMI055_GYR_OFC3          0x38
#define BMI055_GYR_OFC4          0x39
#define BMI055_GYR_TRIM_GP0      0x3A
#define BMI055_GYR_TRIM_GP1      0x3B
#define BMI055_GYR_BIST          0x3C
#define BMI055_GYR_FIFO_CONFIG_0 0x3D
#define BMI055_GYR_FIFO_CONFIG_1 0x3E
#define BMI055_GYR_FIFO_DATA     0x3F


class BMI055 : public IMUBase {
public:
	BMI055() {};

	// Inherited via IMUBase
	int init(calData cal, uint8_t address) override;

	void update() override;
	void getAccel(AccelData* out) override;
	void getGyro(GyroData* out) override;
	void getMag(MagData* out) override {};
	void getQuat(float* out) override {};
	float getTemp() override { return temperature; };

	void calibrateAccelGyro(float* out_accelBias, float* out_gyroBias) override;
	virtual void calibrateMag(float* out_magBias, float* out_magScale) override {};

	bool hasMagnetometer() override {
		return false;
	}
	bool hasTemperature() override {
		return true;
	}
	bool hasQuatOutput() override {
		return false;
	}

private:
	float aRes = 16.f / 32768.f;			//ares value for full range (16g) readings
	float gRes = 2000.f / 32768.f;			//gres value for full range (2000dps) readings

	float temperature = 0.f;
	AccelData accel = { 0 };
	GyroData gyro = { 0 };

	calData calibration;

	uint8_t AccelAddress;
	uint8_t GyroAddress;


	void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
	{
		Wire.beginTransmission(address);  // Initialize the Tx buffer
		Wire.write(subAddress);           // Put slave register address in Tx buffer
		Wire.write(data);                 // Put data in Tx buffer
		Wire.endTransmission();           // Send the Tx buffer
	}

	uint8_t readByte(uint8_t address, uint8_t subAddress)
	{
		uint8_t data; 						   // `data` will store the register data
		Wire.beginTransmission(address);         // Initialize the Tx buffer
		Wire.write(subAddress);                  // Put slave register address in Tx buffer
		Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
		Wire.requestFrom(address, (uint8_t)1);  // Read one byte from slave register address
		data = Wire.read();                      // Fill Rx buffer with result
		return data;                             // Return data read from slave register
	}

	void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest)
	{
		Wire.beginTransmission(address);   // Initialize the Tx buffer
		Wire.write(subAddress);            // Put slave register address in Tx buffer
		Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
		uint8_t i = 0;
		Wire.requestFrom(address, count);  // Read bytes from slave register address
		while (Wire.available()) {
			dest[i++] = Wire.read();
		}         // Put read results in the Rx buffer
	}
};
#endif /* _F_BMI055_H_ */
