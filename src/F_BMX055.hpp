#pragma once

#ifndef _F_BMX055_H_
#define _F_BMX055_H_

#include "IMUBase.hpp"
/*

	BMX055 REGISTERS

*/

#define BMX055_ACCEL_ID       0xFA
#define BMX055_GYRO_ID        0x0F
#define BMX055_MAG_ID         0x32

#define BMX055_ACCD_CHIPID    0x00
#define BMX055_ACCD_X_LSB    0x02
#define BMX055_ACCD_X_MSB    0x03
#define BMX055_ACCD_Y_LSB    0x04
#define BMX055_ACCD_Y_MSB    0x05
#define BMX055_ACCD_Z_LSB    0x06
#define BMX055_ACCD_Z_MSB    0x07
#define BMX055_ACCD_TEMP     0x08
#define BMX055_PMU_RANGE     0x0F
#define BMX055_PMU_BW        0x10
#define BMX055_PMU_LPW       0x11
#define BMX055_BGW_SOFTRESET 0x14

#define BMX055_GYR_CHIP_ID       0x00
#define BMX055_GYR_RATE_X_LSB    0x02
#define BMX055_GYR_RATE_X_MSB    0x03
#define BMX055_GYR_RATE_Y_LSB    0x04
#define BMX055_GYR_RATE_Y_MSB    0x05
#define BMX055_GYR_RATE_Z_LSB    0x06
#define BMX055_GYR_RATE_Z_MSB    0x07
#define BMX055_GYR_RANGE         0x0F
#define BMX055_GYR_BW            0x10
#define BMX055_GYR_LPM1          0x11
#define BMX055_GYR_BGW_SOFTRESET 0x14

#define BMX055_MAG_CHIP_ID			 0x40
#define BMX055_MAG_X_LSB			 0x42
#define BMX055_MAG_X_MSB			 0x43
#define BMX055_MAG_Y_LSB			 0x44
#define BMX055_MAG_Y_MSB			 0x45
#define BMX055_MAG_Z_LSB			 0x46
#define BMX055_MAG_Z_MSB			 0x47
#define BMX055_MAG_RHALL_LSB_DRDY	 0x48
#define BMX055_MAG_RHALL_MSB_DRDY    0x49
#define BMX055_MAG_INT_STATUS		 0x4A
#define BMX055_MAG_PWR_CTRL_SR		 0x4B
#define BMX055_MAG_OM_ODR_SELF		 0x4C
#define BMX055_MAG_INT_REG_0		 0x4D
#define BMX055_MAG_INT_REG_1		 0x4E
#define BMX055_MAG_LOW_TH_INT		 0x4F
#define BMX055_MAG_HIGH_TH_INT		 0x50
#define BMX055_MAG_REP_CTRL_XY		 0x51
#define BMX055_MAG_REP_CTRL_Z		 0x52


class BMX055 : public IMUBase {
public:
	BMX055() {};

	// Inherited via IMUBase
	int init(calData cal, uint8_t address) override;

	void update() override;
	void getAccel(AccelData* out) override;
	void getGyro(GyroData* out) override;
	void getMag(MagData* out) override;
	void getQuat(Quaternion* out) override {};
	float getTemp() override { return temperature; };

	int setGyroRange(int range) override;
	int setAccelRange(int range) override;
	int setIMUGeometry(int index) override { geometryIndex = index; return 0; };

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
		return "BMX-055";
	}
	String IMUType() override {
		return "BMX055";
	}
	String IMUManufacturer() override {
		return "Bosch";
	}

private:
	float aRes = 16.f / 2048.f;				//ares value for full range (16g) readings (12 bit)
	float gRes = 2000.f / 32768.f;			//gres value for full range (+-2000dps) readings
	float mResXY = 1300.f / 4096.f;			//mres value for full range (+-1300uT) 13 bit readings
	float mResZ = 2500.f / 16384.f;			//mres value for full range (+-2500uT) 15 bit readings
	int geometryIndex = 0;

	float temperature = 0.f;
	AccelData accel = { 0 };
	GyroData gyro = { 0 };
	MagData mag = { 0 };

	calData calibration;

	uint8_t AccelAddress;
	uint8_t GyroAddress;
	uint8_t MagAddress;


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
#endif /* _F_BMX055_H_ */
