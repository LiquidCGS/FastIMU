#pragma once

#ifndef _F_MPU6886_H_
#define _F_MPU6886_H_

#include "IMUBase.hpp"
/*

	MPU6886 REGISTERS

*/
#define MPU6886_SELF_TEST_X_GYRO 0x00
#define MPU6886_SELF_TEST_Y_GYRO 0x01
#define MPU6886_SELF_TEST_Z_GYRO 0x02

// #define MPU6886_X_FINE_GAIN      0x03 // [7:0] fine gain
// #define MPU6886_Y_FINE_GAIN      0x04
// #define MPU6886_Z_FINE_GAIN      0x05
// #define MPU6886_XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
// #define MPU6886_XA_OFFSET_L_TC   0x07
// #define MPU6886_YA_OFFSET_H      0x08
// #define MPU6886_YA_OFFSET_L_TC   0x09
// #define MPU6886_ZA_OFFSET_H      0x0A
// #define MPU6886_ZA_OFFSET_L_TC   0x0B

#define MPU6886_SELF_TEST_X_ACCEL 0x0D
#define MPU6886_SELF_TEST_Y_ACCEL 0x0E
#define MPU6886_SELF_TEST_Z_ACCEL 0x0F

#define MPU6886_SELF_TEST_A      0x10

#define MPU6886_XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define MPU6886_XG_OFFSET_L      0x14
#define MPU6886_YG_OFFSET_H      0x15
#define MPU6886_YG_OFFSET_L      0x16
#define MPU6886_ZG_OFFSET_H      0x17
#define MPU6886_ZG_OFFSET_L      0x18
#define MPU6886_SMPLRT_DIV       0x19
#define MPU6886_MPU_CONFIG       0x1A
#define MPU6886_GYRO_CONFIG      0x1B
#define MPU6886_ACCEL_CONFIG     0x1C
#define MPU6886_ACCEL_CONFIG2    0x1D
#define MPU6886_LP_ACCEL_ODR     0x1E
#define MPU6886_WOM_THR          0x1F

#define MPU6886_MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define MPU6886_ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define MPU6886_ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define MPU6886_FIFO_EN          0x23
#define MPU6886_I2C_MST_CTRL     0x24
#define MPU6886_I2C_SLV0_ADDR    0x25
#define MPU6886_I2C_SLV0_REG     0x26
#define MPU6886_I2C_SLV0_CTRL    0x27
#define MPU6886_I2C_SLV1_ADDR    0x28
#define MPU6886_I2C_SLV1_REG     0x29
#define MPU6886_I2C_SLV1_CTRL    0x2A
#define MPU6886_I2C_SLV2_ADDR    0x2B
#define MPU6886_I2C_SLV2_REG     0x2C
#define MPU6886_I2C_SLV2_CTRL    0x2D
#define MPU6886_I2C_SLV3_ADDR    0x2E
#define MPU6886_I2C_SLV3_REG     0x2F
#define MPU6886_I2C_SLV3_CTRL    0x30
#define MPU6886_I2C_SLV4_ADDR    0x31
#define MPU6886_I2C_SLV4_REG     0x32
#define MPU6886_I2C_SLV4_DO      0x33
#define MPU6886_I2C_SLV4_CTRL    0x34
#define MPU6886_I2C_SLV4_DI      0x35
#define MPU6886_I2C_MST_STATUS   0x36
#define MPU6886_INT_PIN_CFG      0x37
#define MPU6886_INT_ENABLE       0x38
#define MPU6886_DMP_INT_STATUS   0x39  // Check DMP interrupt
#define MPU6886_INT_STATUS       0x3A
#define MPU6886_ACCEL_XOUT_H     0x3B
#define MPU6886_ACCEL_XOUT_L     0x3C
#define MPU6886_ACCEL_YOUT_H     0x3D
#define MPU6886_ACCEL_YOUT_L     0x3E
#define MPU6886_ACCEL_ZOUT_H     0x3F
#define MPU6886_ACCEL_ZOUT_L     0x40
#define MPU6886_TEMP_OUT_H       0x41
#define MPU6886_TEMP_OUT_L       0x42
#define MPU6886_GYRO_XOUT_H      0x43
#define MPU6886_GYRO_XOUT_L      0x44
#define MPU6886_GYRO_YOUT_H      0x45
#define MPU6886_GYRO_YOUT_L      0x46
#define MPU6886_GYRO_ZOUT_H      0x47
#define MPU6886_GYRO_ZOUT_L      0x48
#define MPU6886_EXT_SENS_DATA_00 0x49
#define MPU6886_EXT_SENS_DATA_01 0x4A
#define MPU6886_EXT_SENS_DATA_02 0x4B
#define MPU6886_EXT_SENS_DATA_03 0x4C
#define MPU6886_EXT_SENS_DATA_04 0x4D
#define MPU6886_EXT_SENS_DATA_05 0x4E
#define MPU6886_EXT_SENS_DATA_06 0x4F
#define MPU6886_EXT_SENS_DATA_07 0x50
#define MPU6886_EXT_SENS_DATA_08 0x51
#define MPU6886_EXT_SENS_DATA_09 0x52
#define MPU6886_EXT_SENS_DATA_10 0x53
#define MPU6886_EXT_SENS_DATA_11 0x54
#define MPU6886_EXT_SENS_DATA_12 0x55
#define MPU6886_EXT_SENS_DATA_13 0x56
#define MPU6886_EXT_SENS_DATA_14 0x57
#define MPU6886_EXT_SENS_DATA_15 0x58
#define MPU6886_EXT_SENS_DATA_16 0x59
#define MPU6886_EXT_SENS_DATA_17 0x5A
#define MPU6886_EXT_SENS_DATA_18 0x5B
#define MPU6886_EXT_SENS_DATA_19 0x5C
#define MPU6886_EXT_SENS_DATA_20 0x5D
#define MPU6886_EXT_SENS_DATA_21 0x5E
#define MPU6886_EXT_SENS_DATA_22 0x5F
#define MPU6886_EXT_SENS_DATA_23 0x60
#define MPU6886_MOT_DETECT_STATUS 0x61
#define MPU6886_I2C_SLV0_DO      0x63
#define MPU6886_I2C_SLV1_DO      0x64
#define MPU6886_I2C_SLV2_DO      0x65
#define MPU6886_I2C_SLV3_DO      0x66
#define MPU6886_I2C_MST_DELAY_CTRL 0x67
#define MPU6886_SIGNAL_PATH_RESET  0x68
#define MPU6886_MOT_DETECT_CTRL  0x69
#define MPU6886_USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define MPU6886_PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define MPU6886_PWR_MGMT_2       0x6C
#define MPU6886_DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define MPU6886_DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define MPU6886_DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define MPU6886_DMP_REG_1        0x70
#define MPU6886_DMP_REG_2        0x71
#define MPU6886_FIFO_COUNTH      0x72
#define MPU6886_FIFO_COUNTL      0x73
#define MPU6886_FIFO_R_W         0x74
#define MPU6886_WHO_AM_I_MPU6886 0x75 // Should return 0x19
#define MPU6886_WHOAMI_DEFAULT_VALUE 0x19
#define MPU6886_XA_OFFSET_H      0x77
#define MPU6886_XA_OFFSET_L      0x78
#define MPU6886_YA_OFFSET_H      0x7A
#define MPU6886_YA_OFFSET_L      0x7B
#define MPU6886_ZA_OFFSET_H      0x7D
#define MPU6886_ZA_OFFSET_L      0x7E


class MPU6886 : public IMUBase {
public:
	MPU6886() {};

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
		return "MPU-6886";
	}
	String IMUType() override {
		return "MPU6886";
	}
	String IMUManufacturer() override {
		return "InvenSense";
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

	bool dataAvailable(){ return (readByte(IMUAddress, MPU6886_INT_STATUS) & 0x01);}
};
#endif /* _F_MPU6886_H_ */
