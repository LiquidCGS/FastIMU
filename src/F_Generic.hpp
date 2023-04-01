#pragma once

#ifndef _F_Generic_H_
#define _F_Generic_H_

#include "IMUBase.hpp"
/*

	Generic REGISTERS

*/

//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_WHOAMI_DEFAULT_VALUE 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L	 0x03  // data
#define AK8963_XOUT_H	 0x04
#define AK8963_YOUT_L	 0x05
#define AK8963_YOUT_H	 0x06
#define AK8963_ZOUT_L	 0x07
#define AK8963_ZOUT_H	 0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define Generic_SELF_TEST_X_GYRO 0x00
#define Generic_SELF_TEST_Y_GYRO 0x01
#define Generic_SELF_TEST_Z_GYRO 0x02

// #define Generic_X_FINE_GAIN      0x03 // [7:0] fine gain
// #define Generic_Y_FINE_GAIN      0x04
// #define Generic_Z_FINE_GAIN      0x05
// #define Generic_XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
// #define Generic_XA_OFFSET_L_TC   0x07
// #define Generic_YA_OFFSET_H      0x08
// #define Generic_YA_OFFSET_L_TC   0x09
// #define Generic_ZA_OFFSET_H      0x0A
// #define Generic_ZA_OFFSET_L_TC   0x0B

#define Generic_SELF_TEST_X_ACCEL 0x0D
#define Generic_SELF_TEST_Y_ACCEL 0x0E
#define Generic_SELF_TEST_Z_ACCEL 0x0F

#define Generic_SELF_TEST_A      0x10

#define Generic_XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define Generic_XG_OFFSET_L      0x14
#define Generic_YG_OFFSET_H      0x15
#define Generic_YG_OFFSET_L      0x16
#define Generic_ZG_OFFSET_H      0x17
#define Generic_ZG_OFFSET_L      0x18
#define Generic_SMPLRT_DIV       0x19
#define Generic_MPU_CONFIG       0x1A
#define Generic_GYRO_CONFIG      0x1B
#define Generic_ACCEL_CONFIG     0x1C
#define Generic_ACCEL_CONFIG2    0x1D
#define Generic_LP_ACCEL_ODR     0x1E
#define Generic_WOM_THR          0x1F

#define Generic_MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define Generic_ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define Generic_ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define Generic_FIFO_EN          0x23
#define Generic_I2C_MST_CTRL     0x24
#define Generic_I2C_SLV0_ADDR    0x25
#define Generic_I2C_SLV0_REG     0x26
#define Generic_I2C_SLV0_CTRL    0x27
#define Generic_I2C_SLV1_ADDR    0x28
#define Generic_I2C_SLV1_REG     0x29
#define Generic_I2C_SLV1_CTRL    0x2A
#define Generic_I2C_SLV2_ADDR    0x2B
#define Generic_I2C_SLV2_REG     0x2C
#define Generic_I2C_SLV2_CTRL    0x2D
#define Generic_I2C_SLV3_ADDR    0x2E
#define Generic_I2C_SLV3_REG     0x2F
#define Generic_I2C_SLV3_CTRL    0x30
#define Generic_I2C_SLV4_ADDR    0x31
#define Generic_I2C_SLV4_REG     0x32
#define Generic_I2C_SLV4_DO      0x33
#define Generic_I2C_SLV4_CTRL    0x34
#define Generic_I2C_SLV4_DI      0x35
#define Generic_I2C_MST_STATUS   0x36
#define Generic_INT_PIN_CFG      0x37
#define Generic_INT_ENABLE       0x38
#define Generic_DMP_INT_STATUS   0x39  // Check DMP interrupt
#define Generic_INT_STATUS       0x3A
#define Generic_ACCEL_XOUT_H     0x3B
#define Generic_ACCEL_XOUT_L     0x3C
#define Generic_ACCEL_YOUT_H     0x3D
#define Generic_ACCEL_YOUT_L     0x3E
#define Generic_ACCEL_ZOUT_H     0x3F
#define Generic_ACCEL_ZOUT_L     0x40
#define Generic_TEMP_OUT_H       0x41
#define Generic_TEMP_OUT_L       0x42
#define Generic_GYRO_XOUT_H      0x43
#define Generic_GYRO_XOUT_L      0x44
#define Generic_GYRO_YOUT_H      0x45
#define Generic_GYRO_YOUT_L      0x46
#define Generic_GYRO_ZOUT_H      0x47
#define Generic_GYRO_ZOUT_L      0x48
#define Generic_EXT_SENS_DATA_00 0x49
#define Generic_EXT_SENS_DATA_01 0x4A
#define Generic_EXT_SENS_DATA_02 0x4B
#define Generic_EXT_SENS_DATA_03 0x4C
#define Generic_EXT_SENS_DATA_04 0x4D
#define Generic_EXT_SENS_DATA_05 0x4E
#define Generic_EXT_SENS_DATA_06 0x4F
#define Generic_EXT_SENS_DATA_07 0x50
#define Generic_EXT_SENS_DATA_08 0x51
#define Generic_EXT_SENS_DATA_09 0x52
#define Generic_EXT_SENS_DATA_10 0x53
#define Generic_EXT_SENS_DATA_11 0x54
#define Generic_EXT_SENS_DATA_12 0x55
#define Generic_EXT_SENS_DATA_13 0x56
#define Generic_EXT_SENS_DATA_14 0x57
#define Generic_EXT_SENS_DATA_15 0x58
#define Generic_EXT_SENS_DATA_16 0x59
#define Generic_EXT_SENS_DATA_17 0x5A
#define Generic_EXT_SENS_DATA_18 0x5B
#define Generic_EXT_SENS_DATA_19 0x5C
#define Generic_EXT_SENS_DATA_20 0x5D
#define Generic_EXT_SENS_DATA_21 0x5E
#define Generic_EXT_SENS_DATA_22 0x5F
#define Generic_EXT_SENS_DATA_23 0x60
#define Generic_MOT_DETECT_STATUS 0x61
#define Generic_I2C_SLV0_DO      0x63
#define Generic_I2C_SLV1_DO      0x64
#define Generic_I2C_SLV2_DO      0x65
#define Generic_I2C_SLV3_DO      0x66
#define Generic_I2C_MST_DELAY_CTRL 0x67
#define Generic_SIGNAL_PATH_RESET  0x68
#define Generic_MOT_DETECT_CTRL  0x69
#define Generic_USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define Generic_PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define Generic_PWR_MGMT_2       0x6C
#define Generic_DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define Generic_DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define Generic_DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define Generic_DMP_REG_1        0x70
#define Generic_DMP_REG_2        0x71
#define Generic_FIFO_COUNTH      0x72
#define Generic_FIFO_COUNTL      0x73
#define Generic_FIFO_R_W         0x74
#define Generic_WHO_AM_I_Generic 0x75 // Should return 0x71
#define Generic_WHOAMI_DEFAULT_VALUE 0x75
#define Generic_XA_OFFSET_H      0x77
#define Generic_XA_OFFSET_L      0x78
#define Generic_YA_OFFSET_H      0x7A
#define Generic_YA_OFFSET_L      0x7B
#define Generic_ZA_OFFSET_H      0x7D
#define Generic_ZA_OFFSET_L      0x7E


class Generic : public IMUBase {
public:
	Generic() {};

	// Inherited via IMUBase
	int init(calData cal, uint8_t address) override;
	int initMagnetometer();

	void update() override;
	void getAccel(AccelData* out) override;
	void getGyro(GyroData* out) override;
	void getMag(MagData* out) override;
	void getQuat(Quaternion* out) override {};
	float getTemp() override { return temperature; };

	int setGyroRange(int range) override;
	int setAccelRange(int range) override;

	void calibrateAccelGyro(calData* cal) override;
	void calibrateMag(calData* cal) override;

	bool hasMagnetometer() override {
		return (readByte(AK8963_ADDRESS, AK8963_WHO_AM_I) == AK8963_WHOAMI_DEFAULT_VALUE);
	}
	bool hasTemperature() override {
		return true;
	}
	bool hasQuatOutput() override {
		return false;
	}

private:
	float aRes = 16.0 / 32768.0;			//ares value for full range (16g) readings
	float gRes = 2000.0 / 32768.0;			//gres value for full range (2000dps) readings
	float mRes = 10. * 4912. / 32760.0;		//mres value for full range (4912uT) readings

	float temperature = 0.f;
	AccelData accel = { 0 };
	GyroData gyro = { 0 };
	MagData mag = { 0 };

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

	float factoryMagCal[3] = { 0 };

	bool dataAvaliable(){ return (readByte(IMUAddress, Generic_INT_STATUS) & 0x01);}
};
#endif /* _F_Generic_H_ */
