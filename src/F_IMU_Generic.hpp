#pragma once

#ifndef _F_IMU_Generic_H_
#define _F_IMU_Generic_H_

#include "IMUBase.hpp"
/*

	IMU_Generic REGISTERS

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

#define IMU_Generic_SELF_TEST_X_GYRO 0x00
#define IMU_Generic_SELF_TEST_Y_GYRO 0x01
#define IMU_Generic_SELF_TEST_Z_GYRO 0x02

// #define IMU_Generic_X_FINE_GAIN      0x03 // [7:0] fine gain
// #define IMU_Generic_Y_FINE_GAIN      0x04
// #define IMU_Generic_Z_FINE_GAIN      0x05
// #define IMU_Generic_XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
// #define IMU_Generic_XA_OFFSET_L_TC   0x07
// #define IMU_Generic_YA_OFFSET_H      0x08
// #define IMU_Generic_YA_OFFSET_L_TC   0x09
// #define IMU_Generic_ZA_OFFSET_H      0x0A
// #define IMU_Generic_ZA_OFFSET_L_TC   0x0B

#define IMU_Generic_SELF_TEST_X_ACCEL 0x0D
#define IMU_Generic_SELF_TEST_Y_ACCEL 0x0E
#define IMU_Generic_SELF_TEST_Z_ACCEL 0x0F

#define IMU_Generic_SELF_TEST_A      0x10

#define IMU_Generic_XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define IMU_Generic_XG_OFFSET_L      0x14
#define IMU_Generic_YG_OFFSET_H      0x15
#define IMU_Generic_YG_OFFSET_L      0x16
#define IMU_Generic_ZG_OFFSET_H      0x17
#define IMU_Generic_ZG_OFFSET_L      0x18
#define IMU_Generic_SMPLRT_DIV       0x19
#define IMU_Generic_MPU_CONFIG       0x1A
#define IMU_Generic_GYRO_CONFIG      0x1B
#define IMU_Generic_ACCEL_CONFIG     0x1C
#define IMU_Generic_ACCEL_CONFIG2    0x1D
#define IMU_Generic_LP_ACCEL_ODR     0x1E
#define IMU_Generic_WOM_THR          0x1F

#define IMU_Generic_MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define IMU_Generic_ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define IMU_Generic_ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define IMU_Generic_FIFO_EN          0x23
#define IMU_Generic_I2C_MST_CTRL     0x24
#define IMU_Generic_I2C_SLV0_ADDR    0x25
#define IMU_Generic_I2C_SLV0_REG     0x26
#define IMU_Generic_I2C_SLV0_CTRL    0x27
#define IMU_Generic_I2C_SLV1_ADDR    0x28
#define IMU_Generic_I2C_SLV1_REG     0x29
#define IMU_Generic_I2C_SLV1_CTRL    0x2A
#define IMU_Generic_I2C_SLV2_ADDR    0x2B
#define IMU_Generic_I2C_SLV2_REG     0x2C
#define IMU_Generic_I2C_SLV2_CTRL    0x2D
#define IMU_Generic_I2C_SLV3_ADDR    0x2E
#define IMU_Generic_I2C_SLV3_REG     0x2F
#define IMU_Generic_I2C_SLV3_CTRL    0x30
#define IMU_Generic_I2C_SLV4_ADDR    0x31
#define IMU_Generic_I2C_SLV4_REG     0x32
#define IMU_Generic_I2C_SLV4_DO      0x33
#define IMU_Generic_I2C_SLV4_CTRL    0x34
#define IMU_Generic_I2C_SLV4_DI      0x35
#define IMU_Generic_I2C_MST_STATUS   0x36
#define IMU_Generic_INT_PIN_CFG      0x37
#define IMU_Generic_INT_ENABLE       0x38
#define IMU_Generic_DMP_INT_STATUS   0x39  // Check DMP interrupt
#define IMU_Generic_INT_STATUS       0x3A
#define IMU_Generic_ACCEL_XOUT_H     0x3B
#define IMU_Generic_ACCEL_XOUT_L     0x3C
#define IMU_Generic_ACCEL_YOUT_H     0x3D
#define IMU_Generic_ACCEL_YOUT_L     0x3E
#define IMU_Generic_ACCEL_ZOUT_H     0x3F
#define IMU_Generic_ACCEL_ZOUT_L     0x40
#define IMU_Generic_TEMP_OUT_H       0x41
#define IMU_Generic_TEMP_OUT_L       0x42
#define IMU_Generic_GYRO_XOUT_H      0x43
#define IMU_Generic_GYRO_XOUT_L      0x44
#define IMU_Generic_GYRO_YOUT_H      0x45
#define IMU_Generic_GYRO_YOUT_L      0x46
#define IMU_Generic_GYRO_ZOUT_H      0x47
#define IMU_Generic_GYRO_ZOUT_L      0x48
#define IMU_Generic_EXT_SENS_DATA_00 0x49
#define IMU_Generic_EXT_SENS_DATA_01 0x4A
#define IMU_Generic_EXT_SENS_DATA_02 0x4B
#define IMU_Generic_EXT_SENS_DATA_03 0x4C
#define IMU_Generic_EXT_SENS_DATA_04 0x4D
#define IMU_Generic_EXT_SENS_DATA_05 0x4E
#define IMU_Generic_EXT_SENS_DATA_06 0x4F
#define IMU_Generic_EXT_SENS_DATA_07 0x50
#define IMU_Generic_EXT_SENS_DATA_08 0x51
#define IMU_Generic_EXT_SENS_DATA_09 0x52
#define IMU_Generic_EXT_SENS_DATA_10 0x53
#define IMU_Generic_EXT_SENS_DATA_11 0x54
#define IMU_Generic_EXT_SENS_DATA_12 0x55
#define IMU_Generic_EXT_SENS_DATA_13 0x56
#define IMU_Generic_EXT_SENS_DATA_14 0x57
#define IMU_Generic_EXT_SENS_DATA_15 0x58
#define IMU_Generic_EXT_SENS_DATA_16 0x59
#define IMU_Generic_EXT_SENS_DATA_17 0x5A
#define IMU_Generic_EXT_SENS_DATA_18 0x5B
#define IMU_Generic_EXT_SENS_DATA_19 0x5C
#define IMU_Generic_EXT_SENS_DATA_20 0x5D
#define IMU_Generic_EXT_SENS_DATA_21 0x5E
#define IMU_Generic_EXT_SENS_DATA_22 0x5F
#define IMU_Generic_EXT_SENS_DATA_23 0x60
#define IMU_Generic_MOT_DETECT_STATUS 0x61
#define IMU_Generic_I2C_SLV0_DO      0x63
#define IMU_Generic_I2C_SLV1_DO      0x64
#define IMU_Generic_I2C_SLV2_DO      0x65
#define IMU_Generic_I2C_SLV3_DO      0x66
#define IMU_Generic_I2C_MST_DELAY_CTRL 0x67
#define IMU_Generic_SIGNAL_PATH_RESET  0x68
#define IMU_Generic_MOT_DETECT_CTRL  0x69
#define IMU_Generic_USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define IMU_Generic_PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define IMU_Generic_PWR_MGMT_2       0x6C
#define IMU_Generic_DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define IMU_Generic_DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define IMU_Generic_DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define IMU_Generic_DMP_REG_1        0x70
#define IMU_Generic_DMP_REG_2        0x71
#define IMU_Generic_FIFO_COUNTH      0x72
#define IMU_Generic_FIFO_COUNTL      0x73
#define IMU_Generic_FIFO_R_W         0x74
#define IMU_Generic_WHO_AM_I_IMU_Generic 0x75 // Should return 0x71
#define IMU_Generic_WHOAMI_DEFAULT_VALUE 0x75
#define IMU_Generic_XA_OFFSET_H      0x77
#define IMU_Generic_XA_OFFSET_L      0x78
#define IMU_Generic_YA_OFFSET_H      0x7A
#define IMU_Generic_YA_OFFSET_L      0x7B
#define IMU_Generic_ZA_OFFSET_H      0x7D
#define IMU_Generic_ZA_OFFSET_L      0x7E


class IMU_Generic : public IMUBase {
public:
	IMU_Generic() {};

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
	int setIMUGeometry(int index) override { geometryIndex = index; return 0; };

	void calibrateAccelGyro(calData* cal) override;
	void calibrateMag(calData* cal) override;

	bool hasMagnetometer() override {
		return (readByte(AK8963_ADDRESS, AK8963_WHO_AM_I) == AK8963_WHOAMI_DEFAULT_VALUE);
	}
	bool hasTemperature() override {
		return false;
	}
	bool hasQuatOutput() override {
		return false;
	}

	String IMUName() override {
		return "(Fake) MPU6500/MPU9250 Type.";
	}
	String IMUType() override {
		return "IMU_Generic";
	}
	String IMUManufacturer() override {
		return "Unknown manufacturer";
	}
private:
	float aRes = 16.0 / 32768.0;			//ares value for full range (16g) readings
	float gRes = 2000.0 / 32768.0;			//gres value for full range (2000dps) readings
	float mRes = 10. * 4912. / 32760.0;		//mres value for full range (4912uT) readings
	int geometryIndex = 0;

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

	bool dataAvailable(){ return (readByte(IMUAddress, IMU_Generic_INT_STATUS) & 0x01);}
};
#endif /* _F_IMU_Generic_H_ */
