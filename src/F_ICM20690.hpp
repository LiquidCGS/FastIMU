#pragma once

#ifndef _F_ICM20690_H_
#define _F_ICM20690_H_

#include "IMUBase.hpp"
/*

	ICM20690 REGISTERS

*/
#define ICM20690_SELF_TEST_X_GYRO 0x00
#define ICM20690_SELF_TEST_Y_GYRO 0x01
#define ICM20690_SELF_TEST_Z_GYRO 0x02

// #define ICM20690_X_FINE_GAIN      0x03 // [7:0] fine gain
// #define ICM20690_Y_FINE_GAIN      0x04
// #define ICM20690_Z_FINE_GAIN      0x05
// #define ICM20690_XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
// #define ICM20690_XA_OFFSET_L_TC   0x07
// #define ICM20690_YA_OFFSET_H      0x08
// #define ICM20690_YA_OFFSET_L_TC   0x09
// #define ICM20690_ZA_OFFSET_H      0x0A
// #define ICM20690_ZA_OFFSET_L_TC   0x0B

#define ICM20690_SELF_TEST_X_ACCEL 0x0D
#define ICM20690_SELF_TEST_Y_ACCEL 0x0E
#define ICM20690_SELF_TEST_Z_ACCEL 0x0F

#define ICM20690_XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define ICM20690_XG_OFFSET_L      0x14
#define ICM20690_YG_OFFSET_H      0x15
#define ICM20690_YG_OFFSET_L      0x16
#define ICM20690_ZG_OFFSET_H      0x17
#define ICM20690_ZG_OFFSET_L      0x18
#define ICM20690_SMPLRT_DIV       0x19
#define ICM20690_MPU_CONFIG       0x1A
#define ICM20690_GYRO_CONFIG      0x1B
#define ICM20690_ACCEL_CONFIG     0x1C
#define ICM20690_ACCEL_CONFIG2    0x1D
#define ICM20690_LP_MODE_CFG      0x1E

#define ICM20690_ACCEL_WOM_X_THR  0x20  
#define ICM20690_ACCEL_WOM_Y_THR  0x21 
#define ICM20690_ACCEL_WOM_Z_THR  0x22 

#define ICM20690_FIFO_EN          0x23
#define ICM20690_FSYNC_INT		  0x36
#define ICM20690_INT_PIN_CFG      0x37
#define ICM20690_INT_ENABLE       0x38
#define ICM20690_DMP_INT_STATUS   0x39  // Check DMP interrupt
#define ICM20690_INT_STATUS       0x3A
#define ICM20690_ACCEL_XOUT_H     0x3B
#define ICM20690_ACCEL_XOUT_L     0x3C
#define ICM20690_ACCEL_YOUT_H     0x3D
#define ICM20690_ACCEL_YOUT_L     0x3E
#define ICM20690_ACCEL_ZOUT_H     0x3F
#define ICM20690_ACCEL_ZOUT_L     0x40
#define ICM20690_TEMP_OUT_H       0x41
#define ICM20690_TEMP_OUT_L       0x42
#define ICM20690_GYRO_XOUT_H      0x43
#define ICM20690_GYRO_XOUT_L      0x44
#define ICM20690_GYRO_YOUT_H      0x45
#define ICM20690_GYRO_YOUT_L      0x46
#define ICM20690_GYRO_ZOUT_H      0x47
#define ICM20690_GYRO_ZOUT_L      0x48
#define ICM20690_SIGNAL_PATH_RESET   0x68
#define ICM20690_ACCEL_INTEL_CTRL 0x69
#define ICM20690_USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define ICM20690_PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define ICM20690_PWR_MGMT_2       0x6C
#define ICM20690_FIFO_COUNTH      0x72
#define ICM20690_FIFO_COUNTL      0x73
#define ICM20690_FIFO_R_W         0x74
#define ICM20690_WHO_AM_I_ICM20690 0x75 // Should return 0x20
#define ICM20690_WHOAMI_DEFAULT_VALUE 0x20
#define ICM20690_XA_OFFSET_H      0x77
#define ICM20690_XA_OFFSET_L      0x78
#define ICM20690_YA_OFFSET_H      0x7A
#define ICM20690_YA_OFFSET_L      0x7B
#define ICM20690_ZA_OFFSET_H      0x7D
#define ICM20690_ZA_OFFSET_L      0x7E


class ICM20690 : public IMUBase {
public:
	ICM20690() {};

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
		return "ICM-20690";
	}
	String IMUType() override {
		return "ICM20690";
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

	bool dataAvailable(){ return (readByte(IMUAddress, ICM20690_INT_STATUS) & 0x01);}
};
#endif /* _F_ICM20690_H_ */
