#ifndef _F_IMU_AUTO_H_
#define _F_IMU_AUTO_H_
#include "IMUBase.hpp"

class IMU_AUTO : public IMUBase {
public:
	IMU_AUTO() {}

	int init(calData cal, uint8_t address) override {
		
	}
	void update() override {};

	void getAccel(AccelData* out) override {};
	void getGyro(GyroData* out) override {};
	void getMag(MagData* out) override {};
	void getQuat(Quaternion* out) override {};
	float getTemp() override {
		return IMU->getTemp();
	}
	int setGyroRange(int range) override {
	return IMU->setGyroRange(range);
	}
	int setAccelRange(int range) override {
	return IMU->setAccelRange(range);
	}

	void calibrateAccelGyro(calData* cal) override {
		IMU->calibrateAccelGyro(cal);
	};
	void calibrateMag(calData* cal) override {
		IMU->calibrateMag(cal);
	};

	bool hasMagnetometer() override {
		return IMU->hasMagnetometer();
	}
	bool hasTemperature() override {
		return IMU->hasTemperature();
	}
	bool hasQuatOutput() override {
		return IMU->hasQuatOutput();
	}

	String IMUName() override{
		return IMU->IMUName();
	}
	String IMUType() override{
		return IMU->IMUType();
	}
	String IMUManufacturer() override{
		return IMU->IMUManufacturer();
	}
private:
	IMUBase* IMU;
	
	struct TypeIMU {
	  uint8_t Address1;
	  uint8_t Address2;
	  uint8_t Register;
	  uint8_t ExpectedID;
	  int     IMUID;
	};
	
	TypeIMU IMUList[13] =
	{
	  {0x68, 0x69, 0x75, 0x68, 0},		//MPU6050
	  {0x68, 0x69, 0x75, 0x70, 1},		//MPU6500
	  {0x68, 0x69, 0x75, 0x71, 2},		//MPU9250
	  {0x68, 0x69, 0x75, 0x73, 3},		//MPU9255
	  {0x68, 0x69, 0x75, 0x74, 4},		//MPU6515
	  {0x68, 0x69, 0x75, 0x19, 5},		//MPU6886
	  {0x68, 0x69, 0x75, 0x75, 6},		//Fake 6500
	  {0x69, 0x68, 0x00, 0xD1, 7},		//BMI160
	  {0x6B, 0x6A, 0x0F, 0x69, 8},		//LSM6DS3
	  {0x6B, 0x6A, 0x0F, 0x6A, 9},		//LSM6DSL
	  {0x68, 0x69, 0x75, 0x98, 10},		//ICM20689
	  {0x68, 0x69, 0x75, 0x20, 11},		//ICM20690
	  {0x18, 0x19, 0x00, 0xFA, 12}	    //BMX055
	};
};

#endif /* _F_IMU_AUTO_H_ */