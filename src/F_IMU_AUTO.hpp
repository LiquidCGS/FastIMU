#ifndef _F_IMU_AUTO_H_
#define _F_IMU_AUTO_H_
#include "IMUBase.hpp"

class IMU_AUTO : public IMUBase {
public:
	IMU_AUTO() {}

	int init(calData cal, uint8_t address) override {
		uint8_t addr = 0;
		int IMU_ID = -1;
		Wire.setWireTimeout(3000);
		for (int i = 0; i < 13; i++) {
			 if(Wire.getWireTimeoutFlag()){
				 Wire.clearWireTimeoutFlag();
				 return -10;
			 }
			if (readByte(IMUList[i].Address1, IMUList[i].Register) == IMUList[i].ExpectedID)
			{
				addr = IMUList[i].Address1;
				IMU_ID = IMUList[i].IMUID;
			}
			else if (readByte(IMUList[i].Address2, IMUList[i].Register) == IMUList[i].ExpectedID)
			{
				addr = IMUList[i].Address2;
				IMU_ID = IMUList[i].IMUID;
			}
		}
		if(addr = 0){ return -1; }
		switch (IMU_ID){
			case 0:
				IMU = &IMU0;
				break;
			case 1:
				IMU = &IMU1;
				break;
			case 2:
				IMU = &IMU2;
				break;
			case 3:
				IMU = &IMU3;
				break;
			case 4:
				IMU = &IMU4;
				break;
			case 5:
				IMU = &IMU5;
				break;
			case 6:
				IMU = &IMU6;
				break;
			case 7:
				IMU = &IMU7;
				break;
			case 8:
				IMU = &IMU8;
				break;
			case 9:
				IMU = &IMU9;
				break;
			case 10:
				IMU = &IMU10;
				break;
			case 11:
				IMU = &IMU11;
				break;
			case 12:
				IMU = &IMU12;
				break;
		}	
		return IMU->init(cal,addr);
	}
	void update() override {
		IMU->update();
	}
	void getAccel(AccelData* out) override {
		IMU->getAccel(out);
	}
	void getGyro(GyroData* out) override {
		IMU->getGyro(out);
	}
	void getMag(MagData* out) override {
		IMU->getMag(out);
	};
	void getQuat(Quaternion* out) override {
		IMU->getQuat(out);
	};
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
	
	MPU6050 IMU0;
	MPU6500 IMU1;
	MPU9250 IMU2;
	MPU9255 IMU3;
	MPU6515 IMU4;
	MPU6886 IMU5;
	IMU_Generic IMU6;
	BMI160 IMU7;
	LSM6DS3 IMU8;
	LSM6DSL IMU9;
	ICM20689 IMU10;
	ICM20690 IMU11;
	BMX055 IMU12;
	
	TypeIMU IMUList[13] =
	{
	  {0x68, 0x69, 0x75, 0x68, 0},		//MPU6050
	  {0x68, 0x69, 0x75, 0x70, 1},		//MPU6500
	  {0x68, 0x69, 0x75, 0x71, 2},		//MPU9250
	  {0x68, 0x69, 0x75, 0x73, 3},		//MPU9255
	  {0x68, 0x69, 0x75, 0x74, 4},		//MPU6515
	  {0x68, 0x69, 0x75, 0x75, 5},		//Fake 6500
	  {0x68, 0x69, 0x75, 0x19, 6},		//MPU6886
	  {0x68, 0x69, 0x75, 0x98, 7},		//ICM20689
	  {0x68, 0x69, 0x75, 0x20, 8},		//ICM20690
	  {0x18, 0x19, 0x00, 0xFA, 9}	    //BMX055
	  {0x69, 0x68, 0x00, 0xD1, 10},		//BMI160
	  {0x6B, 0x6A, 0x0F, 0x69, 11},		//LSM6DS3
	  {0x6B, 0x6A, 0x0F, 0x6A, 12},		//LSM6DSL

	};
	
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

#endif /* _F_IMU_AUTO_H_ */