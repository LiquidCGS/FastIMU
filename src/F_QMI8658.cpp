#include "F_QMI8658.hpp"

//Original code: https://github.com/hideakitai/MPU9250/blob/master/MPU9250.h

int QMI8658::init(calData cal, uint8_t address) 
{
	//initialize address variable and calibration data.
	IMUAddress = address;

	if (cal.valid == false) 
	{
		calibration = { 0 };
	}
	else
	{
		calibration = cal;
	}

	if (!(readByte(IMUAddress, QMI8658_WHO_AM_I) == QMI8658_WHO_AM_I_DEFAULT_VALUE)) {
		return -1;
	}

	// reset device
	writeByte(IMUAddress, QMI8658_RESET, 0xFF);	    // Toggle softreset
	delay(100);										// wait for reset
	
	writeByte(IMUAddress, QMI8658_CTRL1, 0x40);		// Enable auto increment
	
	writeByte(IMUAddress, QMI8658_CTRL2, 0x34);  	// Set up full scale Accel range. +-16G, 500hz ODR
	writeByte(IMUAddress, QMI8658_CTRL3, 0x74); 	 // Set up full scale Gyro range. +-2000dps, 500hz ODR

	writeByte(IMUAddress, QMI8658_CTRL5, 0x55);  	// Enable LPF for both accel and gyro, set to 5.32% of odr for around 26.6hz

	writeByte(IMUAddress, QMI8658_CTRL7, 0x03);	    // Start up accelerometer and gyro, disable sync
	delay(100);								    	// wait until they're done starting up...

	aRes = 16.f / 32768.f;			//ares value for full range (16g) readings
	gRes = 2048.f / 32768.f;	    //gres value for full range (2000dps) readings

	return 0;
}

void QMI8658::update() {
	
	if(!dataAvailable()) {return;}
	
	int16_t IMUCount[6];                                          // used to read all 16 bytes at once from the accel/gyro
	uint8_t rawData[12];                                          // x/y/z accel register data stored here

	readBytes(IMUAddress, QMI8658_AX_L, 12, &rawData[0]);    // Read the 12 raw data registers into data array

	IMUCount[0] = ((int16_t)rawData[1] << 8) | rawData[0];		  // Turn the MSB and LSB into a signed 16-bit value
	IMUCount[1] = ((int16_t)rawData[3] << 8) | rawData[2];
	IMUCount[2] = ((int16_t)rawData[5] << 8) | rawData[4];
	IMUCount[3] = ((int16_t)rawData[7] << 8) | rawData[6];
	IMUCount[4] = ((int16_t)rawData[9] << 8) | rawData[8];
	IMUCount[5] = ((int16_t)rawData[11] << 8) | rawData[10];

	float ax, ay, az, gx, gy, gz;

	// Calculate the accel value into actual g's per second
	ax = (float)IMUCount[0] * aRes - calibration.accelBias[0];
	ay = (float)IMUCount[1] * aRes - calibration.accelBias[1];
	az = (float)IMUCount[2] * aRes - calibration.accelBias[2];

	// Calculate the gyro value into actual degrees per second
	gx = (float)IMUCount[3] * gRes - calibration.gyroBias[0];
	gy = (float)IMUCount[4] * gRes - calibration.gyroBias[1];
	gz = (float)IMUCount[5] * gRes - calibration.gyroBias[2];

	switch (geometryIndex) {
	case 0:
		accel.accelX = ax;		gyro.gyroX = gx;
		accel.accelY = ay;		gyro.gyroY = gy;
		accel.accelZ = az;		gyro.gyroZ = gz;
		break;
	case 1:
		accel.accelX = -ay;		gyro.gyroX = -gy;
		accel.accelY = ax;		gyro.gyroY = gx;
		accel.accelZ = az;		gyro.gyroZ = gz;
		break;
	case 2:
		accel.accelX = -ax;		gyro.gyroX = -gx;
		accel.accelY = -ay;		gyro.gyroY = -gy;
		accel.accelZ = az;		gyro.gyroZ = gz;
		break;
	case 3:
		accel.accelX = ay;		gyro.gyroX = gy;
		accel.accelY = -ax;		gyro.gyroY = -gx;
		accel.accelZ = az;		gyro.gyroZ = gz;
		break;
	case 4:
		accel.accelX = -az;		gyro.gyroX = -gz;
		accel.accelY = -ay;		gyro.gyroY = -gy;
		accel.accelZ = -ax;		gyro.gyroZ = -gx;
		break;
	case 5:
		accel.accelX = -az;		gyro.gyroX = -gz;
		accel.accelY = ax;		gyro.gyroY = gx;
		accel.accelZ = -ay;		gyro.gyroZ = -gy;
		break;
	case 6:
		accel.accelX = -az;		gyro.gyroX = -gz;
		accel.accelY = ay;		gyro.gyroY = gy;
		accel.accelZ = ax;		gyro.gyroZ = gx;
		break;
	case 7:
		accel.accelX = -az;		gyro.gyroX = -gz;
		accel.accelY = -ax;		gyro.gyroY = -gx;
		accel.accelZ = ay;		gyro.gyroZ = gy;
		break;
	}

	uint8_t buf[2];
	readBytes(IMUAddress, QMI8658_TEMP_L, 2, &buf[0]);
	temperature = ((float)buf[1] + (float)buf[0] / 256);	
}

void QMI8658::getAccel(AccelData* out)
{
	memcpy(out, &accel, sizeof(accel));
}
void QMI8658::getGyro(GyroData* out)
{
	memcpy(out, &gyro, sizeof(gyro));
}

int QMI8658::setAccelRange(int range) {
	writeByte(IMUAddress, QMI8658_CTRL7, 0x00);	    // disable accelerometer and gyro, disable sync
	uint8_t c;
	if (range == 16) {
		aRes = 16.f / 32768.f;			//ares value for full range (16g) readings
		c = 0x68;
	}
	else if (range == 8) {
		aRes = 8.f / 32768.f;			//ares value for range (8g) readings
		c = 0x48;
	}
	else if (range == 4) {
		aRes = 4.f / 32768.f;			//ares value for range (4g) readings
		c = 0x28;
	}
	else if (range == 2) {
		aRes = 2.f / 32768.f;			//ares value for range (2g) readings
		c = 0x08;
	}
	else {
		return -1;
	}
	writeByte(IMUAddress, QMI8658_CTRL2, c); 	// Write new ACCEL_CONFIG register value
	writeByte(IMUAddress, QMI8658_CTRL7, 0x03);	    // Start up accelerometer and gyro, disable sync
	return 0;
}

int QMI8658::setGyroRange(int range) {
	writeByte(IMUAddress, QMI8658_CTRL7, 0x00);	    // disable accelerometer and gyro, disable sync
	uint8_t c;
	if (range == 2048) {
		gRes = 2048.f / 32768.f;			//gres value for full range (2000dps) readings
		c = 0x74;
	}
	else if (range == 1024) {
		gRes = 1024.f / 32768.f;			//gres value for range (1000dps) readings
		c = 0x64;
	}
	else if (range == 512) {
		gRes = 512.f / 32768.f;			//gres value for range (500dps) readings
		c = 0x54;
	}
	else if (range == 256){
		gRes = 256.f / 32768.f;			//gres value for range (250dps) readings
		c = 0x44;
	}
	else if (range == 128){
		gRes = 128.f / 32768.f;			//gres value for range (250dps) readings
		c = 0x34;
	}
	else {
		return -1;
	}
	writeByte(IMUAddress, QMI8658_CTRL3, c); 	// Write new ACCEL_CONFIG register value
	writeByte(IMUAddress, QMI8658_CTRL7, 0x03);	    // Start up accelerometer and gyro, disable sync
	return 0;
}

void QMI8658::calibrateAccelGyro(calData* cal)
{
	uint8_t data[12];
	uint16_t packet_count = 100; // How many sets of full gyro and accelerometer data for averaging;
	float gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	float  gyrosensitivity = 128.f / 32768.f;			
	float  accelsensitivity = 2.f / 32768.f;

	// reset device
	writeByte(IMUAddress, QMI8658_RESET, 0xFF);	    // Toggle softreset
	delay(100);										// wait for reset
	
	writeByte(IMUAddress, QMI8658_CTRL1, 0x40);		// Enable auto increment

	writeByte(IMUAddress, QMI8658_CTRL2, 0x04);  	// Set up full scale Accel range. +-2G, 500hz ODR
	writeByte(IMUAddress, QMI8658_CTRL3, 0x34); 	 // Set up Gyro range. +-1024dps, 500hz ODR

	writeByte(IMUAddress, QMI8658_CTRL5, 0x77);  	// Enable LPF for both accel and gyro, set to 14% of odr for around 70hz
	
	writeByte(IMUAddress, QMI8658_CTRL7, 0x03);	    // Start up accelerometer and gyro, disable sync
	delay(100);								    	//wait until they're done starting up...

	for (int i = 0; i < packet_count; i++)
	{
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };

		readBytes(IMUAddress, QMI8658_AX_L, 12, &data[0]);    // Read the 12 raw data registers into data array

		accel_temp[0] = ((int16_t)data[1] << 8) | data[0];		  // Turn the MSB and LSB into a signed 16-bit value
		accel_temp[1] = ((int16_t)data[3] << 8) | data[2];
		accel_temp[2] = ((int16_t)data[5] << 8) | data[4];

		gyro_temp[0] = ((int16_t)data[7] << 8) | data[6];
		gyro_temp[1] = ((int16_t)data[9] << 8) | data[8];
		gyro_temp[2] = ((int16_t)data[11] << 8) | data[10];


		accel_bias[0] += accel_temp[0] * accelsensitivity; // Sum individual signed 16-bit biases to get accumulated biases
		accel_bias[1] += accel_temp[1] * accelsensitivity;
		accel_bias[2] += accel_temp[2] * accelsensitivity;
		
		gyro_bias[0] += gyro_temp[0] * gyrosensitivity;
		gyro_bias[1] += gyro_temp[1] * gyrosensitivity;
		gyro_bias[2] += gyro_temp[2] * gyrosensitivity;
		delay(4);
	}
	
	accel_bias[0] /= packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= packet_count;
	accel_bias[2] /= packet_count;

	gyro_bias[0] /= packet_count;
	gyro_bias[1] /= packet_count;
	gyro_bias[2] /= packet_count;

	switch (geometryIndex) {
	case 0:
	case 1:
	case 2:
	case 3:
		if (accel_bias[2] > 0.f) {
			accel_bias[2] -= 1.f; // Remove gravity from the z-axis accelerometer bias calculation
		}
		else {
			accel_bias[2] += 1.f;
		}
		break;
	case 4:
	case 6:
		if (accel_bias[0] > 0.f) {
			accel_bias[0] -= 1.f; // Remove gravity from the z-axis accelerometer bias calculation
		}
		else {
			accel_bias[0] += 1.f;
		}
		break;
	case 5:
	case 7:
		if (accel_bias[1] > 0.f) {
			accel_bias[1] -= 1.f; // Remove gravity from the z-axis accelerometer bias calculation
		}
		else {
			accel_bias[1] += 1.f;
		}
		break;
	}
	// Output scaled accelerometer biases for display in the main program
	cal->accelBias[0] = (float)accel_bias[0];
	cal->accelBias[1] = (float)accel_bias[1];
	cal->accelBias[2] = (float)accel_bias[2];
	// Output scaled gyro biases for display in the main program
	cal->gyroBias[0] = (float)gyro_bias[0];
	cal->gyroBias[1] = (float)gyro_bias[1];
	cal->gyroBias[2] = (float)gyro_bias[2];
	cal->valid = true;
}