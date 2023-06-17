#include "F_LSM6DS3.hpp"

//Original code: https://github.com/hideakitai/MPU9250/blob/master/MPU9250.h

int LSM6DS3::init(calData cal, uint8_t address)
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

	if (!(readByte(IMUAddress, LSM6DS3_WHO_AM_I) == LSM6DS3_WHOAMI_DEFAULT_VALUE)) {
		return -1;
	}

	// reset device
	writeByte(IMUAddress, LSM6DS3_CTRL3_C, 0x01);   // Toggle softreset
	delay(100);										// wait for reset
	
	writeByte(IMUAddress, LSM6DS3_CTRL1_XL, 0x47);	// Start up accelerometer, set range to +-16g, set output data rate to 104hz, BW_XL bits to 11.
	writeByte(IMUAddress, LSM6DS3_CTRL2_G, 0x4C);	// Start up gyroscope, set range to -+2000dps, output data rate to 104hz.
	writeByte(IMUAddress, LSM6DS3_CTRL4_C, 0x80);	// Set XL_BW_SCAL_ODR;
	//writeByte(IMUAddress, LSM6DS3_CTRL8_XL, 0x80);

	aRes = 16.f / 32768.f;			//ares value for full range (16g) readings
	gRes = 2000.f / 32768.f;	    //gres value for full range (2000dps) readings
	return 0;
}

void LSM6DS3::update() {
	if (!(readByte(IMUAddress, LSM6DS3_STATUS_REG) & 0x03)) { return; }

	int16_t IMUCount[6];                                          // used to read all 16 bytes at once from the accel/gyro
	uint8_t rawData[14];                                          // x/y/z accel register data stored here

	readBytes(IMUAddress, LSM6DS3_OUT_TEMP_L, 14, &rawData[0]);    // Read the 12 raw data registers into data array

	IMUCount[0] = ((int16_t)rawData[3] << 8) | rawData[2];		  // Turn the MSB and LSB into a signed 16-bit value
	IMUCount[1] = ((int16_t)rawData[5] << 8) | rawData[4];
	IMUCount[2] = ((int16_t)rawData[7] << 8) | rawData[6];
	IMUCount[3] = ((int16_t)rawData[9] << 8) | rawData[8];
	IMUCount[4] = ((int16_t)rawData[11] << 8) | rawData[10];
	IMUCount[5] = ((int16_t)rawData[13] << 8) | rawData[12];

	float ax, ay, az, gx, gy, gz;

	// Calculate the accel value into actual g's per second
	ay = -((float)IMUCount[3] * aRes - calibration.accelBias[0]);
	ax = -((float)IMUCount[4] * aRes - calibration.accelBias[1]);
	az = ((float)IMUCount[5] * aRes - calibration.accelBias[2]);

	// Calculate the gyro value into actual degrees per second
	gy = ((float)IMUCount[0] * gRes - calibration.gyroBias[0]);
	gx = ((float)IMUCount[1] * gRes - calibration.gyroBias[1]);
	gz = ((float)IMUCount[2] * gRes - calibration.gyroBias[2]);

	float temp = ((((int16_t)rawData[1]) << 8) | rawData[0]);
	temperature = (temp / 8) + 25.f;

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
}

void LSM6DS3::getAccel(AccelData* out)
{
	memcpy(out, &accel, sizeof(accel));
}
void LSM6DS3::getGyro(GyroData* out)
{
	memcpy(out, &gyro, sizeof(gyro));
}

int LSM6DS3::setAccelRange(int range) {
	uint8_t c;
	if (range == 16) {
		aRes = 16.f / 32768.f;			//ares value for full range (16g) readings
		c = 0x67;
	}
	else if (range == 8) {
		aRes = 8.f / 32768.f;			//ares value for range (8g) readings
		c = 0x6F;
	}
	else if (range == 4) {
		aRes = 4.f / 32768.f;			//ares value for range (4g) readings
		c = 0x6B;
	}
	else if (range == 2) {
		aRes = 2.f / 32768.f;			//ares value for range (2g) readings
		c = 0x63;
	}
	else {
		return -1;
	}
	writeByte(IMUAddress, LSM6DS3_CTRL1_XL, c); // Write new ACCEL_CONFIG register value
	return 0;
}

int LSM6DS3::setGyroRange(int range) {
	uint8_t c;
	if (range == 2000) {
		gRes = 2000.f / 32768.f;			//ares value for full range (2000dps) readings
		c = 0x6C;
	}
	else if (range == 1000) {
		gRes = 1000.f / 32768.f;			//ares value for range (1000dps) readings
		c = 0x68;
	}
	else if (range == 500) {
		gRes = 500.f / 32768.f;			//ares value for range (500dps) readings
		c = 0x64;
	}
	else if (range == 250){
		gRes = 250.f / 32768.f;			//ares value for range (250dps) readings
		c = 0x60;
	}
	else {
		return -1;
	}
	writeByte(IMUAddress, LSM6DS3_CTRL2_G, c); // Write new GYRO_CONFIG register value
	return 0;
}

void LSM6DS3::calibrateAccelGyro(calData* cal)
{
	uint8_t data[12];
	uint16_t packet_count = 64; // How many sets of full gyro and accelerometer data for averaging;
	float gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	float  gyrosensitivity = 250.f / 32768.f;
	float  accelsensitivity = 2.f / 32768.f;

	// reset device
	writeByte(IMUAddress, LSM6DS3_CTRL3_C, 0x01);   // Toggle softreset
	delay(100);										// wait for reset

	writeByte(IMUAddress, LSM6DS3_CTRL1_XL, 0x70);	// Start up accelerometer, set range to +-2g, set output data rate to 104hz
	writeByte(IMUAddress, LSM6DS3_CTRL2_G, 0x70);	// Start up gyroscope, set range to -+250dps, output data rate to 104hz.
	delay(100);										// wait...

	for (int i = 0; i < packet_count; i++)
	{
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };

		readBytes(IMUAddress, LSM6DS3_OUTX_L_G, 12, &data[0]);    // Read the 12 raw data registers into data array

		gyro_temp[0] = ((int16_t)data[1] << 8) | data[0];		  // Turn the MSB and LSB into a signed 16-bit value
		gyro_temp[1] = ((int16_t)data[3] << 8) | data[2];
		gyro_temp[2] = ((int16_t)data[5] << 8) | data[4];

		accel_temp[0] = ((int16_t)data[7] << 8) | data[6];
		accel_temp[1] = ((int16_t)data[9] << 8) | data[8];
		accel_temp[2] = ((int16_t)data[11] << 8) | data[10];


		accel_bias[0] += accel_temp[0] * accelsensitivity; // Sum individual signed 16-bit biases to get accumulated biases
		accel_bias[1] += accel_temp[1] * accelsensitivity;
		accel_bias[2] += accel_temp[2] * accelsensitivity;

		gyro_bias[0] += gyro_temp[0] * gyrosensitivity;
		gyro_bias[1] += gyro_temp[1] * gyrosensitivity;
		gyro_bias[2] += gyro_temp[2] * gyrosensitivity;
		delay(20);
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