#include "F_BMI055.hpp"

int BMI055::init(calData cal, uint8_t address) 
{
	//initialize address variable and calibration data.
	if (address == 0x18 || address == 0x68) {
		AccelAddress = 0x18;
		GyroAddress = 0x68;
	}
	else if (address == 0x19 || address == 0x69) {
		AccelAddress = 0x19;
		GyroAddress = 0x69;
	}
	else {
		return -1;
	}

	if (cal.valid == false) 
	{
		calibration = { 0 };
	}
	else
	{
		calibration = cal;
	}

	if (!(readByte(AccelAddress, BMI055_ACCD_CHIPID) == BMI055_ACCEL_ID)) {
		return -2;
	}
	if (!(readByte(GyroAddress, BMI055_GYR_CHIP_ID) == BMI055_GYRO_ID)) {
		return -3;
	}

	// Reset sensor.
	writeByte(AccelAddress, BMI055_BGW_SOFTRESET, 0xB6);
	writeByte(GyroAddress, BMI055_GYR_BGW_SOFTRESET, 0xB6);
	delay(100);

	// Set accelerometer range
	writeByte(AccelAddress, BMI055_PMU_RANGE, 0x0C); // Write '1100' into bits 3:0, setting accelerometer into 16g range.

	// Set LPF
	writeByte(AccelAddress, BMI055_PMU_BW, 0x0B); // Write '01011' into bits 4:0, setting the accelerometer lpf bandwidth to 62.5hz

	// Enter normal mode
	writeByte(AccelAddress, BMI055_PMU_LPW, 0x00); 

	// Set Gyro range
	writeByte(GyroAddress, BMI055_GYR_RANGE, 0x00);	// Write '000' into bits 2:0, setting gyro into 2000dps range.

	// Set LPF
	writeByte(GyroAddress, BMI055_GYR_BW, 0x03); // Write '0011' into bits 3:0, setting the gyro lpf bandwidth to 47hz ;;;;;;;;;;;; THIS LIMITS ODR TO 400HZ

	// Enter normal mode
	writeByte(GyroAddress, BMI055_GYR_LPM1, 0x00);

	delay (100);

	return 0;
}

void BMI055::update() 
{
	int16_t AccelCount[3];                                        // used to read all 6 bytes at once from the BMI055 accel
	int16_t GyroCount[3];										  // used to read all 6 bytes at once from the BMI055 gyro
	uint8_t rawDataAccel[7];                                          // x/y/z accel register data stored here
	uint8_t rawDataGyro[6];

	readBytes(AccelAddress, BMI055_ACCD_X_LSB, 7, &rawDataAccel[0]);       // Read the 7 raw accelerometer data registers into data array
	readBytes(GyroAddress, BMI055_GYR_RATE_X_LSB , 6, &rawDataGyro[0]);   // Read the 6 raw gyroscope data registers into data array

	//accel registers
	AccelCount[0] = ((rawDataAccel[1] << 8) | (rawDataAccel[0] & 0xF0)) >> 4;		  // Turn the MSB and LSB into a signed 12-bit value
	AccelCount[1] = ((rawDataAccel[3] << 8) | (rawDataAccel[2] & 0xF0)) >> 4;	      // praise sign extension, making this code clean and simple.
	AccelCount[2] = ((rawDataAccel[5] << 8) | (rawDataAccel[4] & 0xF0)) >> 4;

	//gyro registers
	GyroCount[0] = (rawDataGyro[1] << 8) | (rawDataGyro[0]);
	GyroCount[1] = (rawDataGyro[3] << 8) | (rawDataGyro[2]);
	GyroCount[2] = (rawDataGyro[5] << 8) | (rawDataGyro[4]);

	float ax, ay, az, gx, gy, gz;

	// Calculate the accel value into actual g's per second
	ax = AccelCount[0] * (float)aRes - calibration.accelBias[0];
	ay = AccelCount[1] * (float)aRes - calibration.accelBias[1];
	az = AccelCount[2] * (float)aRes - calibration.accelBias[2];

	// Calculate the gyro value into actual degrees per second
	gx = GyroCount[0] * (float)gRes - calibration.gyroBias[0];
	gy = GyroCount[1] * (float)gRes - calibration.gyroBias[1];
	gz = GyroCount[2] * (float)gRes - calibration.gyroBias[2];

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

	// Calculate the temperature value into actual deg c
	temperature = -((rawDataAccel[6] * -0.5f) * (86.5f - -40.5f) / (float)(128.f) - 40.5f) - 20.f;
}

void BMI055::getAccel(AccelData* out) 
{
	memcpy(out, &accel, sizeof(accel));
}
void BMI055::getGyro(GyroData* out) 
{
	memcpy(out, &gyro, sizeof(gyro));
}

int BMI055::setAccelRange(int range) {
	uint8_t c;
	if (range == 16) {
		aRes = 16.f / 2048.f;			//ares value for full range (16g) readings
		c = 0x0C;
	}
	else if (range == 8) {
		aRes = 8.f / 2048.f;			//ares value for range (8g) readings
		c = 0x08;
	}
	else if (range == 4) {
		aRes = 4.f / 2048.f;			//ares value for range (4g) readings
		c = 0x05;
	}
	else if (range == 2) {
		aRes = 2.f / 2048.f;			//ares value for range (2g) readings
		c = 0x03;
	}
	else {
		return -1;
	}
	writeByte(AccelAddress, BMI055_PMU_RANGE, c); // Write new BMI055_PMU_RANGE register value
	return 0;
}

int BMI055::setGyroRange(int range) {
	uint8_t c;
	if (range == 2000) {
		gRes = 2000.f / 32768.f;			//ares value for full range (2000dps) readings
		c = 0x00;
	}
	else if (range == 1000) {
		gRes = 1000.f / 32768.f;			//ares value for range (1000dps) readings
		c = 0x01;
	}
	else if (range == 500) {
		gRes = 500.f / 32768.f;			//ares value for range (500dps) readings
		c = 0x02;
	}
	else if (range == 250) {
		gRes = 250.f / 32768.f;			//ares value for range (250dps) readings
		c = 0x03;
	}
	else if (range == 125) {
		gRes = 125.f / 32768.f;			//ares value for range (125dps) readings
		c = 0x04;
	}
	else {
		return -1;
	}
	writeByte(GyroAddress, BMI055_GYR_RANGE, c); // Write new BMX055_GYR_RANGE register value
	return 0;
}

void BMI055::calibrateAccelGyro(calData* cal)
{
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t packet_count = 64; // How many sets of full gyro and accelerometer data for averaging;
	float gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	float  gyrosensitivity = 125.f / 32768.f;			//gres value for full range (2000dps) readings (16 bit)
	float  accelsensitivity = 2.f / 2048.f;				//ares value for full range (16g) readings (12 bit)

	// Reset sensor.
	writeByte(AccelAddress, BMI055_BGW_SOFTRESET, 0xB6);
	writeByte(GyroAddress, BMI055_GYR_BGW_SOFTRESET, 0xB6);
	delay(100);
	// Set accelerometer range
	writeByte(AccelAddress, BMI055_PMU_RANGE, 0x03); // Write '0011' into bits 3:0, setting accelerometer into 2g range, maximum sensitivity
	// Set LPF
	writeByte(AccelAddress, BMI055_PMU_BW, 0x0C); // Write '01100' into bits 4:0, setting the accelerometer lpf bandwidth to 125hz
	// Enter normal mode
	writeByte(AccelAddress, BMI055_PMU_LPW, 0x00);
	// Reset sensor.
	// Set Gyro range
	writeByte(GyroAddress, BMI055_GYR_RANGE, 0x04);	// Write '100' into bits 2:0, setting gyro into 125dps range, maximum sensitivity
	// Set LPF
	writeByte(GyroAddress, BMI055_GYR_BW, 0x03); // Write '0011' into bits 3:0, setting the gyro lpf bandwidth to 47hz ;;;;;;;;;;;; THIS LIMITS ODR TO 400HZ
	// Enter normal mode
	writeByte(GyroAddress, BMI055_GYR_LPM1, 0x00);
	delay(10);

	for (int i = 0; i < packet_count; i++)
	{
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };

		readBytes(AccelAddress, BMI055_ACCD_X_LSB, 6, &data[0]);       // Read the 7 raw accelerometer data registers into data array
		readBytes(GyroAddress, BMI055_GYR_RATE_X_LSB, 6, &data[6]);   // Read the 6 raw gyroscope data registers into data array
		
		accel_temp[0] = ((data[1] << 8) | (data[0] & 0xF0)) >> 4;  // Form signed 16-bit integer for each sample
		accel_temp[1] = ((data[3] << 8) | (data[2] & 0xF0)) >> 4;
		accel_temp[2] = ((data[5] << 8) | (data[4] & 0xF0)) >> 4;

		gyro_temp[0] = (data[7] << 8) | (data[6]);
		gyro_temp[1] = (data[9] << 8) | (data[8]);
		gyro_temp[2] = (data[11] << 8) | (data[10]);


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