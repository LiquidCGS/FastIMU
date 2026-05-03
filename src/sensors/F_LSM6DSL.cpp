#include "F_LSM6DSL.hpp"

//Original code: https://github.com/hideakitai/MPU9250/blob/master/MPU9250.h

int LSM6DSL::init(calData cal, uint8_t address)
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
	
	// Wait up to 100ms for IMU to become ready.
	uint8_t IMUWhoAmI = checkReady(IMUAddress, 100);
	if (!(IMUWhoAmI == LSM6DSL_WHOAMI_DEFAULT_VALUE_A) && !(IMUWhoAmI == LSM6DSL_WHOAMI_DEFAULT_VALUE_B)) {
		return -1;
	}

	// reset device
	writeByteI2C(wire, IMUAddress, LSM6DSL_CTRL3_C, 0x01);   // Toggle softreset
	while (!checkReady(IMUAddress, 100));			// wait for reset
	
	writeByteI2C(wire, IMUAddress, LSM6DSL_CTRL1_XL, 0x47);	// Start up accelerometer, set range to +-16g, set output data rate to 104hz, BW_XL bits to 11.
	writeByteI2C(wire, IMUAddress, LSM6DSL_CTRL2_G, 0x4C);	// Start up gyroscope, set range to -+2000dps, output data rate to 104hz.
	writeByteI2C(wire, IMUAddress, LSM6DSL_CTRL4_C, 0x80);	// Set XL_BW_SCAL_ODR;
	//writeByteI2C(wire, IMUAddress, LSM6DSL_CTRL8_XL, 0x80);

	aRes = 16.f / 32768.f;			//ares value for full range (16g) readings
	gRes = 2000.f / 32768.f;	    //gres value for full range (2000dps) readings
	return 0;
}

void LSM6DSL::update() {
	uint8_t status = readByteI2C(wire, IMUAddress, LSM6DSL_STATUS_REG);
	bool accelReady = status & 0x01;  // XLDA
	bool gyroReady  = status & 0x02;  // GDA
	if (!accelReady && !gyroReady) return;

	int16_t IMUCount[6];
	uint8_t rawData[14];

	readBytesI2C(wire, IMUAddress, LSM6DSL_OUT_TEMP_L, 14, &rawData[0]);

	uint32_t now = micros();
	if (accelReady) accel.timestamp = now;
	if (gyroReady)  gyro.timestamp  = now;

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

void LSM6DSL::getAccel(AccelData* out)
{
	memcpy(out, &accel, sizeof(accel));
}
void LSM6DSL::getGyro(GyroData* out)
{
	memcpy(out, &gyro, sizeof(gyro));
}

int LSM6DSL::setAccelRange(int range) {
	uint8_t c;
	if (range == 16) {
		aRes = 16.f / 32768.f;
		c = 0x04;  // FS_XL = 01 at bits[3:2]
	}
	else if (range == 8) {
		aRes = 8.f / 32768.f;
		c = 0x0C;  // FS_XL = 11 at bits[3:2]
	}
	else if (range == 4) {
		aRes = 4.f / 32768.f;
		c = 0x08;  // FS_XL = 10 at bits[3:2]
	}
	else if (range == 2) {
		aRes = 2.f / 32768.f;
		c = 0x00;  // FS_XL = 00 at bits[3:2]
	}
	else {
		return -1;
	}
	rmwByteI2C(wire, IMUAddress, LSM6DSL_CTRL1_XL, 0x0C, c);
	return 0;
}

int LSM6DSL::setGyroRange(int range) {
	uint8_t c;
	if (range == 2000) {
		gRes = 2000.f / 32768.f;
		c = 0x0C;  // FS_G = 11 at bits[3:2], FS_125 = 0
	}
	else if (range == 1000) {
		gRes = 1000.f / 32768.f;
		c = 0x08;  // FS_G = 10 at bits[3:2]
	}
	else if (range == 500) {
		gRes = 500.f / 32768.f;
		c = 0x04;  // FS_G = 01 at bits[3:2]
	}
	else if (range == 250){
		gRes = 250.f / 32768.f;
		c = 0x00;  // FS_G = 00 at bits[3:2]
	}
	else {
		return -1;
	}
	rmwByteI2C(wire, IMUAddress, LSM6DSL_CTRL2_G, 0x0E, c);
	return 0;
}

void LSM6DSL::calibrateAccelGyro(calData* cal)
{
	uint8_t data[12];
	uint16_t packet_count = 256;
	float gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	float  gyrosensitivity = 250.f / 32768.f;
	float  accelsensitivity = 2.f / 32768.f;

	// reset device
	writeByteI2C(wire, IMUAddress, LSM6DSL_CTRL3_C, 0x01);
	delay(100);

	// 104 Hz ODR, ±2g accel, ±250dps gyro
	writeByteI2C(wire, IMUAddress, LSM6DSL_CTRL1_XL, 0x40);
	writeByteI2C(wire, IMUAddress, LSM6DSL_CTRL2_G,  0x40);
	delay(200);

	for (int i = 0; i < packet_count; i++)
	{
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };

		// Wait for fresh data on both sensors
		while (!(readByteI2C(wire, IMUAddress, LSM6DSL_STATUS_REG) & 0x03)) {}

		readBytesI2C(wire, IMUAddress, LSM6DSL_OUTX_L_G, 12, &data[0]);

		gyro_temp[0] = ((int16_t)data[1] << 8) | data[0];
		gyro_temp[1] = ((int16_t)data[3] << 8) | data[2];
		gyro_temp[2] = ((int16_t)data[5] << 8) | data[4];

		accel_temp[0] = ((int16_t)data[7] << 8) | data[6];
		accel_temp[1] = ((int16_t)data[9] << 8) | data[8];
		accel_temp[2] = ((int16_t)data[11] << 8) | data[10];

		accel_bias[0] += accel_temp[0] * accelsensitivity;
		accel_bias[1] += accel_temp[1] * accelsensitivity;
		accel_bias[2] += accel_temp[2] * accelsensitivity;

		gyro_bias[0] += gyro_temp[0] * gyrosensitivity;
		gyro_bias[1] += gyro_temp[1] * gyrosensitivity;
		gyro_bias[2] += gyro_temp[2] * gyrosensitivity;
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

static const int LSM6DSL_ODR_TABLE[] = {13, 26, 52, 104, 208, 416, 833, 1666};
static const uint8_t LSM6DSL_ODR_REG[] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80};

int LSM6DSL::setAccelODR(int odr_hz) {
	if (odr_hz <= 0) return -1;
	int actual = nearestHigherODR(LSM6DSL_ODR_TABLE, 8, odr_hz);
	int idx = 0;
	while (LSM6DSL_ODR_TABLE[idx] != actual) idx++;
	rmwByteI2C(wire, IMUAddress, LSM6DSL_CTRL1_XL, 0xF0, LSM6DSL_ODR_REG[idx]);
	currentAccelODR = actual;
	return actual;
}

int LSM6DSL::setGyroODR(int odr_hz) {
	if (odr_hz <= 0) return -1;
	int actual = nearestHigherODR(LSM6DSL_ODR_TABLE, 8, odr_hz);
	int idx = 0;
	while (LSM6DSL_ODR_TABLE[idx] != actual) idx++;
	rmwByteI2C(wire, IMUAddress, LSM6DSL_CTRL2_G, 0xF0, LSM6DSL_ODR_REG[idx]);
	currentGyroODR = actual;
	return actual;
}