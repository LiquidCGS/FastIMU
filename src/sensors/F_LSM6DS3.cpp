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

	uint8_t IMUWhoAmI = checkReady(IMUAddress, 100);
	if (!(IMUWhoAmI == LSM6DS3_WHOAMI_DEFAULT_VALUE)) {
		if (!(IMUWhoAmI == LSM6DS3TR_C_WHOAMI_DEFAULT_VALUE)) {
			return -1;
		}
	}

	// reset device
	writeByteI2C(wire, IMUAddress, LSM6DS3_CTRL3_C, 0x01);   // Toggle softreset
	while (!checkReady(IMUAddress, 100));			// wait for reset

	setAccelODR(104);
	setAccelRange(16);
	setGyroODR(104);
	setGyroRange(2000);
	setAccelLPF(0);
	return 0;
}

void LSM6DS3::update() {
	uint8_t status = readByteI2C(wire, IMUAddress, LSM6DS3_STATUS_REG);
	bool accelReady = status & 0x01;  // XLDA
	bool gyroReady  = status & 0x02;  // GDA
	if (!accelReady && !gyroReady) return;

	int16_t IMUCount[6];
	uint8_t rawData[14];

	readBytesI2C(wire, IMUAddress, LSM6DS3_OUT_TEMP_L, 14, &rawData[0]);

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

	float aArr[3] = {ax, ay, az};
	float gArr[3] = {gx, gy, gz};
	const int8_t* gm = GEO_MAP[geometryIndex];
	accel.accelX = applyGeo(gm[0], aArr);
	accel.accelY = applyGeo(gm[1], aArr);
	accel.accelZ = applyGeo(gm[2], aArr);
	gyro.gyroX   = applyGeo(gm[0], gArr);
	gyro.gyroY   = applyGeo(gm[1], gArr);
	gyro.gyroZ   = applyGeo(gm[2], gArr);
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
	rmwByteI2C(wire, IMUAddress, LSM6DS3_CTRL1_XL, 0x0C, c);
	return 0;
}

int LSM6DS3::setGyroRange(int range) {
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
	rmwByteI2C(wire, IMUAddress, LSM6DS3_CTRL2_G, 0x0E, c);
	return 0;
}

void LSM6DS3::calibrateAccelGyro(calData* cal)
{
	uint8_t data[12];
	uint16_t packet_count = 256;
	float gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	float  gyrosensitivity = 250.f / 32768.f;
	float  accelsensitivity = 2.f / 32768.f;

	// reset device
	writeByteI2C(wire, IMUAddress, LSM6DS3_CTRL3_C, 0x01);
	delay(100);

	// 104 Hz ODR, ±2g accel, ±250dps gyro
	setAccelODR(104);
	setAccelRange(2);
	setGyroODR(104);
	setGyroRange(250);
	delay(200);

	for (int i = 0; i < packet_count; i++)
	{
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };

		// Wait for fresh data on both sensors
		while (!(readByteI2C(wire, IMUAddress, LSM6DS3_STATUS_REG) & 0x03)) {}

		readBytesI2C(wire, IMUAddress, LSM6DS3_OUTX_L_G, 12, &data[0]);

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

// ODR encoding in bits[7:4]: 0x1=13Hz, 0x2=26Hz, 0x3=52Hz, 0x4=104Hz, 0x5=208Hz, 0x6=416Hz, 0x7=833Hz, 0x8=1666Hz
static const int LSM6DS3_ODR_TABLE[] = {13, 26, 52, 104, 208, 416, 833, 1666};
static const uint8_t LSM6DS3_ODR_REG[] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80};

int LSM6DS3::setAccelODR(int odr_hz) {
	if (odr_hz <= 0) return -1;
	int actual = nearestHigherODR(LSM6DS3_ODR_TABLE, 8, odr_hz);
	int idx = 0;
	while (LSM6DS3_ODR_TABLE[idx] != actual) idx++;
	rmwByteI2C(wire, IMUAddress, LSM6DS3_CTRL1_XL, 0xF0, LSM6DS3_ODR_REG[idx]);
	currentAccelODR = actual;
	return actual;
}

int LSM6DS3::setGyroODR(int odr_hz) {
	if (odr_hz <= 0) return -1;
	int actual = nearestHigherODR(LSM6DS3_ODR_TABLE, 8, odr_hz);
	int idx = 0;
	while (LSM6DS3_ODR_TABLE[idx] != actual) idx++;
	rmwByteI2C(wire, IMUAddress, LSM6DS3_CTRL2_G, 0xF0, LSM6DS3_ODR_REG[idx]);
	currentGyroODR = actual;
	return actual;
}

static const int LSM6DS3_ACCEL_LPF_TABLE[] = {50, 100, 200, 400};
static const uint8_t LSM6DS3_ACCEL_LPF_CFG[] = {3, 2, 1, 0};

int LSM6DS3::setAccelLPF(int lpf_hz) {
	if (lpf_hz == 0) {
		rmwByteI2C(wire, IMUAddress, LSM6DS3_CTRL1_XL, 0x03, 0x00);
		rmwByteI2C(wire, IMUAddress, LSM6DS3_CTRL4_C, 0x80, 0x80);
		currentAccelLPF = 0;
		return 0;
	}
	int actual = nearestVal(LSM6DS3_ACCEL_LPF_TABLE, 4, lpf_hz);
	int idx = 0;
	while (LSM6DS3_ACCEL_LPF_TABLE[idx] != actual) idx++;
	rmwByteI2C(wire, IMUAddress, LSM6DS3_CTRL4_C, 0x80, 0x00);
	rmwByteI2C(wire, IMUAddress, LSM6DS3_CTRL1_XL, 0x03, LSM6DS3_ACCEL_LPF_CFG[idx]);
	currentAccelLPF = actual;
	return actual;
}

int LSM6DS3::setGyroLPF(int lpf_hz) {
	if (lpf_hz == 0) {
		rmwByteI2C(wire, IMUAddress, LSM6DS3_CTRL6_C, 0x03, 0x01);
		currentGyroLPF = 0;
		return 0;
	}
	static const float factors[] = {0.317f, 0.480f, 0.350f, 0.221f};
	int best_idx = 0;
	int best_bw = (int)(factors[0] * currentGyroODR);
	int best_dist = abs(best_bw - lpf_hz);
	for (int i = 1; i < 4; i++) {
		int bw = (int)(factors[i] * currentGyroODR);
		int d = abs(bw - lpf_hz);
		if (d < best_dist) { best_dist = d; best_bw = bw; best_idx = i; }
	}
	rmwByteI2C(wire, IMUAddress, LSM6DS3_CTRL6_C, 0x03, (uint8_t)best_idx);
	currentGyroLPF = best_bw;
	return best_bw;
}
