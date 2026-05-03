#include "F_BMX055.hpp"

int BMX055::init(calData cal, uint8_t address)
{
	//initialize address variable and calibration data.

	if (address == 0x18 || address == 0x68 || address == 0x10) {
		AccelAddress = 0x18;
		GyroAddress = 0x68;
		MagAddress = 0x10;
	}
	else if (address == 0x19 || address == 0x69 || address == 0x13) {
		AccelAddress = 0x19;
		GyroAddress = 0x69;
		MagAddress = 0x13;
	}
	else {
		return -1;
	}

	if (cal.valid == false)
	{
		calibration = { 0 };
		calibration.magScale[0] = 1.f;
		calibration.magScale[1] = 1.f;
		calibration.magScale[2] = 1.f;
	}
	else
	{
		calibration = cal;
	}

	if (!(readByteI2C(wire, AccelAddress, BMX055_ACCD_CHIPID) == BMX055_ACCEL_ID)) {
		return -2;
	}
	if (!(readByteI2C(wire, GyroAddress, BMX055_GYR_CHIP_ID) == BMX055_GYRO_ID)) {
		return -3;
	}
	if (!(readByteI2C(wire, MagAddress, BMX055_MAG_CHIP_ID) == BMX055_MAG_ID)) {
		return -4;
	}

	// Reset sensor.
	writeByteI2C(wire, AccelAddress, BMX055_BGW_SOFTRESET, 0xB6);
	writeByteI2C(wire, GyroAddress, BMX055_GYR_BGW_SOFTRESET, 0xB6);
	writeByteI2C(wire, MagAddress, BMX055_MAG_PWR_CTRL_SR, 0x83);
	delay(100);

	// Set accelerometer range
	writeByteI2C(wire, AccelAddress, BMX055_PMU_RANGE, 0x0C); // Write '1100' into bits 3:0, setting accelerometer into 16g range.

	// Set LPF
	writeByteI2C(wire, AccelAddress, BMX055_PMU_BW, 0x0B); // Write '01011' into bits 4:0, setting the accelerometer lpf bandwidth to 62.5hz

	// Enter normal mode
	writeByteI2C(wire, AccelAddress, BMX055_PMU_LPW, 0x00); 

	// Set Gyro range
	writeByteI2C(wire, GyroAddress, BMX055_GYR_RANGE, 0x00);	// Write '000' into bits 2:0, setting gyro into 2000dps range.

	// Set LPF
	writeByteI2C(wire, GyroAddress, BMX055_GYR_BW, 0x03); // Write '0011' into bits 3:0, setting the gyro lpf bandwidth to 47hz ;;;;;;;;;;;; THIS LIMITS ODR TO 400HZ

	// Enter normal mode
	writeByteI2C(wire, GyroAddress, BMX055_GYR_LPM1, 0x00);

	// Enable all magnetometer axis
	writeByteI2C(wire, MagAddress, BMX055_MAG_INT_REG_1, 0x38);

	// Set magnetometer repetitions for X and Y axis to 29
	writeByteI2C(wire, MagAddress, BMX055_MAG_REP_CTRL_XY, 0x0E);

	// Set magnetometer repetitions for Z axis to 53
	writeByteI2C(wire, MagAddress, BMX055_MAG_REP_CTRL_Z, 0x32);

	// Set magnetometer output data rate to 30hz and enter normal mode
	writeByteI2C(wire, MagAddress, BMX055_MAG_OM_ODR_SELF, 0x38);

	delay (100);
	return 0;
}

void BMX055::update()
{
	bool accelReady = readByteI2C(wire, AccelAddress, BMX055_ACCD_X_LSB) & 0x01;
	bool gyroReady  = readByteI2C(wire, GyroAddress,  BMX055_GYR_INT_STATUS_0) & 0x80;
	bool magReady   = readByteI2C(wire, MagAddress,   BMX055_MAG_RHALL_LSB_DRDY) & 0x01;
	if (!accelReady && !gyroReady && !magReady) return;

	int16_t AccelCount[3] = {0};
	int16_t GyroCount[3]  = {0};
	int16_t MagCount[3]   = {0};
	uint8_t rawDataAccel[7];
	uint8_t rawDataGyro[6];
	uint8_t rawDataMag[6];

	if (accelReady) {
		readBytesI2C(wire, AccelAddress, BMX055_ACCD_X_LSB, 7, &rawDataAccel[0]);
		accel.timestamp = micros();
		AccelCount[0] = ((rawDataAccel[1] << 8) | (rawDataAccel[0] & 0xF0)) >> 4;
		AccelCount[1] = ((rawDataAccel[3] << 8) | (rawDataAccel[2] & 0xF0)) >> 4;
		AccelCount[2] = ((rawDataAccel[5] << 8) | (rawDataAccel[4] & 0xF0)) >> 4;
		temperature = -((rawDataAccel[6] * -0.5f) * (86.5f - -40.5f) / (float)(128.f) - 40.5f) - 20.f;
	}

	if (gyroReady) {
		readBytesI2C(wire, GyroAddress, BMX055_GYR_RATE_X_LSB, 6, &rawDataGyro[0]);
		gyro.timestamp = micros();
		GyroCount[0] = (rawDataGyro[1] << 8) | rawDataGyro[0];
		GyroCount[1] = (rawDataGyro[3] << 8) | rawDataGyro[2];
		GyroCount[2] = (rawDataGyro[5] << 8) | rawDataGyro[4];
	}

	if (magReady) {
		readBytesI2C(wire, MagAddress, BMX055_MAG_X_LSB, 6, &rawDataMag[0]);
		mag.timestamp = micros();
		MagCount[0] = ((rawDataMag[1] << 8) | (rawDataMag[0] & 0xF8)) >> 3;
		MagCount[1] = ((rawDataMag[3] << 8) | (rawDataMag[2] & 0xF8)) >> 3;
		MagCount[2] = ((rawDataMag[5] << 8) | (rawDataMag[4] & 0xFE)) >> 1;
	}

	float ax = AccelCount[0] * aRes - calibration.accelBias[0];
	float ay = AccelCount[1] * aRes - calibration.accelBias[1];
	float az = AccelCount[2] * aRes - calibration.accelBias[2];

	float gx = GyroCount[0] * gRes - calibration.gyroBias[0];
	float gy = GyroCount[1] * gRes - calibration.gyroBias[1];
	float gz = GyroCount[2] * gRes - calibration.gyroBias[2];

	float mx = (float)(MagCount[0] * mResXY - calibration.magBias[0]) * calibration.magScale[0];
	float my = (float)(MagCount[1] * mResXY - calibration.magBias[1]) * calibration.magScale[1];
	float mz = (float)(MagCount[2] * mResZ  - calibration.magBias[2]) * calibration.magScale[2];

	switch (geometryIndex) {
	case 0:
		accel.accelX = ax;		gyro.gyroX = gx;		mag.magX = mx;
		accel.accelY = ay;		gyro.gyroY = gy;		mag.magY = my;
		accel.accelZ = az;		gyro.gyroZ = gz;		mag.magZ = mz;
		break;
	case 1:
		accel.accelX = -ay;		gyro.gyroX = -gy;		mag.magX = -my;
		accel.accelY = ax;		gyro.gyroY = gx;		mag.magY = mx;
		accel.accelZ = az;		gyro.gyroZ = gz;		mag.magZ = mz;
		break;
	case 2:
		accel.accelX = -ax;		gyro.gyroX = -gx;		mag.magX = mx;
		accel.accelY = -ay;		gyro.gyroY = -gy;		mag.magY = my;
		accel.accelZ = az;		gyro.gyroZ = gz;		mag.magZ = mz;
		break;
	case 3:
		accel.accelX = ay;		gyro.gyroX = gy;		mag.magX = my;
		accel.accelY = -ax;		gyro.gyroY = -gx;		mag.magY = -mx;
		accel.accelZ = az;		gyro.gyroZ = gz;		mag.magZ = mz;
		break;
	case 4:
		accel.accelX = -az;		gyro.gyroX = -gz;		mag.magX = -mz;
		accel.accelY = -ay;		gyro.gyroY = -gy;		mag.magY = -my;
		accel.accelZ = -ax;		gyro.gyroZ = -gx;		mag.magZ = -mx;
		break;
	case 5:
		accel.accelX = -az;		gyro.gyroX = -gz;		mag.magX = -mz;
		accel.accelY = ax;		gyro.gyroY = gx;		mag.magY = mx;
		accel.accelZ = -ay;		gyro.gyroZ = -gy;		mag.magZ = -my;
		break;
	case 6:
		accel.accelX = -az;		gyro.gyroX = -gz;		mag.magX = -mz;
		accel.accelY = ay;		gyro.gyroY = gy;		mag.magY = my;
		accel.accelZ = ax;		gyro.gyroZ = gx;		mag.magZ = mx;
		break;
	case 7:
		accel.accelX = -az;		gyro.gyroX = -gz;		mag.magX = -mz;
		accel.accelY = -ax;		gyro.gyroY = -gx;		mag.magY = -mx;
		accel.accelZ = ay;		gyro.gyroZ = gy;		mag.magZ = my;
		break;
	}
}

void BMX055::getAccel(AccelData* out) 
{
	memcpy(out, &accel, sizeof(accel));
}
void BMX055::getGyro(GyroData* out) 
{
	memcpy(out, &gyro, sizeof(gyro));
}
void BMX055::getMag(MagData* out)
{
	memcpy(out, &mag, sizeof(mag));
}

int BMX055::setAccelRange(int range) {
	uint8_t c;
	if (range == 16) {
		aRes = 16.f / 2048.f;			//ares value for full range (16g) readings
		c = 0b1100;
	}
	else if (range == 8) {
		aRes = 8.f / 2048.f;			//ares value for range (8g) readings
		c = 0b1000;
	}
	else if (range == 4) {
		aRes = 4.f / 2048.f;			//ares value for range (4g) readings
		c = 0b0101;
	}
	else if (range == 2) {
		aRes = 2.f / 2048.f;			//ares value for range (2g) readings
		c = 0b0011;
	}
	else {
		return -1;
	}
	rmwByteI2C(wire, AccelAddress, BMX055_PMU_RANGE, 0b00001111, c); // Write new BMI055_PMU_RANGE register value
	return 0;
}

int BMX055::setGyroRange(int range) {
	uint8_t c;
	if (range == 2000) {
		gRes = 2000.f / 32768.f;			//ares value for full range (2000dps) readings
		c = 0b000;
	}
	else if (range == 1000) {
		gRes = 1000.f / 32768.f;			//ares value for range (1000dps) readings
		c = 0b001;
	}
	else if (range == 500) {
		gRes = 500.f / 32768.f;			//ares value for range (500dps) readings
		c = 0b010;
	}
	else if (range == 250) {
		gRes = 250.f / 32768.f;			//ares value for range (250dps) readings
		c = 0b011;
	}
	else if (range == 125) {
		gRes = 125.f / 32768.f;			//ares value for range (125dps) readings
		c = 0b100;
	}
	else {
		return -1;
	}
	rmwByteI2C(wire, AccelAddress, BMX055_GYR_RANGE, 0b00000111, c); // Write new BMX055_GYR_RANGE register value
	return 0;
}

void BMX055::calibrateAccelGyro(calData* cal) 
{
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t packet_count = 64; // How many sets of full gyro and accelerometer data for averaging;
	float gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	float  gyrosensitivity = 125.f / 32768.f;			//gres value for full range (2000dps) readings (16 bit)
	float  accelsensitivity = 2.f / 2048.f;				//ares value for full range (16g) readings (12 bit)

	// Reset sensor.
	writeByteI2C(wire, AccelAddress, BMX055_BGW_SOFTRESET, 0xB6);
	writeByteI2C(wire, GyroAddress, BMX055_GYR_BGW_SOFTRESET, 0xB6);
	delay(100);
	// Set accelerometer range
	writeByteI2C(wire, AccelAddress, BMX055_PMU_RANGE, 0x03); // Write '0011' into bits 3:0, setting accelerometer into 2g range, maximum sensitivity
	// Set LPF
	writeByteI2C(wire, AccelAddress, BMX055_PMU_BW, 0x0C); // Write '01100' into bits 4:0, setting the accelerometer lpf bandwidth to 125hz
	// Enter normal mode
	writeByteI2C(wire, AccelAddress, BMX055_PMU_LPW, 0x00);
	// Reset sensor.
	// Set Gyro range
	writeByteI2C(wire, GyroAddress, BMX055_GYR_RANGE, 0x04);	// Write '100' into bits 2:0, setting gyro into 125dps range, maximum sensitivity
	// Set LPF
	writeByteI2C(wire, GyroAddress, BMX055_GYR_BW, 0x03); // Write '0011' into bits 3:0, setting the gyro lpf bandwidth to 47hz ;;;;;;;;;;;; THIS LIMITS ODR TO 400HZ
	// Enter normal mode
	writeByteI2C(wire, GyroAddress, BMX055_GYR_LPM1, 0x00);
	delay(10);

	for (int i = 0; i < packet_count; i++)
	{
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };

		readBytesI2C(wire, AccelAddress, BMX055_ACCD_X_LSB, 6, &data[0]);       // Read the 7 raw accelerometer data registers into data array
		readBytesI2C(wire, GyroAddress, BMX055_GYR_RATE_X_LSB, 6, &data[6]);   // Read the 6 raw gyroscope data registers into data array

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

void BMX055::calibrateMag(calData* cal)
{
	uint16_t ii = 0, sample_count = 0;
	float mag_bias[3] = { 0, 0, 0 }, mag_scale[3] = { 0, 0, 0 };
	float mag_max[3] = { 0.f, 0.f, 0.f }, mag_min[3] = { 0.f, 0.f, 0.f };

	// shoot for ~fifteen seconds of mag data
	sample_count = 450;  // at 30 Hz ODR, new mag data is available every 33 ms

	for (ii = 0; ii < sample_count; ii++)
	{
		int16_t MagCount[3];
		uint8_t rawDataMag[6];
		float magReading[3];

		readBytesI2C(wire, MagAddress, BMX055_MAG_X_LSB, 6, &rawDataMag[0]);				// Read the 6 raw magnetometer data registers into data array

		MagCount[0] = ((rawDataMag[1] << 8) | (rawDataMag[0] & 0xF8)) >> 3;		  // Turn the MSB and LSB into a signed 13-bit value
		MagCount[1] = ((rawDataMag[3] << 8) | (rawDataMag[2] & 0xF8)) >> 3;
		MagCount[2] = ((rawDataMag[5] << 8) | (rawDataMag[4] & 0xFE)) >> 1;	      // Turn the MSB and LSB into a signed 15-bit value

		magReading[0] = (float)(MagCount[0] * mResXY);
		magReading[1] = (float)(MagCount[1] * mResXY);
		magReading[2] = (float)(MagCount[2] * mResZ);

		for (int jj = 0; jj < 3; jj++)
		{
			if (MagCount[jj] > mag_max[jj]) mag_max[jj] = MagCount[jj];
			if (MagCount[jj] < mag_min[jj]) mag_min[jj] = MagCount[jj];
		}
		delay(33); // at 30 Hz ODR, new mag data is available every 33 ms
	}

	// Get hard iron correction
	mag_bias[0] = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias
	mag_bias[1] = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias
	mag_bias[2] = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias

	cal->magBias[0] = mag_bias[0];
	cal->magBias[1] = mag_bias[1];
	cal->magBias[2] = mag_bias[2];

	// Get soft iron correction estimate
	mag_scale[0] = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
	mag_scale[1] = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
	mag_scale[2] = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	cal->magScale[0] = avg_rad / ((float)mag_scale[0]);
	cal->magScale[1] = avg_rad / ((float)mag_scale[1]);
	cal->magScale[2] = avg_rad / ((float)mag_scale[2]);
}

// Accel ODR = 2 * bandwidth. PMU_BW values 0x08-0x0F give BW 7.81-1000 Hz.
static const int BMX055_ACCEL_ODR_TABLE[] = {15, 31, 62, 125, 250, 500, 1000, 2000};
static const uint8_t BMX055_ACCEL_BW_REG[] = {0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};

// Gyro: distinct ODR values with highest available bandwidth for each.
static const int BMX055_GYRO_ODR_TABLE[] = {100, 200, 400, 1000};
static const uint8_t BMX055_GYRO_BW_REG[] = {0x07, 0x06, 0x00, 0x01};

int BMX055::setAccelODR(int odr_hz) {
	if (odr_hz <= 0) return -1;
	int actual = nearestHigherODR(BMX055_ACCEL_ODR_TABLE, 8, odr_hz);
	int idx = 0;
	while (BMX055_ACCEL_ODR_TABLE[idx] != actual) idx++;
	rmwByteI2C(wire, AccelAddress, BMX055_PMU_BW, 0b00011111, BMX055_ACCEL_BW_REG[idx]);
	currentAccelODR = actual;
	return actual;
}

int BMX055::setGyroODR(int odr_hz) {
	if (odr_hz <= 0) return -1;
	int actual = nearestHigherODR(BMX055_GYRO_ODR_TABLE, 4, odr_hz);
	int idx = 0;
	while (BMX055_GYRO_ODR_TABLE[idx] != actual) idx++;
	rmwByteI2C(wire, GyroAddress, BMX055_GYR_BW, 0b00001111, BMX055_GYRO_BW_REG[idx]);
	currentGyroODR = actual;
	return actual;
}
// BMX055 mag MAG_OM_ODR_SELF bits[5:3] ODR encoding (ascending Hz order with matching codes)
static const int BMX055_MAG_ODR_TABLE[] = {2, 6, 8, 10, 15, 20, 25, 30};
static const uint8_t BMX055_MAG_ODR_REG[] = {0x08, 0x10, 0x18, 0x00, 0x20, 0x28, 0x30, 0x38};

int BMX055::setMagODR(int odr_hz) {
	if (odr_hz <= 0) return -1;
	int actual = nearestHigherODR(BMX055_MAG_ODR_TABLE, 8, odr_hz);
	int idx = 0;
	while (BMX055_MAG_ODR_TABLE[idx] != actual) idx++;
	rmwByteI2C(wire, MagAddress, BMX055_MAG_OM_ODR_SELF, 0x38, BMX055_MAG_ODR_REG[idx]);
	currentMagODR = actual;
	return actual;
}