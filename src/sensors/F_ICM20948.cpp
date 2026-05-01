#include "F_ICM20948.hpp"

int ICM20948::init(calData cal, uint8_t address)
{
	IMUAddress = address;

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

	selectBank(0);
	if (readByteI2C(wire, IMUAddress, ICM20948_WHO_AM_I) != ICM20948_WHOAMI_DEFAULT_VALUE) {
		return -1;
	}

	// Reset
	writeByteI2C(wire, IMUAddress, ICM20948_PWR_MGMT_1, 0x80);
	delay(100);

	// Wake up
	writeByteI2C(wire, IMUAddress, ICM20948_PWR_MGMT_1, 0x01);
	writeByteI2C(wire, IMUAddress, ICM20948_PWR_MGMT_2, 0x00); // Enable accel and gyro
	delay(200);

	selectBank(2);
	writeByteI2C(wire, IMUAddress, ICM20948_GYRO_SMPLRT_DIV, 0x00);   // Gyro ODR = 1.1 kHz
	// GYRO_CONFIG_1: DLPF_CFG=001 (~119 Hz BW), FS_SEL=11 (2000 dps), FCHOICE=1
	uint8_t c = (0x01 << 3) | (0x03 << 1) | 0x01;
	writeByteI2C(wire, IMUAddress, ICM20948_GYRO_CONFIG_1, c);

	writeByteI2C(wire, IMUAddress, ICM20948_ACCEL_SMPLRT_DIV_1, 0x00);
	writeByteI2C(wire, IMUAddress, ICM20948_ACCEL_SMPLRT_DIV_2, 0x00); // Accel ODR = 1125 Hz
	// ACCEL_CONFIG: DLPF_CFG=001 (~119 Hz BW), FS_SEL=11 (16g), FCHOICE=1
	c = (0x01 << 3) | (0x03 << 1) | 0x01;
	writeByteI2C(wire, IMUAddress, ICM20948_ACCEL_CONFIG, c);

	selectBank(0);
	// Enable I2C master
	writeByteI2C(wire, IMUAddress, ICM20948_USER_CTRL, 0x20);
	selectBank(3);
	writeByteI2C(wire, IMUAddress, ICM20948_I2C_MST_CTRL, 0x07);  // 345.6 kHz clock
	selectBank(0);
	delay(10);

	// Soft-reset AK09916
	writeAK(AK09916_CNTL3, 0x01);
	delay(100);

	bool found = false;
	for (int i = 0; i < 10 && !found; i++) {
		if (readAK(AK09916_WIA2) == 0x09) {
			found = true;
		} else {
			writeByteI2C(wire, IMUAddress, ICM20948_USER_CTRL, 0x02);  // I2C_MST_RST
			delay(10);
			writeByteI2C(wire, IMUAddress, ICM20948_USER_CTRL, 0x20);  // I2C_MST_EN
			delay(10);
		}
	}
	if (!found) return -2;

	// Power-down then start 100 Hz continuous measurement
	writeAK(AK09916_CNTL2, 0x00);
	delay(10);
	writeAK(AK09916_CNTL2, 0x08);
	delay(10);

	// Configure SLV0 to automatically burst-read 8 bytes from AK09916 HXL each cycle
	selectBank(3);
	writeByteI2C(wire, IMUAddress, ICM20948_I2C_SLV0_ADDR, 0x80 | AK09916_ADDRESS);
	writeByteI2C(wire, IMUAddress, ICM20948_I2C_SLV0_REG,  AK09916_HXL);
	writeByteI2C(wire, IMUAddress, ICM20948_I2C_SLV0_CTRL, 0x88);  // EN + 8 bytes
	selectBank(0);
	delay(10);

	magInitialized = true;
	return 0;
}

void ICM20948::update()
{
	// Read 20 bytes: ACCEL(6) + GYRO(6) + TEMP(2) + EXT_SLV_SENS_DATA(6 of 8 mag bytes)
	uint8_t rawData[20] = {0};
	readBytesI2C(wire, IMUAddress, ICM20948_ACCEL_XOUT_H, 20, &rawData[0]);

	accel.timestamp = micros();
	gyro.timestamp = accel.timestamp;
	mag.timestamp = gyro.timestamp;

	int16_t rawAccel[3], rawGyro[3], rawTemp;
	rawAccel[0] = ((int16_t)rawData[0]  << 8) | rawData[1];
	rawAccel[1] = ((int16_t)rawData[2]  << 8) | rawData[3];
	rawAccel[2] = ((int16_t)rawData[4]  << 8) | rawData[5];
	rawGyro[0]  = ((int16_t)rawData[6]  << 8) | rawData[7];
	rawGyro[1]  = ((int16_t)rawData[8]  << 8) | rawData[9];
	rawGyro[2]  = ((int16_t)rawData[10] << 8) | rawData[11];
	rawTemp     = ((int16_t)rawData[12] << 8) | rawData[13];

	temperature = (float)rawTemp / 333.87f + 21.0f;

	float ax = (float)rawAccel[0] * aRes - calibration.accelBias[0];
	float ay = (float)rawAccel[1] * aRes - calibration.accelBias[1];
	float az = (float)rawAccel[2] * aRes - calibration.accelBias[2];

	float gx = (float)rawGyro[0] * gRes - calibration.gyroBias[0];
	float gy = (float)rawGyro[1] * gRes - calibration.gyroBias[1];
	float gz = (float)rawGyro[2] * gRes - calibration.gyroBias[2];

	// Bytes 14-19: AK09916 HXL,HXH,HYL,HYH,HZL,HZH
	if (magInitialized) {
		int16_t mc[3];
		mc[0] = (int16_t)((rawData[15] << 8) | rawData[14]);
		mc[1] = (int16_t)((rawData[17] << 8) | rawData[16]);
		mc[2] = (int16_t)((rawData[19] << 8) | rawData[18]);

		// Remap AK09916 axis to align with ICM20948
		float mx = ((float)mc[1] * mRes - calibration.magBias[1]) * calibration.magScale[1];
		float my = ((float)mc[0] * mRes - calibration.magBias[0]) * calibration.magScale[0];
		float mz = -((float)mc[2] * mRes - calibration.magBias[2]) * calibration.magScale[2];

		switch (geometryIndex) {
		case 0: mag.magX = mx;  mag.magY = my;  mag.magZ = mz;  break;
		case 1: mag.magX = -my; mag.magY = mx;  mag.magZ = mz;  break;
		case 2: mag.magX = -mx; mag.magY = -my; mag.magZ = mz;  break;
		case 3: mag.magX = my;  mag.magY = -mx; mag.magZ = mz;  break;
		case 4: mag.magX = -mz; mag.magY = -my; mag.magZ = -mx; break;
		case 5: mag.magX = -mz; mag.magY = mx;  mag.magZ = -my; break;
		case 6: mag.magX = -mz; mag.magY = my;  mag.magZ = mx;  break;
		case 7: mag.magX = -mz; mag.magY = -mx; mag.magZ = my;  break;
		}
	}

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

void ICM20948::getAccel(AccelData* out)
{
	memcpy(out, &accel, sizeof(accel));
}

void ICM20948::getGyro(GyroData* out)
{
	memcpy(out, &gyro, sizeof(gyro));
}

void ICM20948::getMag(MagData* out)
{
	memcpy(out, &mag, sizeof(mag));
}

int ICM20948::setAccelRange(int range)
{
	uint8_t fsSel;
	if (range == 16) {
		aRes = 16.f / 32768.f;
		fsSel = 0x03;
	}
	else if (range == 8) {
		aRes = 8.f / 32768.f;
		fsSel = 0x02;
	}
	else if (range == 4) {
		aRes = 4.f / 32768.f;
		fsSel = 0x01;
	}
	else if (range == 2) {
		aRes = 2.f / 32768.f;
		fsSel = 0x00;
	}
	else {
		return -1;
	}
	// DLPFCFG=1
	uint8_t c = (0x01 << 3) | (fsSel << 1) | 0x01;
	selectBank(2);
	writeByteI2C(wire, IMUAddress, ICM20948_ACCEL_CONFIG, c);
	selectBank(0);
	return 0;
}

int ICM20948::setGyroRange(int range)
{
	uint8_t fsSel;
	if (range == 2000) {
		gRes = 2000.f / 32768.f;
		fsSel = 0x03;
	}
	else if (range == 1000) {
		gRes = 1000.f / 32768.f;
		fsSel = 0x02;
	}
	else if (range == 500) {
		gRes = 500.f / 32768.f;
		fsSel = 0x01;
	}
	else if (range == 250) {
		gRes = 250.f / 32768.f;
		fsSel = 0x00;
	}
	else {
		return -1;
	}
	// DLPFCFG=1
	uint8_t c = (0x01 << 3) | (fsSel << 1) | 0x01;
	selectBank(2);
	writeByteI2C(wire, IMUAddress, ICM20948_GYRO_CONFIG_1, c);
	selectBank(0);
	return 0;
}

void ICM20948::calibrateAccelGyro(calData* cal)
{
	int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	// Reset
	selectBank(0);
	writeByteI2C(wire, IMUAddress, ICM20948_PWR_MGMT_1, 0x80);
	delay(100);

	// Wake up
	writeByteI2C(wire, IMUAddress, ICM20948_PWR_MGMT_1, 0x01);
	writeByteI2C(wire, IMUAddress, ICM20948_PWR_MGMT_2, 0x00);
	delay(200);

	// 250 dps and 2g for maximum sensitivity
	selectBank(2);
	writeByteI2C(wire, IMUAddress, ICM20948_GYRO_SMPLRT_DIV,    0x00);
	writeByteI2C(wire, IMUAddress, ICM20948_GYRO_CONFIG_1,      0x01); // 250 dps, DLPF on
	writeByteI2C(wire, IMUAddress, ICM20948_ACCEL_SMPLRT_DIV_1, 0x00);
	writeByteI2C(wire, IMUAddress, ICM20948_ACCEL_SMPLRT_DIV_2, 0x00);
	writeByteI2C(wire, IMUAddress, ICM20948_ACCEL_CONFIG,       0x01); // 2g, DLPF on
	selectBank(0);
	delay(100);

	// Collect 1000 samples directly from output registers
	const int NUM_SAMPLES = 1000;
	uint8_t rawData[12];
	for (int ii = 0; ii < NUM_SAMPLES; ii++)
	{
		readBytesI2C(wire, IMUAddress, ICM20948_ACCEL_XOUT_H, 12, rawData);
		accel_bias[0] += (int16_t)((rawData[0]  << 8) | rawData[1]);
		accel_bias[1] += (int16_t)((rawData[2]  << 8) | rawData[3]);
		accel_bias[2] += (int16_t)((rawData[4]  << 8) | rawData[5]);
		gyro_bias[0]  += (int16_t)((rawData[6]  << 8) | rawData[7]);
		gyro_bias[1]  += (int16_t)((rawData[8]  << 8) | rawData[9]);
		gyro_bias[2]  += (int16_t)((rawData[10] << 8) | rawData[11]);
		delay(1);
	}
	accel_bias[0] /= NUM_SAMPLES;
	accel_bias[1] /= NUM_SAMPLES;
	accel_bias[2] /= NUM_SAMPLES;
	gyro_bias[0]  /= NUM_SAMPLES;
	gyro_bias[1]  /= NUM_SAMPLES;
	gyro_bias[2]  /= NUM_SAMPLES;

	// Remove gravity from the axis pointing up
	const int32_t accelsensitivity = 16384; // LSB/g at 2g
	switch (geometryIndex) {
	case 0: case 1: case 2: case 3:
		if (accel_bias[2] > 0L) accel_bias[2] -= accelsensitivity;
		else                     accel_bias[2] += accelsensitivity;
		break;
	case 4: case 6:
		if (accel_bias[0] > 0L) accel_bias[0] -= accelsensitivity;
		else                     accel_bias[0] += accelsensitivity;
		break;
	case 5: case 7:
		if (accel_bias[1] > 0L) accel_bias[1] -= accelsensitivity;
		else                     accel_bias[1] += accelsensitivity;
		break;
	}

	cal->accelBias[0] = (float)accel_bias[0] / 16384.f;
	cal->accelBias[1] = (float)accel_bias[1] / 16384.f;
	cal->accelBias[2] = (float)accel_bias[2] / 16384.f;
	cal->gyroBias[0]  = (float)gyro_bias[0]  / 131.f;
	cal->gyroBias[1]  = (float)gyro_bias[1]  / 131.f;
	cal->gyroBias[2]  = (float)gyro_bias[2]  / 131.f;
	cal->valid = true;
}

void ICM20948::calibrateMag(calData* cal)
{
	int32_t mag_bias[3] = { 0, 0, 0 }, mag_scale[3] = { 0, 0, 0 };
	int16_t mag_max[3] = { -32767, -32767, -32767 }, mag_min[3] = { 32767, 32767, 32767 }, mag_temp[3] = { 0, 0, 0 };

	// ~15 seconds at 100 Hz
	for (uint16_t ii = 0; ii < 1500; ii++) {
		uint8_t rawData[6] = {0};
		// SLV0 deposits AK09916 data here; little-endian byte order
		readBytesI2C(wire, IMUAddress, ICM20948_EXT_SLV_SENS_DATA_00, 6, rawData);
		mag_temp[0] = (int16_t)((rawData[1] << 8) | rawData[0]);
		mag_temp[1] = (int16_t)((rawData[3] << 8) | rawData[2]);
		mag_temp[2] = (int16_t)((rawData[5] << 8) | rawData[4]);
		for (int jj = 0; jj < 3; jj++) {
			if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
			if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
		}
		delay(12);
	}

	mag_bias[0] = (mag_max[0] + mag_min[0]) / 2;
	mag_bias[1] = (mag_max[1] + mag_min[1]) / 2;
	mag_bias[2] = (mag_max[2] + mag_min[2]) / 2;

	cal->magBias[0] = (float)mag_bias[0] * mRes;
	cal->magBias[1] = (float)mag_bias[1] * mRes;
	cal->magBias[2] = (float)mag_bias[2] * mRes;

	mag_scale[0] = (mag_max[0] - mag_min[0]) / 2;
	mag_scale[1] = (mag_max[1] - mag_min[1]) / 2;
	mag_scale[2] = (mag_max[2] - mag_min[2]) / 2;

	float avg_rad = (mag_scale[0] + mag_scale[1] + mag_scale[2]) / 3.0f;

	cal->magScale[0] = avg_rad / (float)mag_scale[0];
	cal->magScale[1] = avg_rad / (float)mag_scale[1];
	cal->magScale[2] = avg_rad / (float)mag_scale[2];

	cal->valid = true;
}
