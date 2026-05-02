#include "F_BMM150.hpp"

int BMM150::init(calData cal, uint8_t address)
{
	IMUAddress = address;
	if (cal.valid == false) {
		calibration = { 0 };
		calibration.magScale[0] = 1.f;
		calibration.magScale[1] = 1.f;
		calibration.magScale[2] = 1.f;
	} else {
		calibration = cal;
	}

	// Enter suspend mode, then power on
	writeByteI2C(wire, IMUAddress, BMM150_PWR, 0x00);
	delay(5);
	writeByteI2C(wire, IMUAddress, BMM150_PWR, 0x01);
	delay(3);
	if (readByteI2C(wire, IMUAddress, BMM150_WHOAMI) != BMM150_WHOAMI_VALUE) {
		return -1;
	}

	// Soft reset (bit7=1), then re-enable power
	writeByteI2C(wire, IMUAddress, BMM150_PWR, 0x82);
	delay(10);
	writeByteI2C(wire, IMUAddress, BMM150_PWR, 0x01);
	delay(3);

	loadTrim();

	// Normal mode, 30 Hz ODR
	writeByteI2C(wire, IMUAddress, BMM150_CONFIG, 0x38);

	// Regular preset repetitions
	writeByteI2C(wire, IMUAddress, BMM150_REPXY, 0x04);
	writeByteI2C(wire, IMUAddress, BMM150_REPZ, 0x07);

	// Enable all axes (DIS_X/Y/Z bits 3:5 = 0)
	writeByteI2C(wire, IMUAddress, BMM150_INT1, 0x00);
	return 0;
}

void BMM150::loadTrim()
{
	uint8_t buf2[2], buf4[4], buf10[10];

	readBytesI2C(wire, IMUAddress, BMM150_TRIM_X1Y1, 2,  buf2);
	readBytesI2C(wire, IMUAddress, BMM150_TRIM_Z4,   4,  buf4);
	readBytesI2C(wire, IMUAddress, BMM150_TRIM_Z2,   10, buf10);

	trim.digX1   = (int8_t)buf2[0];
	trim.digY1   = (int8_t)buf2[1];
	trim.digZ4   = (int16_t)(((uint16_t)buf4[1] << 8) | buf4[0]);
	trim.digX2   = (int8_t)buf4[2];
	trim.digY2   = (int8_t)buf4[3];
	trim.digZ2   = (int16_t)(((uint16_t)buf10[1] << 8) | buf10[0]);
	trim.digZ1   = (uint16_t)(((uint16_t)buf10[3] << 8) | buf10[2]);
	trim.digXYZ1 = (uint16_t)((((uint16_t)buf10[5] & 0x7F) << 8) | buf10[4]);
	trim.digZ3   = (int16_t)(((uint16_t)buf10[7] << 8) | buf10[6]);
	trim.digXY2  = (int8_t)buf10[8];
	trim.digXY1  = buf10[9];
}

float BMM150::compensateX(int16_t raw, uint16_t rhall)
{
	if (raw == BMM150_OVF_XY || rhall == 0 || trim.digXYZ1 == 0)
		return 0.f;
	float c0 = ((float)trim.digXYZ1) * 16384.f / rhall;
	float r  = c0 - 16384.f;
	float c1 = (float)trim.digXY2 * (r * r / 268435456.f);
	float c2 = c1 + r * (float)trim.digXY1 / 16384.f;
	float c3 = (float)trim.digX2 + 160.f;
	float c4 = raw * ((c2 + 256.f) * c3);
	return (c4 / 8192.f + (float)trim.digX1 * 8.f) / 16.f;
}

float BMM150::compensateY(int16_t raw, uint16_t rhall)
{
	if (raw == BMM150_OVF_XY || rhall == 0 || trim.digXYZ1 == 0)
		return 0.f;
	float c0 = ((float)trim.digXYZ1) * 16384.f / rhall;
	float r  = c0 - 16384.f;
	float c1 = (float)trim.digXY2 * (r * r / 268435456.f);
	float c2 = c1 + r * (float)trim.digXY1 / 16384.f;
	float c3 = (float)trim.digY2 + 160.f;
	float c4 = raw * ((c2 + 256.f) * c3);
	return (c4 / 8192.f + (float)trim.digY1 * 8.f) / 16.f;
}

float BMM150::compensateZ(int16_t raw, uint16_t rhall)
{
	if (raw == BMM150_OVF_Z || trim.digZ2 == 0 || trim.digZ1 == 0 || rhall == 0 || trim.digXYZ1 == 0)
		return 0.f;
	float z0 = (float)raw - (float)trim.digZ4;
	float z1 = (float)rhall - (float)trim.digXYZ1;
	float z2 = (float)trim.digZ3 * z1;
	float z3 = (float)trim.digZ1 * (float)rhall / 32768.f;
	float z4 = (float)trim.digZ2 + z3;
	return ((z0 * 131072.f - z2) / (z4 * 4.f)) / 16.f;
}

void BMM150::update()
{
	uint8_t rawData[8] = { 0 };
	readBytesI2C(wire, IMUAddress, BMM150_DATAX, 8, &rawData[0]);

	if (!(rawData[6] & 0x01)) return; // data ready bit
	
	mag.timestamp = micros();

	// X and Y: 13-bit signed (sign-extend from bit 12)
	int16_t rawX = (int16_t)(((int16_t)(int8_t)rawData[1]) * 32) | (rawData[0] >> 3);
	int16_t rawY = (int16_t)(((int16_t)(int8_t)rawData[3]) * 32) | (rawData[2] >> 3);

	// Z: 15-bit signed (sign-extend from bit 14)
	int16_t rawZ = (int16_t)(((int16_t)(int8_t)rawData[5]) * 128) | (rawData[4] >> 1);
	
	// RHALL: 14-bit unsigned
	uint16_t rhall = (uint16_t)(((uint16_t)rawData[7] << 6) | (rawData[6] >> 2));

	if (rhall == 0) return;

	float mx = (compensateX(rawX, rhall) - calibration.magBias[0]) * calibration.magScale[0];
	float my = (compensateY(rawY, rhall) - calibration.magBias[1]) * calibration.magScale[1];
	float mz = (compensateZ(rawZ, rhall) - calibration.magBias[2]) * calibration.magScale[2];

	switch (geometryIndex) {
	case 0:
		mag.magX = mx;  
		mag.magY = my;  
		mag.magZ = mz;
		break;
	case 1:
		mag.magX = -my;
		mag.magY = mx;  
		mag.magZ = mz;
		break;
	case 2:
		mag.magX = mx;  
		mag.magY = my;  
		mag.magZ = mz;
		break;
	case 3:
		mag.magX = my;  
		mag.magY = -mx; 
		mag.magZ = mz;
		break;
	case 4:
		mag.magX = -mz;
		mag.magY = -my; 
		mag.magZ = -mx;
		break;
	case 5:
		mag.magX = -mz;
		mag.magY = mx;  
		mag.magZ = -my;
		break;
	case 6:
		mag.magX = -mz; 
		mag.magY = my;  
		mag.magZ = mx;
		break;
	case 7:
		mag.magX = -mz;
		mag.magY = -mx;
		mag.magZ = my;
		break;
	}
}

void BMM150::getMag(MagData* out)
{
	memcpy(out, &mag, sizeof(mag));
}

void BMM150::calibrateMag(calData* cal)
{
	float mag_max[3] = { -1e9f, -1e9f, -1e9f };
	float mag_min[3] = {  1e9f,  1e9f,  1e9f };

	// ~15 seconds at 30 Hz
	for (uint16_t ii = 0; ii < 450; ii++) {
		uint8_t rawData[8] = { 0 };
		readBytesI2C(wire, IMUAddress, BMM150_DATAX, 8, &rawData[0]);
		if (rawData[6] & 0x01) {
			int16_t rawX = (int16_t)(((int16_t)(int8_t)rawData[1]) * 32) | (rawData[0] >> 3);
			int16_t rawY = (int16_t)(((int16_t)(int8_t)rawData[3]) * 32) | (rawData[2] >> 3);
			int16_t rawZ = (int16_t)(((int16_t)(int8_t)rawData[5]) * 128) | (rawData[4] >> 1);
			uint16_t rhall = (uint16_t)(((uint16_t)rawData[7] << 6) | (rawData[6] >> 2));
			if (rhall == 0) { delay(35); continue; }

			float v[3];
			v[0] = compensateX(rawX, rhall);
			v[1] = compensateY(rawY, rhall);
			v[2] = compensateZ(rawZ, rhall);
			for (int j = 0; j < 3; j++) {
				if (v[j] > mag_max[j]) mag_max[j] = v[j];
				if (v[j] < mag_min[j]) mag_min[j] = v[j];
			}
		}
		delay(35);
	}

	cal->magBias[0] = (mag_max[0] + mag_min[0]) / 2.f;
	cal->magBias[1] = (mag_max[1] + mag_min[1]) / 2.f;
	cal->magBias[2] = (mag_max[2] + mag_min[2]) / 2.f;

	float half[3] = {
		(mag_max[0] - mag_min[0]) / 2.f,
		(mag_max[1] - mag_min[1]) / 2.f,
		(mag_max[2] - mag_min[2]) / 2.f
	};
	float avg_rad = (half[0] + half[1] + half[2]) / 3.f;

	cal->magScale[0] = avg_rad / half[0];
	cal->magScale[1] = avg_rad / half[1];
	cal->magScale[2] = avg_rad / half[2];

	cal->valid = true;
}

// BMM150 CONFIG bits[5:3] ODR encoding (ascending Hz order with matching codes)
static const int BMM150_ODR_TABLE[] = {2, 6, 8, 10, 15, 20, 25, 30};
static const uint8_t BMM150_ODR_CODE[] = {1, 2, 3, 0, 4, 5, 6, 7};

int BMM150::setMagODR(int odr_hz) {
	if (odr_hz <= 0) return -1;
	int actual = nearestHigherODR(BMM150_ODR_TABLE, 8, odr_hz);
	int idx = 0;
	while (BMM150_ODR_TABLE[idx] != actual) idx++;
	uint8_t cfg = readByteI2C(wire, IMUAddress, BMM150_CONFIG);
	cfg = (cfg & 0xC7) | (uint8_t)(BMM150_ODR_CODE[idx] << 3);
	writeByteI2C(wire, IMUAddress, BMM150_CONFIG, cfg);
	currentMagODR = actual;
	return actual;
}
