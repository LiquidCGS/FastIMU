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

	if (!(readByteI2C(wire, IMUAddress, QMI8658_WHO_AM_I) == QMI8658_WHO_AM_I_DEFAULT_VALUE)) {
		return -1;
	}

	// reset device
	writeByteI2C(wire, IMUAddress, QMI8658_RESET, 0xFF);	    // Toggle softreset
	delay(100);										// wait for reset
	
	writeByteI2C(wire, IMUAddress, QMI8658_CTRL1, 0x40);		// Enable auto increment

	setAccelODR(500);
	setAccelRange(16);
	setGyroODR(500);
	setGyroRange(2000);
	setAccelLPF(70);
	setGyroLPF(70);

	writeByteI2C(wire, IMUAddress, QMI8658_CTRL7, 0x03);	    // Start up accelerometer and gyro, disable sync
	delay(100);								    	//wait until they're done starting up...

	return 0;
}

void QMI8658::update() {
	uint8_t status = readByteI2C(wire, IMUAddress, QMI8658_STATUS0);
	bool accelReady = status & 0x01;
	bool gyroReady  = status & 0x02;
	if (!accelReady && !gyroReady) return;

	int16_t IMUCount[6];                                          // used to read all 16 bytes at once from the accel/gyro
	uint8_t rawData[12];                                          // x/y/z accel register data stored here

	readBytesI2C(wire, IMUAddress, QMI8658_AX_L, 12, &rawData[0]);    // Read the 12 raw data registers into data array

	uint32_t now = micros();
	if (accelReady) accel.timestamp = now;
	if (gyroReady)  gyro.timestamp  = now;

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

	float aArr[3] = {ax, ay, az};
	float gArr[3] = {gx, gy, gz};
	const int8_t* gm = GEO_MAP[geometryIndex];
	accel.accelX = applyGeo(gm[0], aArr);
	accel.accelY = applyGeo(gm[1], aArr);
	accel.accelZ = applyGeo(gm[2], aArr);
	gyro.gyroX   = applyGeo(gm[0], gArr);
	gyro.gyroY   = applyGeo(gm[1], gArr);
	gyro.gyroZ   = applyGeo(gm[2], gArr);

	uint8_t buf[2];
	readBytesI2C(wire, IMUAddress, QMI8658_TEMP_L, 2, &buf[0]);
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
	uint8_t c;
	if (range == 16) {
		aRes = 16.f / 32768.f;
		c = 0x30;  // aFS = 011 at bits[6:4]
	}
	else if (range == 8) {
		aRes = 8.f / 32768.f;
		c = 0x20;  // aFS = 010 at bits[6:4]
	}
	else if (range == 4) {
		aRes = 4.f / 32768.f;
		c = 0x10;  // aFS = 001 at bits[6:4]
	}
	else if (range == 2) {
		aRes = 2.f / 32768.f;
		c = 0x00;  // aFS = 000 at bits[6:4]
	}
	else {
		return -1;
	}
	writeByteI2C(wire, IMUAddress, QMI8658_CTRL7, 0x00);
	rmwByteI2C(wire, IMUAddress, QMI8658_CTRL2, 0x70, c);
	writeByteI2C(wire, IMUAddress, QMI8658_CTRL7, 0x03);
	return 0;
}

int QMI8658::setGyroRange(int range) {
	uint8_t c;
	if (range == 2048 || range == 2000) {
		gRes = 2048.f / 32768.f;
		c = 0x70;  // gFS = 111 at bits[6:4]
	}
	else if (range == 1024 || range == 1000) {
		gRes = 1024.f / 32768.f;
		c = 0x60;  // gFS = 110 at bits[6:4]
	}
	else if (range == 512 || range == 500) {
		gRes = 512.f / 32768.f;
		c = 0x50;  // gFS = 101 at bits[6:4]
	}
	else if (range == 256 || range == 250){
		gRes = 256.f / 32768.f;
		c = 0x40;  // gFS = 100 at bits[6:4]
	}
	else if (range == 128 || range == 125){
		gRes = 128.f / 32768.f;
		c = 0x30;  // gFS = 011 at bits[6:4]
	}
	else {
		return -1;
	}
	writeByteI2C(wire, IMUAddress, QMI8658_CTRL7, 0x00);
	rmwByteI2C(wire, IMUAddress, QMI8658_CTRL3, 0x70, c);
	writeByteI2C(wire, IMUAddress, QMI8658_CTRL7, 0x03);
	return 0;
}

void QMI8658::calibrateAccelGyro(calData* cal)
{
	uint8_t data[12];
	uint16_t packet_count = 256; // How many sets of full gyro and accelerometer data for averaging;
	float gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	float  gyrosensitivity = 128.f / 32768.f;			
	float  accelsensitivity = 2.f / 32768.f;

	// reset device
	writeByteI2C(wire, IMUAddress, QMI8658_RESET, 0xFF);	    // Toggle softreset
	delay(100);										// wait for reset
	
	writeByteI2C(wire, IMUAddress, QMI8658_CTRL1, 0x40);		// Enable auto increment

	setAccelODR(500);
	setAccelRange(2);
	setGyroODR(500);
	setGyroRange(128);
	setAccelLPF(70);
	setGyroLPF(70);

	writeByteI2C(wire, IMUAddress, QMI8658_CTRL7, 0x03);	    // Start up accelerometer and gyro, disable sync
	delay(100);								    	//wait until they're done starting up...

	for (int i = 0; i < packet_count; i++)
	{
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };

		readBytesI2C(wire, IMUAddress, QMI8658_AX_L, 12, &data[0]);    // Read the 12 raw data registers into data array

		accel_temp[0] = ((int16_t)data[1] << 8) | data[0];		  // Turn the MSB and LSB into a signed 16-bit value
		accel_temp[1] = ((int16_t)data[3] << 8) | data[2];
		accel_temp[2] = ((int16_t)data[5] << 8) | data[4];

		gyro_temp[0] = ((int16_t)data[7] << 8) | data[6];
		gyro_temp[1] = ((int16_t)data[9] << 8) | data[8];
		gyro_temp[2] = ((int16_t)data[11] << 8) | data[10];


		accel_bias[0] += accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated biases
		accel_bias[1] += accel_temp[1];
		accel_bias[2] += accel_temp[2];
		
		gyro_bias[0] += gyro_temp[0];
		gyro_bias[1] += gyro_temp[1];
		gyro_bias[2] += gyro_temp[2];
		delay(20);
	}
	
	accel_bias[0] /= packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= packet_count;
	accel_bias[2] /= packet_count;

	accel_bias[0] *= accelsensitivity;
	accel_bias[1] *= accelsensitivity;
	accel_bias[2] *= accelsensitivity;

	gyro_bias[0] /= packet_count;
	gyro_bias[1] /= packet_count;
	gyro_bias[2] /= packet_count;
	
	gyro_bias[0] *= gyrosensitivity;
	gyro_bias[1] *= gyrosensitivity;
	gyro_bias[2] *= gyrosensitivity;

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

// QMI8658 ODR encoding: lower code = higher rate (0x00=8000Hz … 0x07=62Hz).
static const int QMI8658_ODR_TABLE[] = {62, 125, 250, 500, 1000, 2000, 4000, 8000};
static const uint8_t QMI8658_ODR_REG[] = {0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00};

int QMI8658::setAccelODR(int odr_hz) {
	if (odr_hz <= 0) return -1;
	int actual = nearestHigherODR(QMI8658_ODR_TABLE, 8, odr_hz);
	int idx = 0;
	while (QMI8658_ODR_TABLE[idx] != actual) idx++;
	uint8_t ctrl7 = readByteI2C(wire, IMUAddress, QMI8658_CTRL7);
	writeByteI2C(wire, IMUAddress, QMI8658_CTRL7, 0x00);
	rmwByteI2C(wire, IMUAddress, QMI8658_CTRL2, 0x0F, QMI8658_ODR_REG[idx]);
	writeByteI2C(wire, IMUAddress, QMI8658_CTRL7, ctrl7);
	currentAccelODR = actual;
	return actual;
}

int QMI8658::setGyroODR(int odr_hz) {
	if (odr_hz <= 0) return -1;
	int actual = nearestHigherODR(QMI8658_ODR_TABLE, 8, odr_hz);
	int idx = 0;
	while (QMI8658_ODR_TABLE[idx] != actual) idx++;
	uint8_t ctrl7 = readByteI2C(wire, IMUAddress, QMI8658_CTRL7);
	writeByteI2C(wire, IMUAddress, QMI8658_CTRL7, 0x00);
	rmwByteI2C(wire, IMUAddress, QMI8658_CTRL3, 0x0F, QMI8658_ODR_REG[idx]);
	writeByteI2C(wire, IMUAddress, QMI8658_CTRL7, ctrl7);
	currentGyroODR = actual;
	return actual;
}

int QMI8658::setAccelLPF(int lpf_hz) {
	uint8_t ctrl7 = readByteI2C(wire, IMUAddress, QMI8658_CTRL7);
	writeByteI2C(wire, IMUAddress, QMI8658_CTRL7, 0x00);
	if (lpf_hz == 0) {
		rmwByteI2C(wire, IMUAddress, QMI8658_CTRL5, 0xD0, 0x00);
		writeByteI2C(wire, IMUAddress, QMI8658_CTRL7, ctrl7);
		currentAccelLPF = 0;
		return 0;
	}
	static const float factors[] = {0.025f, 0.14f, 0.33f, 0.50f};
	int best_idx = 0;
	int best_bw = (int)(factors[0] * currentAccelODR);
	int best_dist = abs(best_bw - lpf_hz);
	for (int i = 1; i < 4; i++) {
		int bw = (int)(factors[i] * currentAccelODR);
		int d = abs(bw - lpf_hz);
		if (d < best_dist) { best_dist = d; best_bw = bw; best_idx = i; }
	}
	rmwByteI2C(wire, IMUAddress, QMI8658_CTRL5, 0xD0, ((uint8_t)best_idx << 6) | 0x10);
	writeByteI2C(wire, IMUAddress, QMI8658_CTRL7, ctrl7);
	currentAccelLPF = best_bw;
	return best_bw;
}

int QMI8658::setGyroLPF(int lpf_hz) {
	uint8_t ctrl7 = readByteI2C(wire, IMUAddress, QMI8658_CTRL7);
	writeByteI2C(wire, IMUAddress, QMI8658_CTRL7, 0x00);
	if (lpf_hz == 0) {
		rmwByteI2C(wire, IMUAddress, QMI8658_CTRL5, 0x0D, 0x00);
		writeByteI2C(wire, IMUAddress, QMI8658_CTRL7, ctrl7);
		currentGyroLPF = 0;
		return 0;
	}
	static const float factors[] = {0.025f, 0.14f, 0.33f, 0.50f};
	int best_idx = 0;
	int best_bw = (int)(factors[0] * currentGyroODR);
	int best_dist = abs(best_bw - lpf_hz);
	for (int i = 1; i < 4; i++) {
		int bw = (int)(factors[i] * currentGyroODR);
		int d = abs(bw - lpf_hz);
		if (d < best_dist) { best_dist = d; best_bw = bw; best_idx = i; }
	}
	rmwByteI2C(wire, IMUAddress, QMI8658_CTRL5, 0x0D, ((uint8_t)best_idx << 2) | 0x01);
	writeByteI2C(wire, IMUAddress, QMI8658_CTRL7, ctrl7);
	currentGyroLPF = best_bw;
	return best_bw;
}