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

	writeByteI2C(wire, IMUAddress, QMI8658_CTRL2, 0x04);  	// Set up full scale Accel range. +-2G, 500hz ODR
	writeByteI2C(wire, IMUAddress, QMI8658_CTRL3, 0x34); 	 // Set up Gyro range. +-128dps, 500hz ODR

	writeByteI2C(wire, IMUAddress, QMI8658_CTRL5, 0x55);  	// Enable LPF for both accel and gyro, set to 14% of odr for around 70hz
	
	writeByteI2C(wire, IMUAddress, QMI8658_CTRL7, 0x03);	    // Start up accelerometer and gyro, disable sync
	delay(100);								    	//wait until they're done starting up...

	aRes = 2.f / 32768.f;			//ares value for full range (16g) readings
	gRes = 128.f / 32768.f;	    //gres value for full range (2048dps) readings

	return 0;
}

void QMI8658::update() {
	
	if(!dataAvailable()) {return;}
	
	int16_t IMUCount[6];                                          // used to read all 16 bytes at once from the accel/gyro
	uint8_t rawData[12];                                          // x/y/z accel register data stored here

	readBytesI2C(wire, IMUAddress, QMI8658_AX_L, 12, &rawData[0]);    // Read the 12 raw data registers into data array

	accel.timestamp = micros();
	gyro.timestamp = accel.timestamp;

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
	writeByteI2C(wire, IMUAddress, QMI8658_CTRL7, 0x00);
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
	rmwByteI2C(wire, IMUAddress, QMI8658_CTRL2, 0x70, c);
	writeByteI2C(wire, IMUAddress, QMI8658_CTRL7, 0x03);
	return 0;
}

int QMI8658::setGyroRange(int range) {
	writeByteI2C(wire, IMUAddress, QMI8658_CTRL7, 0x00);
	uint8_t c;
	if (range == 2048) {
		gRes = 2048.f / 32768.f;
		c = 0x70;  // gFS = 111 at bits[6:4]
	}
	else if (range == 1024) {
		gRes = 1024.f / 32768.f;
		c = 0x60;  // gFS = 110 at bits[6:4]
	}
	else if (range == 512) {
		gRes = 512.f / 32768.f;
		c = 0x50;  // gFS = 101 at bits[6:4]
	}
	else if (range == 256){
		gRes = 256.f / 32768.f;
		c = 0x40;  // gFS = 100 at bits[6:4]
	}
	else if (range == 128){
		gRes = 128.f / 32768.f;
		c = 0x30;  // gFS = 011 at bits[6:4]
	}
	else {
		return -1;
	}
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

	writeByteI2C(wire, IMUAddress, QMI8658_CTRL2, 0x04);  	// Set up full scale Accel range. +-2G, 500hz ODR
	writeByteI2C(wire, IMUAddress, QMI8658_CTRL3, 0x34); 	 // Set up Gyro range. +-128dps, 500hz ODR

	writeByteI2C(wire, IMUAddress, QMI8658_CTRL5, 0x55);  	// Enable LPF for both accel and gyro, set to 14% of odr for around 70hz
	
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

// Ascending ODR table (same encoding for both accel and gyro).
// Accel ODR field: CTRL2 bits[6:4]. Gyro ODR field: CTRL3 bits[7:5].
static const int QMI8658_ODR_TABLE[] = {62, 125, 250, 500, 1000, 2000, 4000, 8000};
static const uint8_t QMI8658_ACCEL_ODR_REG[] = {0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70};
static const uint8_t QMI8658_GYRO_ODR_REG[]  = {0x00, 0x20, 0x40, 0x60, 0x80, 0xA0, 0xC0, 0xE0};

int QMI8658::setAccelODR(int odr_hz) {
	if (odr_hz <= 0) return -1;
	int actual = nearestHigherODR(QMI8658_ODR_TABLE, 8, odr_hz);
	int idx = 0;
	while (QMI8658_ODR_TABLE[idx] != actual) idx++;
	rmwByteI2C(wire, IMUAddress, QMI8658_CTRL2, 0x70, QMI8658_ACCEL_ODR_REG[idx]);
	currentAccelODR = actual;
	return actual;
}

int QMI8658::setGyroODR(int odr_hz) {
	if (odr_hz <= 0) return -1;
	int actual = nearestHigherODR(QMI8658_ODR_TABLE, 8, odr_hz);
	int idx = 0;
	while (QMI8658_ODR_TABLE[idx] != actual) idx++;
	rmwByteI2C(wire, IMUAddress, QMI8658_CTRL3, 0xE0, QMI8658_GYRO_ODR_REG[idx]);
	currentGyroODR = actual;
	return actual;
}