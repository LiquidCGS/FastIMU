#include "F_MPU6500.hpp"

//Original code: https://github.com/hideakitai/MPU9250/blob/master/MPU9250.h

int MPU6500::init(calData cal, uint8_t address) 
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

	if (!(readByteI2C(wire, IMUAddress, MPU6500_WHO_AM_I_MPU6500) == MPU6500_WHOAMI_DEFAULT_VALUE)) {
		return -1;
	}

	// reset device
	writeByteI2C(wire, IMUAddress, MPU6500_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	delay(100);
	// wake up device
	writeByteI2C(wire, IMUAddress, MPU6500_PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
	delay(100); // Wait for all registers to reset

	// get stable time source
	writeByteI2C(wire, IMUAddress, MPU6500_PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
	delay(200);

	setGyroODR(333);
	setGyroRange(2000);
	setAccelRange(16);
	setGyroLPF(42);
	setAccelLPF(41);

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
	// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	writeByteI2C(wire, IMUAddress, MPU6500_INT_PIN_CFG, 0x22);	//enable Magnetometer bypass
	writeByteI2C(wire, IMUAddress, MPU6500_INT_ENABLE, 0x01);    // Enable data ready (bit 0) interrupt
	delay(100);
	return 0;
}

void MPU6500::update() {
	if (!dataAvailable()) return;

	int16_t IMUCount[7];                                          // used to read all 14 bytes at once from the MPU6500 accel/gyro
	uint8_t rawData[14];                                          // x/y/z accel register data stored here

	readBytesI2C(wire, IMUAddress, MPU6500_ACCEL_XOUT_H, 14, &rawData[0]);    // Read the 14 raw data registers into data array

	accel.timestamp = micros();
	gyro.timestamp = accel.timestamp;

	IMUCount[0] = ((int16_t)rawData[0] << 8) | rawData[1];		  // Turn the MSB and LSB into a signed 16-bit value
	IMUCount[1] = ((int16_t)rawData[2] << 8) | rawData[3];
	IMUCount[2] = ((int16_t)rawData[4] << 8) | rawData[5];
	IMUCount[3] = ((int16_t)rawData[6] << 8) | rawData[7];
	IMUCount[4] = ((int16_t)rawData[8] << 8) | rawData[9];
	IMUCount[5] = ((int16_t)rawData[10] << 8) | rawData[11];
	IMUCount[6] = ((int16_t)rawData[12] << 8) | rawData[13];

	float ax, ay, az, gx, gy, gz;

	// Calculate the accel value into actual g's per second
	ax = (float)IMUCount[0] * aRes - calibration.accelBias[0];
	ay = (float)IMUCount[1] * aRes - calibration.accelBias[1];
	az = (float)IMUCount[2] * aRes - calibration.accelBias[2];

	// Calculate the temperature value into actual deg c
	temperature = (((float)IMUCount[3] - 21.0f) / 333.87f) + 21.0f;

	// Calculate the gyro value into actual degrees per second
	gx = (float)IMUCount[4] * gRes - calibration.gyroBias[0];
	gy = (float)IMUCount[5] * gRes - calibration.gyroBias[1];
	gz = (float)IMUCount[6] * gRes - calibration.gyroBias[2];

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

void MPU6500::getAccel(AccelData* out) 
{
	memcpy(out, &accel, sizeof(accel));
}
void MPU6500::getGyro(GyroData* out) 
{
	memcpy(out, &gyro, sizeof(gyro));
}

int MPU6500::setAccelRange(int range) {
	uint8_t c;
	if (range == 16) {
		aRes = 16.f / 32768.f;			//ares value for full range (16g) readings
		c = 0x03 << 3;
	}
	else if (range == 8) {
		aRes = 8.f / 32768.f;			//ares value for range (8g) readings
		c = 0x02 << 3;
	}
	else if (range == 4) {
		aRes = 4.f / 32768.f;			//ares value for range (4g) readings
		c = 0x01 << 3;
	}
	else if (range == 2) {
		aRes = 2.f / 32768.f;			//ares value for range (2g) readings
		c = 0x00 << 3;
	}
	else {
		return -1;
	}
	rmwByteI2C(wire, IMUAddress, MPU6500_ACCEL_CONFIG, 0x18, c);
	return 0;
}

int MPU6500::setGyroRange(int range) {
	uint8_t c;
	if (range == 2000) {
		gRes = 2000.f / 32768.f;			//ares value for full range (2000dps) readings
		c = 0x03 << 3;
	}
	else if (range == 1000) {
		gRes = 1000.f / 32768.f;			//ares value for range (1000dps) readings
		c = 0x02 << 3;
	}
	else if (range == 500) {
		gRes = 500.f / 32768.f;			//ares value for range (500dps) readings
		c = 0x01 << 3;
	}
	else if (range == 250) {
		gRes = 250.f / 32768.f;			//ares value for range (250dps) readings
		c = 0x00 << 3;
	}
	else {
		return -1;
	}
	rmwByteI2C(wire, IMUAddress, MPU6500_GYRO_CONFIG, 0x18, c);
	return 0;
}

void MPU6500::calibrateAccelGyro(calData* cal) 
{
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	// reset device
	writeByteI2C(wire, IMUAddress, MPU6500_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	delay(100);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	// else use the internal oscillator, bits 2:0 = 001
	writeByteI2C(wire, IMUAddress, MPU6500_PWR_MGMT_1, 0x01);
	writeByteI2C(wire, IMUAddress, MPU6500_PWR_MGMT_2, 0x00);
	delay(200);

	// Configure device for bias calculation
	writeByteI2C(wire, IMUAddress, MPU6500_INT_ENABLE, 0x00);   // Disable all interrupts
	writeByteI2C(wire, IMUAddress, MPU6500_FIFO_EN, 0x00);      // Disable FIFO
	writeByteI2C(wire, IMUAddress, MPU6500_PWR_MGMT_1, 0x00);   // Turn on internal clock source
	writeByteI2C(wire, IMUAddress, MPU6500_I2C_MST_CTRL, 0x00); // Disable I2C master
	writeByteI2C(wire, IMUAddress, MPU6500_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	writeByteI2C(wire, IMUAddress, MPU6500_USER_CTRL, 0x0C);    // Reset FIFO and DMP
	delay(15);

	// Configure MPU6500 gyro and accelerometer for bias calculation
	setGyroODR(1000);
	setGyroRange(250);
	setAccelRange(2);
	setGyroLPF(188);

	uint16_t  gyrosensitivity = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeByteI2C(wire, IMUAddress, MPU6500_USER_CTRL, 0x40);   // Enable FIFO
	writeByteI2C(wire, IMUAddress, MPU6500_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	writeByteI2C(wire, IMUAddress, MPU6500_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	readBytesI2C(wire, IMUAddress, MPU6500_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count / 12;// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++)
	{
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
		readBytesI2C(wire, IMUAddress, MPU6500_FIFO_R_W, 12, &data[0]); // read data for averaging
		accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
		accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
		gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
		gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
		gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

		accel_bias[0] += (int32_t)accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t)accel_temp[1];
		accel_bias[2] += (int32_t)accel_temp[2];
		gyro_bias[0] += (int32_t)gyro_temp[0];
		gyro_bias[1] += (int32_t)gyro_temp[1];
		gyro_bias[2] += (int32_t)gyro_temp[2];
	}
	accel_bias[0] /= (int32_t)packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t)packet_count;
	accel_bias[2] /= (int32_t)packet_count;
	gyro_bias[0] /= (int32_t)packet_count;
	gyro_bias[1] /= (int32_t)packet_count;
	gyro_bias[2] /= (int32_t)packet_count;

	switch (geometryIndex) {
	case 0:
	case 1:
	case 2:
	case 3:
		if (accel_bias[2] > 0L) {
			accel_bias[2] -= (int32_t)accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
		}
		else {
			accel_bias[2] += (int32_t)accelsensitivity;
		}
		break;
	case 4:
	case 6:
		if (accel_bias[0] > 0L) {
			accel_bias[0] -= (int32_t)accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
		}
		else {
			accel_bias[0] += (int32_t)accelsensitivity;
		}
		break;
	case 5:
	case 7:
		if (accel_bias[1] > 0L) {
			accel_bias[1] -= (int32_t)accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
		}
		else {
			accel_bias[1] += (int32_t)accelsensitivity;
		}
		break;
	}
	// Output scaled accelerometer biases for display in the main program
	cal->accelBias[0] = (float)accel_bias[0] / (float)accelsensitivity;
	cal->accelBias[1] = (float)accel_bias[1] / (float)accelsensitivity;
	cal->accelBias[2] = (float)accel_bias[2] / (float)accelsensitivity;
	// Output scaled gyro biases for display in the main program
	cal->gyroBias[0] = (float)gyro_bias[0] / (float)gyrosensitivity;
	cal->gyroBias[1] = (float)gyro_bias[1] / (float)gyrosensitivity;
	cal->gyroBias[2] = (float)gyro_bias[2] / (float)gyrosensitivity;
	cal->valid = true;
}

int MPU6500::setGyroODR(int odr_hz) {
	if (odr_hz <= 0) return -1;
	uint8_t div = (uint8_t)constrain(1000 / odr_hz - 1, 0, 255);
	writeByteI2C(wire, IMUAddress, MPU6500_SMPLRT_DIV, div);
	currentODR = 1000 / ((int)div + 1);
	return currentODR;
}

int MPU6500::setAccelODR(int odr_hz) {
	return setGyroODR(odr_hz);
}

static const int MPU6500_GYRO_LPF_TABLE[] = {5, 10, 20, 42, 98, 188, 250};
static const uint8_t MPU6500_GYRO_LPF_CFG[] = {6, 5, 4, 3, 2, 1, 0};

static const int MPU6500_ACCEL_LPF_TABLE[] = {5, 10, 20, 41, 92, 184, 460};
static const uint8_t MPU6500_ACCEL_LPF_CFG[] = {6, 5, 4, 3, 2, 1, 0};

int MPU6500::setGyroLPF(int lpf_hz) {
	if (lpf_hz == 0) {
		rmwByteI2C(wire, IMUAddress, MPU6500_GYRO_CONFIG, 0x03, 0x01);
		currentGyroLPF = 0;
		return 0;
	}
	int actual = nearestVal(MPU6500_GYRO_LPF_TABLE, 7, lpf_hz);
	int idx = 0;
	while (MPU6500_GYRO_LPF_TABLE[idx] != actual) idx++;
	rmwByteI2C(wire, IMUAddress, MPU6500_GYRO_CONFIG, 0x03, 0x00);
	rmwByteI2C(wire, IMUAddress, MPU6500_MPU_CONFIG, 0x07, MPU6500_GYRO_LPF_CFG[idx]);
	currentGyroLPF = actual;
	return actual;
}

int MPU6500::setAccelLPF(int lpf_hz) {
	if (lpf_hz == 0) {
		rmwByteI2C(wire, IMUAddress, MPU6500_ACCEL_CONFIG2, 0x0F, 0x08);
		currentAccelLPF = 0;
		return 0;
	}
	int actual = nearestVal(MPU6500_ACCEL_LPF_TABLE, 7, lpf_hz);
	int idx = 0;
	while (MPU6500_ACCEL_LPF_TABLE[idx] != actual) idx++;
	rmwByteI2C(wire, IMUAddress, MPU6500_ACCEL_CONFIG2, 0x0F, MPU6500_ACCEL_LPF_CFG[idx]);
	currentAccelLPF = actual;
	return actual;
}