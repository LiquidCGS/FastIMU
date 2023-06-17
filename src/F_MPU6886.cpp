#include "F_MPU6886.hpp"

//Original code: https://github.com/hideakitai/MPU9250/blob/master/MPU9250.h

int MPU6886::init(calData cal, uint8_t address) 
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

	if (!(readByte(IMUAddress, MPU6886_WHO_AM_I_MPU6886) == MPU6886_WHOAMI_DEFAULT_VALUE)) {
		return -1;
	}

	// reset device
	writeByte(IMUAddress, MPU6886_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	delay(100);
	// wake up device
	writeByte(IMUAddress, MPU6886_PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
	delay(100); // Wait for all registers to reset

	// get stable time source
	writeByte(IMUAddress, MPU6886_PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
	delay(200);

	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
	// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
	// be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both 
	// With the MPU6886, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
	writeByte(IMUAddress, MPU6886_MPU_CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	writeByte(IMUAddress, MPU6886_SMPLRT_DIV, 0x02);  // Use a 500 Hz rate; a rate consistent with the filter update rate
	// determined inset in CONFIG above

	// Set gyroscope full scale range
	// Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t c = readByte(IMUAddress, MPU6886_GYRO_CONFIG); // get current GYRO_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x03; // Clear Fchoice bits [1:0]
	c = c & ~0x18; // Clear GFS bits [4:3]
	c = c | (uint8_t)3 << 3; // Set full scale range for the gyro (11 on 4:3)
	// c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
	writeByte(IMUAddress, MPU6886_GYRO_CONFIG, c); // Write new GYRO_CONFIG value to register

	// Set accelerometer full-scale range configuration
	c = readByte(IMUAddress, MPU6886_ACCEL_CONFIG); // get current ACCEL_CONFIG register value
	// c = c & ~0xE0; // Clear self-test bits [7:5]
	c = c & ~0x18;  // Clear AFS bits [4:3]
	c = c | (uint8_t)3 << 3; // Set full scale range for the accelerometer (11 on 4:3)
	writeByte(IMUAddress, MPU6886_ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	c = readByte(IMUAddress, MPU6886_ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
	c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	writeByte(IMUAddress, MPU6886_ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
	// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	writeByte(IMUAddress, MPU6886_INT_PIN_CFG, 0x22);	//enable Magnetometer bypass
	writeByte(IMUAddress, MPU6886_INT_ENABLE, 0x01);    // Enable data ready (bit 0) interrupt
	delay(100);
	return 0;
}

void MPU6886::update() {
	if (!dataAvailable()) return;

	int16_t IMUCount[7];                                          // used to read all 14 bytes at once from the MPU6886 accel/gyro
	uint8_t rawData[14];                                          // x/y/z accel register data stored here

	readBytes(IMUAddress, MPU6886_ACCEL_XOUT_H, 14, &rawData[0]);    // Read the 14 raw data registers into data array

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

void MPU6886::getAccel(AccelData* out) 
{
	memcpy(out, &accel, sizeof(accel));
}
void MPU6886::getGyro(GyroData* out) 
{
	memcpy(out, &gyro, sizeof(gyro));
}

int MPU6886::setAccelRange(int range) {
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
	writeByte(IMUAddress, MPU6886_ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value
	return 0;
}

int MPU6886::setGyroRange(int range) {
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
	writeByte(IMUAddress, MPU6886_GYRO_CONFIG, c); // Write new GYRO_CONFIG register value
	return 0;
}

void MPU6886::calibrateAccelGyro(calData* cal) 
{
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	// reset device
	writeByte(IMUAddress, MPU6886_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	delay(100);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	// else use the internal oscillator, bits 2:0 = 001
	writeByte(IMUAddress, MPU6886_PWR_MGMT_1, 0x01);
	writeByte(IMUAddress, MPU6886_PWR_MGMT_2, 0x00);
	delay(200);

	// Configure device for bias calculation
	writeByte(IMUAddress, MPU6886_INT_ENABLE, 0x00);   // Disable all interrupts
	writeByte(IMUAddress, MPU6886_FIFO_EN, 0x00);      // Disable FIFO
	writeByte(IMUAddress, MPU6886_PWR_MGMT_1, 0x00);   // Turn on internal clock source
	writeByte(IMUAddress, MPU6886_I2C_MST_CTRL, 0x00); // Disable I2C master
	writeByte(IMUAddress, MPU6886_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	writeByte(IMUAddress, MPU6886_USER_CTRL, 0x0C);    // Reset FIFO and DMP
	delay(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	writeByte(IMUAddress, MPU6886_MPU_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	writeByte(IMUAddress, MPU6886_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	writeByte(IMUAddress, MPU6886_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeByte(IMUAddress, MPU6886_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t  gyrosensitivity = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeByte(IMUAddress, MPU6886_USER_CTRL, 0x40);   // Enable FIFO
	writeByte(IMUAddress, MPU6886_FIFO_EN, 0x18);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	writeByte(IMUAddress, MPU6886_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	readBytes(IMUAddress, MPU6886_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count / 12;// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++)
	{
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
		readBytes(IMUAddress, MPU6886_FIFO_R_W, 12, &data[0]); // read data for averaging
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