#include "F_AK09918.hpp"


int AK09918::init(calData cal, uint8_t address)
{
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

	uint8_t wai[2];
	// Check wai for correct device
	readBytesI2C(wire, AK09918_ADDRESS, AK09918_WIA1, 2, wai);
	if (!(wai[0] == 0x48 && wai[1] == 0x0C)) {
        return -1; // Failure
    }

	writeByteI2C(wire, AK09918_ADDRESS, AK09918_CNTL2, AK09918_NORMAL); // Enter Fuse ROM access mode
	delay(10);

	return 0;
}

void AK09918::update() {	
	if (dataAvailable())	
	{
		int16_t magCount[3] = { 0, 0, 0 };                           // Stores the 16-bit signed magnetometer sensor output
		uint8_t rawData[8];                                          // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
		readBytesI2C(wire, AK09918_ADDRESS, AK09918_HXL, 8, &rawData[0]);    // Read the six raw data and ST2 registers sequentially into data array
		uint8_t c = rawData[7];                                      // End data read by reading ST2 register
		if (!(c & AK09918_HOFL_BIT)) {                                           // Check if magnetic sensor overflow set, if not then report data
			magCount[0] = ((int16_t)rawData[1] << 8) | rawData[0];   // Turn the MSB and LSB into a signed 16-bit value
			magCount[1] = ((int16_t)rawData[3] << 8) | rawData[2];   // Data stored as little Endian
			magCount[2] = ((int16_t)rawData[5] << 8) | rawData[4];
		}
		
		float mx, my, mz;

		mx = (float)(magCount[1] * mRes - calibration.magBias[1]) * calibration.magScale[1];
		my = (float)(magCount[0] * mRes - calibration.magBias[0]) * calibration.magScale[0];  // get actual magnetometer value, this depends on scale being set
		mz = -(float)(magCount[2] * mRes - calibration.magBias[2]) * calibration.magScale[2];

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
	else
	{
		if(!readByteI2C(wire, AK09918_ADDRESS, AK09918_CNTL2) & 0x01){
			writeByteI2C(wire, AK09918_ADDRESS, AK09918_CNTL2, 0x01); // Set magnetometer to single measure mode.
		}
	}
	//    // Apply mag soft iron error compensation
	//    mx = x * calibration.mag_softiron_matrix[0][0] + y * calibration.mag_softiron_matrix[0][1] + z * calibration.mag_softiron_matrix[0][2];
	//    my = x * calibration.mag_softiron_matrix[1][0] + y * calibration.mag_softiron_matrix[1][1] + z * calibration.mag_softiron_matrix[1][2];
	//    mz = x * calibration.mag_softiron_matrix[2][0] + y * calibration.mag_softiron_matrix[2][1] + z * calibration.mag_softiron_matrix[2][2];
}

void AK09918::getAccel(AccelData* out) 
{
	return;
}
void AK09918::getGyro(GyroData* out) 
{
	return;
}
void AK09918::getMag(MagData* out) 
{
	memcpy(out, &mag, sizeof(mag));
}

int AK09918::setAccelRange(int range) {
	return -1;
}

int AK09918::setGyroRange(int range) {
	return -1;
}

void AK09918::calibrateAccelGyro(calData* cal) 
{
	return;
}

void AK09918::calibrateMag(calData* cal) 
{
	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3] = { 0, 0, 0 }, mag_scale[3] = { 0, 0, 0 };
	int16_t mag_max[3] = { -32767, -32767, -32767 }, mag_min[3] = { 32767, 32767, 32767 }, mag_temp[3] = { 0, 0, 0 };

	// shoot for ~fifteen seconds of mag data
	sample_count = 1024;  // at 100 Hz ODR, new mag data is available every 10 ms

	for (ii = 0; ii < sample_count; ii++)
	{	
		writeByteI2C(wire, AK09918_ADDRESS, AK09918_CNTL2, 0x01); // Set magnetometer to single measure mode.
		delay(15); // at 100 Hz ODR, new mag data is available every 10 ms
		uint8_t rawData[8];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
		if (readByteI2C(wire, AK09918_ADDRESS, AK09918_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
			readBytesI2C(wire, AK09918_ADDRESS, AK09918_HXL, 8, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
			uint8_t c = rawData[7];	 // End data read by reading ST2 register
			if (!(c & AK09918_HOFL_BIT)) { // Check if magnetic sensor overflow set, if not then report data
				mag_temp[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
				mag_temp[1] = ((int16_t)rawData[3] << 8) | rawData[2];  // Data stored as little Endian
				mag_temp[2] = ((int16_t)rawData[5] << 8) | rawData[4];
			}
			for (int jj = 0; jj < 3; jj++)
			{
				if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
				if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
			}
		}
	}

	// Get hard iron correction
	mag_bias[0] = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
	mag_bias[1] = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
	mag_bias[2] = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

	cal->magBias[0] = (float)mag_bias[0] * mRes; // save mag biases in G for main program
	cal->magBias[1] = (float)mag_bias[1] * mRes;
	cal->magBias[2] = (float)mag_bias[2] * mRes;

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
