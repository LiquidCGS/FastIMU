#include "F_QMC5883L.hpp"

int QMC5883L::init(calData cal, uint8_t address)
{
	IMUAddress = address;
	//check sensor
	if (!(readByte(IMUAddress, QMC5883L_WHOAMI) == QMC5883L_WHOAMI_VALUE && readByte(IMUAddress, 0x0C) == 0x01)) {
		return -1;
	}
	//load cal
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
	// Reset sensor.
	writeByte(IMUAddress, QMC5883L_RESET1, 0x80);
	delay(100);
	writeByte(IMUAddress, QMC5883L_RESET2, 0x01);
	delay(100);
	//start up sensor, 200hz ODR, 128 OSR, 8G, Continuous mode.
	writeByte(IMUAddress, QMC5883L_CTRL, 0x1D);
	return 0;
}

void QMC5883L::update()
{
	if (!(readByte(IMUAddress, QMC5883L_STATUS) & 0x01)) {
		mag.magX = 0.f;
		mag.magY = 0.f;
		mag.magZ = 0.f;
		return;
	}
	uint8_t rawData[6] = { 0 };
	int16_t magCount[3] = { 0, 0, 0 };
	readBytes(IMUAddress, QMC5883L_X_LSB, 6, &rawData[0]);

	if (!(readByte(IMUAddress, QMC5883L_STATUS) & 0x02)) {                                           // Check if magnetic sensor overflow set, if not then report data
	magCount[0] = ((int16_t)rawData[1] << 8) | rawData[0];   // Turn the MSB and LSB into a signed 16-bit value
	magCount[1] = ((int16_t)rawData[3] << 8) | rawData[2];   // Data stored as little Endian
	magCount[2] = ((int16_t)rawData[5] << 8) | rawData[4];
	}

	// Calculate the mag value
	float mx, my, mz;
	mx = ((float)(magCount[0] * mRes - calibration.magBias[0]) * calibration.magScale[0]) * 100.f;
	my = ((float)(magCount[1] * mRes - calibration.magBias[1]) * calibration.magScale[1]) * 100.f;  // get actual magnetometer value, this depends on scale being set
	mz = ((float)(magCount[2] * mRes - calibration.magBias[2]) * calibration.magScale[2]) * 100.f;  //mul by 100 to convert from G to µT
	
	readBytes(IMUAddress, QMC5883L_T_LSB, 2, &rawData[0]);
	temperature = (float)((((int16_t)rawData[1] << 8) | rawData[0]) * tRes) + 20.f;
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

void QMC5883L::getMag(MagData* out)
{
	memcpy(out, &mag, sizeof(mag));
}

void QMC5883L::calibrateMag(calData* cal) 
{
	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3] = { 0, 0, 0 }, mag_scale[3] = { 0, 0, 0 };
	int16_t mag_max[3] = { -32767, -32767, -32767 }, mag_min[3] = { 32767, 32767, 32767 }, mag_temp[3] = { 0, 0, 0 };

	// shoot for ~fifteen seconds of mag data
	sample_count = 3000;  // at 200 Hz ODR, new mag data is available every 5 ms

	for (ii = 0; ii < sample_count; ii++)
	{
		uint8_t rawData[6];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
		if ((readByte(IMUAddress, QMC5883L_STATUS) & 0x01) && !(readByte(IMUAddress, QMC5883L_STATUS) & 0x02)) { // wait for magnetometer data ready bit to be set and overflow not to be.
			readBytes(IMUAddress, QMC5883L_X_LSB, 6, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
			mag_temp[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
			mag_temp[1] = ((int16_t)rawData[3] << 8) | rawData[2];  // Data stored as little Endian
			mag_temp[2] = ((int16_t)rawData[5] << 8) | rawData[4];
			for (int jj = 0; jj < 3; jj++)
			{
				if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
				if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
			}
		}
		delay(6); // at 200 Hz ODR, new mag data is available every 5 ms
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