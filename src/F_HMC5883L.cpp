#include "F_HMC5883L.hpp"

int HMC5883L::init(calData cal, uint8_t address)
{
	IMUAddress = address;
	//check sensor
	if (!(readByteI2C(wire, IMUAddress, HMC5883L_IDC) == HMC5883L_WHOAMI_VALUE)) {
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
	// setup sensor. 75hz odr, normal mode, 8 samples
	writeByteI2C(wire, IMUAddress, HMC5883L_CFGA, 0x78);
	delay(1);
	// +- 8100uT
	writeByteI2C(wire, IMUAddress, HMC5883L_CFGB, 0xE0);
	delay(1);
	// cont mode.
	writeByteI2C(wire, IMUAddress, HMC5883L_MODE, 0x00);
	delay(1);
	return 0;
}

void HMC5883L::update()
{
	if (!(readByteI2C(wire, IMUAddress, HMC5883L_STATUS) & 0x01) && (readByteI2C(wire, IMUAddress, HMC5883L_STATUS) & 0x02)) {
		mag.magX = 0.f;
		mag.magY = 0.f;
		mag.magZ = 0.f;
		return;
	}
	uint8_t rawData[6] = { 0 };
	int16_t magCount[3] = { 0, 0, 0 };
	readBytesI2C(wire, IMUAddress, HMC5883L_X_MSB, 6, &rawData[0]);

	magCount[0] = ((int16_t)rawData[0] << 8) | rawData[1];   // Turn the MSB and LSB into a signed 16-bit value
	magCount[1] = ((int16_t)rawData[2] << 8) | rawData[3];  
	magCount[2] = ((int16_t)rawData[4] << 8) | rawData[5];


	// Calculate the mag value
	float mx, my, mz;
	mx = ((float)(magCount[0] * mRes - calibration.magBias[0]) * calibration.magScale[0]);
	mz = ((float)(magCount[1] * mRes - calibration.magBias[2]) * calibration.magScale[2]);  // get actual magnetometer value, this depends on scale being set
	my = ((float)(magCount[2] * mRes - calibration.magBias[1]) * calibration.magScale[1]);  //mul by 100 to convert from G to µT
	
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

void HMC5883L::getMag(MagData* out)
{
	memcpy(out, &mag, sizeof(mag));
}

void HMC5883L::calibrateMag(calData* cal) 
{
	// setup sensor. 75hz odr, normal mode, 8 samples
	writeByteI2C(wire, IMUAddress, HMC5883L_CFGA, 0x78);
	delay(1);
	// +- 8100uT
	writeByteI2C(wire, IMUAddress, HMC5883L_CFGB, 0xE0);
	delay(1);
	// cont mode.
	writeByteI2C(wire, IMUAddress, HMC5883L_MODE, 0x00);
	delay(1);
	
	uint16_t ii = 0, sample_count = 0;
	int32_t mag_bias[3] = { 0, 0, 0 }, mag_scale[3] = { 0, 0, 0 };
	int16_t mag_max[3] = { -32767, -32767, -32767 }, mag_min[3] = { 32767, 32767, 32767 }, mag_temp[3] = { 0, 0, 0 };

	// shoot for ~fifteen seconds of mag data
	sample_count = 1000;  // at 75 Hz ODR, new mag data is available every 20 ms

	for (ii = 0; ii < sample_count; ii++)
	{
		uint8_t rawData[6];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
		if ((readByteI2C(wire, IMUAddress, HMC5883L_STATUS) & 0x01) && !(readByteI2C(wire, IMUAddress, HMC5883L_STATUS) & 0x02)) { // wait for magnetometer data ready bit to be set and lcok not to be
			readBytesI2C(wire, IMUAddress, HMC5883L_X_MSB, 6, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
			mag_temp[0] = ((int16_t)rawData[0] << 8) | rawData[1];  // Turn the MSB and LSB into a signed 16-bit value
			mag_temp[2] = ((int16_t)rawData[2] << 8) | rawData[3];  // Data stored as little Endian
			mag_temp[1] = ((int16_t)rawData[4] << 8) | rawData[5];
			for (int jj = 0; jj < 3; jj++)
			{
				if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
				if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
			}
		}
		delay(14); // at 75 Hz ODR, new mag data is available every 13.3 ms
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

	Serial.print("MagBias Max: ");
	Serial.print(mag_max[0]);
	Serial.print(" MagBias min: ");
	Serial.print(mag_min[0]);
	Serial.print(" MagBias: ");
	Serial.print(mag_bias[0]);
	Serial.print(" MagBias scaled: ");
	Serial.println((float)mag_bias[0] * mRes);
	

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	cal->magScale[0] = avg_rad / ((float)mag_scale[0]);
	cal->magScale[1] = avg_rad / ((float)mag_scale[1]);
	cal->magScale[2] = avg_rad / ((float)mag_scale[2]);
}