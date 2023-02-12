#include "FastIMU.h"
#include "Madgwick.h"
#include "HID.h"

//This example is for use with the Relativty steamvr driver. it outputs a rotation quaternion over HID that the driver can interpret as HMD rotation.

#define IMU_ADDRESS 0x69    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
BMI160 IMU;                 //Change to the name of any supported IMU!

// Currently supported IMUS: MPU9255 MPU9250 MPU6500 MPU6050 ICM20689 ICM20690 BMI055 BMX055 BMI160 LSM6DS3 LSM6DSL

calData calib = { 0 };  //Calibration data
AccelData IMUAccel;    //Sensor data
GyroData IMUGyro;
MagData IMUMag;
Madgwick filter;

static const uint8_t _hidReportDescriptor[] PROGMEM = {

  0x06, 0x03, 0x00,         // USAGE_PAGE (vendor defined)
  0x09, 0x00,         // USAGE (Undefined)
  0xa1, 0x01,         // COLLECTION (Application)
  0x15, 0x00,         //   LOGICAL_MINIMUM (0)
  0x26, 0xff, 0x00,   //   LOGICAL_MAXIMUM (255)
  0x85, 0x01,         //   REPORT_ID (1)
  0x75, 0x08,         //   REPORT_SIZE (16)

  0x95, 0x3f,         //   REPORT_COUNT (1)

  0x09, 0x00,         //   USAGE (Undefined)
  0x81, 0x02,         //   INPUT (Data,Var,Abs) - to the host
  0xc0

};
float quat[4];

void setup() {
  static HIDSubDescriptor node (_hidReportDescriptor, sizeof(_hidReportDescriptor));
  HID().AppendDescriptor(&node);
  IMU.init(calib, IMU_ADDRESS);

#ifdef PERFORM_CALIBRATION
  delay(5000);
  IMU.calibrateAccelGyro(&calib);
  IMU.init(calib, IMU_ADDRESS);
#endif
  filter.begin(0.025f);
}

void loop() {
  IMU.update();
  IMU.getAccel(&IMUAccel);
  IMU.getGyro(&IMUGyro);
  filter.updateIMU(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ);
  quat[0] = filter.getQuatW();
  quat[1] = filter.getQuatY();
  quat[2] = filter.getQuatZ();
  quat[3] = filter.getQuatX();
  
  HID().SendReport(1,quat,63);
}
