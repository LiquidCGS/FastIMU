#include "FastIMU.h"
#include "HID.h"
#include "EEPROM.h"
#include <Wire.h>

//This example is for use with the Relativty steamvr driver. it outputs a rotation quaternion over HID that the driver can interpret as HMD rotation.

#define IMU_ADDRESS 0x68    //Change to the address of the IMU
MPU6050 IMU;                //Change to the name of any supported IMU
#define IMU_GEOMETRY 0		//Change to your current IMU geometry (check docs for a reference pic).

// Currently supported IMUS: MPU9255 MPU9250 MPU6886 MPU6500 MPU6050 ICM20689 ICM20690 BMI055 BMX055 BMI160 LSM6DS3 LSM6DSL QMI8658

calData calib = { 0 };  //Calibration data
AccelData IMUAccel;    //Sensor data
GyroData IMUGyro;
MagData IMUMag;

struct HMDRAWPacket
{
  uint8_t  PacketID;

  int16_t AccX;
  int16_t AccY;
  int16_t AccZ;

  int16_t GyroX;
  int16_t GyroY;
  int16_t GyroZ;

  int16_t MagX;
  int16_t MagY;
  int16_t MagZ;

  uint16_t HMDData;

  uint8_t Padding[30];
};
static HMDRAWPacket HMDRawData;

bool flag;

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

void setup() {
  Wire.begin();
  Wire.setClock(400000); //400khz clock

  int calStatus = 0;
  Serial.begin(9600);
  static HIDSubDescriptor node (_hidReportDescriptor, sizeof(_hidReportDescriptor));
  HID().AppendDescriptor(&node);
  EEPROM.get(100, calStatus);
  if (calStatus == 99) {
    EEPROM.get(200, calib);
  }
  IMU.setIMUGeometry(IMU_GEOMETRY);
  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    while(!Serial){
      ;
    }
    Serial.print("Error initializing IMU! e:");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  HMDRawData.PacketID = 3;
}

void loop() {
  if (Serial) {
    if (!flag) {
      Serial.println("Serial monitor open, do you want to enter calibration mode? (y/n)");
    }
    flag = true;
    if (Serial.read() == 'y') {
      calib = { 0 };                    //this looks important
      IMU.init(calib, IMU_ADDRESS);
      Serial.println("Calibrating IMU... Keep headset still on a flat and level surface...");
      delay(10000);
      IMU.calibrateAccelGyro(&calib);
      IMU.init(calib, IMU_ADDRESS);
      Serial.println("Accelerometer and Gyroscope calibrated!");
      if (IMU.hasMagnetometer()) {
        delay(1000);
        Serial.println("Magnetometer calibration: move IMU in figure 8 pattern until done.");
        delay(5000);
        IMU.calibrateMag(&calib);
        Serial.println("Magnetic calibration done!");
      }
      Serial.println("IMU Calibration complete!");
      Serial.println("Accel biases X/Y/Z: ");
      Serial.print(calib.accelBias[0]);
      Serial.print(", ");
      Serial.print(calib.accelBias[1]);
      Serial.print(", ");
      Serial.println(calib.accelBias[2]);
      Serial.println("Gyro biases X/Y/Z: ");
      Serial.print(calib.gyroBias[0]);
      Serial.print(", ");
      Serial.print(calib.gyroBias[1]);
      Serial.print(", ");
      Serial.println(calib.gyroBias[2]);
      if (IMU.hasMagnetometer()) {
        Serial.println("Mag biases X/Y/Z: ");
        Serial.print(calib.magBias[0]);
        Serial.print(", ");
        Serial.print(calib.magBias[1]);
        Serial.print(", ");
        Serial.println(calib.magBias[2]);
        Serial.println("Mag Scale X/Y/Z: ");
        Serial.print(calib.magScale[0]);
        Serial.print(", ");
        Serial.print(calib.magScale[1]);
        Serial.print(", ");
        Serial.println(calib.magScale[2]);
      }
      Serial.println("Saving Calibration values to EEPROM!");
      EEPROM.put(200, calib);
      EEPROM.put(100, 99);
      delay(1000);
      Serial.println("You can now close the Serial monitor.");
      delay(5000);
    }
  }
  IMU.update();
  IMU.getAccel(&IMUAccel);
  IMU.getGyro(&IMUGyro);

  if (IMU.hasMagnetometer()) {
    IMU.getMag(&IMUMag);
    HMDRawData.MagX = (short)(IMUMag.magX * 5);
    HMDRawData.MagY = (short)(IMUMag.magY * 5);
    HMDRawData.MagZ = (short)(IMUMag.magZ * 5);
  }
  else {
    IMUMag.magX = 0;
	IMUMag.magY = 0;
	IMUMag.magZ = 0;
  }
  
  HMDRawData.AccX = (short)(IMUAccel.accelX * 2048);
  HMDRawData.AccY = (short)(IMUAccel.accelY * 2048);
  HMDRawData.AccZ = (short)(IMUAccel.accelZ * 2048);

  HMDRawData.GyroX = (short)(IMUGyro.gyroX * 16);
  HMDRawData.GyroY = (short)(IMUGyro.gyroY * 16);
  HMDRawData.GyroZ = (short)(IMUGyro.gyroZ * 16);
	
  HID().SendReport(1, &HMDRawData, 63);
}
