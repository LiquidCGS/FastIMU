#include "FastIMU.h"
#include "Madgwick.h"
#include "EEPROM.h"
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEHIDDevice.h>
#include <BLE2902.h>

//This example is for use with the Relativty steamvr driver. it outputs a rotation quaternion over HID that the driver can interpret as HMD rotation.

#define IMU_ADDRESS 0x68    // Change to the address of the IMU
MPU6500 IMU;                // Change to the name of any supported IMU!

#define IMU_GEOMETRY 0      // Change to your current IMU geometry (check docs for a reference pic).

// Currently supported IMUS: MPU9255 MPU9250 MPU6886 MPU6500 MPU6050 ICM20689 ICM20690 BMI055 BMX055 BMI160 LSM6DS3 LSM6DSL QMI8658

#define FILTER_MAX_BETA 0.15
#define FILTER_MIN_BETA 0.015
#define FILTER_DROPOFF  0.85      // filter values

calData calib = { 0 };  // Calibration data
AccelData IMUAccel;    // Sensor data
GyroData IMUGyro;
MagData IMUMag;

GyroData GyroVel;   // used for angular velocity based filter beta

Madgwick filter;
bool flag;

const uint8_t customHidReportDescriptor[] = {
    0x06, 0x03, 0x00,         // USAGE_PAGE (vendor defined)
    0x09, 0x00,               // USAGE (Undefined)
    0xa1, 0x01,               // COLLECTION (Application)
    0x15, 0x00,               // LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,         // LOGICAL_MAXIMUM (255)
    0x85, 0x01,               // REPORT_ID (1)
    0x75, 0x08,               // REPORT_SIZE (8)
    0x95, 0x3f,               // REPORT_COUNT (63)
    0x09, 0x00,               // USAGE (Undefined)
    0x81, 0x02,               // INPUT (Data,Var,Abs) - to the host
    0xc0
};

BLEHIDDevice* hid;
BLECharacteristic* inputReport;

float quat[4];

void setup() {
  #if defined(ARDUINO_ARCH_ESP32)
    Wire.begin(21, 22);
    #else
    Wire.begin(35, 36);
  #endif
    Wire.setClock(400000); // 400kHz clock

    int calStatus = 0;
    Serial.begin(115200);

    BLEDevice::init("RelativityVR");
    BLEServer* pServer = BLEDevice::createServer();
    hid = new BLEHIDDevice(pServer);

    inputReport = hid->inputReport(1); // Report ID

    hid->manufacturer()->setValue("Relativity");
    hid->pnp(0x01, 0x02E5, 0xABCD, 0x0110);
    hid->hidInfo(0x00, 0x01);

    hid->reportMap((uint8_t*)customHidReportDescriptor, sizeof(customHidReportDescriptor));
    hid->startServices();

    BLESecurity* pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);

    BLEAdvertising* pAdvertising = pServer->getAdvertising();
    pAdvertising->setAppearance(HID_KEYBOARD);
    pAdvertising->addServiceUUID(hid->hidService()->getUUID());
    pAdvertising->start();

    hid->setBatteryLevel(100);

    EEPROM.get(100, calStatus);
    if (calStatus == 99) {
        EEPROM.get(200, calib);
    }
    IMU.setIMUGeometry(IMU_GEOMETRY);
    int err = IMU.init(calib, IMU_ADDRESS);
    if (err != 0) {
        Serial.print("Error initializing IMU! e:");
        Serial.println(err);
    }

    filter.begin(2.f);                                                      // warm up filter before use
    for (int i = 0; i < 2000; i++) {
        IMU.update();
        IMU.getAccel(&IMUAccel);
        IMU.getGyro(&IMUGyro);
        if (IMU.hasMagnetometer()) {
            IMU.getMag(&IMUMag);
            filter.update(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ, IMUMag.magX, IMUMag.magY, IMUMag.magZ);
        } else {
            filter.updateIMU(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ);
        }
    }
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
	  Serial.println("Please remember to set hmdIMUdmpPackets to false in the driver settings.");
      Serial.println("You can now close the Serial monitor.");
      delay(5000);
    }
  }
  IMU.update();
  IMU.getAccel(&IMUAccel);
  IMU.getGyro(&IMUGyro);

  float Av = GyroVel.gyroX * GyroVel.gyroX + GyroVel.gyroY * GyroVel.gyroY + GyroVel.gyroZ * GyroVel.gyroZ; //sqr magnitude
  if (Av > 100.f) Av = 100.f;
  filter.changeBeta(Av * (FILTER_MAX_BETA - FILTER_MIN_BETA) / 100 + FILTER_MIN_BETA);                      //some stuff

  if (IMU.hasMagnetometer()) {
    IMU.getMag(&IMUMag);
    filter.update(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ, IMUMag.magX, IMUMag.magY, IMUMag.magZ);
  }
  else {
    filter.updateIMU(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ, IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ);
  }

  GyroVel.gyroX += IMUGyro.gyroX * filter.delta_t;
  GyroVel.gyroY += IMUGyro.gyroY * filter.delta_t;
  GyroVel.gyroZ += IMUGyro.gyroZ * filter.delta_t;
  GyroVel.gyroX *= FILTER_DROPOFF;
  GyroVel.gyroY *= FILTER_DROPOFF;
  GyroVel.gyroZ *= FILTER_DROPOFF;                  //velocity calculations and dropoff...

  quat[0] = filter.getQuatW();
  quat[1] = filter.getQuatY();
  quat[2] = filter.getQuatZ();
  quat[3] = filter.getQuatX();

inputReport->setValue((uint8_t*)quat, 63);
inputReport->notify();
}
