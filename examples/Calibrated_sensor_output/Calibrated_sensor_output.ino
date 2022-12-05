#include "FastIMU.h"

MPU9250 IMU;    //Change "MPU9250" to the name of any supported IMU!

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
MagData magData;

void setup() {
  float accelBias[3] = { 0 };
  float gyroBias[3] = { 0 };
  float magScale[3] = { 0 };
  float magBias[3] = { 0 };
  IMU.init(calib, 0x68);

  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  Serial.println("FastIMU calibration & data example");

  if (IMU.hasMagnetometer()) {
    delay(1000);
    Serial.println("Move IMU in figure 8 pattern until done.");
    delay(3000);
    IMU.calibrateMag(magBias, magScale);
    Serial.println("Magnetic calibration done!");
  }
  else {
    delay(5000);
  }

  delay(5000);
  Serial.println("Keep IMU level.");
  delay(5000);
  IMU.calibrateAccelGyro(accelBias, gyroBias);
  Serial.println("Calibration done!");
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(accelBias[0]);
  Serial.print(", ");
  Serial.print(accelBias[1]);
  Serial.print(", ");
  Serial.println(accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(gyroBias[0]);
  Serial.print(", ");
  Serial.print(gyroBias[1]);
  Serial.print(", ");
  Serial.println(gyroBias[2]);
  if (IMU.hasMagnetometer()) {
    Serial.println("Mag biases X/Y/Z: ");
    Serial.print(magBias[0]);
    Serial.print(", ");
    Serial.print(magBias[1]);
    Serial.print(", ");
    Serial.println(magBias[2]);
    Serial.println("Mag Scale X/Y/Z: ");
    Serial.print(magScale[0]);
    Serial.print(", ");
    Serial.print(magScale[1]);
    Serial.print(", ");
    Serial.println(magScale[2]);
  }
  delay(5000);

  calib.valid = true;
  calib.gyroBias[0] = gyroBias[0];
  calib.gyroBias[1] = gyroBias[1];
  calib.gyroBias[2] = gyroBias[2];
  calib.accelBias[0] = accelBias[0];
  calib.accelBias[1] = accelBias[1];
  calib.accelBias[2] = accelBias[2];
  calib.magBias[0] = magBias[0];
  calib.magBias[1] = magBias[1];
  calib.magBias[2] = magBias[2];
  calib.magScale[0] = magScale[0];
  calib.magScale[1] = magScale[1];
  calib.magScale[2] = magScale[2];

  IMU.init(calib, 0x68);
}

void loop() {
  IMU.update();
  IMU.getAccel(&accelData);
  Serial.print(accelData.accelX);
  Serial.print("\t");
  Serial.print(accelData.accelY);
  Serial.print("\t");
  Serial.print(accelData.accelZ);
  Serial.print("\t");
  IMU.getGyro(&gyroData);
  Serial.print(gyroData.gyroX);
  Serial.print("\t");
  Serial.print(gyroData.gyroY);
  Serial.print("\t");
  Serial.print(gyroData.gyroZ);
  if (IMU.hasMagnetometer()) {
    IMU.getMag(&magData);
    Serial.print("\t");
    Serial.print(magData.magX);
    Serial.print("\t");
    Serial.print(magData.magY);
    Serial.print("\t");
    Serial.print(magData.magZ);
  }
  Serial.print("\t");
  Serial.println(IMU.getTemp());
  delay(50);
}
