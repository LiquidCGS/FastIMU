#include <Wire.h>
// Similar to https://github.com/Levi--G/IMU-WhoAmIVerifier

//Do be aware that MPU9150's will report as 6050s, this is because a 9150 is a 6050 with a magnetometer
//if you have a 9250 board and it reports as a 6050, it's most likely a 9150.

//This code will check for an IMU when reset and, if one is found, it will report what it is.
//To re run the check without resetting the Arduino, pull pin 4 to GND.

#define NUM_IMUS 40

bool errorflag;

typedef struct IMU {
  uint8_t Address1;
  uint8_t Address2;
  uint8_t Register;
  uint8_t ExpectedID;
  const char*  IMUName PROGMEM;
  const char*  IMUCapabilities PROGMEM;
  bool    LibSupported;
};

const IMU IMUList[NUM_IMUS] =
{
  {0x68, 0x69, 0x75, 0x68, "MPU6050",   "3A,3G",	true},
  {0x68, 0x69, 0x75, 0x70, "MPU6500",   "3A,3G",	true},
  {0x68, 0x69, 0x75, 0x74, "MPU6515",   "3A,3G",	true},
  {0x68, 0x69, 0x75, 0x19, "MPU6886",   "3A,3G",	true},
  {0x68, 0x69, 0x75, 0x71, "MPU9250",   "3A,3G,3M",	true},
  {0x68, 0x69, 0x75, 0x73, "MPU9255",   "3A,3G,3M",	true},
  {0x69, 0x68, 0x00, 0xD1, "BMI160",    "3A,3G",	true},
  {0x6B, 0x6A, 0x0F, 0x69, "LSM6DS3",   "3A,3G",	true},
  {0x6B, 0x6A, 0x0F, 0x6A, "LSM6DSL",   "3A,3G",	true},
  {0x68, 0x69, 0x75, 0x98, "ICM20689",  "3A,3G",	true},
  {0x68, 0x69, 0x75, 0x20, "ICM20690",  "3A,3G",	true},
  {0x6B, 0x6A, 0x00, 0x05, "QMI8658",   "3A,3G",	true},
  {0x18, 0x19, 0x00, 0xFA, "BMI055 or BMX055", "3A,3G or 3A,3G,3M", true},
  {0x0D, 0x0D, 0x0D, 0xFF, "QMC5883L",   "3M",		true},
  {0x68, 0x69, 0x75, 0x75, "Unknown or fake Invensense IMU, use 'IMU_Generic'",   "3A,3G, possibly 3M?",    true},
  {0x6B, 0x6A, 0x0F, 0x6B, "LSM6DSR",   "3A,3G",	false},
  {0x6B, 0x6A, 0x0F, 0x6C, "LSM6DSO",   "3A,3G",	false},
  {0x6B, 0x6A, 0x00, 0xFC, "QMI8610",   "3A,3G",	false},
  {0x68, 0x69, 0x75, 0x92, "ICG20330",  "3G",		false},
  {0x68, 0x69, 0x75, 0xB5, "IAM20380",  "3A",		false},
  {0x68, 0x69, 0x75, 0xB6, "IAM20381",  "3G",		false},
  {0x68, 0x69, 0x75, 0x11, "ICM20600",  "3A,3G",	false},
  {0x68, 0x69, 0x75, 0xAC, "ICM20601",  "3A,3G",	false},
  {0x68, 0x69, 0x75, 0x12, "ICM20602",  "3A,3G",	false},
  {0x68, 0x69, 0x75, 0xAF, "ICM20608-G", "3A,3G",	false},
  {0x68, 0x69, 0x75, 0xA6, "ICM20609",  "3A,3G",	false},
  {0x68, 0x69, 0x00, 0xE0, "ICM20648",  "3A,3G",	false},
  {0x68, 0x69, 0x00, 0xE1, "ICM20649",  "3A,3G",	false},
  {0x68, 0x69, 0x75, 0xA9, "ICG20660",  "3A,3G",	false},
  {0x68, 0x69, 0x75, 0x91, "IAM20680",  "3A,3G",	false},
  {0x68, 0x69, 0x00, 0xEA, "ICM20948",  "3A,3G,3M",	false},
  {0x68, 0x69, 0x75, 0x6C, "IIM42351",  "3A",		false},
  {0x68, 0x69, 0x75, 0x6D, "IIM42352",  "3A",		false},
  {0x68, 0x69, 0x75, 0x4E, "ICM40627",  "3A,3G",	false},
  {0x68, 0x69, 0x75, 0x42, "ICM42605",  "3A,3G",	false},
  {0x68, 0x69, 0x75, 0x6F, "IIM42652",  "3A,3G",	false},
  {0x68, 0x69, 0x75, 0x67, "ICM42670-P", "3A,3G",	false},
  {0x68, 0x69, 0x75, 0xDB, "ICM42688-V", "3A,3G",	false},
  {0x68, 0x69, 0x00, 0x68, "MPU3050",   "3G",		false},
  {0x1E, 0x1E, 0x0C, 0x33, "HMC5883L",   "3M",		false},
};

void setup() {
  Serial.begin(9600);
  while (!Serial) ;
  errorflag = false;
  pinMode(4, INPUT_PULLUP);
  Wire.begin();
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(3000);
#endif
  Serial.println(F("\n=========== IMU Identifier ==========="));
}

void loop() {
  static int a = 0;
  while (digitalRead(4) && a != 0) ;   //do once
  a = 1;
  bool detected = false;
  for (int i = 0; i < NUM_IMUS; i++)
  {
#ifdef WIRE_HAS_TIMEOUT
    if (errorflag || Wire.getWireTimeoutFlag()) {
      Serial.print(F("Error while reading address 0x"));
      Serial.print(IMUList[i].Address1, HEX);
      Serial.print(F(": "));
      if (Wire.getWireTimeoutFlag()) {
        Serial.println(F("I2C bus timed out. (Bad IMU? check wiring.)"));
      }
      else {
        Serial.println(F("Unknown error while reading/writing"));
      }
      Serial.println(F("======================================"));
      Wire.clearWireTimeoutFlag();
      errorflag = false;
      delay(2000);
      return;
    }
#endif
    if (readByte(IMUList[i].Address1, IMUList[i].Register) == IMUList[i].ExpectedID)
    {
      detected = true;
      Serial.print(F("IMU Found: "));
      Serial.print(IMUList[i].IMUName);
      Serial.print(F(" On address: 0x"));
      Serial.println(IMUList[i].Address1, HEX);
      Serial.print(F("This IMU is capable of the following axis: "));
      Serial.println(IMUList[i].IMUCapabilities);
      if (IMUList[i].LibSupported) {
        Serial.println(F("This IMU is supported by the FastIMU library."));
      }
      else
      {
        Serial.println(F("This IMU is not supported by the FastIMU library."));
      }
      Serial.println(F("======================================"));
    }
    else if (readByte(IMUList[i].Address2, IMUList[i].Register) == IMUList[i].ExpectedID)
    {
      detected = true;
      Serial.print(F("IMU Found: "));
      Serial.print(IMUList[i].IMUName);
      Serial.print(F(" On address: 0x"));
      Serial.println(IMUList[i].Address2, HEX);
      Serial.print(F("This IMU is capable of the following axis: "));
      Serial.println(IMUList[i].IMUCapabilities);
      if (IMUList[i].LibSupported) {
        Serial.println(F("This IMU is supported by the FastIMU library."));
      }
      else
      {
        Serial.println(F(" This IMU is not supported by the FastIMU library."));
      }
      Serial.println(F("======================================"));
    }
  }
  if (!detected) {
    Serial.println(F("No IMU detected"));
    Serial.println(F("======================================"));
  }
  delay(1000);
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data;                            // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  int i = Wire.endTransmission(false);     // Send the Tx buffer, but send a restart to keep connection alive
  if (i == 5) {
    return 0;
    errorflag = true;
  }
  i = Wire.requestFrom(address, (uint8_t) 1, true); // Read one byte from slave register address
  if (i == 0) {
    return 0;
    errorflag = true;
  }
  if (Wire.available()) {
    data = Wire.read();                      // Fill Rx buffer with result
  }
  return data;                             // Return data read from slave register
}
