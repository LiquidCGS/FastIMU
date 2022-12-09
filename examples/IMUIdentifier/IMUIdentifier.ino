#include <Wire.h>
// Similar to https://github.com/Levi--G/IMU-WhoAmIVerifier

//Do be aware that MPU9150's will report as 6050s, this is because a 9150 is a 6050 with a magnetometer
//if you have a 9250 board and it reports as a 6050, it's most likely a 9150.

//This code will check for an IMU when reset and, if one is found, it will report what it is.
//To re run the check without resetting the Arduino, pull pin 4 to GND.

#define NUM_IMUS 28

typedef struct IMU {
  uint8_t Address1;
  uint8_t Address2;
  uint8_t Register;
  uint8_t ExpectedID;
  String  IMUName;
  String  IMUCapabilities;
  bool    LibSupported;
};

IMU IMUList[NUM_IMUS] =
{
  {0x68, 0x69, 0x00, 0x68, "MPU3050",   "3G",       false},
  {0x68, 0x69, 0x75, 0x68, "MPU6050",   "3A,3G",    true},
  {0x68, 0x69, 0x75, 0x70, "MPU6500",   "3A,3G",    true},
  {0x68, 0x69, 0x75, 0x71, "MPU9250",   "3A,3G,3M", true},
  {0x68, 0x69, 0x75, 0x73, "MPU9255",   "3A,3G,3M", true},
  {0x68, 0x69, 0x75, 0x92, "ICG20330",  "3G",       false},
  {0x68, 0x69, 0x75, 0xB5, "IAM20380",  "3A",       false},
  {0x68, 0x69, 0x75, 0xB6, "IAM20381",  "3G",       false},
  {0x68, 0x69, 0x75, 0x11, "ICM20600",  "3A,3G",    false},
  {0x68, 0x69, 0x75, 0xAC, "ICM20601",  "3A,3G",    false},
  {0x68, 0x69, 0x75, 0x12, "ICM20602",  "3A,3G",    false},
  {0x68, 0x69, 0x75, 0xAF, "ICM20608-G", "3A,3G",    false},
  {0x68, 0x69, 0x75, 0xA6, "ICM20609",  "3A,3G",    false},
  {0x68, 0x69, 0x00, 0xE0, "ICM20648",  "3A,3G",    false},
  {0x68, 0x69, 0x00, 0xE1, "ICM20649",  "3A,3G",    false},
  {0x68, 0x69, 0x75, 0xA9, "ICG20660",  "3A,3G",    false},
  {0x68, 0x69, 0x75, 0x91, "IAM20680",  "3A,3G",    false},
  {0x68, 0x69, 0x75, 0x98, "ICM20689",  "3A,3G",    true},
  {0x68, 0x69, 0x75, 0x20, "ICM20690",  "3A,3G",    true},
  {0x68, 0x69, 0x00, 0xEA, "ICM20948",  "3A,3G,3M", false},
  {0x68, 0x69, 0x75, 0x6C, "IIM42351",  "3A",       false},
  {0x68, 0x69, 0x75, 0x6D, "IIM42352",  "3A",       false},
  {0x68, 0x69, 0x75, 0x4E, "ICM40627",  "3A,3G",    false},
  {0x68, 0x69, 0x75, 0x42, "ICM42605",  "3A,3G",    false},
  {0x68, 0x69, 0x75, 0x6F, "IIM42652",  "3A,3G",    false},
  {0x68, 0x69, 0x75, 0x67, "ICM42670-P", "3A,3G",    false},
  {0x68, 0x69, 0x75, 0xDB, "ICM42688-V", "3A,3G",    false},
  {0x18, 0x19, 0x00, 0xFA, "BMI055 or BMX055", "3A,3G or 3A,3G,3M", true},
};

void setup() {
  pinMode(4, INPUT_PULLUP);
  Wire.begin();
  Serial.begin(9600);
  while (!Serial) ;
  Serial.println("\n=========== IMU Identifier ===========");
}

void loop() {
  static int a = 0;
  while (digitalRead(4) && a != 0) ;   //do once
  a = 1;
  bool detected = false;
  for (int i = 0; i < NUM_IMUS; i++)
  {
    if (readByte(IMUList[i].Address1, IMUList[i].Register) == IMUList[i].ExpectedID)
    {
      detected = true;
      Serial.print("IMU Found: ");
      Serial.print(IMUList[i].IMUName);
      Serial.print(" On address: 0x");
      Serial.println(IMUList[i].Address1, HEX);
      Serial.print("This IMU is capable of the following axis: ");
      Serial.println(IMUList[i].IMUCapabilities);
      if (IMUList[i].LibSupported) {
        Serial.println("This IMU is supported by the FastIMU library.");
      }
      else
      {
        Serial.println("This IMU is not supported by the FastIMU library.");
      }
      Serial.println("======================================");
    }
    else if (readByte(IMUList[i].Address2, IMUList[i].Register) == IMUList[i].ExpectedID)
    {
      detected = true;
      Serial.print("IMU Found: ");
      Serial.print(IMUList[i].IMUName);
      Serial.print(" On address: 0x");
      Serial.println(IMUList[i].Address2, HEX);
      Serial.print("This IMU is capable of the following axis: ");
      Serial.println(IMUList[i].IMUCapabilities);
      if (IMUList[i].LibSupported) {
        Serial.println("This IMU is supported by the FastIMU library.");
      }
      else
      {
        Serial.println(" This IMU is not supported by the FastIMU library.");
      }
      Serial.println("======================================");
    }
  }
  if (!detected) {
    Serial.println("No IMU detected");
  }
  delay(1000);
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data;                            // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}
