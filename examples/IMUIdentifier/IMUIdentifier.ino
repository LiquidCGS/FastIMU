#include <Wire.h>
// Similar to https://github.com/Levi--G/IMU-WhoAmIVerifier except for Arduinos instead of ESP's.

//Do be aware that MPU9150's will report as 6050s, this is because a 9150 is a 6050 with a magnetometer
//if you have a 9250 board and it reports as a 6050, it's most likely a 9150.

//This code will check for an IMU when reset and, if one is found, it will report what it is.
//To re run the check without resetting the Arduino, pull pin 4 to GND.

//Curently supported ones are:

//MPU3050           //ICM20649
//MPU6050           //ICG20660
//MPU6500           //IAM20680
//MPU9250           //IAM20680HP
//MPU9255           //ICM20689
//ICG20330          //ICM20690
//IAM20380          //ICM20948
//IAM20381          //ICM40627 
//ICM20600          //IIM42351
//ICM20601          //IIM42352
//ICM20602          //ICM42605 
//ICM20608-G        //IIM42652  
//ICM20609          //ICM42670-P
//ICM20648          //ICM42688-V


uint8_t Addresses[2] = {0x68, 0x69};
uint8_t ADOState[2] = {0, 1};

uint8_t Registers[2] = {0x75, 0x00};

uint8_t WhoamIsReg1[25] = {0x68     , 0x70     , 0x71     , 0x73     , 0x11      , 0xAC      , 0x12      , 0xAF        , 0xA6      , 0x98      , 0x20      , 0x4E      , 0x42      , 0x67        , 0xDB        , 0x47        , 0x91      , 0x92       , 0xB5      , 0xB6       , 0xA9      , 0xF8        , 0x6F      , 0x6C      , 0x6D};
String  IMUNameReg1[25] = {"MPU6050", "MPU6500", "MPU9250", "MPU9255", "ICM20600", "ICM20601", "ICM20602", "ICM20608-G", "ICM20609", "ICM20689", "ICM20690", "ICM40627", "ICM42605", "ICM42670-P", "ICM42688-V", "ICM42688-P", "ICG20660", "ICG20330" , "IAM20380", "IAM20381" , "IAM20680", "IAM20680HP", "IIM42652", "IIM42351", "IIM42352"};
String  IMUCapbReg1[25] = {"3A,3G"  , "3A,3G"  , "3A,3G,3M", "3A,3G,3M", "3A,3G" , "3A,3G"   , "3A,3G"   , "3A,3G"     , "3A,3G"   , "3A,3G"   , "3A,3G"   , "3A,3G"   , "3A,3G"   , "3A,3G"     , "3A,3G"     , "3A,3G"     , "3A,3G"   , "3G"       , "3A"      , "3G"       , "3A,3G"   , "3A,3G"     , "3A,3G"   , "3A"      , "3A"};

uint8_t WhoamIsReg2[4] = {0xE0      , 0xE1      , 0xEA      , 0x68                                                        ,};
String  IMUNameReg2[4] = {"ICM20648", "ICM20649", "ICM20948", "(IXZ2510/IDG-2020/IDG-2021/IXZ-2020/IXZ-2020) or (MPU3050)",};
String  IMUCapbReg2[4] = {"3A,3G"   , "3A,3G"   , "3A,3G,3M", "(2G) or (3G)"                                              ,};

bool detected = false;

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
  detected = false;
  for (int i = 0; i < sizeof(Addresses); i++) {
    for (int j = 0; j < sizeof(WhoamIsReg1); j++) {
      if (readByte(Addresses[i], Registers[0]) == WhoamIsReg1[j]) {
        Serial.print("IMU Detected: ");
        Serial.print(IMUNameReg1[j]);
        Serial.print(" On address: 0x");
        Serial.print(Addresses[i], HEX);
        Serial.print(" (ADO=");
        Serial.print(ADOState[i]);
        Serial.print("), WhoAmI = 0x");
        Serial.print(WhoamIsReg1[j], HEX);
        Serial.print(" IMU Has the following axis: ");
        Serial.println(IMUCapbReg1[j]);
        Serial.println("----------------------------------");
        detected = true;
      }
    }
    for (int j = 0; j < sizeof(WhoamIsReg2); j++) {
      if (readByte(Addresses[i], Registers[0]) == WhoamIsReg2[j]) {
        Serial.print("IMU Detected: ");
        Serial.print(IMUNameReg2[j]);
        Serial.print(" On address: 0x");
        Serial.print(Addresses[i], HEX);
        Serial.print(" (ADO=");
        Serial.print(ADOState[i]);
        Serial.print("), WhoAmI = 0x");
        Serial.print(WhoamIsReg2[j], HEX);
        Serial.print(" IMU Has the following axis: ");
        Serial.println(IMUCapbReg2[j]);
        Serial.println("----------------------------------");
        detected = true;
      }
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
