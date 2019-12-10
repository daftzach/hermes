#include "Adafruit_FONA.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>

#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

const int MPU_addr=0x68;  // I2C address of the MPU-6050

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;


int16_t temperature, altitude;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
uint16_t vbat, charge, voltage;

String data;

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");
  
  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial )) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }

  delay(5000);

  fona.enableGPRS(false);

  delay(5000);
  
  if (!fona.enableGPRS(true)) {
    Serial.println("GRPS failed");
  }

  delay(5000);

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  } else {
    Serial.println("BME680 is go");
  }

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);

    Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void loop() {
    // Wait for launch command (a phone call)
  int8_t callState = fona.getCallStatus();
  switch (callState) {
          case 0: Serial.println(F("Ready")); break;
          case 1: Serial.println(F("Could not get status")); break;
          case 3: Serial.println(F("Ringing (incoming)")); break;
          case 4: Serial.println(F("Ringing/in progress (outgoing)")); break;
          default: Serial.println(F("Unknown")); break;
        }

  data = String("");
  
  uint16_t statuscode;
  int16_t length;
  char url[] = "itp.zacl.me/test/api/createrecord.php";

  Serial.println("Init data...");

  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); 
  Wire.endTransmission(false);

   Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  temperature = bme.temperature;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  fona.getBattPercent(&vbat);
  charge = vbat;

  fona.getBattVoltage(&vbat);
  voltage = vbat;
  
  data += "temperature=";
  data += temperature;
  
  data += "&altitude=";
  data += altitude;
  
  data += "&x_accel=";
  data += AcX;

  data += "&y_accel=";
  data += AcY;

  data += "&z_accel=";
  data += AcZ;

  data += "&x_gyro=";
  data += GyX;

  data += "&y_gyro=";
  data += GyY;
  
  data += "&z_gyro=";
  data += GyZ;
  
  data += "&battery_charge=";
  data += charge;

  data += "&battery_voltage=";
  data += voltage;  

  Serial.println("Posting results...");
  Serial.println(data);

  uint8_t postData[data.length()];
  data.toCharArray(postData, data.length());

 
}
