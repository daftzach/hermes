#ifndef _onboard_h
#define _onboard_h

// For Adafruit FONA SIM808 component - handles telemetry streaming
#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>

#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#include <Wire.h>
#include <SPI.h>

#define POST_URL "http://itp.zacl.me/hermes/scripts/uploadTelemetry.php"

#define FONA_RX  2
#define FONA_TX  3
#define FONA_RST 4

#define BME_SCK  13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS   10

#define MPU6050_I2C_ADDRESS 0x68

#define SEALEVELPRESSURE_HPA 1013.25

#endif