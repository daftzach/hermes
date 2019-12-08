#ifndef _onboard_h
#define _onboard_h

// For Adafruit FONA SIM808 component - handles telemetry streaming
#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>

#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#include <Wire.h>
#include <SPI.h>

#include <Queue.h>

// This limits flight data collection to approx. 5 minutes
#define MAX_QUEUE_SIZE 500

#define POST_URL "itp.zacl.me/test/api/createrecord.php"

#define FONA_RX  2
#define FONA_TX  3
#define FONA_RST 4

#define BME_SCK  13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS   10

#define MPU_ADDR 0x68

#define SEALEVELPRESSURE_HPA 1013.25

#endif