/************************************************

A Hermes object will represent the entire system.

************************************************/

#ifndef hermes_h
#define hermes_h

#include "onboard.h"

// FONA SIM808 Pins
#define FONA_RX  2
#define FONA_TX  3
#define FONA_RST 4

class Hermes {
public:
	bool initSystems(SoftwareSerial softwareSerial, SoftwareSerial &serialRef);
	Adafruit_FONA getFona();
private:
	SoftwareSerial fonaSS;
	SoftwareSerial *fonaSerial;
	Adafruit_FONA& fona;
	bool initFona(SoftwareSerial softwareSerial, SoftwareSerial &serialRef);
};

#endif
