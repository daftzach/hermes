#include "onboard.h"

Hermes* sys;

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

void setup() {
	if (!sys->initSystems(fonaSS, *fonaSerial)) {
		Serial.println("ERROR! Failed to start systems.");
	} else {
		Serial.println("GO FOR LAUNCH!");
	}
}

void loop() {}
