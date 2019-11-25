#include "onboard.h"

bool Hermes::initSystems(SoftwareSerial softwareSerial, SoftwareSerial &serialRef) {
	Serial.begin(4800);

	return initFona(softwareSerial, serialRef);
}

bool Hermes::initFona(SoftwareSerial softwareSerial, SoftwareSerial &serialRef) {
	Adafruit_FONA newFona = Adafruit_FONA(FONA_RST);

	serialRef.begin(4800);

	// Initialize FONA board
	if (!newFona.begin(serialRef)) {
		Serial.println(F("FONA: Failed to initialize!"));
		return false;
	} else {
		Serial.println("FONA: Initialized...");
	}

	// Disable GPRS just in case it is already enabled
	newFona.enableGPRS(false);

	// Initialize GPRS
	if (!newFona.enableGPRS(true)) {
		Serial.println(F("FONA: Failed to initialize GPRS!"));
		return false;
	}
	
	return true;
}

Adafruit_FONA Hermes::getFona() {
	return fona;
}
