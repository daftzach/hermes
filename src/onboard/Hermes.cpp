#include "Hermes.h"

Hermes::Hermes() {
	
}

bool Hermes::initSystems() {
	Serial.begin(9600);
	Serial.println("Serial initiated...");

	return true;
}