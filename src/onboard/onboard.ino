#include "onboard.h"

Adafruit_FONA fona  = Adafruit_FONA(FONA_RST);
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

// Store telemetry data for upload
DataQueue<uint8_t> telemetryQueue(MAX_QUEUE_SIZE);

int8_t deviceState = 0;

bool getTelemetry(DataQueue<uint8_t> &queue) {
	uint16_t temperature, altitude;
	String telemetry = String("");

	if(!bme.performReading()) {
		Serial.println(F("BME: Failed to perform reading!"));
		return false;
	}

	temperature = bme.temperature;
	telemetry += "temperature=" + temperature;

	altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
	telemetry += "&altitude=" + altitude;

	if(!queue.isFull()) {
		uint8_t telemetryElement[telemetry.length()];
		telemetry.toCharArray(telemetryElement, telemetry.length());
		queue.enqueue(telemetryElement);
	}

	return true;
}

void setup() {
	Serial.begin(115200);

	fonaSerial->begin(4800);
	if(!fona.begin(*fonaSerial)) {
		Serial.println(F("FONA: Could not initialize!"));
		return false;
	}

	delay(5000);

	fona.enableGPRS(false);

	delay(5000);

	if(!fona.enableGPRS(true)) {
		Serial.println(F("FONA: Trying to enable GPRS..."));
	}

	if(!bme.begin()) {
		Serial.println(F("BME: Could not initialize!"));
		return false;
	}

	bme.setTemperatureOversampling(BME680_OS_8X);
	bme.setPressureOversampling(BME680_OS_4X);
}

void loop() {
	// Wait for launch command (a phone call)
	if(deviceState == 0) {
			while(fona.getCallStatus() != 3) {
				Serial.println("INFO: Standing by for launch command...");
			}
		fona.hangUp();
		deviceState = 1;
	} else if(deviceState == 1) {
		if(getTelemetry(telemetryQueue)) {
			Serial.println(telemetryQueue.item_count());
		} else {
			Serial.println(F("ERROR: Could not get telemetry"));
		}

		if(fona.getCallStatus() == 3) {
			deviceState = 2;
		}
	} else if(deviceState == 2) {
		Serial.println("INFO: Uploading launch telemetry...");
		while(1);
	}

	delay(1000);
}