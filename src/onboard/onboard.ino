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

bool postReq(uint8_t requestData) {
	uint16_t statuscode;
	int16_t length;

	if(!fona.HTTP_POST_start(POST_URL, F("application/x-www-form-urlencoded"), (uint8_t *) requestData, strlen(requestData), &statuscode, (uint16_t *)&length)) {
		Serial.println("ERROR: Failed to send POST request!");
	}
	while (length > 0) {
		while (fona.available()) {
			char c = fona.read();

			#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
            	loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
            	UDR0 = c;
			#else
				Serial.write(c);
			#endif

			length--;
			if (! length) break;
          }
        }
	fona.HTTP_POST_end();
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

	while(!fona.enableGPRS(true)) {
		Serial.println(F("FONA: Trying to enable GPRS..."));
	}

	delay(5000);

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
			delay(5000);
		}

		fona.hangUp();
		deviceState = 1;
	} else if(deviceState == 1) {
		if(getTelemetry(telemetryQueue)) {
			Serial.println(telemetryQueue.item_count());
		} else {
			Serial.println(F("ERROR: Could not get telemetry!"));
		}

		if(fona.getCallStatus() == 3) {
			deviceState = 2;
		}
	} else if(deviceState == 2) {
		Serial.println("INFO: Uploading flight telemetry...");
		for(int i = 0; i < telemetryQueue.item_count(); i++) {
			postReq(telemetryQueue.dequeue());
		}
		while(1);
	}

	delay(1000);
}