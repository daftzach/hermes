#include "onboard.h"

Adafruit_FONA fona  = Adafruit_FONA(FONA_RST);
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

int8_t deviceState = 0;

uint8_t getTelemetry() {
	// BME680 Telemetry
	int16_t temperature, altitude, pressure, humidity, gas;

	// MPU-6050 Telemetry
	int16_t acX, acY, acZ, gyX, gyY, gyZ;

	// FONA Telemetry
	uint16_t charge, voltage;

	// Contain telemetry recorded
	String telemetry = String("");

	if(!bme.performReading()) {
		Serial.println(F("BME: Failed to perform reading!"));
		return false;
	}

	Wire.beginTransmission(MPU6050_I2C_ADDRESS);
	Wire.write(0x3B); 
	Wire.endTransmission(false);

	Wire.requestFrom(MPU6050_I2C_ADDRESS, 14, true);  // request a total of 14 registers

	// Acceleration Telemetry
	acX = Wire.read() << 8 | Wire.read() / 16384.0;  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
	telemetry += "xA=" + acX;

	acY = Wire.read() << 8 | Wire.read() / 16384.0;  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
	telemetry += "&yA=" + acY;

	acZ = Wire.read() << 8 | Wire.read() / 16384.0;  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
	telemetry += "&zA=" + acZ;

	// Gyroscopic Telemetry
	gyX = Wire.read() << 8 | Wire.read() / 131.0;  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
	telemetry += "&xG=" + gyX;

	gyY = Wire.read() << 8 | Wire.read() / 131.0;  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
	telemetry += "&yG=" + gyY;

	gyZ = Wire.read() << 8 | Wire.read() / 131.0;  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
	telemetry += "&zG=" + gyZ;

	altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
	telemetry += "&altitude=" + altitude;

	temperature = bme.temperature;
	telemetry += "&temperature=" + temperature;

	pressure = bme.pressure;
	telemetry += "&pressure=" + pressure;

	humidity = bme.humidity;
	telemetry += "&humidity=" + humidity;

	gas = bme.gas_resistance / 1000.0;
	telemetry += "&voc=" + gas;

	fona.getBattPercent(&charge);
	telemetry += "&charge=" + charge;

	fona.getBattVoltage(&voltage);
	telemetry += "&voltage=" + voltage;

	uint8_t telemetryElement[telemetry.length()];
	telemetry.toCharArray(telemetryElement, telemetry.length());

	return telemetryElement;
}

bool postReq(uint8_t requestData) {
	uint16_t statuscode;
	int16_t length;

	flushSerial();

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

	while(!fona.enableGPRS(true)) {
		Serial.println(F("FONA: Trying to enable GPRS..."));
	}

	delay(5000);

	if(!bme.begin()) {
		Serial.println(F("BME: Could not initialize!"));
		return false;
	}

	bme.setTemperatureOversampling(BME680_OS_8X);
	bme.setHumidityOversampling(BME680_OS_2X);
	bme.setPressureOversampling(BME680_OS_4X);
	bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
	bme.setGasHeater(320, 150);

	Wire.begin();
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(0x6B);  // PWR_MGMT_1 register
	Wire.write(0);     // set to zero (wakes up the MPU-6050)
	Wire.endTransmission(true);
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
		if(!postReq(getTelemetry())) {
			Serial.println(F("ERROR: Could not get/post telemetry!"));
		}

		if(fona.getCallStatus() == 3) {
			deviceState = 2;
		}
	} else if(deviceState == 2) {
		while(1);
	}

	delay(1000);
}