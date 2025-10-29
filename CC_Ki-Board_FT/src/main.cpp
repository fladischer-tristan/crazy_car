#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "Types.hpp"
#include "AuxUart.hpp"

// ESP32S3 Pinout
#define ESP_S2 D0
#define ESP_S3 D1
#define ESP_S4 D2
#define ESP_NEO_PIXEL D3
#define ESP_SDA D4
#define ESP_SCL D5
#define ESP_STEERING_PWM_IN D10
#define ESP_ESC_PWM_IN D9
#define ESP_S1 D8

// WiFi ssid and pwd
const char* SSID = "HUAWEI-E5180-E375";
const char* PASS = "Y271HJ02FFT";

// TCP Server details
const char* host = "192.168.8.106";  // IP deines PCs im selben WLAN
const uint16_t port = 5000;          // Port, den dein Server abhört

// length of UART packet
const byte SENSOR_FRAME_LENGTH = sizeof(SensorData);

WiFiClient client;

// Prototyping functions
void sensorDataDebugPrint(SensorData& sensorData);

void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  	switch (event) {
    	case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      		Serial.printf("[WiFi] Disconnected, reason=%d\n", info.wifi_sta_disconnected.reason);
      		break;
    	case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      		Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
      		break;
    	default:
      		break;
  }
}

void setup() {
	delay(3000); // ESP Serial needs the delay to actually work in the setup
	Serial.begin(115200); // DEBUG Serial
	delay(100);
	auxUartInit(); // Serial to Arduino Mega
	delay(100);

	// RC-Receiver pins
	pinMode(ESP_STEERING_PWM_IN, INPUT);
	pinMode(ESP_ESC_PWM_IN, INPUT);

	// small wifi test
	WiFi.begin(SSID, PASS);
	Serial.print("Verbinde mit WLAN...");
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}
	Serial.println("\n[WiFi] Verbunden!");
	Serial.println(WiFi.localIP());

	Serial.print("Verbinde mit Server ");
	Serial.print(host);
	Serial.print(":");
	Serial.println(port);

	if (client.connect(host, port)) {
		Serial.println("✅ Verbindung erfolgreich!");
		client.println("Hallo vom ESP32S3!");
	} else {
		Serial.println("❌ Verbindung fehlgeschlagen!");
	}
}

/*
* 1. Wait for START_OF_COMM_BYTE
* 2. Read Motor Pulses and send them to ESP
* 3. Wait for incoming SensorData struct
* 4. Send struct away over WiFi (ignore for now, just print to Serial)
*/
void loop() {
	static long lastTimeDebug = millis();
	if (millis() - lastTimeDebug >= 50) {
		Serial.println("Hello from Loop");

		// Send some debug data to TCP Server
		if (client.connected()) {
			client.print("ServoPulse(us): ");
			client.println(pulseIn(ESP_STEERING_PWM_IN, HIGH, 25000));
			client.print("EscPulse(us): ");
			client.println(pulseIn(ESP_ESC_PWM_IN, HIGH, 25000));

		} else {
			Serial.println("Client getrennt, versuche Reconnect...");
			client.connect(host, port);
		}

		lastTimeDebug = millis();
	}
	
	// 1. Wait for START_OF_COMM_BYTE
	if (auxUart.available() && auxUart.peek() == START_OF_COMM_BYTE) {
		while (auxUart.available()) auxUart.read();
		Serial.println("SoC byte received! Comm started");

		while(true) {
			// check for END_OF_COMM_BYTE
			if (auxUart.available() && auxUart.peek() == END_OF_COMM_BYTE) {
				auxUart.read();
				Serial.println("EoC byte received! Comm stopped");
				while (auxUart.available()) auxUart.read(); // clearing UART buffer
				break;
			}

			// 2. Read and send Motor Pulses
			unsigned long startTime = millis();
			unsigned long servoPulseUS = pulseIn(ESP_STEERING_PWM_IN, HIGH, 25000); // 25ms max delay
			unsigned long escPulseUS = pulseIn(ESP_ESC_PWM_IN, HIGH, 25000);

			Serial.print("escPulse: ");
			Serial.println(escPulseUS);
			Serial.print("servoPulse: ");
			Serial.println(servoPulseUS);

			SensorData motorData;
			motorData.escPulse = escPulseUS;
			motorData.servoPulse = servoPulseUS;

			Serial.print("sizeof(SensorData): ");
			Serial.println((unsigned long)sizeof(motorData));
			
			sendMotorDataToMega(&motorData);
			unsigned long elapsedTime = millis() - startTime;
			Serial.print("Sending took ");
			Serial.print(elapsedTime);
			Serial.println("ms.");

			// 3. Wait for SensorData
			while (auxUart.available() < SENSOR_FRAME_LENGTH) {
				// TODO this loop is not really working for the end of comm, it will be stuck here
			}

			SensorData sensorData;
			auxUart.readBytes(reinterpret_cast<byte*>(&sensorData), sizeof(SensorData));
			while (auxUart.available()) auxUart.read();

			// 4. Send data to PC / print to terminal for now
			sensorDataDebugPrint(sensorData);
		}
	}
}

// Printing the contents of a SensorData struct to the Serial for DEBUG purpose
void sensorDataDebugPrint(SensorData& sensorData) {
	Serial.print("sizeof(SensorData): ");
	Serial.println((unsigned long)sizeof(sensorData));
	Serial.print(F("id: ")); Serial.print(sensorData.packetNumber);
	Serial.print(F(" vel: ")); Serial.print(sensorData.velocity, 2);
	Serial.print(F(" bat: ")); Serial.print(sensorData.batteryVoltage, 2);

	Serial.print(F(" | dist L/M/R: "));
	Serial.print(sensorData.leftDistance, 2); Serial.print(" / ");
	Serial.print(sensorData.middleDistance, 2); Serial.print(" / ");
	Serial.print(sensorData.rightDistance, 2);

	Serial.print(F(" | acc: "));
	Serial.print(sensorData.ax, 2); Serial.print(" ");
	Serial.print(sensorData.ay, 2); Serial.print(" ");
	Serial.print(sensorData.az, 2);

	Serial.print(F(" | gyro: "));
	Serial.print(sensorData.gx, 2); Serial.print(" ");
	Serial.print(sensorData.gy, 2); Serial.print(" ");
	Serial.print(sensorData.gz, 2);

	Serial.print(F(" | servo: "));
	Serial.print(sensorData.servoPulse);
	Serial.print(F(" esc: "));
	Serial.println(sensorData.escPulse);
}