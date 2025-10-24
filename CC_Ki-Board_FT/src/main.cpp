#include <Arduino.h>
#include <WiFi.h>
#include "Types.hpp"

// ESP32S3 Pinout
#define ESP_S2 D0
#define ESP_S3 D1
#define ESP_S4 D2
#define ESP_NEO_PIXEL D3
#define ESP_SDA D4
#define ESP_SCL D5
#define ESP_TX D7
#define ESP_RX D6
#define ESP_STEERING_PWM_IN D10
#define ESP_ESC_PWM_IN D9
#define ESP_S1 D8

const byte SENSOR_FRAME_LENGTH = sizeof(SensorData); // Length of incoming sensor frame (coming from auxUart)

// Wifi
const char* ssid = "test";
const char* password = "12345678";

WiFiServer server(3333);
HardwareSerial auxUart(1);
 
void setup() {
	Serial.begin(115200); // DEBUG Serial
	delay(100);
	auxUart.begin(115200, SERIAL_8N1, ESP_RX, ESP_TX); // Data connection to Arduino
	delay(100);

	// WIFI connection for sending parsed sensorData (to laptop/pc)
	WiFi.begin(ssid, password);
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	} // Note: not able to connect at the moment. Reason unknown, worked already
	Serial.println("Connected!");
	Serial.print("IP: "); Serial.println(WiFi.localIP());
	server.begin();

	Serial.begin(115200);
	pinMode(ESP_STEERING_PWM_IN, INPUT);
	pinMode(ESP_ESC_PWM_IN, INPUT);
}
 
void loop() {
	if (auxUart.available() >= SENSOR_FRAME_LENGTH) {
		
		SensorData sensorData;

		auxUart.readBytes(reinterpret_cast<byte*>(&sensorData), sizeof(sensorData));
		
		// debug print
		Serial.print("sizeof(SensorData): ");
		Serial.println(sizeof(SensorData));
		Serial.print(F("dir: ")); Serial.print(sensorData.direction);
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

		static WiFiClient client;
		if (!client || !client.connected()) {
			client = server.available();
		}
		if (client && client.connected()) {
			client.printf(
			// need to send sensorData struct here
			"foo"
			);
		}

		// For Testing purpose: using pulseIn() to determine motorpulses and printing to Serial
		// pulseIn should be replaced by 2 hardware timers (more precise signal, no delay)
		ulong servoPulseUS = pulseIn(ESP_STEERING_PWM_IN, HIGH);
		ulong escPulseUS = pulseIn(ESP_ESC_PWM_IN, HIGH);

		Serial.print("Servo pulse in us: ");
		Serial.println(servoPulseUS);
		Serial.print("ESC pulse in us: ");
		Serial.println(escPulseUS);
	}
	vTaskDelay(pdMS_TO_TICKS(50));
}