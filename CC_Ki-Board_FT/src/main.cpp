#include <Arduino.h>
#include <WiFi.h>
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

const byte SENSOR_FRAME_LENGTH = sizeof(SensorData); // Length of incoming sensor frame (coming from auxUart)

// Wifi
const char ssid[] = "test";
const char password[] = "12345678";
WiFiServer server(3333);

void sensorDataDebugPrint(SensorData& sensorData);
 
void setup() {
	Serial.begin(115200); // DEBUG Serial
	delay(100);
	auxUartInit();
	delay(100);

	pinMode(ESP_STEERING_PWM_IN, INPUT);
	pinMode(ESP_ESC_PWM_IN, INPUT);

	Serial.println("Hello from Setup");

	while (auxUart.available()) auxUart.read(); // CLEAR RX BUFFER
  	Serial.println("UART buffer cleared");

	// // WIFI connection for sending parsed sensorData (to laptop/pc)
	// WiFi.begin(ssid, password);
	// while (WiFi.status() != WL_CONNECTED) {
	// 	delay(500);
	// 	Serial.print(".");
	// } // Note: not able to connect at the moment. Reason unknown, worked already
	// Serial.println("Connected!");
	// Serial.print("IP: "); Serial.println(WiFi.localIP());
	// server.begin();
}
/*
* 1. Wait for START_OF_COMM_BYTE
* 2. Read Motor Pulses and send them to ESP
* 3. Wait for incoming SensorData struct
* 4. Send struct away over WiFi (ignore for now, just print to Serial)
*/
void loop() {
	//Serial.println("Hello from Loop");
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
				// Serial.print("auxUart.available():");
				// Serial.println(auxUart.available());
			}

			SensorData sensorData;
			auxUart.readBytes(reinterpret_cast<byte*>(&sensorData), sizeof(SensorData));
			while (auxUart.available()) auxUart.read();

			// 4. Send data to PC / print to terminal for now
			sensorDataDebugPrint(sensorData);

			// static WiFiClient client;
			// if (!client || !client.connected()) {
			// 	client = server.available();
			// }
			// if (client && client.connected()) {
			// 	client.printf(
			// 	// need to send sensorData struct here
			// 	"foo"
			// 	);
			// }
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