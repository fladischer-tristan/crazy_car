/**
 * @file main.cpp
 * @author Tristan Fladischer
 * @brief ESP32S3 src code for CrazyCar.
 * @version 0.1
 * @date 2025-10-31
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "Types.hpp"
#include "AuxUart.hpp"
#include "secrets.hpp"
#include "WifiUtils.hpp"
#include "Debug.hpp"

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

// TCP Server details
const char* HOST = "192.168.8.106"; 
const uint16_t PORT = 5000;

// length of UART packet
const byte SENSOR_FRAME_LENGTH = sizeof(SensorData);

// Queue and TCP task
QueueHandle_t packetQueue;

// enums for our control logic (2 state machines)
enum CommState {WAIT_FOR_START, RUNNING};
enum CycleState {SEND_MOTOR_PULSES, READ_SENSOR_DATA, APPEND_DATA_TO_QUEUE};

CommState commState = WAIT_FOR_START;
CycleState cycleState = SEND_MOTOR_PULSES;

void tcpSenderTask(void *pvParameters);

void setup() {
	vTaskDelay(pdMS_TO_TICKS(3000)); // ESP32S3 Serial needs the delay to actually work in the setup

	Serial.begin(115200); // DEBUG Serial
	vTaskDelay(pdMS_TO_TICKS(100));

	auxUartInit(); // Serial to Arduino Mega
	vTaskDelay(pdMS_TO_TICKS(100));

	// RC-Receiver pins
	pinMode(ESP_STEERING_PWM_IN, INPUT);
	pinMode(ESP_ESC_PWM_IN, INPUT);

	/*
	* creating a queue so we can store our SensorData
	* packets, so a seperate task can send them away
	* over TCP - the task is handled by its own core,
	* core 1
	*/
	packetQueue = xQueueCreate(1000, sizeof(SensorData));
	xTaskCreatePinnedToCore(tcpSenderTask, "TCP Sender", 4096, NULL, 1, NULL, 1);

	//Connect to WiFi and TCPServer
	connectToNetwork(SSID, PASS);
	connectToServer(HOST, PORT);
}

/*
* Sending Motor-Pulses to Arduino Mega over UART (auxUart),
* then waiting for Arduino to send back the whole SensorData struct,
* and finally send the received SensorData struct away to TCP Server
* -  This cycle should repeat as fast as possible, in fixed timestamps
*
* 1. Wait for START_OF_COMM_BYTE
* 2. Read Motor Pulses and send them to Arduino MEGA
* 3. Wait for incoming SensorData struct and read it
* 4. Send struct away over WiFi (this happens outside loop via FreeRtos task)
*/
void loop() {
	static SensorData motorData, sensorData;

	switch (commState) {
		case WAIT_FOR_START:
			if (auxUart.available() && auxUart.read() == START_OF_COMM_BYTE) {
				Serial.println("SoC...");

				// clearing auxUart buf to ensure stable synchronization
				while (auxUart.available()) auxUart.read();

				// Start of Communication deteced, switch to RUNNING
				cycleState = SEND_MOTOR_PULSES;
				commState = RUNNING;
			}
			break;

		case RUNNING:
			// First we need to check if the communication ended:
			if (auxUart.available() && auxUart.peek() == END_OF_COMM_BYTE) {
				auxUart.read();
				Serial.println("EoC...");
				while (auxUart.available()) auxUart.read(); // reset uart buffer
				commState = WAIT_FOR_START;
				break;
			}

			switch (cycleState) {
				case SEND_MOTOR_PULSES:
					Serial.println("Reading Motorpulses ...");
					// reading RC-Receiver pins and sending the Signals to Arduino MEGA
					motorData.escPulse = pulseIn(ESP_ESC_PWM_IN, HIGH, 25000);
					motorData.servoPulse = pulseIn(ESP_STEERING_PWM_IN, HIGH, 25000);
					Serial.print("ESC: ");
					Serial.println(motorData.escPulse);
					Serial.print("SERVO: ");
					Serial.println(motorData.escPulse);
					sendMotorDataToMega(&motorData);
					Serial.println("Sent Motorpulses ...");

					// switch to next state
					cycleState = READ_SENSOR_DATA;
					break;

				case READ_SENSOR_DATA:
					//Serial.println("Fetching data from UART...");
					if (readSensorFrame(sensorData)) {
						sensorDataDebugPrint(sensorData);
						cycleState = APPEND_DATA_TO_QUEUE;
						Serial.println("Fetch successful ...");
					}
					break;

				case APPEND_DATA_TO_QUEUE:
					Serial.println("Appending data to queue ...");
					xQueueSend(packetQueue, &sensorData, 0);

					// switch to original state -> repeat cycle
					cycleState = SEND_MOTOR_PULSES;
					break;
			}
			break;
	}
}

/**
 * @brief Sends SensorData to TCP Server, operates on reserved CPU core
 * 
 * @param pvParameters NULL
 */
void tcpSenderTask(void *pvParameters) {
	for (;;) {
		SensorData packet;
		if (xQueueReceive(packetQueue, &packet, portMAX_DELAY)) {
			if (client.connected()) {
				// Sending a SensorData packet to TCP server
				client.write((uint8_t*)&packet, sizeof(packet));
			}
			else {
				// If we are not connected to server, try reconnect
				// since this task does not block the main loop, we
				// can safely wait for the reconnect. the queue will
				// buffer any incoming SensorData from loop.
				client.connect(HOST, PORT);
			}
		}
	}
}

// // 4. Send data to TCPServer
			// sensorDataDebugPrint(sensorData);
			// if (client.connected()) {
			// 	client.print("ID: ");
			// 	client.print(sensorData.packetNumber);
			// 	client.print(", servo: ");
			// 	client.print(pulseIn(ESP_STEERING_PWM_IN, HIGH, 25000));
			// 	client.print(", esc: ");
			// 	client.print(pulseIn(ESP_ESC_PWM_IN, HIGH, 25000));
			// 	client.print(", speed: ");
			// 	client.print(sensorData.velocity, 2);
			// 	client.print(", vbat: ");
			// 	client.print(sensorData.batteryVoltage, 2);
			// 	client.print(", ld: ");
			// 	client.print(sensorData.leftDistance, 2);
			// 	client.print(", md: ");
			// 	client.print(sensorData.middleDistance, 2);
			// 	client.print(", rd: ");
			// 	client.print(sensorData.rightDistance, 2);
			// 	client.print(", ax: ");
			// 	client.print(sensorData.ax, 2);
			// 	client.print(", ay: ");
			// 	client.print(sensorData.ay, 2);
			// 	client.print(", az: ");
			// 	client.print(sensorData.az, 2);
			// 	client.print(", gx: ");
			// 	client.print(sensorData.gx, 2);
			// 	client.print(", gy: ");
			// 	client.print(sensorData.gy, 2);
			// 	client.print(", gz: ");
			// 	client.println(sensorData.gz, 2);

			// } else {
			// 	Serial.println("Client getrennt, versuche Reconnect...");
			// 	client.connect(HOST, PORT);
			// }