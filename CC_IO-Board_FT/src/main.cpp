/**
 * @file main.cpp
 * @author Tristan Fladischer
 * @brief CrazyCar Training-data collection program
 * @version 0.1
 * @date 2025-10-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <Arduino.h>
#include <Servo.h>
#include "AuxUart.hpp"
#include "Sensors.hpp"
#include "Types.hpp"
#include "Debug.hpp"

constexpr uint16_t SAMPLE_RATE = 100; // data sample time in ms

const uint8_t SENSOR_FRAME_LENGTH = sizeof(SensorData); // length of each UART packet

// physical buttons - negative logic
constexpr uint8_t STARTBUTTON = 12;
constexpr uint8_t STOPBUTTON = 13;

// motor pins
constexpr uint8_t SERVO_PIN = 5; 
constexpr uint8_t ESC_PIN = 2;

/*
* MAX/MIN allowed values for our Motors in microseconds
* these values have been measured using an RC-Receiver
*/
const uint32_t ESC_PULSE_MIN = 1011;
const uint32_t ESC_PULSE_MAX = 1923;
const uint32_t SERVO_PULSE_MIN = 1011;
const uint32_t SERVO_PULSE_MAX = 1963;

unsigned long packetCounter = 0; // used to add identifier to each SensorData struct
unsigned long lastTime = 0;
volatile long hallPulseCount = 0;

// runMode enum used for loop logic
enum RunMode {START, STOP};
RunMode runMode = STOP;

// Motor objects
Servo steeringServo;
Servo esc;

// Protyping ISR
void onHallPulse();

void setup() {
	Serial.begin(115200); // DEBUG Serial
	uart3Init(); // start Serial3
	sensorInit();


	steeringServo.attach(SERVO_PIN);
	esc.attach(ESC_PIN);

	attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_1_PIN), onHallPulse, RISING);

	while (Serial3.available()) Serial3.read(); // CLEAR RX BUFFER
  	Serial.println("UART buffer cleared");
}

void loop() {
	unsigned long now = millis();
	static unsigned long lastStartPressed = 0;

	/*
	* Polling the two pyhsical buttons and setting our runMode variable
	* when runMode is active, 
	*/
	if (digitalRead(STARTBUTTON) == LOW) {
		// debouncing - caused problems on esp side since SoC byte was sent twice
		if ((millis() - lastStartPressed) >= 500) {
			runMode = START;
			Serial3.write(START_OF_COMM_BYTE); // StartOfCommunication ('S')
		}
		lastStartPressed = millis();
	}
	else {
		if (digitalRead(STOPBUTTON) == LOW) {
			runMode = STOP;
			Serial3.write(END_OF_COMM_BYTE); // EndOfCommunication ('E')
			packetCounter = 0;
		
			while (Serial3.available()) Serial3.read(); // clearing UART buffer
		}
	}

	/*
	* RUNMODE
	* 1. Initiate UART comm with ESP32
	* 2. Wait for incoming MotorPulses (SensorData struct)
	* 3. Feed MotorPulses to our Motors
	* 4. Send back a SensorData struct with all Sensor data + Motorpulses + Identifier
	*/
	if (runMode == START) {
		if (now - lastTime >= SAMPLE_RATE) {
			unsigned long startTime = millis();

			// 2. Wait for incoming MotorPulses (SensorData struct)
			while (Serial3.available() < SENSOR_FRAME_LENGTH) 
			{
				
			}
			
			// 3. Feed MotorPulses to our Motors
			SensorData motorData;
			Serial3.readBytes(reinterpret_cast<byte*>(&motorData), sizeof(SensorData));
			while (Serial3.available()) Serial3.read();

			Serial.print("sizeof(SensorData): ");
			Serial.println((unsigned long)sizeof(motorData));

			uint32_t servoPulseUS = motorData.servoPulse;
			uint32_t escPulseUS = motorData.escPulse;

			if (servoPulseUS >= SERVO_PULSE_MIN && servoPulseUS <= SERVO_PULSE_MAX) {
				steeringServo.writeMicroseconds(servoPulseUS);
			}

			if (escPulseUS >= ESC_PULSE_MIN && escPulseUS <= ESC_PULSE_MAX) {
				esc.writeMicroseconds(escPulseUS);
			}

			// 4. Send back a SensorData struct with all Sensor data + Motorpulses + Identifier

			// getting our hall pulseCount
			noInterrupts();
			long count = hallPulseCount;
			hallPulseCount = 0;
			interrupts();

			// assembling sensorData struct
			SensorData sensorData;

			sensorData.packetNumber = packetCounter; // add unique identifier
			packetCounter++; // make sure to increment the counter for next packet
			readAllSensorData(sensorData); // add the sensor values
			sensorData.velocity = calculateVelocity(count, SAMPLE_RATE); // calculate and add speed value
			sensorData.servoPulse = servoPulseUS; // servo pulse from esp32
			sensorData.escPulse = escPulseUS; // esc pulse from esp32

			sendSensorDataToEsp(&sensorData); // send the whole thing to ESP32
			delayMicroseconds(500);

			// measure time (for DEBUG purpose)
			unsigned long elapsedTime = millis() - startTime;
			Serial.print("Sensor fetch took ");
			Serial.print(elapsedTime);
			Serial.println("ms.");

			sensorDataDebugPrint(sensorData); // printing our struct for DEBUG purpose
			lastTime = millis();
		}
	}
}

/**
 * @brief ISR for hall pulse - increments/decrements pulse counter
 */
void onHallPulse() {
	if (digitalRead(HALL_SENSOR_2_PIN) == HIGH) {
		hallPulseCount++;
	}
	else {
		hallPulseCount--;
	}
}