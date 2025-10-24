#include <Arduino.h>
#include "AuxUart.hpp"
#include "Sensors.hpp"
#include "Types.hpp"

constexpr uint16_t SAMPLE_RATE = 500; // sensor sample rate in ms

// physical buttons - negative logic
constexpr uint8_t STARTBUTTON = 12;
constexpr uint8_t STOPBUTTON = 13;

unsigned long lastTime = 0;
volatile long hallPulseCount = 0;
volatile uint8_t dir = 0; // direction of vehicle

// Protyping ISRs
void onHallPulse();
void onStartButton();
void onStopButton();

enum RunMode {
	START,
	STOP
};

RunMode runMode = STOP;

void setup() {
	Serial.begin(115200); // DEBUG Serial
	uart3Init();
	sensorInit();

	// need to have hall ISR here (?) since we would need a global variable in Sensors.hpp otherwise
	attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_1_PIN), onHallPulse, RISING);
	// attachInterrupt(digitalPinToInterrupt(STARTBUTTON), onStartButton, FALLING);
	// attachInterrupt(digitalPinToInterrupt(STOPBUTTON), onStopButton, FALLING);
}

void loop() {
	unsigned long now = millis();
	//Serial.print(runMode);

	if (digitalRead(STARTBUTTON) == LOW) {
		runMode = START;
	}
	else {
		if (digitalRead(STOPBUTTON) == LOW) {
			runMode = STOP;
		}
	}

	if (runMode == START) {
		if (now - lastTime >= SAMPLE_RATE) {
			noInterrupts();
			long count = hallPulseCount;
			hallPulseCount = 0;
			interrupts();

			SensorData sensorData;

			// assembling sensorData
			sensorData.direction = dir;
			sensorData.velocity = calculateVelocity(count, SAMPLE_RATE);

			readDistanceSensors(
				sensorData.leftDistance,
				sensorData.middleDistance,
				sensorData.rightDistance
			);
			readGyroSensor(
				sensorData.ax,
				sensorData.ay,
				sensorData.az,
				sensorData.gx,
				sensorData.gy,
				sensorData.gz
			);
			// NOTE: Servo and ESC pulses require some additional hardware setup, not availiable yet.
			sensorData.servoPulse = 0;
			sensorData.escPulse = 0;

			sendSensorDataToEsp(&sensorData); // send over UART3

			// debug print
			Serial.print("sizeof(SensorData): ");
			Serial.println((unsigned long)sizeof(SensorData));
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
		dir = 1;
	}
	else {
		hallPulseCount--;
		dir = 0;
	}
}

// ISR for start button
void onStartButton() {
	Serial.print("Start Button ISR");
	// debouncing button
	static unsigned long lastPress = 0;
	if (millis() - lastPress > 200) {
		runMode = START;
		lastPress = millis();
	}
}

// ISR for stop button
void onStopButton() {
	Serial.print("Stop Button ISR");
	// debouncing button
	static unsigned long lastPress = 0;
	if (millis() - lastPress > 200) {
		runMode = STOP;
		lastPress = millis();
	}
}