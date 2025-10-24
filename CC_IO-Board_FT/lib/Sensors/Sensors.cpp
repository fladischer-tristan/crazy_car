#include "Sensors.hpp"


void sensorInit() {
    Wire.begin();
    pinMode(HALL_SENSOR_1_PIN, INPUT_PULLUP);
    pinMode(HALL_SENSOR_2_PIN, INPUT_PULLUP);
    analogReference(INTERNAL2V56);
}

// Converts ATMEGA2560 ADC Value (0 - 2^10-1) to Voltage
float adcToVoltage(int adc) {
	return ((float)adc * V_REF) / ADC_MAX;
}

/**
 * @brief calculates velocity of vehicle
 * 
 * @param pulseCount amount of hallsensor pulses detected
 * @return float - velocity in mm/s
 */
float calculateVelocity(const int16_t pulseCount, const unsigned long sampleRate) { 
    return (float)(pulseCount * PULSE_DISTANCE) / (float)(sampleRate / 1000.0f); // mm/s
}

/**
 * @brief reads and normalizes acceleration and gyroscope values of MPU9250 (I2C)
 * 
 * @param ax pass reference - acceleration in x dir
 * @param ay pass reference - acceleration in y dir
 * @param az pass reference - acceleration in z dir
 * @param gx pass reference - angular speed in x dir
 * @param gy pass reference - angular speed in y dir
 * @param gz pass reference - angular speed in z dir
 */
void readGyroSensor(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
    const byte accelReg = 0x3B; // first acceleration register
    const byte gyroReg = 0x43; // first gyroscope register

    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(accelReg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU9250_ADDR, 6, true); // requesting 6 bytes from MPU9250 (2 bytes each for 3 values)

    // acceleration values (g)
    ax = (float)((Wire.read() << 8) | Wire.read()) / ACCEL_SENSITIVITY; // read 1st byte, bit shift 8 to left, then add 2nd byte
    ay = (float)((Wire.read() << 8) | Wire.read()) / ACCEL_SENSITIVITY;
    az = (float)((Wire.read() << 8) | Wire.read()) / ACCEL_SENSITIVITY;

    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(gyroReg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU9250_ADDR, 6, true);

    // gyroscope values (°/s)
    gx = (float)((Wire.read() << 8) | Wire.read()) / GYRO_SENSITIVITY;
    gy = (float)((Wire.read() << 8) | Wire.read()) / GYRO_SENSITIVITY;
    gz = (float)((Wire.read() << 8) | Wire.read()) / GYRO_SENSITIVITY;
}

/**
 * @brief reads all IR sensors and stores in references
 * 
 * @param leftDistance - reference var
 * @param middleDistance - reference var
 * @param rightDistance - reference var
 */
void readDistanceSensors(float& leftDistance, float& middleDistance, float& rightDistance) {
    leftDistance = adcToVoltage(analogRead(LEFTSENSOR_PIN));
    middleDistance = adcToVoltage(analogRead(MIDDLESENSOR_PIN));
    rightDistance = adcToVoltage(analogRead(RIGHTSENSOR_PIN));
}

// reads and returns battery voltage
float readBatteryVoltage() {
    return adcToVoltage(analogRead(VBAT_PIN));
}