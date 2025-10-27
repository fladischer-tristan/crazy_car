#pragma once

#include <Arduino.h>
#include "Sensors.hpp"
#include "Types.hpp"

constexpr unsigned long BAUD = 115200; // UART3 Baudrate
constexpr uint8_t START_OF_COMM_BYTE = 'S';
constexpr uint8_t END_OF_COMM_BYTE = 'E';

void uart3Init();
void sendSensorDataToEsp(SensorData* data);

/**
 * @brief takes pointer to a sensor-value, casts it to
 * a uint8_t (byte) pointer, so that UART can send raw
 * bytes.
 * 
 * @tparam T
 * @param value - pointer to sensor-value (can be of any pointer type)
 */
template<typename T>
void sendValue(T* value) {
    uint8_t* pValue = reinterpret_cast<uint8_t*>(value); // casting to byte pointer
    Serial3.write(pValue, sizeof(T)); // send over UART3
}