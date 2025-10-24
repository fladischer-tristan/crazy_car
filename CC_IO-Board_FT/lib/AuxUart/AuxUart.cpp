#include "AuxUart.hpp"

void uart3Init() {
    Serial3.begin(BAUD);
}

/**
 * @brief Send SensorData struct over UART3 in binary
 * 
 * @param data SensorData struct - holds all data from sensor measurements
 */
void sendSensorDataToEsp(SensorData* data) {
    if (!Serial3) {
        return;
    }

    // transmitt each primitive in binary format
    sendValue(&data->direction);
    sendValue(&data->velocity);
    sendValue(&data->batteryVoltage);
    sendValue(&data->leftDistance);
    sendValue(&data->middleDistance);
    sendValue(&data->rightDistance);
    sendValue(&data->ax);
    sendValue(&data->ay);
    sendValue(&data->az);
    sendValue(&data->gx);
    sendValue(&data->gy);
    sendValue(&data->gz);
    sendValue(&data->servoPulse);
    sendValue(&data->escPulse);
}