#include "AuxUart.hpp"

HardwareSerial auxUart(1);

void auxUartInit() {
    auxUart.begin(BAUD, SERIAL_8N1, ESP_RX, ESP_TX); // Data connection to Arduino
}

/**
 * @brief Send SensorData struct over AuxUart in binary (only the motor pulses matter)
 * 
 * @param data SensorData struct - holds all data from sensor measurements
 */
void sendMotorDataToMega(SensorData* data) {
    // transmitt each primitive in binary format
    sendValue(&data->packetNumber);
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