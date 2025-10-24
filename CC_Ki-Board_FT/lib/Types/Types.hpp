#pragma once

#pragma pack(push, 1)  // disable struct padding

// struct for storing all sensor-data in compact packet (good for Serial)
struct SensorData {
    uint8_t direction = 0;
    float velocity = 0.0F;
    float batteryVoltage = 0.0F;
    float leftDistance = 0.0F, middleDistance = 0.0F, rightDistance = 0.0F;
    float ax = 0.0F, ay = 0.0F, az = 0.0F;
    float gx = 0.0F, gy = 0.0F, gz = 0.0F;
    uint32_t servoPulse = 0, escPulse = 0;
};

#pragma pack(pop)