#pragma once

#include <WiFi.h>
#include "esp_wifi.h"

/*
 * If we are connecting to a router, channel 11 should work.
 * For local hotspots, any channel should work.
 */

extern WiFiClient client; // client object

void connectToNetwork(const char *ssid, const char *pass);
void connectToServer(const char* host, const uint16_t port);