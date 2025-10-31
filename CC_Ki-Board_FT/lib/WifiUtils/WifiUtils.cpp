#include "WifiUtils.hpp"

WiFiClient client; // client object

/**
 * @brief Connecting to Network over WiFi
 * 
 */
void connectToNetwork(const char *ssid, const char *pass) {
    // small wifi test
	WiFi.begin(ssid, pass);
	Serial.print("Connecting to wifi...");
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}
	Serial.println("Connected!");
	Serial.println(WiFi.localIP());
}

/**
 * @brief Connecting to WiFi-Server
 * 
 */
void connectToServer(const char* host, const uint16_t port) {
    Serial.print("Connecting to server ");
	Serial.print(host);
	Serial.print(":");
	Serial.println(port);

	if (client.connect(host, port)) {
		Serial.println("Connection successful");
	} else {
		Serial.println("Connection failed");
	}
}