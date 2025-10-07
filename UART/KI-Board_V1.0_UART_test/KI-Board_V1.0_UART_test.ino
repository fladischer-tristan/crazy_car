#define ESP_TX 6 // laut Schaltung
#define ESP_RX 7 // laut Schaltung

/*
Aktuell funktioniert die UART Verbindung in beide Richtungen nicht.
*/

void setup() {
  Serial.begin(9600); // USB Debug
  Serial1.begin(9600, SERIAL_8N1, ESP_RX, ESP_TX); // An Arduino Seriel3 angeschlossen (?)
  Serial.println("ESP gestartet, sende Hello..."); 
}

void loop() {
  Serial1.println("HELLO from ESP32"); // Sende Daten an Arduino Seriel3
  Serial.println("HELLO gesendet"); // Debug print
  delay(1000);
}