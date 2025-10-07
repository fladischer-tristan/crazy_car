void setup() {
  Serial.begin(9600);    // UART0 zum debuggen
  Serial3.begin(9600);   // UART3, RX=15, TX=14 laut Schaltplan
  Serial.println("Arduino gestartet, warte auf Daten...");
}

// Versuche Daten vom ESP32S3 zu erhalten, dann über UART ausgeben
void loop() {
  if (Serial3.available()) {
    int b = Serial3.read();
    Serial.write(b);     // USB Monitor
  }
}