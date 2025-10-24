#include <Arduino.h>
 
// PINs für KI Board
 
#define ESP_S2 D0
#define ESP_S3 D1
#define ESP_S4 D2
#define ESP_NEO_PIXEL D3
#define ESP_SDA D4
#define ESP_SCL D5
#define ESP_TX D7
#define ESP_RX D6
#define ESP_STEERING_PWM_IN D10
#define ESP_ESC_PWM_IN D9
#define ESP_S1 D8
 
HardwareSerial mySerial(1);  // UART1
 
void setup() {
  Serial.begin(115200);
  delay(100);
  mySerial.begin(115200, SERIAL_8N1, ESP_RX, ESP_TX);
  delay(100);
}
 
uint32_t lastSerialPrint = 0;
uint16_t counter=0;
void loop() {
  if (millis() -lastSerialPrint > 1000 ) {
    mySerial.print("Hello World from esp32-s3:");
    mySerial.println(counter);
    Serial.println("+");
    counter++;
    lastSerialPrint = millis();
  }
 
  while (mySerial.available()) {
    char c = mySerial.read();
    Serial.print(c);
  }    
}