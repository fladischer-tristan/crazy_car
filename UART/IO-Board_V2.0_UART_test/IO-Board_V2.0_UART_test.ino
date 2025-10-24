#include <Arduino.h>
 
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial3.begin(115200);
  delay(100);
}
 
uint32_t lastSerialPrint = 0;
uint16_t counter=0;
void loop() {
  if (millis() -lastSerialPrint > 1000 ) {
    Serial3.print("Hello World from arduino Mega:");
    Serial3.println(counter);
    Serial.println("*");
    counter++;
    lastSerialPrint = millis();
  }
 
  while (Serial3.available()) {
    char c = Serial3.read();
    Serial.print(c);
  }    
}