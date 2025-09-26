#include <arduino.h>
#include <Servo.h>

// Physische Taster
#define STARTBUTTON 12 // schwarze Taste // PB6
#define STOPBUTTON 13  // rote Taste // PB7

// Motoren
#define PIN_SPEED_SERVO 2 // Geschwindigkeits-Servo (Speed) // PE4 und auf PH1 (Serial2)
#define PIN_STEERING_SERVO 5 // µC Port PE3 

// Infrarot Sensoren
#define LEFTSENSOR_PIN A1
#define RIGHTSENSOR_PIN A2
#define MIDDLESENSOR_PIN A3

// Batteriespannungssensor
#define VBAT_PIN A0 


Servo speedServo;
Servo steeringServo; 
bool runMode = 0; // 0 -> Motoren aus


void setup() {
  Serial.begin(115200);
  speedServo.attach(PIN_SPEED_SERVO);
  steeringServo.attach(PIN_STEERING_SERVO); 
}


void loop() {
  // run-flag setzen
  if (digitalRead(STOPBUTTON) == LOW) {
    runMode = 0;
  }
	else if (digitalRead(STARTBUTTON) == LOW) {
    runMode = 1;
  }

  if (runMode) {
    // Kleines Testprogramm
    Serial.println("car in runmode");
    speedServo.writeMicroseconds(1500); // neutral
    delay(1000);
    speedServo.writeMicroseconds(1600); // leicht vorwärts
    delay(1000);
    speedServo.writeMicroseconds(1500); // neutral
  }
  else {
    Sterial.println("car stopped");
  }
}



/* Alter Setup Codeabschnitt von FH Website:
void setupESCPWM() {
  Serial.println("KALIBRIERUNG");
  const int CONFIG_DELAY = 5000; //Zeit wie lange das Konfig Signal anliegt
  speedServo.writeMicroseconds(1000); // Voll rückwärts
  Serial.println("VORWÄRTS");
  delay(CONFIG_DELAY);
  speedServo.writeMicroseconds(1500); // Neutral
  Serial.println("NEUTRAL");
  delay(CONFIG_DELAY);
  speedServo.writeMicroseconds(2000); // Voll vorwärts
  Serial.println("RÜCKWÄRTS");
  delay(CONFIG_DELAY);
  speedServo.writeMicroseconds(1500); // Neutral
  Serial.println("NEUTRAL");
  delay(CONFIG_DELAY);
  // FÜR DIESEN MOTOR (DC BRUSHLESS) OBSOLET
} */