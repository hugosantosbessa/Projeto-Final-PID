#include <Servo_ESP32.h>

int servoPin = 18;
Servo_ESP32 myservo;

void setup() {
  // put your setup code here, to run once:
  myservo.attach(servoPin);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()){
    int angle = Serial.parseInt();
    Serial.println(angle);
    myservo.write(angle);
  }
}
