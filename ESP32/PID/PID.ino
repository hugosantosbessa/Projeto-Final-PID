#include <Servo_ESP32.h>

Servo_ESP32 servo;

#define pinServo 19
#define pinTrig 5
#define pinEcho 18

////////////////////////Variables///////////////////////
int Read = 0;
float distance = 0.0;
// float elapsedTime, time, timePrev;        //Variables for time control
float distance_previous_error, distance_error;
int period = 50;  //Refresh rate period of the loop is 50ms
///////////////////////////////////////////////////////


///////////////////PID constants///////////////////////
float kp=8; //Mine was 8
float ki=0.2; //Mine was 0.2
float kd=3100; //Mine was 3100
float distance_setpoint = 21;           //Should be the distance from sensor to the middle of the bar in mm
float PID_p, PID_i, PID_d, PID_total;
///////////////////////////////////////////////////////


double priError = 0;
double toError = 0;

void setup() {
  Serial.begin(115200);
  pinMode(pinTrig, OUTPUT);
  pinMode(pinEcho, INPUT);
  servo.attach(pinServo);
  servo.write(0);
}
void loop() {
  PID();
}

float distance_HCSR04() {
  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);
  int duration = pulseIn(pinEcho, HIGH);
  float distance_cm = duration/58;
  return distance_cm;
}

void PID(){
  distance = distance_HCSR04();   
  Serial.println(distance);
  distance_error = distance_setpoint - distance;   
  PID_p = kp * distance_error;
  float dist_diference = distance_error - distance_previous_error;     
  PID_d = kd*((distance_error - distance_previous_error)/period);
    
  if(-3 < distance_error && distance_error < 3)
  {
    PID_i = PID_i + (ki * distance_error);
  }
  else
  {
    PID_i = 0;
  }

  PID_total = PID_p + PID_i + PID_d;  
  PID_total = map(PID_total, -150, 150, 0, 150);

  if(PID_total < 20){PID_total = 20;}
  if(PID_total > 160) {PID_total = 160; } 

  servo.write(PID_total+30);  
  distance_previous_error = distance_error;
}
