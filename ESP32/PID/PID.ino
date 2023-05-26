/* 
  Projeto Bola e Viga
 */

// Biblioteca servo 
#include <Servo_ESP32.h>

// Pinos utilizados
#define pinServo 19
#define pinTrig 5
#define pinEcho 18

// Constantes PID;
double Kp = 0, Ki = 0, Kd = 0;

// Variavel de posicao atual e posicao desejada
int pos = 0;
int setPoint = 12;

// Variaveis PID
unsigned long currentTime, previousTime;
double elapsedTime;
double error, lastError, sumError, rateError;
double outPut;

// distancia
float dis;

// servo motor
Servo_ESP32 myservo;

void setup() {
  Serial.begin(115200);
  pinMode(pinTrig, OUTPUT);
  pinMode(pinEcho, INPUT);
  myservo.attach(pinServo);
  myservo.write(0);
}

void loop() {
// Obtem a distancia do sensor
  dis = distance_HCSR04();
// A posicao do servo motor eh atribuida em graus
  pos = PID(dis) + 70;
// Limita o calor da posicao do servo para que nao exceda 
// os limites
  limite();
// Envia a posicao ao servo motor
  myservo.write(pos);

// Se imprimen los valores para graficar en el serial plotter
  Serial.print(setPoint);
  Serial.print(" ");
  Serial.println(dis);
  delay(10);
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

void limite(void)
{ 
  // Verifica posicao maxima e minima do angulo
  if(pos > 140)
    pos = 140;
  else if(pos < 0) 
    pos = 0;
  
}

double PID(float input)
{ 
  // Tempo atual
  currentTime = millis();
  // Tempo decorrido
  elapsedTime = currentTime - previousTime;
  // Erro de posicao
  error = setPoint - input;
  // Calcula integral do erro
  sumError += error * elapsedTime;
  // Calcula a derivada do erro
  rateError = (error - lastError) / elapsedTime;
  // Calcula a saida do controlador
  outPut = Kp * error + Ki * sumError + Kd * rateError;

  // Atualiza o ultimo erro com o erro atual
  lastError = error;
  // Atualiza o tempo anterior com o tempo atual
  previousTime = currentTime;

  // Saida do controlador eh retornada
  return outPut;
}
