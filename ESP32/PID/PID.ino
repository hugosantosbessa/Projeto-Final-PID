/* 
  Projeto Bola e Viga
 */
// Biblioteca servo 
#include <Servo_ESP32.h>

// Pinos utilizados
#define pinServo 33
#define pinTrig 5
#define pinEcho 18

// Constantes PID;
double Kp = 1, Ki = 0.001, Kd = 0.03;

// Variavel de posicao atual e posicao desejada
int pos = 0;
int setPoint = 15;

// Variaveis PID
unsigned long currentTime, previousTime;
double elapsedTime;
double error, lastError, sumError, rateError;
double outPut;

// Variaveis de distancia
float dis, dis_corr;

// servo motor
Servo_ESP32 myservo;

void setup() {
  Serial.begin(115200);
  pinMode(pinTrig, OUTPUT);
  pinMode(pinEcho, INPUT);
  myservo.attach(pinServo);
  myservo.write(75);
}

void loop() {
// Obtem a distancia do sensor
  dis = distance_HCSR04(); 

  /*
    Afim de corrigir valores "errados" retornado pelo modulo HCSR04,
    eh atribuido o valor medio 15, quando a distancia lida pelo modulo 
    eh maior que 40. Se valor esta entre em [0, 40], entao a distancia lida
    esta correta.
   */
  if(dis > 40) 
    dis_corr = 15; 
  else 
    dis_corr = dis;

// A posicao do servo motor eh atribuida em graus
// 75 -> angulo em que a plataforma fica reta (sem inclinacao)
  pos = PID(dis_corr) + 75;

// Limita o valor da posicao do servo para que nao exceda os limites
  limite();

// Envia a posicao ao servo motor
  myservo.write(pos);
 
// Os valores a serem plotados são impressos na plotter serial
  Serial.print(setPoint);
  Serial.print(" ");
  Serial.println(dis);
}

float distance_HCSR04() {
  digitalWrite(pinTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(4);
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
  error = input - setPoint;

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
