/* Autor: Ana Júlia da Silva
    Ano: 2023
    Este código é parte integrante do TCC e tem a seguinte funcionalidade:
     Emular o funcionamento de um controle PID do separador bifásico de óleo e gás;
*/
// Include library
#include <Adafruit_MCP4725.h>
Adafruit_MCP4725 dac;

//variable
float Kp = -10;
float Ti = 10;
float Td = 0;
float setpointLV = 2; //m
float setpointP = 8; //bar
float LV = A3;
float P = A4;
float dt = 0.01; //tempo de amostragem
float outputLV;
float outputP;
float errorP = 0;
float errorLV = 0;
unsigned long time;
float T = 1;

float error_oldLV = 0.0;
float error_oldLV2 = 0.0;
float integralTermLV = 0.0;
float Output_oldLV = 0.0;
float error_oldP = 0.0;
float error_oldP2 = 0.0;
float integralTermP = 0.0;
float Output_oldP = 0.0;
float uLV;
float uP = 0;
//float uLV_old = 0;
float uLV_old = 0.5;
float uP_old = 0;

  //incialização do temporizador
unsigned long previousMillis = 0;
const long interval = 1000; // Intervalo de 1 segundo

void setup(){
  Serial.begin(9600);
  dac.begin(0x60);
}

void PID() {
  // Calcule o erro
  float inputLV = analogRead(LV) *5.2/ (1023); //o sinal é recebido em bits aqui estamos convertando de bits para volts e de volts para metros
  //float inputP = analogRead(P) * 5.00 *8/ (1023.00*2.5); //bar

  errorLV = - inputLV + setpointLV;
  //errorP = inputP - setpointP;

//PID LEVEL
  //equação do sinal de controle em tempo discreto u(k)
  float r0 = Kp + Kp*Td/T;
  float r1 = -Kp + Kp*T/Ti - 2*Kp*Td/T;
  float r2 = Kp*Td/T;

  //uLV = uLV_old + r0*errorLV + r1*error_oldLV + r2*error_oldLV2 + 0.5;
  uLV = uLV_old + r0*errorLV + r1*error_oldLV + r2*error_oldLV2;

  if (uLV>1){ //saturação do sistema
      uLV=1;
    }

  if (uLV<0){ //saturação do sistema
      uLV=0;
  }

  // Atualize as variáveis para a próxima iteração
  error_oldLV2 = error_oldLV;
  error_oldLV = errorLV;
  uLV_old = uLV;
  
//PID PRESSURE
  //equação do sinal de controle em tempo discreto u(k)
  //float r0 = Kp + Kp*Td/dt;
  //float r1 = -Kp + Kp*dt/Ti - 2*Kp*Td/dt;
  //float r2 = Kp*Td/dt;

//   uP = 0.5 + r0*errorP + r1*error_oldP + r2*error_oldP2;

//   // if (uP>1){ //saturação do sistema
//   //     uP=1;
//   //   }

//   // if (uP<0){ //saturação do sistema
//   //     uP=0;
//   // }
//   // Atualize as variáveis para a próxima iteração
//   error_oldP2 = error_oldP;
//   error_oldP = errorP;
//   uP_old = uP;
}

void loop(){
dac.setVoltage(uLV*4095/5,false);

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    PID();
    previousMillis = currentMillis;  // Salva o tempo atual
  }

    //válvulas de saída

Serial.print(uLV);
Serial.print(" ");
//Serial.print(" signal control P:");
//Serial.print(uP);
//Serial.print(" errorP:");
//Serial.print(errorP);
//Serial.print(" errorLV:");
//Serial.print(errorLV);
//Serial.print("\n");

float time;
time = millis()/1000;
Serial.println(time);
delay(1000);
}