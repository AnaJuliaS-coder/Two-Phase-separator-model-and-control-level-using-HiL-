/* Autor: Ana Julia da Silva
    Ano: 2023
    Este código é parte integrante do TCC e tem a seguinte funcionalidade:
     Emular o funcionamento de um separador bifásico de óleo e gás, reproduzindo
     suas variáveis de nível de óleo e pressão interna conforme o modelo matemático;
     Servir como base para a implementação de sistemas de controle digital via HiL.
*/
// Inclusão de bibliotecas
#include <math.h>
#include <TimerThree.h>
//#include <Wire.h>
#include <Adafruit_MCP4725.h>
Adafruit_MCP4725 dac1;
Adafruit_MCP4725 dac2;

// Definição dos termos constantes
volatile double pi = 3.1415926;
//volatile double t0 = 0;  // Tempo inicial
//volatile double tf = 100; // Tempo final
volatile double dt = 0.01; // Período de amostragem
//volatile double ni = 1000000; // Numero total de iterações (1 milhão)
//volatile double Vmax = 5.00; // Limite de tensão superior do Arduíno
unsigned long time;
//Declaração dos pinos analógicos
//volatile double analogPinNivel = A5; // nível (m)
//volatile double analogPinPressao = A1; // pressão (bar)
volatile double analogPinLiqValve = A2; // taxa de abertura da válvula de líquidos (de 0 a 1, onde 0 é totalmente fechada e 1 é totalmente aberta)
//volatile double analogPinGasValve = A4; // taxa de abertura da válvula de gases (de 0 a 1, onde 0 é totalmente fechada e 1 é totalmente aberta)

//Declaração das constantes do separador bifásico
volatile double V = 56.60; // volume total do tanque (m³)
volatile double MMg = 0.021; // Massa Molar do gás
volatile double T = 303.15; // Temperatura no interior do tanque
volatile double MMar = 0.029; // Massa Molar do gás
volatile double Tar = 293.15; // Temperatura no interior do tanque
volatile double C = 8.00; // Comprimento do tanque (m)
volatile double D = 3.00; // Diâmetro do tanque (m)
volatile double ro_h2o = 999.20; // Densidade específica da água
volatile double ro_L = 850; // densidade específica do hidrocarboneto
volatile double xL = 0.5; // abertura da válvula de líquidos em regime permanente
volatile double xG;// = 0.5; // abertura da válvula de gases em regime permanente
volatile double Cvl = 896.74910; // Coeficiente de vazão da válvula de óleo
volatile double Cvg = 370.49128; // Coeficiente de vazão da válvula de gás
volatile double P1 = 6.00; // Pressão à jusante da válvula de óleo
volatile double P2 = 6.00; // Pressão à jusante da válvula de gás

// Condições iniciais da planta
//volatile double dh_k = 0.00;
//volatile double dh_km1 = 0.00;
volatile double h_k = 0.00;
//volatile double h_km1 = 2.00;

volatile double h = 2.00;
//volatile double dP_k = 0.00;
//volatile double dP_km1 = 0.00;
volatile double P_k = 0.00;
//volatile double P_km1 = 8.00;

volatile double P = 8.00;

volatile double ro_g_k = 0.00;
volatile double Vl_k = 0.00;
volatile double Lout_k = 0.00;
volatile double Gout_k = 0.00;
//volatile double Lout_km1 = (2.4e-4) * xL * Cvl * sqrt((P_km1 - P1) / (ro_L / ro_h2o));
//volatile double Gout_km1 = (2.4e-4) * xG * Cvg * sqrt(((P_km1 - P2) * (P_km1 + P2)*MMar*T) / (P_km1*MMg*Tar));


// volatile double Lout_km1 = (2.4e-4) * xL * Cvl * sqrt((P - P1) / (ro_L / ro_h2o));
// volatile double Gout_km1 = (2.4e-4) * xG * Cvg * sqrt(((P - P2) * (P + P2)*MMar*T) / (P *MMg*Tar));
double Lout_km1;
double Gout_km1;


// tempo para as golfadas
volatile double t = 0.00;
volatile double Lin_ss = 0.165; //ss - steady state (valor de regime)
volatile double Gin_ss = 0.1;
volatile double Lin = Lin_ss;
volatile double Gin = Gin_ss;
volatile double dp = 0;
volatile double dh = 1;
// int contador = 0;

void setup() {

  Serial.begin(9600); // COMENTAR DURANTE OS TESTES
  //Definições da interrupção por tempo:
  Timer3.initialize(1000000);  // Inicializa Timer3 com período de amostragem (em us) (10 ms)
  Timer3.attachInterrupt(planta);  // Define a interrupção de tempo
  
  //dac1.begin(0x60);
  dac2.begin(0x61);
  //Configurando o status dos pinos
  // pinMode (PWM, OUTPUT);
  // pinMode (PWM2, OUTPUT);
  // pinMode(analogPinNivel, OUTPUT);
  // pinMode(analogPinPressao, OUTPUT);
  // pinMode(analogPinLiqValve, INPUT);
  // pinMode(analogPinGasValve, INPUT);
}

void planta() {
  
  //Implementação da planta via Runge Kutta de 4ªOrdem

//xL = (analogRead(analogPinLiqValve) * 1.00 / 1023)*5;
//xG = analogRead(analogPinGasValve) * 1.00 / 1023;
  xL = 0.5;
  xG = 0.5;
 
  //golfadas
  t = t + dt;
  Lin  =  0.165* sin(0.333*time); //0.300;//
  Gin  =  0.165* sin(0.333*time); //0.300;//
  if (Lin<0){ //saturação das golfadas sem numeros negativos
      Lin=0;
    }
  if (Gin<0){ //saturação das golfadas sem numeros negativos
      Gin=0;
    }

  //Lin  = 0.165;
  //Lin  = 0.165;
  //Gin  = 0.100;

  //implementação da modelagem da planta
  // Lout_km1 = (2.4e-4) * xL * Cvl * sqrt((P_km1 - P1) / (ro_L / ro_h2o));
  // Gout_km1 = (2.4e-4) * xG * Cvg * sqrt((P_km1 - P2) * (P_km1 + P2)*MMar*T / P_km1*MMg*Tar);
  
  // Vl_k = ((C * D * D) / 4) * (acos((D - 2 * h_km1) / D) - (2 * (sqrt(D - h_km1) * h_km1) / D) * ((D - 2 * h_km1) / D));
  
  // Lout_k = (2.4e-4) * xL * Cvl * sqrt((P_km1 - P1) / (ro_L / ro_h2o));
  // Gout_k = (2.4e-4) * xG * Cvg * sqrt((P_km1 - P2) * (P_km1 + P2)*MMar*T / P_km1*MMg*Tar);
 

 Lout_km1 = (2.4e-4) * xL * Cvl * sqrt((P - P1) / (ro_L / ro_h2o));
 Gout_km1 = (2.4e-4) * xG * Cvg * sqrt(((P - P2) * (P + P2)*MMar*T) / (P*MMg*Tar));
  
  Vl_k = ((C * D * D) / 4) * (acos((D - 2 * h) / D) - (2 * (sqrt(D - h) * h) / D) * ((D - 2 * h) / D));
  
  Lout_k = Lout_km1;
  Gout_k = Gout_km1;
 

//////////////////////////////////////////INTEGRANDO COMPLICADO COM RK4

float k1_h =  (Lin - Lout_km1) / (2 * C * sqrt((D - h) * h));
float k2_h = (Lin - Lout_km1) / (2 * C * sqrt((D - (h+(dh/2)*k1_h)) * (h+(dh/2)*k1_h)));
float k3_h = (Lin - Lout_km1) / (2 * C * sqrt((D - (h+(dh/2)*k2_h)) * (h+(dh/2)*k2_h)));
float k4_h = (Lin - Lout_km1) / (2 * C * sqrt((D - (h+(dh)*k3_h)) * (h+(dh)*k3_h)));
h += dh*(k1_h + 2*k2_h+ 2*k3_h + k4_h) / 6;
//h=h_k;  

float k1_P =  (P * (Gin - Gout_km1 + Lin - Lout_km1)) / (V - Vl_k);
float k2_P =  ((P+(dh/2)*k1_P) * (Gin - Gout_km1 + Lin - Lout_km1)) / (V - Vl_k);
float k3_P = ((P+(dh/2)*k2_P) * (Gin - Gout_km1 + Lin - Lout_km1)) / (V - Vl_k);
float k4_P = ((P+(dh)*k3_P) * (Gin - Gout_km1 + Lin - Lout_km1)) / (V - Vl_k);
P +=   dh*(k1_P + 2*k2_P+ 2*k3_P + k4_P) / 6;
dp = dh*(k1_P + 2*k2_P+ 2*k3_P + k4_P) / 6;
//P=P_k;  
////////////////////////////////////////



/////////////////////////////////////////////////////////////INTEGRANDO SIMPLES VIA EULER
  // dP_k = (P_km1 * (Gin - Gout_km1 + Lin - Lout_km1)) / (V - Vl_k);
  // P_k = dP_k * dt + P_km1;
  
  // dh_k = (Lin - Lout_km1) / (2 * C * sqrt((D - h_km1) * h_km1));
  // h_k = dh_k * dt + h_km1;
//////////////////////////////////////////////////////////////////////////////////////////////
  //atualizando as variáveis para a iteração
  // dh_km1 = dh_k;
  // h_km1 = h_k;
  // dP_km1 = dP_k;
  // P_km1 = P_k;
  // ro_g_km1 = ro_g_k;
  // Lout_km1 = Lout_k;
  // Gout_km1 = Gout_k;

 }

void loop() {

//dac1.setVoltage(h,false);//*4095/5,false);
dac2.setVoltage(h*4095/5,false);//*4095/5,false);

//Serial.print(" nivel:");
Serial.print(h);
Serial.print(" ");
time = millis()/1000;
Serial.println(time);


//Serial.println(xL);

//válvulas de saída
// Serial.print("pressão:");
// Serial.print(P);
// Serial.print(" lout:");
// Serial.print(Lout_k);
// Serial.print(" gout:");
// Serial.print(Gout_k);
// Serial.print(" nivel:");
// Serial.println(h);
//Serial.println(dp*1000);
// contador ++;
// Serial.print("Contador: ");
// Serial.println(contador);
delay(1000); 
}
