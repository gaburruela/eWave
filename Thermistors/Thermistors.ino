//Librería para logaritmo
#include <math.h>

// Pines del Arduino
#define Term_Water A1
#define Term_Motor A0

float V_ref = 5; //Voltaje del Arduino

//DATOS DEL TERMISTOR WATER
float R_1_W = 5200; //Resistencia fija
float V_O_W; //Tensión de salida - Medición
float R_T_W; //Resistencia variable del termistor - Cálculo

int SensorValue_W; //Entrada analógica del Arduino

float T_W; //Temperatura medida - Cálculo

//Parámetros del termistor - Beta model
float R0_W = 10000; // [Ω]
float T0_W = 298.15; // [K]
float B_W = 3435; // [K]

//DATOS DEL TERMISTOR MOTOR
float R_1_M = 5088; //Resistencia fija
float V_O_M; //Tensión de salida - Medición
float R_T_M; //Resistencia variable del termistor - Cálculo

int SensorValue_M; //Entrada analógica del Arduino

float T_M; //Temperatura medida - Cálculo

//Parámetros del termistor - Beta model
float R0_M = 1000; // [Ω]
float T0_M = 298.15; // [K]
float B_M = 3800; // [K]

void setup() {
  Serial.begin(9600);
  pinMode(Term_Water, INPUT);
  pinMode(Term_Motor, INPUT);
}

void loop() {
  //Lectura del Arduino
  SensorValue_W = analogRead(Term_Water);
  V_O_W = SensorValue_W * V_ref / 1023;

  SensorValue_M = analogRead(Term_Motor);
  V_O_M = SensorValue_M * V_ref / 1023; //Mapeo

  //Fórmulas obtenidas
  R_T_W = R_1_W * V_O_W / (V_ref - V_O_W); //Divisor de tensión
  T_W = 1/(log(R_T_W/R0_W)/B_W + 1/T0_W) - 273.15; //Ecuación de beta - Conversión a °C

  R_T_M = R_1_M * V_O_M / (V_ref - V_O_M);
  T_M = 1/(log(R_T_M/R0_M)/B_M + 1/T0_M) - 273.15;
  
  // Display results
  Serial.print("Water resistance: ");
  Serial.print(R_T_W);
  Serial.print(" kΩ - Water temperature: ");
  Serial.print(T_W);
  Serial.print(" °C\n");

  Serial.print("Motor resistance: ");
  Serial.print(R_T_M);
  Serial.print(" kΩ - Motor temperature: ");
  Serial.print(T_M);
  Serial.print(" °C\n\n");
  delay(2000);
}
