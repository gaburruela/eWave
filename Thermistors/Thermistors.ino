//Librería para logaritmo
#include <math.h>

//Datos del termistor motor
float V_ref = 5; //Voltaje del Arduino
float R_1 = 5100; //Resistencia fija
float V_O; //Tensión de salida - Medición
float R_T; //Resistencia variable del termistor - Cálculo

int SensorValue; //Entrada analógica del Arduino

float T; //Temperatura medida - Cálculo

//Parámetros del termistor

// First point - R(60°C) = 1k
// Second point - R(20°C) = 5k
// Thrird point - R(85°C) = 0.4k

float A = 1.676866998*pow(10,-3);
float B = 1.690093223*pow(10,-4);
float C = 4.772460971*pow(10,-7);


void setup() {
  Serial.begin(9600);
}

void loop() {
  //Lectura del Arduino
  SensorValue = analogRead(A1);
  V_O = SensorValue * V_ref / 1023; //Mapeo

  //Fórmulas obtenidas
  R_T = R_1 * V_O / (V_ref - V_O); //Divisor de tensión

  T = 1/(A+B*log(R_T)+C*pow(log(R_T), 3)) - 273.15; //Ecuación de Steinhart-Hart - Conversión a °C
  
  // Display results
  Serial.print("Resistance: ");
  Serial.print(R_T);
  Serial.print(" kΩ - Temperature: ");
  Serial.print(T);
  Serial.print(" °C\n");
  //Serial.print(SensorValue);
  delay(2000);
}
