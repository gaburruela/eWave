// Se toman n mediciones y se saca el promedio de todas esas, dura como 3 segundos cada promedio

const int num_med = 1; // Mediciones por cada punto -> En 1 para lecturas reales
const int med_zero = 100; // Mediciones para hacer el zero level

// First sensor variables
const int sensor1 = A0;
float voltage1;
float distance_avg1 = 0;
float zero_lvl1 = 0;

// Second sensor variables
const int sensor2 = A4;
float voltage2;
float distance_avg2 = 0;
float zero_lvl2 = 0;

void setup() {
  Serial.begin(9600);      
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);

  zero_lvl1 = 0; // Hacer reset por favor!
  zero_lvl2 = 0; // Hacer reset por favor!
  for (int i = 0; i < med_zero; i++) {
    voltage1 = analogRead(sensor1);
    voltage2 = analogRead(sensor2);
    zero_lvl1 = zero_lvl1 + voltage1 / 1024 * 930 + 70; // Mapeo manual para tener decimales (regla de 3)
    zero_lvl2 = zero_lvl2 + voltage2 / 1024 * 930 + 70;
    delay(50);
  }
  
  // Zero level setting
  zero_lvl1 = zero_lvl1 / med_zero; // Saca el promedio
  zero_lvl2 = zero_lvl2 / med_zero;

  Serial.println(zero_lvl1);
  Serial.println(zero_lvl2);
}

void loop() {
  // Hace las n mediciones y las suma
  distance_avg1 = 0; // Hacer reset por favor!
  distance_avg2 = 0;
  for (int i = 0; i < num_med; i++) {
    voltage1 = analogRead(sensor1);
    voltage2 = analogRead(sensor2);
    distance_avg1 = distance_avg1 + voltage1 / 1024 * 930 + 70; // Mapeo manual para tener decimales (regla de 3)
    distance_avg2 = distance_avg2 + voltage2 / 1024 * 930 + 70;
    // distance_avg = map(voltage,0,1024,30,500); // Mapeo automático de Arduino
    delay(50);
  }

  distance_avg1 = distance_avg1 / num_med;
  distance_avg2 = distance_avg2 / num_med;

  // Calibración hecha en el excel
  Serial.print(millis());
  Serial.print(",");
  //Serial.print("Sensor 1:");
  Serial.print(distance_avg1 - zero_lvl1);
  Serial.print(",");
  //Serial.print(" - Sensor 2:");
  Serial.println(distance_avg2 - zero_lvl2);
}
