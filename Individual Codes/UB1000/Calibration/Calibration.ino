// Se toman n mediciones y se saca el promedio de todas esas, dura como 3 segundos cada promedio

const int sensor1 = A0; // No Bond
const int sensor2 = A4; // Bond
const int num_med = 169; //nice

float s1_voltage;
float s2_voltage;
float distance;
float s1_distance_avg = 0;
float s2_distance_avg = 0;
float s1_distance_calibrated = 0;
float s2_distance_calibrated = 0;

// Manual mapping No Bond -> UPDATED
float s1_min_Volt = 176;
float s1_max_Volt = 887;
float s1_min_Dist = 89;
float s1_max_Dist = 983;

// Manual mapping Bond -> UPDATED
float s2_min_Volt = 176;
float s2_max_Volt = 884;
float s2_min_Dist = 89;
float s2_max_Dist = 990;

void setup() {
  Serial.begin(9600);
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
}

void loop() {
  // Hace las n mediciones y las suma
  s1_distance_avg = 0; // Hacer reset por favor!
  s2_distance_avg = 0; 
  for (int i = 0; i < num_med; i++) {
    s1_voltage = analogRead(sensor1);
    s2_voltage = analogRead(sensor2);
    //Serial.print("voltaje sensor 1: ");
    //Serial.print(s1_voltage);
    //Serial.print(" voltaje sensor 2: ");
    //Serial.println(s2_voltage);
    s1_distance_avg = s1_distance_avg + (s1_voltage - s1_min_Volt) / (s1_max_Volt - s1_min_Volt) * (s1_max_Dist - s1_min_Dist) + s1_min_Dist; // Mapeo manual sensor 1 para tener decimales (regla de 3)
    s2_distance_avg = s2_distance_avg + (s2_voltage - s2_min_Volt) / (s2_max_Volt - s2_min_Volt) * (s2_max_Dist - s2_min_Dist) + s2_min_Dist; // Mapeo manual sensor 2 para tener decimales (regla de 3)
    //s1_distance_avg = map(s1_voltage,0,1024,30,500); // Mapeo automático de Arduino
    delay(50);
  }
  
  s1_distance_avg = s1_distance_avg / num_med; // Saca el promedio
  s2_distance_avg = s2_distance_avg / num_med; // Saca el promedio

  // Calibración hecha en el excel: NotBond
  s1_distance_calibrated = (s1_distance_avg - 2.344994118) / 0.998825735 - 436;
  //Serial.println(s1_distance_avg);
  Serial.println(s1_distance_calibrated);

  // Calibración hecha en el excel: Bond
  s2_distance_calibrated = (s2_distance_avg + 2.293051471) / 1.020507353 - 435;
  //Serial.println(s2_distance_avg);
  Serial.println(s2_distance_calibrated);
}
