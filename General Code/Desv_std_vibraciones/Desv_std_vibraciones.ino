// Se toman n mediciones y se saca el promedio de todas esas, dura como 3 segundos cada promedio

const int sensor1 = A3; // No Bond
const int sensor2 = A2; // Bond
// const int num_med = 169; //nice
const int num_med = 200; //nice

float s1_voltage;
float s2_voltage;
float distance;
float s1_distance_avg = 0;
float s2_distance_avg = 0;
float s1_distance_calibrated = 0;
float s2_distance_calibrated = 0;
int s1_distance_int;
int s2_distance_int;

// Manual mapping No Bond -> NOT UPDATED
float s1_min_Volt = 176;
float s1_max_Volt = 887;
float s1_min_Dist = 89;
float s1_max_Dist = 983;

// Manual mapping Bond -> NOT UPDATED
float s2_min_Volt = 176;
float s2_max_Volt = 884;
float s2_min_Dist = 89;
float s2_max_Dist = 990;

// Calculos de desviacion estandar
float Bond_stdev = 0;
float NoBond_stdev = 0;
float NoBond_val = 0;
float Bond_val = 0;

void setup() {
  Serial.begin(9600);
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
}

void loop() {

float mediciones_Bond[num_med];
float mediciones_NoBond[num_med]; 

  // Hace las n mediciones y las suma
  s1_distance_avg = 0; // Hacer reset por favor!
  s2_distance_avg = 0; 
  NoBond_val = 0;
  Bond_val = 0;

  for (int i = 0; i < num_med; i++) {
    
    s1_voltage = analogRead(sensor1);
    s2_voltage = analogRead(sensor2);

    NoBond_val = (s1_voltage - s1_min_Volt) / (s1_max_Volt - s1_min_Volt) * (s1_max_Dist - s1_min_Dist) + s1_min_Dist;
    mediciones_NoBond[i] = (NoBond_val + 0.09645) / 0.994539706;

    Bond_val =  (s2_voltage - s2_min_Volt) / (s2_max_Volt - s2_min_Volt) * (s2_max_Dist - s2_min_Dist) + s2_min_Dist;
    mediciones_Bond[i] = (Bond_val + 6.587792647) / 1.016928676;


    Serial.print(i);
    //Serial.print(" voltaje sensor 2: ");
    //Serial.println(s2_voltage);
    s1_distance_avg = s1_distance_avg + NoBond_val; // Mapeo manual sensor 1 para tener decimales (regla de 3)
    s2_distance_avg = s2_distance_avg + Bond_val; // Mapeo manual sensor 2 para tener decimales (regla de 3)
    //s1_distance_avg = map(s1_voltage,0,1024,30,500); // Mapeo automático de Arduino
    delay(50);
  }
  
  s1_distance_avg = s1_distance_avg / num_med; // Saca el promedio de NoBond
  s2_distance_avg = s2_distance_avg / num_med; // Saca el promedio de Bond

  // Calibración hecha en el excel: NotBond
  s1_distance_calibrated = (s1_distance_avg + 0.09645) / 0.994539706;
  s1_distance_int = round(s1_distance_calibrated); // Redondea a int


  // Calibración hecha en el excel: Bond
  s2_distance_calibrated = (s2_distance_avg + 6.587792647) / 1.016928676;
  s2_distance_int = round(s2_distance_calibrated); // Redondea a int

  Bond_stdev = 0;
  NoBond_stdev = 0;

 for(int i = 0; i < num_med; i++) {
  Bond_stdev += pow(mediciones_Bond[i]-s2_distance_calibrated,2);
  NoBond_stdev += pow(mediciones_NoBond[i]-s1_distance_calibrated,2);
 }

 Bond_stdev = pow(Bond_stdev/(num_med-1),0.5);
 NoBond_stdev = pow(NoBond_stdev/(num_med-1),0.5);

  Serial.print("Bond stdev:");
  Serial.println(Bond_stdev);
  Serial.print("NoBond stdev:");
  Serial.println(NoBond_stdev);
}
