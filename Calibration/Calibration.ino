// Se toman n mediciones y se saca el promedio de todas esas, dura como 3 segundos cada promedio

const int sensor1 = A3; // No Bond
const int sensor2 = A2; // Bond
const int sync = 7;
const int num_med = 169; //nice

float noBond_voltage;
float Bond_voltage;
float distance;
float noBond_distance_avg = 0;
float Bond_distance_avg = 0;
float noBond_distance_calibrated = 0;
float Bond_distance_calibrated = 0;

// Manual mapping No Bond -> UPDATED
float noBond_min_Volt = 175;
float noBond_max_Volt = 883;
float noBond_min_Dist = 88;
float noBond_max_Dist = 973;

// Manual mapping Bond -> UPDATED
float Bond_min_Volt = 176;
float Bond_max_Volt = 882;
float Bond_min_Dist = 87;
float Bond_max_Dist = 967;

float noBond_slope = 1;
float noBond_intercept = -1.5;
float Bond_slope = 1;
float Bond_intercept = -1.5;


void setup() {
  Serial.begin(115200);
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sync, OUTPUT);
}

void loop() {
  // Hace las n mediciones y las suma
  noBond_distance_avg = 0; // Hacer reset por favor!
  Bond_distance_avg = 0;
  for (int i = 0; i < num_med; i++) {
    // Synchronization
    digitalWrite(sync, HIGH);
    delay(2);
    digitalWrite(sync, LOW);
    delay(2);
    noBond_voltage = analogRead(sensor1);
    Bond_voltage = analogRead(sensor2);
    //Serial.print("voltaje sensor 1: ");
    //Serial.println(noBond_voltage);
    //Serial.print(" voltaje sensor 2: ");
    //Serial.println(Bond_voltage);
    noBond_distance_avg = noBond_distance_avg + (noBond_voltage - noBond_min_Volt) / (noBond_max_Volt - noBond_min_Volt) * (noBond_max_Dist - noBond_min_Dist) + noBond_min_Dist; // Mapeo manual sensor 1 para tener decimales (regla de 3)
    Bond_distance_avg = Bond_distance_avg + (Bond_voltage - Bond_min_Volt) / (Bond_max_Volt - Bond_min_Volt) * (Bond_max_Dist - Bond_min_Dist) + Bond_min_Dist; // Mapeo manual sensor 2 para tener decimales (regla de 3)
    //noBond_distance_avg = map(noBond_voltage,0,1024,30,500); // Mapeo automático de Arduino
    delay(50);
  }
  
  noBond_distance_avg = noBond_distance_avg / num_med; // Saca el promedio
  Bond_distance_avg = Bond_distance_avg / num_med; // Saca el promedio

  // Calibración hecha en el excel: NotBond
  noBond_distance_calibrated = (noBond_distance_avg - noBond_intercept) / noBond_slope;
  //Serial.println(noBond_distance_avg);
  //Serial.println(noBond_distance_calibrated);

  // Calibración hecha en el excel: Bond
  Bond_distance_calibrated = (Bond_distance_avg - Bond_intercept) / Bond_slope;
  //Serial.println(Bond_distance_avg);
  Serial.println(Bond_distance_calibrated);
}
