// Se toman n mediciones y se saca el promedio de todas esas, dura como 3 segundos cada promedio

const int sensor = A0;
const int num_med = 1;

float voltage;
float distance;
float distance_avg = 0;
float distance_calibrated = 0;

const int sensor2 = A4;
float voltage2;
float distance2;
float distance_avg2 = 0;

void setup() {
  Serial.begin(9600);      
  pinMode(sensor, INPUT);
  pinMode(sensor2, INPUT);
}

void loop() {
  // Hace las n mediciones y las suma
  distance_avg = 0; // Hacer reset por favor!
  distance_avg2 = 0; // Hacer reset por favor!
  for (int i = 0; i < num_med; i++) {
    voltage = analogRead(sensor);
    voltage2 = analogRead(sensor2);
    distance_avg = distance_avg + voltage / 1024 * 930 + 70; // Mapeo manual para tener decimales (regla de 3)
    distance_avg2 = distance_avg2 + voltage2 / 1024 * 930 + 70;
    // distance_avg = map(voltage,0,1024,30,500); // Mapeo automático de Arduino
    delay(50);
  }

  distance_avg = distance_avg / num_med; // Saca el promedio
  distance_avg2 = distance_avg2 / num_med;

  // Calibración hecha en el excel
  //distance_calibrated = distance_avg/1.0629 + 21.48;
  Serial.print("Sensor 1:");
  Serial.print(distance_avg);
  Serial.print(" - Sensor 2:");
  Serial.println(distance_avg2);
  //Serial.println(distance_calibrated);
}

// Desfase de ~20 mm experimentales (para obtener medidas reales)
