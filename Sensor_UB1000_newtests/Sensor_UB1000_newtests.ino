// Se toman n mediciones y se saca el promedio de todas esas, dura como 3 segundos cada promedio

const int sensor = A0;
const int num_med = 1;

float voltage;
float distance;
float distance_avg = 0;
float distance_calibrated = 0;

void setup() {
  Serial.begin(9600);
  pinMode(sensor, INPUT);
}

void loop() {
  // Hace las n mediciones y las suma
  distance_avg = 0; // Hacer reset por favor!
  for (int i = 0; i < num_med; i++) {
    voltage = analogRead(sensor);
    distance_avg = distance_avg + voltage / 1024 * 930 + 70; // Mapeo manual para tener decimales (regla de 3)
    // distance_avg = map(voltage,0,1024,30,500); // Mapeo automático de Arduino
    delay(50);
  }

  distance_avg = distance_avg / num_med; // Saca el promedio

  // Calibración hecha en el excel
  //distance_calibrated = distance_avg/1.0629 + 21.48;
  Serial.print(distance_avg);
  //Serial.println(distance_calibrated);
}

// Desfase de ~20 mm experimentales (para obtener medidas reales)
