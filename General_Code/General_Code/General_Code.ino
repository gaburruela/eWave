// Código general v1
// Includes: UB1000, Inductive, Accelerometer, Thermistors and Humidity

// General variables
const int num_med = 1; // average
const int med_zero = 100; // zero-leveling

// Time variables [ms]
unsigned long millis_previous = 0;
unsigned long millis_current;
const long time_interval = 125; // Response delay from datasheet

// Define sensor pins
const int s1 = A5; // Not Bond
const int s2 = A4; // Bond

// Sensor 1 variables
float s1_voltage;
float s1_distance_avg = 0;
float s1_distance_calibrated = 0;
float s1_zero_lvl = 0;
// Sensor 2 variables
float s2_voltage;
float s2_distance_avg = 0;
float s2_distance_calibrated = 0;
float s2_zero_lvl = 0;

// Manual mapping
// Not Bond
const float s1_min_Volt = 176; // [bits]
const float s1_max_Volt = 887; // [bits]
const float s1_min_Dist = 89; // [mm]
const float s1_max_Dist = 983; // [mm]
// Bond
const float s2_min_Volt = 176;
const float s2_max_Volt = 884;
const float s2_min_Dist = 89;
const float s2_max_Dist = 990;

// Parámetros de calibración - Revisar Excel
// Not Bond
const float s1_slope = 0.998825735;
const float s1_intercept = 2.344994118;
// Bond
const float s2_slope = 1.020507353;
const float s2_intercept = -2.293051471;


// Manual mapping - get decimals - sensor is just which of the 2 sensors we're mapping
float mapping(float signal, int sensor) {
  if (sensor == 1) {
    return (signal - s1_min_Volt) / (s1_max_Volt - s1_min_Volt) * (s1_max_Dist - s1_min_Dist) + s1_min_Dist; // regla de 3
  }
  if (sensor == 2) {
    return (signal - s2_min_Volt) / (s2_max_Volt - s2_min_Volt) * (s2_max_Dist - s2_min_Dist) + s2_min_Dist;
  }
  return;
}


void setup() {
  Serial.begin(9600);
  pinMode(s1, INPUT);
  pinMode(s2, INPUT);

  // Zero-leveling
  for (int i = 0; i < med_zero; i++) {
    s1_voltage = analogRead(s1);
    s2_voltage = analogRead(s2);
    s1_zero_lvl = s1_zero_lvl + mapping(s1_voltage, 1);
    s2_zero_lvl = s2_zero_lvl + mapping(s2_voltage, 2);
    delay(50);
  }
  
  // Zero level setting - Averages
  s1_zero_lvl = s1_zero_lvl / med_zero;
  s2_zero_lvl = s2_zero_lvl / med_zero;

  // Calibrate sensors
  s1_zero_lvl = s1_zero_lvl * s1_slope + s1_intercept;
  s2_zero_lvl = s2_zero_lvl * s2_slope + s2_intercept;

  Serial.println(s1_zero_lvl);
  Serial.println(s2_zero_lvl);
}


void loop() {
  millis_current = millis();

  // Take new measurements
  if (millis_current - millis_previous >= time_interval) {
    millis_previous = millis_current;
    
    s1_distance_avg = 0;
    s2_distance_avg = 0;
    for (int i = 0; i < num_med; i++) {
      s1_voltage = analogRead(s1);
      s2_voltage = analogRead(s2);
      s1_distance_avg = s1_distance_avg + mapping(s1_voltage, 1);
      s2_distance_avg = s2_distance_avg + mapping(s2_voltage, 2);

      // Only for averages
      if (num_med > 1) {
        delay(50);
      }
    }
    
    // Get averages
    s1_distance_avg = s1_distance_avg / num_med;
    s2_distance_avg = s2_distance_avg / num_med;

    // Calibration
    s1_distance_calibrated = s1_distance_avg * s1_slope + s1_intercept;
    s2_distance_calibrated = s2_distance_avg * s2_slope + s2_intercept;


    // Print results to csv
    Serial.print(float(millis_current) / 1000); // Time in seconds
    Serial.print(",");
    Serial.print(s1_distance_calibrated - s1_zero_lvl);
    Serial.print(",");
    Serial.println(s2_distance_calibrated - s2_zero_lvl);
  }
}
