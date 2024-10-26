// Código unificado para UB1000
// Includes: Dual measurements, calibration, zero-leveling, data to csv

// General variables
const int num_med = 1; // average
const int med_zero = 100; // zero-leveling

// Time variables [ms]
unsigned long millis_previous = 0;
unsigned long millis_current;
const long time_interval = 125; // Response delay from datasheet

// Define sensor pins
const int s1 = A0; // Not Bond
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
const float s1_min_Volt = 162; // [bits]
const float s1_max_Volt = 813; // [bits]
const float s1_min_Dist = 89; // [mm]
const float s1_max_Dist = 976; // [mm]
// Bond
const float s2_min_Volt = 158;
const float s2_max_Volt = 794;
const float s2_min_Dist = 85;
const float s2_max_Dist = 967;

// Parámetros de calibración - Revisar Excel
// Not Bond
const float s1_slope = 1.016270686;
const float s1_intercept = -19.43567117;
// Bond
const float s2_slope = 1;
const float s2_intercept = 0;


// Manual mapping - get decimals
float mapping(float signal, float min_Volt, float max_Volt, float min_Dist, float max_Dist) {
  return (signal - min_Volt) / (max_Volt - min_Volt) * (max_Dist - min_Dist) + min_Dist; // regla de 3
}


void setup() {
  Serial.begin(9600);
  pinMode(s1, INPUT);
  pinMode(s2, INPUT);

  // Zero-leveling
  for (int i = 0; i < med_zero; i++) {
    s1_voltage = analogRead(s1);
    s2_voltage = analogRead(s2);
    s1_zero_lvl = s1_zero_lvl + mapping(s1_voltage, s1_min_Volt, s1_max_Volt, s1_min_Dist, s1_max_Dist);
    s2_zero_lvl = s2_zero_lvl + mapping(s2_voltage, s2_min_Volt, s2_max_Volt, s2_min_Dist, s2_max_Dist);
    delay(50);
  }
  
  // Zero level setting - Averages
  s1_zero_lvl = s1_zero_lvl / med_zero;
  s2_zero_lvl = s2_zero_lvl / med_zero;

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
      s1_zero_lvl = s1_zero_lvl + mapping(s1_voltage, s1_min_Volt, s1_max_Volt, s1_min_Dist, s1_max_Dist);
      s2_zero_lvl = s2_zero_lvl + mapping(s2_voltage, s2_min_Volt, s2_max_Volt, s2_min_Dist, s2_max_Dist);

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


    // Print results
    Serial.print(millis_current * 1000); // Time in seconds
    Serial.print(",");
    Serial.print(s1_distance_calibrated - s1_zero_lvl);
    Serial.print(",");
    Serial.println(s2_distance_calibrated - s2_zero_lvl);
  }
}
