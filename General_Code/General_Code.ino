// Código unificado general :)

// ACCELEROMETER MPU6050 - I2C
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;


// INDUCTIVE
const int sensorPin = 8; // Digital pin connected to the infrared/inductive sensor
const int numMeasurements = 3; // Number of measurements to average

bool firstPulseDetected = false; // Flag to check if the first pulse has been detected (only used once)
unsigned long lastPulseTime = 0; // Time of the last detected signal
unsigned long currentPulseTime = 0; // Time of the current detected signal
unsigned long timeInterval = 0; // Time between last and current detected signals

int measurementIndex = 0; // Index for storing measurements
float measurements[numMeasurements] = {0}; // Array to store measurements
float averageFrequency = 0;
float angularVelocity = 0;

bool inductive_flag = false;


// Ultrasonics UB1000
// General variables
const int num_med = 1; // average
const int med_zero = 100; // zero-leveling

// Time variables [ms]
unsigned long millis_previous = 0;
unsigned long millis_current;
const long time_interval = 40; // Around 40

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




void setup(void) {
  Serial.begin(9600);

  // ACCELEROMETER
	// Try to initialize!
	if (!mpu.begin()) {
		Serial.println("Failed to find MPU6050 chip");
		while (1) {
		  delay(10);
		}
	}
	Serial.println("MPU6050 Found!");

	// set accelerometer range to +-8G
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

	// set gyro range to +- 500 deg/s
	mpu.setGyroRange(MPU6050_RANGE_500_DEG);

	// set filter bandwidth to 21 Hz
	mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

	//delay(100);


  // INDUCTIVE
  pinMode(sensorPin, INPUT); // Sensor pin as input with pull-up resistor


  // ULTRASONICS
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

  Serial.println("Zero levels:")
  Serial.println(s1_zero_lvl);
  Serial.println(s2_zero_lvl);
}

void loop() {
  // UNIFORM INTERVALS
  millis_current = millis();

  // Take new measurements only if inside the time interval
  if (millis_current - millis_previous >= time_interval) {
    millis_previous = millis_current;

    // ACCELEROMETER
    // Get new sensor events with the readings
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    //delay(50);


    // INDUCTIVE
    // Check the sensor pin
    if (digitalRead(sensorPin) == HIGH) { // LOW for infrared, HIGH for inductive
      currentPulseTime = millis(); // Store current time

      // If this is the first pulse detected, initialize the lastPulseTime
      if (!firstPulseDetected) {
        firstPulseDetected = true;
        lastPulseTime = currentPulseTime; // Set the initial pulse time
        }
      
      // Any subsequent pulse
      else{
        // Calculate time between pulses
        timeInterval = currentPulseTime - lastPulseTime;
        // Update the last pulse time
        lastPulseTime = currentPulseTime;


        //Still in-bounds
        if (measurementIndex < numMeasurements - 1) {
          // Calculate frequency and update measurements
          measurements[measurementIndex] = 1000.0 / timeInterval; // Frecuency in Hz
          measurementIndex += 1;
        }
        // Out of bounds
        else {
          measurements[measurementIndex] = 1000.0 / timeInterval; // Last measurement
          measurementIndex = 0; // Reset measurement index

        // Take the average of all measurements
        averageFrequency = 0; // Reset previous value
          for (int i = 0; i < numMeasurements; i++) {
            averageFrequency += measurements[i];
          }
          averageFrequency /= numMeasurements;
          inductive_flag = true;


          // Print results
          angularVelocity = averageFrequency * 60; // rpm
          Serial.print("Average Angular Velocity: ");
          Serial.print(angularVelocity);
          Serial.println(" rpm");
        }
      }
      delay(400); // Delay to avoid rapid measurements
  }

    // ULTRASONICS
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



    // PRINT OUT ALL THE VALUES
    // First line per sensor is title, delete when actually saving data
    // Also change all final println to regular print for each sensor, all sensors should be within the same line

    // Print results to csv - With filter for if ultrasonics turn off
    if (s1_distance_calibrated - s1_zero_lvl < 300 && s2_distance_calibrated - s2_zero_lvl < 300) {

      // Time
      Serial.println("Time (s)");
      Serial.print(float(millis_current) / 1000); // Time in seconds
      Serial.print(",");
      Serial.println("");

      // Accelerometer
      Serial.println("Acceleration (m^2/s) x,y,z")
      Serial.print(a.acceleration.x);
      Serial.print(",");
      Serial.print(a.acceleration.y);
      Serial.print(",");
      Serial.println(a.acceleration.z);
      Serial.print(",");
      Serial.println("");

      // Ultrasonics
      Serial.println("Height (mm) s1,s2");
      Serial.print(s1_distance_calibrated - s1_zero_lvl);
      Serial.print(",");
      Serial.println(s2_distance_calibrated - s2_zero_lvl);
    }
  }
}
