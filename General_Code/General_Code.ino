// Código unificado general :)

// ACCELEROMETER MPU6050 - I2C
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;


// INDUCTIVE
const int ind_pin = 8; // Digital pin connected to the infrared/inductive sensor
const int ind_measurements = 3; // Number of measurements to average

bool ind_firstPulseDetected = false; // Flag to check if the first pulse has been detected (only used once)
unsigned long ind_lastPulseTime = 0; // Time of the last detected signal
unsigned long ind_currentPulseTime = 0; // Time of the current detected signal
unsigned long ind_timeInterval = 0; // Time between last and current detected signals

float ind_freq = 0; // Value to store measurements
float ind_avg_freq = 0;

int ind_counter = 1;


// HUMIDITY DHT11
#include "DHTStable.h"
DHTStable DHT;
#define DHT11_PIN 4


// THERMISTORS
//Librería para logaritmo
#include <math.h>

// Pines del Arduino
#define Term_Water A1
#define Term_Motor A0

float V_ref = 5; //Voltaje del Arduino

//Datos del termistor water

float R_1_W = 5200; //Resistencia fija
float V_O_W; //Tensión de salida - Medición
float R_T_W; //Resistencia variable del termistor - Cálculo

int SensorValue_W; //Entrada analógica del Arduino

float T_W; //Temperatura medida - Cálculo

//Parámetros del termistor - Beta model
float R0_W = 10000; // [Ω]
float T0_W = 298.15; // [K]
float B_W = 3435; // [K]

//Datos del termistor motor

float R_1_M = 5088; //Resistencia fija
float V_O_M; //Tensión de salida - Medición
float R_T_M; //Resistencia variable del termistor - Cálculo

int SensorValue_M; //Entrada analógica del Arduino

float T_M; //Temperatura medida - Cálculo

//Parámetros del termistor - Beta model
float R0_M = 1000; // [Ω]
float T0_M = 298.15; // [K]
float B_M = 3800; // [K]


// ULTRASONICS UB1000
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
  pinMode(ind_pin, INPUT); // Sensor pin as input with pull-up resistor

  // HUMIDITY
  // Nothing to do, yay


  // THERMISTORS
  pinMode(Term_Water, INPUT);
  pinMode(Term_Motor, INPUT);


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
    if (digitalRead(ind_pin) == HIGH) { // LOW for infrared, HIGH for inductive
      ind_currentPulseTime = millis(); // Store current time

      // If this is the first pulse detected, initialize the ind_lastPulseTime
      if (!ind_firstPulseDetected) {
        ind_firstPulseDetected = true;
        ind_lastPulseTime = ind_currentPulseTime; // Set the initial pulse time
        }
      
      // Any subsequent pulse
      else{
        // Calculate time between pulses
        ind_timeInterval = ind_currentPulseTime - ind_lastPulseTime;
        // Update the last pulse time
        ind_lastPulseTime = ind_currentPulseTime;

        // Calculate frequency and update ind_freq
        ind_freq += 1000.0 / ind_timeInterval; // Frecuency in Hz

        //Still in-bounds
        if (ind_counter < ind_measurements) {          
          ind_counter += 1;
        }
        // Out of bounds
        else {
          ind_counter = 0; // Reset measurement index

          // Take the average of all measurements
          ind_avg_freq = ind_freq / ind_measurements;
          ind_avg_freq = ind_avg_freq * 60; // convert to rpm
        }
      }
      //delay(400); // Delay to avoid rapid measurements
    }


    // HUMIDITY
    // Read data
    /*
    int chk = DHT.read11(DHT11_PIN);
    switch (chk) {
      case DHTLIB_OK:
        Serial.print("OK,\t");
        break;
      case DHTLIB_ERROR_CHECKSUM:
        Serial.print("Checksum error,\t");
        break;
      case DHTLIB_ERROR_TIMEOUT:
        Serial.print("Time out error,\t");
        break;
      default:
        Serial.print("Unknown error,\t");
        break;
    }
    */
    //delay(2000);


    // THERMISTORS
    //Lectura del Arduino
    SensorValue_W = analogRead(Term_Water);
    V_O_W = SensorValue_W * V_ref / 1023;

    SensorValue_M = analogRead(Term_Motor);
    V_O_M = SensorValue_M * V_ref / 1023; //Mapeo

    //Fórmulas obtenidas
    R_T_W = R_1_W * V_O_W / (V_ref - V_O_W); //Divisor de tensión
    T_W = 1/(log(R_T_W/R0_W)/B_W + 1/T0_W) - 273.15; //Ecuación de beta - Conversión a °C

    R_T_M = R_1_M * V_O_M / (V_ref - V_O_M);
    T_M = 1/(log(R_T_M/R0_M)/B_M + 1/T0_M) - 273.15;

    delay(2000);


    // ULTRASONICS
    s1_distance_avg = 0;
    s2_distance_avg = 0;

    s1_voltage = analogRead(s1);
    s2_voltage = analogRead(s2);
    s1_distance_avg += mapping(s1_voltage, 1);
    s2_distance_avg += mapping(s2_voltage, 2);
    
    // Get averages
    s1_distance_avg = s1_distance_avg / num_med;
    s2_distance_avg = s2_distance_avg / num_med;

    // Calibration
    s1_distance_calibrated = s1_distance_avg * s1_slope + s1_intercept;
    s2_distance_calibrated = s2_distance_avg * s2_slope + s2_intercept;




    // PRINT OUT ALL THE VALUES
    // First line per sensor is title, delete when actually saving data
    // Also change all final println to regular print for each sensor (except ultrasonics), all sensors should be within the same line

    // Print results to csv - With filter for if ultrasonics turn off
    if (s1_distance_calibrated - s1_zero_lvl < 300 && s2_distance_calibrated - s2_zero_lvl < 300) {

      // Time
      Serial.println("Time (s)");
      Serial.print(float(millis_current) / 1000); // Time in seconds
      //Serial.print(",");
      Serial.println("");

      // Accelerometer
      Serial.println("Acceleration (m^2/s) x,y,z")
      Serial.print(a.acceleration.x);
      Serial.print(",");
      Serial.print(a.acceleration.y);
      Serial.print(",");
      Serial.println(a.acceleration.z);
      //Serial.print(",");
      Serial.println("");

      // Inductive
      Serial.println("Angular Velocity (rpm)");
      Serial.print(ind_avg_freq);
      //Serial.print(",");
      Serial.println("");

      // Humidity
      Serial.println("Humidity (%), Temperature (°C)");
      Serial.print(DHT.getHumidity(), 1);
      Serial.print(",");
      Serial.print(DHT.getTemperature(), 1);
      //Serial.print(",");
      Serial.println("");

      // Thermistors
      Serial.println("Water_temp,Motor_temp (°C)");
      Serial.print(T_W);
      Serial.print(",");
      Serial.print(T_M);
      //Serial.print(",");
      Serial.println("");

      // Ultrasonics
      Serial.println("Height (mm) s1,s2");
      Serial.print(s1_distance_calibrated - s1_zero_lvl);
      Serial.print(",");
      Serial.println(s2_distance_calibrated - s2_zero_lvl);
    }
  }
}
