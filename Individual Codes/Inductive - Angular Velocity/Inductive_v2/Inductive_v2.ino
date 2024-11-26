// This code takes n measurements and then takes the average of those to determine the angular velocity of a motor
// Make sure to check the value of sensorPin as LOW or HIGH depending on the sensor used

const int ind_pin = 8; // Digital pin connected to the infrared/inductive sensor
const int ind_measurements = 3; // Number of measurements to average

bool ind_firstPulseDetected = false; // Flag to check if the first pulse has been detected (only used once)
unsigned long ind_lastPulseTime = 0; // Time of the last detected signal
unsigned long ind_currentPulseTime = 0; // Time of the current detected signal
unsigned long ind_timeInterval = 0; // Time between last and current detected signals

float ind_freq = 0; // Value to store measurements
float ind_avg_freq = 0;

int ind_counter = 1;


void setup() {
  pinMode(ind_pin, INPUT); // Sensor pin as input with pull-up resistor
  Serial.begin(9600); // Start serial communication
}

void loop() {
  // Check the sensor pin
  if (digitalRead(ind_pin) == HIGH) { // LOW for infrared, HIGH for inductive
    ind_currentPulseTime = millis(); // Store current time
    // If this is the first pulse detected, initialize the ind_lastPulseTime
    if (!ind_firstPulseDetected) {
      ind_firstPulseDetected = true;
      ind_lastPulseTime = ind_currentPulseTime; // Set the initial pulse time
    }
    
    // Any subsequent pulse
    else {
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
        ind_counter = 1; // Reset measurement index

        // Take the average of all measurements
        ind_avg_freq = ind_freq / ind_measurements;
        ind_avg_freq = ind_avg_freq * 60; // convert to rpm

        ind_freq = 0; // Reset frequency variable
      }
    }
    delay(400); // Delay to avoid rapid measurements
    Serial.println("Angular Velocity (rpm)");
    Serial.print(ind_avg_freq);
    //Serial.print(",");
    Serial.println("");
  }
}
