// This code takes n measurements and then takes the average of those to determine the angular velocity of a motor
// Make sure to check the value of sensorPin as LOW or HIGH depending on the sensor used

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


void setup() {
  pinMode(sensorPin, INPUT_PULLUP); // Sensor pin as input with pull-up resistor
  Serial.begin(9600); // Start serial communication
}

void loop() {
  // Check the sensor pin
  if (digitalRead(sensorPin) == LOW) { // LOW for infrared, HIGH for inductive
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


        // Print results
        angularVelocity = averageFrequency * 2*PI; // rad/s
        Serial.print("Average Angular Velocity: ");
        Serial.print(angularVelocity);
        Serial.println(" rad/s");
      }
    }
    delay(400); // Delay to avoid rapid measurements
  }
}
