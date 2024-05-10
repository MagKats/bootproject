// Include the necessary libraries
#include <Arduino.h>

// Define the GPIO pin connected to the servo
const int servoPin = 23; // Change this to the appropriate GPIO pin

// Define variables for servo control
const int minPulseWidth = 1000; // Minimum pulse width for the TowerPro SG90D
const int maxPulseWidth = 2000; // Maximum pulse width for the TowerPro SG90D
const int servoRange = maxPulseWidth - minPulseWidth; // Range of pulse width

// Function to set the servo angle
void setServoAngle(int angle) {
  // Calculate the pulse width based on the desired angle
  int pulseWidth = map(angle, 0, 360, minPulseWidth, maxPulseWidth);

  // Generate the PWM signal on the servo pin
  ledcWrite(0, pulseWidth);
}

void setup() {
  // Initialize the GPIO pin for servo control
  ledcAttachPin(servoPin, 0);
  ledcSetup(0, 50, 16); // Configure LEDC channel 0 for 50 Hz PWM with 16-bit resolution

  // Set the initial position of the servo (e.g., 180 degrees)
  setServoAngle(180);
}

void loop() {
  // Rotate the servo continuously (for demonstration purposes)
  for (int angle = 0; angle <= 360; angle += 10) {
    setServoAngle(angle);
    delay(500);
  }
}
