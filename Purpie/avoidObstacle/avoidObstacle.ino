#include <NewPing.h> // Include the NewPing library for the ultrasonic sensor

#define TRIGGER_PIN 4 // Define the Arduino pin connected to the sensor's trigger pin
#define ECHO_PIN 3  // Define the Arduino pin connected to the sensor's echo pin
#define MAX_DISTANCE 200 // Define the maximum distance to measure in centimeters (adjust according to your requirements)

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // Create a NewPing object

// Motor A
int motorPin1 = 11; // Connected to pin 2 on Arduino Nano
int motorPin2 = 10; // Connected to pin 3 on Arduino Nano

// Motor B
int motorPin3 = 6; // Connected to pin 4 on Arduino Nano
int motorPin4 = 5; // Connected to pin 5 on Arduino Nano

void setup() {
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(motorPin3, OUTPUT);
    pinMode(motorPin4, OUTPUT);
}

void loop() {
    int distance = sonar.ping_cm(); // Get the distance measured by the ultrasonic sensor in centimeters

    // If obstacle detected within 20cm, turn right
    if (distance < 20) {
        analogWrite(motorPin1, 245); //forword, left
        analogWrite(motorPin2, 0); //back
        analogWrite(motorPin3, 0); //forword, right
        analogWrite(motorPin4, 0); //back
    } else {
        // Move forward if no obstacle detected
      analogWrite(motorPin1, 235); //forword, left
      analogWrite(motorPin2, 0); //back
      analogWrite(motorPin3, 220); //forword, right
      analogWrite(motorPin4, 0); //back
    }
}
