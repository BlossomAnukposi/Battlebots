#include <Servo.h>

// Define the servo pin
const int servoPin = 12;

// Define servo object
Servo gripperServo;

// Define gripper positions
const int closedPosition = 0; // Adjust as needed
const int openPosition = 120; // Adjust as needed

// Define ultrasonic sensor pins
const int trigPin = 2; // Trig pin of ultrasonic sensor
const int echoPin = 3; // Echo pin of ultrasonic sensor

// Define variables for gripper state
bool gripperOpen = true;

void setup() {
  // Attach the servo to the pin
  gripperServo.attach(servoPin);

  // Initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initially open the gripper
  gripperServo.write(openPosition);
}

void loop() {
  // Trigger the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the echo time to determine distance
  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;

  // If an object is within range, close the gripper
  if (distance < 10 && gripperOpen) {
    gripperServo.write(closedPosition);
    gripperOpen = false;
    delay(1000); // Adjust delay as needed to ensure stable gripping
  }
  // If no object is detected, open the gripper
  else if (distance >= 10 && !gripperOpen) {
    gripperServo.write(openPosition);
    gripperOpen = true;
    delay(1000); // Adjust delay as needed to ensure stable release
  }

  // Optional: Add a small delay to reduce loop frequency
  delay(100);
}
