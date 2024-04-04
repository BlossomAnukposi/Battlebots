/********************************************************************************
***  _       _________       _______  __   _       _____   ________________   ***
*** | |     / / ____/ |     / /   \ \/ /  | |     / /   | / ____/ ____/ __ \  ***
*** | | /| / / __/  | | /| / / /| |\  /   | | /| / / /| |/ /   / __/ / /_/ /  ***
*** | |/ |/ / /___  | |/ |/ / ___ |/ /    | |/ |/ / ___ / /___/ /___/ _, _/   ***
*** |__/|__/_____/  |__/|__/_/  |_/_/     |__/|__/_/  |_\____/_____/_/ |_|    ***
***                                                                           ***
***        Group IT1G                                                         ***
***        Tim & Quentin                                                      ***
*********************************************************************************/

//====[ LIBRARIES ]=======================

//Neopixles
#include <Adafruit_NeoPixel.h>

//====[ PIN NUMBERS ]=====================

//Neopixles
#define NEOPIN_INPUT    13

//Servomotor: Gripper
#define gripper 7

//Electromotors
#define motorLeftForward 11     // b1
#define motorLeftBackward 5     // a1
#define motorRightForward 6     // a2
#define motorRightBackward 10   // b2

//Ultrasonic sensor
#define rightTrigger 9
#define rightEcho 8
#define frontTrigger 12
#define frontEcho 4

//rotation sensor
#define leftWheelSensor 3
#define rightWheelSensor 2

//====[ VARIABLES ]=======================

//Neopixles
#define WHITE            255, 255, 255
#define RED             0  , 255, 0
#define GREEN           255, 0  , 0
#define BLUE            0  , 0  , 255
#define LIGHT_GREEN     255, 155, 0
#define YELLOW          255, 255, 0
#define OFF             0  , 0  , 0
#define NEO_PIXNUMBER   4                       // number of neopixle LEDs
Adafruit_NeoPixel neoPixel(NEO_PIXNUMBER, NEOPIN_INPUT, NEO_GRB + NEO_KHZ800);

//Servomotor: Gripper
#define gripperOpenPulse 1000
#define gripperClosePulse 500
#define gripperPulseRepeat 20
#define gripperToggleThing 1000

//Ultrasonic sensors
#define SPEED_OF_SOUND 0.034                      // in cm/s
#define NUM_MEASUREMENTS 5                        // average of 5 readings
long measurementsFront[NUM_MEASUREMENTS] = {0};
long measurementsRight[NUM_MEASUREMENTS] = {0};
static long distanceFront;                        // 0 for front sensor
static long distanceRight;                        // 1 for right sensor
static long averageFront;
static long averageRight;

//Rotation sensors
int leftRotationCount =  0;
int rightRotationCount = 0;
int prevLeftRotationCount = leftRotationCount;
int prevRightRotationCount = rightRotationCount;

// Declare an enum for the states
enum RobotState {
    Moving,
    Stopped,
    BackingUp,
};

RobotState robotState = Stopped;
unsigned long moveStartTime = 0;

//Line sensors
int IR1;
int IR2;
int IR3;
int IR4;
int IR5;
int IR6;
int IR7;
int IR8;

float distance, duration; // declared 2 floats in 1 line for organization purposes
const int echoInterval = 245;
unsigned long time;

unsigned long startTime = 0;
bool pickedUpObject = false;
bool raceFinished = false;
bool testing = false;

unsigned long squareStartTime = 0; // Variable to store the time when the robot first detects the square
unsigned long squareEndTime = 0; // Variable to store the time when the robot first detects the square
bool onSquare = false; // Flag to track whether the robot is on the square
bool onSquareStart = false; // Flag to track whether the robot is on the square

//====[ SETUP ]===========================

void setup(){
  Serial.begin(9600);
  
  //Electromotors
  pinMode(motorLeftForward,INPUT); 
  pinMode(motorLeftBackward,INPUT);
  pinMode(motorRightForward,INPUT);
  pinMode(motorRightBackward,INPUT);

  //Servomotor: Gripper
  pinMode(gripper,INPUT);
  
  //Rotation sensors
  pinMode(leftWheelSensor,INPUT);
  pinMode(rightWheelSensor,INPUT);
  attachInterrupt(digitalPinToInterrupt(3),countLeftSensor,CHANGE);
  attachInterrupt(digitalPinToInterrupt(2),countRightSensor,CHANGE);
  
  //Ultrasonic sensors
  pinMode(frontTrigger,OUTPUT);
  pinMode(frontEcho,INPUT);
  pinMode(rightTrigger,OUTPUT);
  pinMode(rightEcho,INPUT);

  //Neopixles
  lightsOff();

  //Gripper
  pinMode(gripper, OUTPUT);
  digitalWrite(gripper, LOW);
}

//Line sensor
void setupIRSensors(){
 pinMode(A0, INPUT);
 pinMode(A1, INPUT);
 pinMode(A2, INPUT);
 pinMode(A3, INPUT);
 pinMode(A4, INPUT);
 pinMode(A5, INPUT);
 pinMode(A6, INPUT);
 pinMode(A7, INPUT);
}

void readSensors(){
  IR1 = analogRead(A0);
  IR2 = analogRead(A1);
  IR3 = analogRead(A2);
  IR4 = analogRead(A3);
  IR5 = analogRead(A4);
  IR6 = analogRead(A5);
  IR7 = analogRead(A6);
  IR8 = analogRead(A7);
}

//====[ LOOP ]============================

void loop() {
  distanceFront = measureDistance(0);
  distanceRight = measureDistance(1);
  averageFront = averageDistanceFront();
  averageRight = averageDistanceRight();
  
//  // If there's a wall in front of the robot
//  if (averageFront < 25) {
//      // If there's also a wall on the right, turn left
//      if (averageRight < 25) {
//          left();
//      }
//      // If there's no wall on the right, turn right
//      else {
//          right();
//      }
//  }
//  // If there's no wall in front of the robot
//    else {
//      // If there's a wall on the right, move forward
//      if (averageRight < 25) {
//          forwardCM(10);
//          fixLeft();
//      }
//      // If there's no wall on the right, turn right
//      else {
//          right();
//      }
//    }

  for (int i = 0; i < 8; i++)
  {
     setGripper(gripperClose Pulse);
  }
}

//====[ FUNCTIONS ]=======================

//Rotation sensor
void countLeftSensor() {
  leftRotationCount++;
}

void countRightSensor() {
  rightRotationCount++;
}

//Ultrasonic sensor
long measureDistance(int sensor) {
    int triggerPin, echoPin;
    long* measurements;
    if (sensor == 0) { // 0 for front sensor
        triggerPin = frontTrigger;
        echoPin = frontEcho;
        measurements = measurementsFront;
    } else { // 1 for right sensor
        triggerPin = rightTrigger;
        echoPin = rightEcho;
        measurements = measurementsRight;
    }

    // Send a 10 microsecond pulse.
    digitalWrite(triggerPin, HIGH);
    wait(1);
    digitalWrite(triggerPin, LOW);

    // Measure the time for the echo.
    long duration = pulseIn(echoPin, HIGH);

    // Calculate the distance.
    long distance = (duration * SPEED_OF_SOUND) / 2;

    // Shift the old measurements and add the new one.
    for (int i = 0; i < NUM_MEASUREMENTS - 1; i++) {
        measurements[i] = measurements[i + 1];
    }
    measurements[NUM_MEASUREMENTS - 1] = distance;
    
    return distance;
}

long averageDistanceFront() {
    long sum = 0;
    for (int i = 0; i < NUM_MEASUREMENTS; i++) {
        sum += measurementsFront[i];
    }
    return round(static_cast<float>(sum) / NUM_MEASUREMENTS);
}

long averageDistanceRight() {
    long sum = 0;
    for (int i = 0; i < NUM_MEASUREMENTS; i++) {
        sum += measurementsRight[i];
    }
    return round(static_cast<float>(sum) / NUM_MEASUREMENTS);
}

//Electromotors
void stopMotors(){
    analogWrite(motorLeftForward,0);
    analogWrite(motorRightForward,0);
    analogWrite(motorLeftBackward,0);
    analogWrite(motorRightBackward,0);
    // Set robotState to Stopped when stopMotors is called
    robotState = Stopped;
}

void forward()
{
    analogWrite(motorLeftForward,226);
    analogWrite(motorRightForward,255);
    // Set robotState to Moving when the robot moves
    robotState = Moving;
}

void forwardCM(int distance)
{
    unsigned long startTime = millis();
    unsigned long timeout = distance * 10; 

    while((rightRotationCount < distance*2) && (leftRotationCount < distance*2))
    {
        if (millis() - startTime > timeout) {
            Serial.println("Timeout reached, stopping motors");
            break;
        }
        forward();
    }
    rightRotationCount = 0;
    leftRotationCount = 0;
    robotState = Moving;
}

void backwards()
{
    analogWrite(motorLeftBackward, 237);
    analogWrite(motorRightBackward, 255);
    // Set robotState to Moving when the robot moves
    robotState = Moving;
}

void right()
{
    Serial.println("right");
    unsigned long startTime = millis();
    unsigned long timeout = 690;

    // Start turning right
    analogWrite(motorLeftBackward, 189);
    analogWrite(motorRightForward, 204);

    // Keep turning until timeout
    while(millis() - startTime < timeout);

    // Stop the motors
    stopMotors();
    distanceFront = measureDistance(0);
    distanceRight = measureDistance(1);
    averageFront = averageDistanceFront();
    averageRight = averageDistanceRight();
    wait(1000);
    // Set robotState to Moving when the robot moves
    robotState = Moving;
}

void wait(unsigned long duration) {
    stopMotors(); // Call the function to stop the motors
    unsigned long startTime = millis();
    while(millis() - startTime < duration);
    robotState = Stopped;
}

void fixLeft()
{
    analogWrite(motorLeftForward, 235);
    analogWrite(motorRightForward, 235);
    delay(5);
    // Set robotState to Moving when the robot moves
    robotState = Moving;
}

void left()
{
    Serial.println("left");
    unsigned long startTime = millis();
    unsigned long timeout = 690;

    // Start turning left
    analogWrite(motorLeftForward, 189);
    analogWrite(motorRightBackward, 204);

    // Keep turning until timeout
    while(millis() - startTime < timeout);

    // Stop the motors
    stopMotors();
    distanceFront = measureDistance(0);
    distanceRight = measureDistance(1);
    averageFront = averageDistanceFront();
    averageRight = averageDistanceRight();
    wait(1000);
    // Set robotState to Moving when the robot moves
    robotState = Moving;
}

bool isStuck() {
    return (leftRotationCount == prevLeftRotationCount || rightRotationCount == prevRightRotationCount) && robotState == Moving && (millis() - moveStartTime > 1000);
}

void backup() {
    wait(100);
    backwards();
    delay(300);
    stopMotors();
    robotState = BackingUp;
}

void avoidStuck() {
    if (isStuck()) {
        backup();
    }
    prevLeftRotationCount = leftRotationCount;
    prevRightRotationCount = rightRotationCount;
}

//Neopixles
void lightsOff() {
    for (int i = 0; i < NEO_PIXNUMBER; i++) {
        neoPixel.setPixelColor(i,neoPixel.Color(OFF));
        neoPixel.show();
    }
}


//Gripper
void setServoAngle(int angle) {
    int pulseWidth = map(angle, 0, 180, 0, 255); 
    analogWrite(gripper, pulseWidth);
}

//GRIPPER TEST FUNCTIONS
void setGripper(int pulse) {
  static unsigned long timer;
  static int pulse1;
  if(pulse > 0)
  {
    pulse1 = pulse;
  }
  if (millis() > timer)
  {
    digitalWrite(gripper, HIGH);
    delayMicroseconds(pulse);
    digitalWrite(gripper, LOW);
    timer = millis() + gripperPulseRepeat;
  }
}

//Start and End functions... WIP
void start()
{
  readSensors();
  if (IR1 > 800 && IR2 > 800 && IR3 > 800 && IR6 > 800 && IR7 > 800 && IR8 > 800) {
        if (!onSquareStart) { // If the robot just entered the square
          onSquareStart = true; // Set the flag to true
          squareStartTime = millis(); // Record the start time
    } else if (millis() - squareStartTime > 200) { // If the robot has been on the square for more than 2 seconds
      stopMotors();
      setServoAngle(85);
      wait(800); // Wait for 1 second
      // Make a left turn
      left();
      pickedUpObject = true; // Set the flag to indicate object picked up
    }
  } else {
    // If the robot is not on the square, reset the flag
    onSquareStart = false;
         }
}

void endRace()
{
  readSensors();
  if (IR1 > 800 && IR2 > 800 && IR3 > 800 && IR6 > 800 && IR7 > 800 && IR8 > 800) {
    if (!onSquare) { // If the robot just entered the square
      onSquare = true; // Set the flag to true
      squareEndTime = millis(); // Record the start time
    } else if (millis() - squareEndTime > 75) { // If the robot has been on the square for more than 2 seconds
      stopMotors();
      // Drop the object
      setServoAngle(130); // Assuming this is how you open the grip to drop the object
      wait(300); // Wait for 1 second to ensure the grip opens fully
      backup(); // Move backward after dropping the object
      raceFinished = true; // Set the raceFinished flag to true
    }
  } else {
    // If the robot is not on the square, reset the flag
    onSquare = false;
         }
}
