#include <Adafruit_NeoPixel.h>

//Neopixels
#define WHITE            255, 255, 255
#define RED             0  , 255, 0
#define GREEN           255, 0  , 0
#define BLUE            0  , 0  , 255
#define LIGHT_GREEN     255, 155, 0
#define YELLOW          255, 255, 0
#define OFF             0  , 0  , 0
#define NEOPIN_INPUT    13
#define NEO_PIXNUMBER   4
Adafruit_NeoPixel neoPixel(NEO_PIXNUMBER, NEOPIN_INPUT, NEO_GRB + NEO_KHZ800);

//Gripper
#define gripper 7
#define gripperOpenPulse 1000
#define gripperClosePulse 971
#define gripperPulseRepeat 10
// TODO: swap pins to 5,6 10, 11
//wheels
#define motorLeftForward 11 // b1
#define motorLeftBackward 5 // a1
#define motorRightForward 6 // a2
#define motorRightBackward 10 // b2

//ultrasonic
#define rightTrigger 9
#define rightEcho 8
#define frontTrigger 12
#define frontEcho 4
#define SPEED_OF_SOUND 0.034 // in cm/s
#define NUM_MEASUREMENTS 5
long measurementsFront[NUM_MEASUREMENTS] = {0};
long measurementsRight[NUM_MEASUREMENTS] = {0};
static long distanceFront; // 0 for front sensor
static long distanceRight; // 1 for right sensor
static long averageFront;
static long averageRight;

//rotation sensor
#define leftWheelSensor 3
#define rightWheelSensor 2
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

//setup
void setup(){
  Serial.begin(9600);
  //Pins
  pinMode(motorLeftForward,INPUT); 
  pinMode(motorLeftBackward,INPUT);
  pinMode(motorRightForward,INPUT);
  pinMode(motorRightBackward,INPUT);
  pinMode(gripper,INPUT);
  //sensor
  pinMode(leftWheelSensor,INPUT);
  pinMode(rightWheelSensor,INPUT);
  attachInterrupt(digitalPinToInterrupt(3),countLeftSensor,CHANGE);
  attachInterrupt(digitalPinToInterrupt(2),countRightSensor,CHANGE);
  //ultrasonic
  pinMode(frontTrigger,OUTPUT);
  pinMode(frontEcho,INPUT);
  pinMode(rightTrigger,OUTPUT);
  pinMode(rightEcho,INPUT);
  lightsOff();
}

void loop()
{
    distanceFront = measureDistance(0);
    distanceRight = measureDistance(1);
    averageFront = averageDistanceFront();
    averageRight = averageDistanceRight();
    
    // If there's a wall in front of the robot
    if (averageFront < 25) {
        wait(100);
        // If there's also a wall on the right, turn left
        if (averageRight < 25) {
            left();
            wait(100);
        }
        // If there's no wall on the right, turn right
        else {
            right();
            wait(100);
        }
    }
    // If there's no wall in front of the robot
      else {
        wait(100);
        // If there's a wall on the right, move forward
        if (averageRight < 25) {
            forwardCM(10);
            fixLeft();
            wait(100);
        }
        // If there's no wall on the right, turn right
        else {
            right();
            wait(100);
       }
//    }
}
}

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
    distanceFront = measureDistance(0);
    distanceRight = measureDistance(1);
    averageFront = averageDistanceFront();
    averageRight = averageDistanceRight();
    
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
    analogWrite(motorLeftBackward, 237 * 0.8);
    analogWrite(motorRightForward, 255 * 0.8);

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
    while(millis() - startTime < duration) {
      distanceFront = measureDistance(0);
      distanceRight = measureDistance(1);
      averageFront = averageDistanceFront();
      averageRight = averageDistanceRight();
    // Wait for the specified duration
    }
    robotState = Stopped;
}

void fixLeft()
{
    analogWrite(motorLeftForward, 235);
    analogWrite(motorRightForward, 235);
    delay(10);
    // Set robotState to Moving when the robot moves
    robotState = Moving;
}

void left()
{
    Serial.println("left");
    unsigned long startTime = millis();
    unsigned long timeout = 690;

    // Start turning left
    analogWrite(motorLeftForward, 237 * 0.8);
    analogWrite(motorRightBackward, 255 * 0.8);

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

void lightsOff() {
    for (int i = 0; i < NEO_PIXNUMBER; i++) {
        neoPixel.setPixelColor(i,neoPixel.Color(OFF));
        neoPixel.show();
    }
}

void countLeftSensor()
{
  leftRotationCount++;
}

void countRightSensor()
{
  rightRotationCount++;
}
