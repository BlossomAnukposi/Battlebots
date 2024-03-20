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

//Gripper
#define gripper 7
#define gripperOpenPulse 1000
#define gripperClosePulse 971
#define gripperPulseRepeat 10
// TODO: swap pins to 5,6 10, 11
//wheels
#define motorLeftForward 11 // a2
#define motorLeftBackward 5 // a1
#define motorRightForward 6 // b1
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

//rotation sensor
#define leftWheelSensor 3
#define rightWheelSensor 2
int leftRotationCount =  0;
int rightRotationCount = 0;


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
}

void loop()
{
    long distanceFront = measureDistance(0); // 0 for front sensor
    long distanceRight = measureDistance(1); // 1 for right sensor
    long averageFront = averageDistanceFront();
    long averageRight = averageDistanceRight();
    if (averageFront <= 30) {
        // An object is detected within 30 cm on average.
        stopMotors();
    } else {
        forward();
    }

    if (averageRight <= 8)
    {
      fixLeft();
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
    delay(10);
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

void forward()
{
    analogWrite(motorLeftForward,235);
    analogWrite(motorRightForward,255);
}

void backwards()
{
    analogWrite(motorLeftBackward, 237);
    analogWrite(motorRightBackward, 255);
}
void right()
{
    analogWrite(motorLeftBackward, 237);
    analogWrite(motorRightForward, 255);
    delay(460);
    stopMotors();
}

void fixLeft()
{
    analogWrite(motorLeftForward, 255);
    analogWrite(motorRightForward, 255);
    delay(1000);
    analogWrite(motorLeftForward,235);
    analogWrite(motorRightForward,255);
}

void left()
{
    analogWrite(motorLeftForward, 233);
    analogWrite(motorRightBackward, 255);
    delay(460);
    stopMotors();
}

void stopMotors(){
    analogWrite(motorLeftForward,0);
    analogWrite(motorRightForward,0);
    analogWrite(motorLeftBackward,0);
    analogWrite(motorRightBackward,0);
}

void countLeftSensor()
{
  leftRotationCount++;
}

void countRightSensor()
{
  rightRotationCount++;
}
