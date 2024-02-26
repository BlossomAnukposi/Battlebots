/****************************************
 ***************Group-G******************
 ****Jia Men Lam and Cristian Trifan*****
 ***********BattleBots:Purpie************
 ****************************************/

// ==========[ LIBRARIES ] ====================
#include <Servo.h>
#include <NewPing.h>
#include <Adafruit_NeoPixel.h>

// ==========[ PIN NUMBERS] ==============================
// NeoPixels
const int neoPin_NO = 7; // Define the pin for NeoPixel strip 1
const int neoPin_NI = 8; // Define the pin for NeoPixel strip 2
const int numNeoPix = 4; // Number of pixels in each strip

// Gripper
const int servoPin = 12; // define the servo pin

//Ultrasonic sensor
const int trigPin = 4; //Trigger pin of ultrasonic sensor
const int echoPin = 3; // Echo pin of ultrasonic sensor


// Motor A
int leftFmp1 = 11;
int leftBmp2 = 10;
int motorPin1 = 11; // Connected to pin 2 on Arduino Nano
int motorPin2 = 10; // Connected to pin 3 on Arduino Nano

// Motor B
int rightFmp3 = 6;
int rightBmp4 = 5;
int motorPin3 = 6; // Connected to pin 4 on Arduino Nano
int motorPin4 = 5; // Connected to pin 5 on Arduino Nano


// ==========[ VARIABLES ]=====================
//Gripper
const int closedGrip = 40; // Close gripper position
const int openGrip = 120; // Open gripper position
Servo gripperServo; // define servo object
bool gripperOpen = true;

//Ultrasonic sensor
const int max_distance = 300; //maximum distance to measure in centimeters
NewPing sonar(trigPin, echoPin, max_distance); // Create a NewPing object

// Create two instances of Adafruit_NeoPixel
Adafruit_NeoPixel strip_NO(numNeoPix, neoPin_NO, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_NI(numNeoPix, neoPin_NI, NEO_GRB + NEO_KHZ800);


//LineSensors
int rightLS; // right line sensor
int leftLS; // left line sensor

int IR1;
int IR2;
int IR3;
int IR4;
int IR5;
int IR6;
int IR7;
int IR8;


//defining variables for fine tuning on line following
int leftValue = 0;
int rightValue = 0;
int adjustSpeed = 0;

// ==========[ SETUP ]=====================

void setup() {

  Serial.begin(9600); // Initializes the serial communication
  setupIRSensors();
  setupMotors();
  
  // Attach the servo to the pin
  gripperServo.attach(servoPin);

  // Initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initially open the gripper
  gripperServo.write(openGrip);
}

// ==========[ LOOP ]=====================
  
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
  if (distance < 4 && gripperOpen) {
    gripperServo.write(closedGrip);
    gripperOpen = false;
    // Adjust delay as needed to ensure stable gripping
    delay(1000);
  }
  // If no object is detected, open the gripper
  else if (distance >= 3 && !gripperOpen) {
    gripperServo.write(openGrip);
    gripperOpen = true;
    // Adjust delay as needed to ensure stable release
    delay(1000);
  }

  // Optional: Add a small delay to reduce loop frequency
  // delay(100);
}
// ==========[ FUNCTIONS ]=====================

void goForward() {
  analogWrite(leftFmp1, 255);
  analogWrite(rightFmp3, 240);
}

void left(){
  analogWrite(leftFmp1, 255);
  analogWrite(rightFmp3, 180);
}

void right(){
  analogWrite(leftFmp1, 180);
  analogWrite(rightFmp3, 255);
}

void followLine(){
  readSensors();
  if(IR3 > 500 && IR4 > 500){
    analogWrite(leftFmp1, 255);
    analogWrite(rightFmp3, 120);
  }else if(IR5 > 500 && IR6 > 500){
    analogWrite(leftFmp1, 120);
    analogWrite(rightFmp3, 255);
  }else if(IR2 > 500 && IR3 > 500){
    analogWrite(leftFmp1, 255);
    analogWrite(rightFmp3, 0);
  }else if(IR6 > 500 && IR7 > 500){
    analogWrite(leftFmp1, 0);
    analogWrite(rightFmp3, 255);
  }else if(IR2 > 500 && IR1 > 500){
    analogWrite(leftFmp1, 255);
    analogWrite(rightFmp3, 0);
  }else if(IR7 > 500 && IR8 > 500){
    analogWrite(leftFmp1, 0);
    analogWrite(rightFmp3, 255);
  }
}

bool squareDetected(){
  if(IR1 > 800 && IR2 > 800 && IR3 > 800 && IR4 > 800 && IR5 > 800 && IR6 > 800 && IR7 > 800 && IR8 > 800){
    return true;
  }
}

void setupMotors(){
  //initializing motor pins
  pinMode(leftFmp1, OUTPUT);
  pinMode(leftBmp2, OUTPUT);
  pinMode(rightFmp3, OUTPUT);
  pinMode(rightBmp4, OUTPUT);
}

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
  IR3 = analogRead(A3);
  IR4 = analogRead(A4);
  IR5 = analogRead(A2);
  IR6 = analogRead(A5);
  IR7 = analogRead(A6);
  IR8 = analogRead(A7);
}
