const int leftB = 10; //motor left - going backwards
const int leftF = 9; // motor left - going forward
const int rightB = 5; //motor right - going backwards
const int rightF = 6; //motor right - going forward

const int servoPin = 11 ; //gripper

#define triggerPin 4 // Define the Arduino pin connected to the sensor's trigger pin
#define echoPin 12 // Define the Arduino pin connected to the sensor's echo pin

int IR1;
int IR2;
int IR3;
int IR4;
int IR5;
int IR6;
int IR7;
int IR8;


const int maxDistance = 20; // 20 cm maximum distance to leave room for error
const int startDistance = 15; // 15 cm maximum distance from the flag
float distance, duration; // declared 2 floats in 1 line for organization purposes
const int echoInterval = 245;

//===============================
//*SETUP*************
//===============================


//===============================
//*FUNCTIONS***********
//===============================
void setServoAngle(int angle) {
    int pulseWidth = map(angle, 0, 180, 0, 255); 
    analogWrite(servoPin, pulseWidth);
}

void goForward() {
  analogWrite(leftF, 255);
  analogWrite(rightF, 240);
  delay(1500);
  stopMotors();
}

void startForward() {
  analogWrite(leftF, 200);
  analogWrite(rightF, 185);
  delay(1500);
  stopMotors();
}

void goCheck() {
  analogWrite(leftF, 200);
  analogWrite(rightF, 180);
  delay(500);
  stopMotors();
}

void left(){
  analogWrite(leftF, 40);
  analogWrite(rightF, 180);
  delay(1000); // Adjust delay for how long you want to move forward
  stopMotors();
}

void right(){
  analogWrite(leftF, 180);
  analogWrite(rightF, 40);
  delay(1000);
}

void go1Sec() {
  analogWrite(leftF, 200);
  analogWrite(rightF, 180);
  delay(600);
  stopMotors();
}

void leftAvoid(){
  analogWrite(leftF, 100);
  analogWrite(rightF, 200);
  delay(1200); // Adjust delay for how long you want to move forward
  stopMotors();
}

void rightAvoid(){
  analogWrite(leftF, 200);
  analogWrite(rightF, 100);
  delay(1000);
}

void moveBackward() {
  // Move both motors backward
  analogWrite(leftB, 255);
  analogWrite(rightB, 255);
  delay(1000); // Adjust delay for how long you want to move backward
  stopMotors();
}

void stopMotors() {
  // Stop both motors
  analogWrite(leftF, 0);
  analogWrite(rightF, 0);
  analogWrite(rightB, 0);
  analogWrite(leftB, 0);
}

void followLine(){
  readSensors();
  if(IR3 > 500 && IR4 > 500){
    analogWrite(leftF, 255);
    analogWrite(rightF, 180);
  }else if(IR5 > 500 && IR6 > 500){
    analogWrite(leftF, 180);
    analogWrite(rightF, 255);
  }else if(IR2 > 500 && IR3 > 500){
    analogWrite(leftF, 255);
    analogWrite(rightF, 80);
  }else if(IR6 > 500 && IR7 > 500){
    analogWrite(leftF, 80);
    analogWrite(rightF, 255);
  }else if(IR2 > 500 && IR1 > 500){
    analogWrite(leftF, 255);
    analogWrite(rightF, 40);
  }else if(IR7 > 500 && IR8 > 500){
    analogWrite(leftF, 40);
    analogWrite(rightF, 255);
  }
}

void distanceReader()
{
  digitalWrite(triggerPin, LOW); // Reset pin
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH); 
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    

    duration = pulseIn(echoPin, HIGH); // Reads pins

    distance = (duration / 2) * 0.0343; // 343 m/s per second as speed of sound
}

void distanceSensor(){
static unsigned long timer;
 if (millis() > timer) 
  {

    distanceReader();

    if (distance <= maxDistance)
      {
        rightAvoid();
        go1Sec();
        leftAvoid();
        goCheck();
        stopMotors();   
      }
    else
      {
        Serial.print(distance);
        Serial.println(" cm");
//        followLine();
      }
       timer = millis() + 200;
    }
}

void setupMotors(){
  //initializing motor pins
  pinMode(leftB, OUTPUT);
  pinMode(leftF, OUTPUT);
  pinMode(rightB, OUTPUT);
  pinMode(rightF, OUTPUT);
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

void setupDistanceSensor(){
    pinMode(echoPin, INPUT);
    pinMode(triggerPin, OUTPUT);
  }
  
void setup(){
  Serial.begin(9600); // Initializes the serial communication
  setupDistanceSensor();
  setupIRSensors();
  setupMotors();
  setServoAngle(130);
}

//===============================
//*LOOP**************
//===============================

unsigned long startTime = 0;
bool pickedUpObject = false;
bool raceFinished = false;
bool testing = false;

unsigned long squareStartTime = 0; // Variable to store the time when the robot first detects the square
bool onSquare = false; // Flag to track whether the robot is on the square


void loop() {
  unsigned long currentTime = millis(); // Get the current time
  // If 3 seconds haven't passed yet, move forward
  if (currentTime - startTime < 150) {
    startForward();
   delay(500); // Wait for 2 seconds
  }
  // After 3 seconds, pick up the object and make a left turn
  else if (!pickedUpObject) {
    // Stop the robot   
    // Close the grip (60 degrees)
    setServoAngle(90);
    delay(1000); // Wait for 1 second
    // Make a left turn
    left();
    delay(500); // Adjust the delay as needed
    pickedUpObject = true; // Set the flag to indicate object picked up
  }
  // After picking up the object and making the left turn, follow the line
  else {      
    if (!raceFinished) {
      followLine();
      distanceSensor();
      
if (IR1 > 900 && IR2 > 900 && IR3 > 900 && IR6 > 900 && IR7 > 900 && IR8 > 900) {
    if (!onSquare) { // If the robot just entered the square
      onSquare = true; // Set the flag to true
      squareStartTime = millis(); // Record the start time
    } else if (millis() - squareStartTime > 80) { // If the robot has been on the square for more than 2 seconds
      stopMotors();
      // Drop the object
      setServoAngle(130); // Assuming this is how you open the grip to drop the object
      delay(500); // Wait for 1 second to ensure the grip opens fully
      moveBackward(); // Move backward after dropping the object
      raceFinished = true; // Set the raceFinished flag to true
    }
  } else {
    // If the robot is not on the square, reset the flag
    onSquare = false;
  }
    }
 }
}
