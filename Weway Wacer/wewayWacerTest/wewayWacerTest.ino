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
#define NEOPIN 13

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
#define WHITE           255, 255, 255
#define RED             0, 255, 0
#define GREEN           255, 0, 0
#define BLUE            0, 0, 255
#define LIGHT_GREEN     255, 155, 0
#define YELLOW          255, 255, 0
#define CALIBRATE       0, 200, 190
#define OFF             0, 0, 0
#define NEO_PIXNUMBER   4                       // number of neopixle LEDs
Adafruit_NeoPixel neoPixel(NEO_PIXNUMBER, NEOPIN, NEO_GRB + NEO_KHZ800);

//Servomotor: Gripper
#define gripperOpenPulse 1400
#define gripperClosePulse 1000
#define gripperPulseRepeat 20

//Ultrasonic sensors
#define SPEED_OF_SOUND 0.034                      // in cm/s
static long distanceFront;                       
static long distanceRight;                        

//Rotation sensors
int leftRotationCount =  0;
int rightRotationCount = 0;
int prevLeftRotationCount = leftRotationCount;
int prevRightRotationCount = rightRotationCount;

// Declare states
enum RobotState {
    Moving,
    Stopped,
    BackingUp,
};

//Line sensors
#define IR_THRESHOLD_HIGH     800
#define IR_THRESHOLD_AVERAGE  600
#define IR_THRESHOLD_LOW      500
#define IRSENSORS             8

int IR1;
int IR2;
int IR3;
int IR4;
int IR5;
int IR6;
int IR7;
int IR8;

//Additonal variables for LineFollow team functions
RobotState robotState = Stopped;
unsigned long moveStartTime = 0;

float distance, duration; // declared 2 floats in 1 line for organization purposes;
unsigned long time;

unsigned long startTime = 0;
bool pickedUpObject = false;
bool raceFinished = false;

unsigned long squareStartTime = 0; // Variable to store the time when the robot first detects the square
unsigned long squareEndTime = 0; // Variable to store the time when the robot first detects the square
bool onSquare = false; // Flag to track whether the robot is on the square
bool onSquareStart = false; // Flag to track whether the robot is on the square

bool insideMaze = false; //Flag to help with the linefollow

//====[ SETUP ]===========================

void setup()
{
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
  attachInterrupt(digitalPinToInterrupt(3),countLeftSensor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2),countRightSensor, CHANGE);
  
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

void readSensors()
{
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

void loop() 
{
  while (!raceFinished)
  {
    if (getDistanceFront() > 30 && !pickedUpObject)
    {
      stopMotors();
    }
    else if (getDistanceFront() < 30 && !pickedUpObject) //start sequence
    {
      for (int i = 0; i < 8; i++)
      {
        setGripper(gripperOpenPulse);
      }
      wait(1000); // wait 1 sec for the other robot to move out of the way
      forward();
      startSequence();
      if (!insideMaze)
      {
        isInsideMaze();
        followLine();
      }
    }
    else if (pickedUpObject == true && !insideMaze) //end sequence
    {
     followLine();
     endSequence(); 
    }
    else //maze logic
    {
      if (getDistanceRight > 30)
      {
        wait(500);
        right();
      }
      else if (getDistanceFront < 15 && getDistanceRight < 15)
      {
        wait(500);
        left();
        if (getDistanceFront < 30)
        {
          left();
        }
      }
      else if (isStuck())
      {
        backup();
        wait(500);
      }
      else
      {
        driveStraight();
      }
    }
  }
}

//====[ FUNCTIONS ]=======================

void wait(unsigned long duration) 
{
  stopMotors(); // Call the function to stop the motors
  unsigned long startTime = millis();
  while (millis() - startTime < duration);
  robotState = Stopped;
}

//Rotation sensor
void countLeftSensor() 
{
  leftRotationCount++;
}

void countRightSensor() 
{
  rightRotationCount++;
}

//Ultrasonic sensor
int getDistanceFront()
{
  digitalWrite(frontTrigger, LOW);
  delay(2);
  
  digitalWrite(frontTrigger, HIGH);
  delay(200);
  
  digitalWrite(frontTrigger, LOW);
  duration = pulseIn(frontEcho, HIGH);
  distanceFront = (duration * 0.034)/2;
  
  Serial.print("front: ");
  Serial.println(distanceFront);

  return distanceFront;
}

int getDistanceRight()
{
  digitalWrite(rightTrigger, LOW);
  delay(2);
  
  digitalWrite(rightTrigger, HIGH);
  delay(200);
  
  digitalWrite(rightTrigger, LOW);
  duration = pulseIn(rightEcho, HIGH);
  distanceRight = (duration * 0.034)/2;
  
  Serial.print("right: ");
  Serial.println(distanceRight);

  return distanceRight;
}

//Electromotors
void stopMotors()
{
  analogWrite(motorLeftForward,0);
  analogWrite(motorRightForward,0);
  analogWrite(motorLeftBackward,0);
  analogWrite(motorRightBackward,0);
  // Set robotState to Stopped when stopMotors is called
  robotState = Stopped;
  signalStop();
  Serial.print("stopmotors\n");
}

void forward()
{
  analogWrite(motorLeftForward, 226);
  analogWrite(motorRightForward, 255);
  // Set robotState to Moving when the robot moves
  robotState = Moving;
  signalForward();
  Serial.print("forward\n");
}

void slowForward()
{
  analogWrite(motorLeftForward, 220);
  analogWrite(motorRightForward, 245);
  robotState = Moving;
  signalForward();
  Serial.print("slowForward\n");
}

void sutterStep()
{
  analogWrite(motorLeftForward, 226);
  analogWrite(motorRightForward, 255);
  // Set robotState to Moving when the robot moves
  robotState = Moving;
  signalForward();
  Serial.print("stutter\n");
  wait(500);  
}

void forwardCM(int distance)
{
  unsigned long startTime = millis();
  unsigned long timeout = distance * 10; 

  while ((rightRotationCount < distance*2) && (leftRotationCount < distance*2))
  {
      if (millis() - startTime > timeout) {
          Serial.println("Timeout reached, stopping motors\n");
          break;
      }
      forward();
  }
  rightRotationCount = 0;
  leftRotationCount = 0;
  robotState = Moving;
  signalForward();
  Serial.print("forwardCM\n");
}

void backward()
{
  analogWrite(motorLeftBackward, 237);
  analogWrite(motorRightBackward, 255);
  
  // Set robotState to Moving when the robot moves
  robotState = Moving;
  signalReverse();
  Serial.print("backward");
}

void right()
{
  unsigned long startTime = millis();
  unsigned long timeout = 690;

  // Start turning right
  analogWrite(motorLeftBackward, 189);
  analogWrite(motorRightForward, 204);

  // Keep turning until timeout
  while (millis() - startTime < timeout);

  // Stop the motors
  stopMotors();
  wait(1000);
  // Set robotState to Moving when the robot moves
  robotState = Moving;
  signalRight();
  Serial.println("right\n");
}

void fixRight()
{
  analogWrite(motorLeftForward, 200);
  analogWrite(motorRightForward, 250);
  robotState = Moving;
  signalRight();
  Serial.print("fixRight\n");
}

void left()
{
  unsigned long startTime = millis();
  unsigned long timeout = 690;

  // Start turning left
  analogWrite(motorLeftForward, 189);
  analogWrite(motorRightBackward, 204);

  // Keep turning until timeout
  while(millis() - startTime < timeout);

  // Stop the motors
  stopMotors();
  wait(1000);
  // Set robotState to Moving when the robot moves
  robotState = Moving;
  signalLeft();
  Serial.print("left\n");
}

void fixLeft()
{
  analogWrite(motorLeftForward, 235);
  analogWrite(motorRightForward, 235);
  delay(5);
  // Set robotState to Moving when the robot moves
  robotState = Moving;
  signalLeft();
  Serial.print("fixLeft\n");
}

void backup() 
{
  backward();
  robotState = BackingUp;
  signalReverse();
}

bool isStuck() 
{
  return (leftRotationCount == prevLeftRotationCount 
  || rightRotationCount == prevRightRotationCount) 
  && robotState == Moving 
  && (millis() - moveStartTime > 1000);
}

void avoidStuck() 
{
  if (isStuck()) 
  {
      backup();
  }
  prevLeftRotationCount = leftRotationCount;
  prevRightRotationCount = rightRotationCount;
}

void driveStraight()
{
  if (getDistanceRight() > 5)
  {
    fixRight();
  }
  else if (getDistanceRight < 5)
  {
    fixLeft();
  }
  else
  {
    forward();
  }
}

//Gripper
void setGripper(int pulse) 
{
  static unsigned long timer;
  static int pulse1;
  if (pulse > 0)
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


//Start and end sequences
void startSequence()
{
  readSensors();
  if (IR1 > IR_THRESHOLD_HIGH && IR2 > IR_THRESHOLD_HIGH && IR3 > IR_THRESHOLD_HIGH && 
      IR6 > IR_THRESHOLD_HIGH && IR7 > IR_THRESHOLD_HIGH && IR8 > IR_THRESHOLD_HIGH) 
  {
      if (!onSquareStart) // If the robot just entered the square
      { 
        onSquareStart = true; // Set the flag to true
        squareStartTime = millis(); // Record the start time
      } 
      else if (millis() - squareStartTime > 200) // If the robot has been on the square for more than 2 seconds
      { 
          stopMotors();
          for (int i = 0; i < 8; i++)
          {
            setGripper(gripperClosePulse);
            Serial.println("gripperClose\n");
          }
          wait(1000); // Wait for 1 second
          left(); // Make a left turn
          pickedUpObject = true; // Set the flag to indicate object picked up
      }
  } 
  else 
  {
    onSquareStart = false; // If the robot is not on the square, reset the flag
  }
}

void endSequence()
{
  readSensors();
  if (IR1 > IR_THRESHOLD_HIGH && IR2 > IR_THRESHOLD_HIGH && IR3 > IR_THRESHOLD_HIGH && 
      IR6 > IR_THRESHOLD_HIGH && IR7 > IR_THRESHOLD_HIGH && IR8 > IR_THRESHOLD_HIGH) 
  {
    if (!onSquare) // If the robot just entered the square
    { 
      onSquare = true; // Set the flag to true
      squareEndTime = millis(); // Record the start time
    } 
    else if (millis() - squareEndTime > 75) // If the robot has been on the square for more than 2 seconds
    { 
      stopMotors();
      for (int i = 0; i < 8; i++)
      {
        setGripper(gripperOpenPulse);
        Serial.print("gripperOpen\n");
      } 
      wait(1000); // Wait for 1 second 
      backup(); // Move backward after dropping the object
      raceFinished = true; // Set the raceFinished flag to true
    }
  } 
  else 
  {
    onSquare = false; // If the robot is not on the square, reset the flag
  }
}

//Line follow
void isInsideMaze()
{
  readSensors();
  if (IR1 < IR_THRESHOLD_HIGH && IR2 < IR_THRESHOLD_HIGH && IR3 < IR_THRESHOLD_HIGH && IR4 < IR_THRESHOLD_HIGH &&
      IR5 < IR_THRESHOLD_HIGH && IR6 < IR_THRESHOLD_HIGH && IR7 < IR_THRESHOLD_HIGH && IR8 < IR_THRESHOLD_HIGH)
  {
    insideMaze = true;
    Serial.print("insideMaze");
  }
}

void followLine()
{
  readSensors();
  if (IR3 > IR_THRESHOLD_AVERAGE && IR4 > IR_THRESHOLD_AVERAGE)
  {
    fixLeft();
  }
  else if (IR5 > IR_THRESHOLD_AVERAGE && IR6 > IR_THRESHOLD_AVERAGE)
  {
    fixRight();
  }
  else if (IR2 > IR_THRESHOLD_AVERAGE && IR3 > IR_THRESHOLD_AVERAGE)
  {
    fixLeft();
  }
  else if (IR6 > IR_THRESHOLD_AVERAGE && IR7 > IR_THRESHOLD_AVERAGE)
  {
    fixRight();
  }
  else if (IR2 > IR_THRESHOLD_AVERAGE && IR1 > IR_THRESHOLD_AVERAGE)
  {
    fixLeft();
  }
  else if (IR7 > IR_THRESHOLD_AVERAGE && IR8 > IR_THRESHOLD_AVERAGE)
  {
    fixRight();
  }
}

//Neopixles
void lightsOff() 
{
  for (int i = 0; i < NEO_PIXNUMBER; i++) 
  {
      neoPixel.setPixelColor(i,neoPixel.Color(OFF));
  }
  neoPixel.show();
}

void signalStarting()
{
  neoPixel.begin();
  for (int i = 0; i < NEO_PIXNUMBER; i++)
  {
    neoPixel.setPixelColor(i, neoPixel.Color(GREEN)); 
  }
  neoPixel.show();
  delay(150);
  for (int i = 0; i < NEO_PIXNUMBER; i++) 
  {
      neoPixel.setPixelColor(i,neoPixel.Color(OFF));
  }
  neoPixel.show();
  for (int i = 0; i < NEO_PIXNUMBER; i++)
  {
    neoPixel.setPixelColor(i, neoPixel.Color(GREEN)); 
  }
  neoPixel.show();
  delay(150);
  for (int i = 0; i < NEO_PIXNUMBER; i++) 
  {
      neoPixel.setPixelColor(i,neoPixel.Color(OFF));
  }
  neoPixel.show();
}

void signalForward()
{
  neoPixel.begin();
  for (int i = 0; i < NEO_PIXNUMBER; i++)
  {
    neoPixel.setPixelColor(i, neoPixel.Color(GREEN));
  }
  neoPixel.show();
}

void signalReverse()
{
  neoPixel.begin();
  for (int i = 0; i < NEO_PIXNUMBER; i++)
  {
    neoPixel.setPixelColor(i, neoPixel.Color(YELLOW));
  }
  neoPixel.show();
}

void signalWait()
{
  neoPixel.begin();
  for (int i = 0; i < NEO_PIXNUMBER; i++)
  {
    neoPixel.setPixelColor(i, neoPixel.Color(WHITE));
  }
  neoPixel.show();
}

void signalCalibrate()
{
  neoPixel.begin();
  for (int i = 0; i < NEO_PIXNUMBER; i++)
  {
    neoPixel.setPixelColor(i, neoPixel.Color(CALIBRATE));
  }
  neoPixel.show();
}

void signalStop()
{
  neoPixel.begin();
  for (int i = 0; i < NEO_PIXNUMBER; i++)
  {
    neoPixel.setPixelColor(i, neoPixel.Color(RED));
  }
  neoPixel.show();
}

void signalLeft()
{
  neoPixel.begin();
  for (int i = 0; i < NEO_PIXNUMBER; i++)
  {
    neoPixel.setPixelColor(i, neoPixel.Color(GREEN));
  }
  neoPixel.setPixelColor(2, neoPixel.Color(BLUE));
  neoPixel.setPixelColor(1, neoPixel.Color(BLUE));
  neoPixel.show();
}

void signalRight()
{
  neoPixel.begin();
  for (int i = 0; i < NEO_PIXNUMBER; i++)
  {
    neoPixel.setPixelColor(i, neoPixel.Color(GREEN));
  }
  neoPixel.setPixelColor(3, neoPixel.Color(BLUE));
  neoPixel.setPixelColor(0, neoPixel.Color(BLUE));
  neoPixel.show();
}
