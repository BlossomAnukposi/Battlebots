// ============= LIBRARIES ===============//
#include <Adafruit_NeoPixel.h>

// neo lights
#define NEOPIN_NO         7 // NEOPIN_OUTPUT
#define NEOPIN_NI         8 // NEOPIN_INPUT
#define NEO_PIXNUMBER     4 
#define BRIGHTNESS_LEVEL 25

// signal colors
#define WHITE           255, 255, 255
#define RED             255,   0,   0
#define GREEN             0, 255,   0
#define LIGHT_GREEN     155, 255,   0
#define BLUE              0,   0, 255
#define YELLOW          255, 255,   0

// servo - gripper
#define GRIPPER_PIN      12
#define GRIPPER_OPEN    120 // ANGLE_OPEN
#define GRIPPER_CLOSE    30 // ANGLE_CLOSED

// ultra sonic sensor
#define TRIGGER_PIN       4
#define ECHO_PIN          9

// motor
#define MOTOR_LEFT_B     10 // MOTOR_LEFT_BACKWARDS
#define MOTOR_LEFT_F     11 // MOTOR_LEFT_FORWARDS
#define MOTOR_RIGHT_B     5// MOTOR_RIGHT_BACKWARDS
#define MOTOR_RIGHT_F     6// MOTOR_RIGHT_FORWARDS

int distance = 0; 


//LineSensors // IR = infrared
int rightLS; 
int leftLS; 
int IR1; 
int IR2;
int IR3;
int IR4;
int IR5;
int IR6;
int IR7;
int IR8;

//variables for fine tuning on line following
int leftValue = 0;
int rightValue = 0;
int adjustSpeed = 0;


Adafruit_NeoPixel strip_NO(NEO_PIXNUMBER, NEOPIN_NO, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_NI(NEO_PIXNUMBER, NEOPIN_NI, NEO_GRB + NEO_KHZ800);


// ============= SETUP ===============//
void setup()
{   
    strip_NO.begin();
    strip_NI.begin();
    pinMode(GRIPPER_PIN, OUTPUT);
    setupMotors();
    setupIRSensors();
}

// ============= LOOP ===============//
void loop ()
{
    followLine();
//    detectObject_openGripper();   
//    digitalWrite(TRIGGER_PIN, LOW);
//    delayMicroseconds(2);
//    digitalWrite(TRIGGER_PIN, HIGH);
//    delayMicroseconds(10);
//    digitalWrite(TRIGGER_PIN, LOW);
//    delayMicroseconds(90);
//    delay(500);
//    setGripperAngle(GRIPPER_CLOSE);
//    delay(500);


  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

    // Measure the echo time to determine distance
  long duration = pulseIn(TRIGGER_PIN, HIGH);
  int distance = duration * 0.034 / 1.5;

//        // Set colors and brightness for NI strip
//    signalRed_all(strip_NI);
//    strip_NI.setBrightness(BRIGHTNESS_LEVEL);
//    strip_NI.show();
//    delay(1000);

      // If an object is within range, close the gripper
//  if (distance < 4 &&) {
//    gripperServo.write(closedPosition);
//    gripperOpen = false;
//    // Adjust delay as needed to ensure stable gripping
//    delay(500);
//  }
//
//  else if (distance >= 9 && !gripperOpen) {
//    gripperServo.write(openPosition);
//    gripperOpen = true;
// 
//    delay(500);
}

// ============= FUNCTIONS ===============//

void moveBackward()
{
    analogWrite(MOTOR_LEFT_B, 255);
    analogWrite(MOTOR_RIGHT_B, 255);
}



void moveForward()
{
    analogWrite(MOTOR_LEFT_F, 255);
    analogWrite(MOTOR_RIGHT_F, 240);
}

void turnLeft()
{
    analogWrite(MOTOR_LEFT_F, 255);
    analogWrite(MOTOR_RIGHT_F, 230);
}


void turnRight()
{
    analogWrite(MOTOR_LEFT_F, 180);
    analogWrite(MOTOR_RIGHT_F, 255);
}

void followLine() 
{
    readSensors();
    if(IR3 > 500 && IR4 > 500) 
    {
        analogWrite(MOTOR_LEFT_F, 255);
        analogWrite(MOTOR_RIGHT_F, 120);
        signalGreenForwardLeft(strip_NO);
        strip_NI.show();
        
    } else if(IR5 > 500 && IR6 > 500) 
    {
        analogWrite(MOTOR_LEFT_F, 120);
        analogWrite(MOTOR_RIGHT_F, 255);
        signalGreenForwardRight(strip_NO);
        strip_NI.show();
        
    } else if(IR2 > 500 && IR3 > 500) {
        analogWrite(MOTOR_LEFT_F, 255); 
        analogWrite(MOTOR_RIGHT_F, 0);
        signalGreenForwardLeft(strip_NO);
        strip_NI.show();
        
    } else if(IR6 > 500 && IR7 > 500) 
    {
        analogWrite(MOTOR_LEFT_F, 0);
        analogWrite(MOTOR_RIGHT_F, 255);
        signalGreenForwardRight(strip_NO);
        strip_NI.show();
        
    } else if(IR2 > 500 && IR1 > 500) 
    {
        analogWrite(MOTOR_LEFT_F, 255);
        analogWrite(MOTOR_RIGHT_F, 0);
        signalGreenForwardLeft(strip_NO);
        strip_NI.show();
        
    } else if(IR7 > 500 && IR8 > 500) 
    {
        analogWrite(MOTOR_LEFT_F, 0);
        analogWrite(MOTOR_RIGHT_F, 255);
        signalGreenForwardRight(strip_NO);
        strip_NI.show();
    }
}

bool squareDetected() 
{
    if(IR1 > 800 && IR2 > 800 && IR3 > 800 && IR4 > 800 && IR5 > 800 && IR6 > 800 && IR7 > 800 && IR8 > 800) {
        return true;
    }
}

void race_start() 
{
  moveForward, followLine();
}

void setupMotors() 
{
    pinMode(MOTOR_LEFT_B, OUTPUT);
    pinMode(MOTOR_LEFT_F, OUTPUT);
    pinMode(MOTOR_RIGHT_B, OUTPUT);
    pinMode(MOTOR_RIGHT_F, OUTPUT);
}

void setupIRSensors() 
{
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
    IR3 = analogRead(A3);
    IR4 = analogRead(A4);
    IR5 = analogRead(A2);
    IR6 = analogRead(A5);
    IR7 = analogRead(A6);
    IR8 = analogRead(A7);
}

void setGripperAngle(int angle) 
{
    int pulseWidth = map(angle, 0, 180, 1000, 2000);
    digitalWrite(GRIPPER_PIN, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(GRIPPER_PIN, LOW);
}


// Define variables for gripper state
bool gripperOpen = true;
//
//void detectObject_openGripper() 
//{
//    if (distance < 4 && gripperOpen) 
//  {
//    setServoAngle(90);
//    gripperOpen = false;
//    // Adjust delay as needed to ensure stable gripping
//    delay(500);
//  }
//  // If no object is detected, open the gripper
//  else if (distance >= 9 && !gripperOpen)
//  {
//    setServoAngle(60);
//    gripperOpen = true;
//    // Adjust delay as needed to ensure stable release
//    delay(500);
//  }
//}

void setServoAngle(int angle) 
{
    // Convert angle to pulse width
    int pulseWidth = map(angle, 0, 180, 1000, 2000); 

    // Send the pulse to the servo pin
    digitalWrite(GRIPPER_PIN, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(GRIPPER_PIN, LOW);
}

void signalGreenForwardLeft(Adafruit_NeoPixel &strip)
{
    strip.setPixelColor(0, strip.Color(GREEN));
}

void signalGreenForwardRight(Adafruit_NeoPixel &strip) 
{
    strip.setPixelColor(1, strip.Color(GREEN));
}
