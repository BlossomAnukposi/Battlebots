//===================== LIBRARIES =====================//
#include <Adafruit_NeoPixel.h>



//===================== VARIABLES =====================//
// neo pixels
#define NEOPIN_NO         7
#define NEOPIN_NI         8
#define BRIGHTNESS_LEVEL 25
#define NEO_PIXNUMBER     4 // neo :  0 - 1 - 2 - 3 
// 0 - back  - left
// 1 - back  - rightq§  
// 2 - front - right
// 3 - front - left

// signal colors
#define WHITE           255, 255, 255
#define RED             0,   255,   0
#define GREEN             255, 0,   0
#define LIGHT_GREEN     155, 255,   0
#define BLUE              0,   0, 255
#define YELLOW          255, 255,   0

Adafruit_NeoPixel STRIP_NO(NEO_PIXNUMBER, NEOPIN_NO, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel STRIP_NI(NEO_PIXNUMBER, NEOPIN_NI, NEO_GRB + NEO_KHZ800);



// motors
#define MOTOR_LEFT_BACKWARD   10
#define MOTOR_LEFT_FORWARD     9
#define MOTOR_RIGHT_BACKWARD   5
#define MOTOR_RIGHT_FORWARD    6

// gripper pin
#define SERVO_PIN 11

// ultra sonic sensors
#define TRIGGER_PIN             4
#define ECHO_PIN               12
#define MAX_DISTANCE           20
#define START_DISTANCE         15
#define ECHO_INTERVAL         245
float distance,duration;
const int obstacleThreshold =  20;

// infrared
int IR1;
int IR2;
int IR3;
int IR4;
int IR5;
int IR6;
int IR7;
int IR8;


unsigned long startTime = 0;
bool pickedUpObject = false;
bool raceFinished = false;
bool testing = false;

unsigned long squareStartTime = 0; // Variable to store the time when the robot first detects the square
bool onSquare = false; // Flag to track whether the robot is on the square

//===================== SETUP =====================//
void setup()
{
  beginSetup();
}


//===================== LOOP =====================//

void loop (){
  readSensors();
  unsigned long currentTime = millis();
  if (currentTime - startTime < 150) 
    {
      startForward();
     delay(500); 
    }
    else if (!pickedUpObject) 
      {
        setServoAngle(85);
        delay(1000); 
        left();
        delay(500); 
        pickedUpObject = true;
      }
      else {      
        if (!raceFinished) 
        {
          followLine();
          distanceSensor();
      
      if (IR1 > 900 && IR2 > 900 && IR3 > 900 && IR6 > 900 && IR7 > 900 && IR8 > 900) 
      {
          if (!onSquare) 
          { 
            onSquare = true; 
            squareStartTime = millis(); 
          } else if (millis() - squareStartTime > 80) 
          {
            stopMotors();
            // Drop the object
            setServoAngle(130); 
            delay(500); 
            moveBackward();
            raceFinished = true; 
          }
        } else 
        {
          onSquare = false;
        }
      }   
  }
}

//===================== FUNCTIONS =====================//

// setup
void beginSetup()
{
  Serial.begin(9600); 
  setupMotors();
  setServoAngle(130);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  STRIP_NI.begin();
  STRIP_NO.begin();
}


void setupMotors()
{
  //initializing motor pins
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
}

// movements

void turn() 
{
    // left
    analogWrite(MOTOR_LEFT_FORWARD, 40);
    analogWrite(MOTOR_RIGHT_FORWARD, 220);
    delay(500);

    // forward
    analogWrite(MOTOR_LEFT_FORWARD, 255);
    analogWrite(MOTOR_RIGHT_FORWARD, 240);
    delay(700);

    // right
    analogWrite(MOTOR_LEFT_FORWARD, 220);
    analogWrite(MOTOR_RIGHT_FORWARD, 40);
    delay(600);

    followLine();
}

void left()
{
  analogWrite(MOTOR_LEFT_FORWARD, 40);
  analogWrite(MOTOR_RIGHT_FORWARD, 180);
  delay(1000); 
  stopMotors();
}

void leftAvoid()
{
  analogWrite(MOTOR_LEFT_FORWARD, 100);
  analogWrite(MOTOR_RIGHT_FORWARD, 200);
  delay(1200); 
  stopMotors();
}


void right()
{
  analogWrite(MOTOR_LEFT_FORWARD, 180);
  analogWrite(MOTOR_RIGHT_FORWARD, 40);
  delay(1000);
  stopMotors();
}

void rightAvoid()
{
  analogWrite(MOTOR_LEFT_FORWARD, 200);
  analogWrite(MOTOR_RIGHT_FORWARD, 100);
  delay(1000);
}

void goForward() 
{
  analogWrite(MOTOR_LEFT_FORWARD, 255);
  analogWrite(MOTOR_RIGHT_FORWARD, 240);
  delay(1500);
  stopMotors();
}

void startForward()
{
  analogWrite(MOTOR_LEFT_FORWARD, 200);
  analogWrite(MOTOR_RIGHT_FORWARD, 185);
  delay(1500);
  stopMotors();
}

void goCheck() 
{
  analogWrite(MOTOR_LEFT_FORWARD, 200);
  analogWrite(MOTOR_RIGHT_FORWARD, 180);
  delay(500);
  stopMotors();
}

void go1Sec() 
{
  analogWrite(MOTOR_LEFT_FORWARD, 200);
  analogWrite(MOTOR_RIGHT_FORWARD, 180);
  delay(600);
  stopMotors();
}

void moveBackward() 
{
  // Move both motors backward
  analogWrite(MOTOR_LEFT_BACKWARD, 255);
  analogWrite(MOTOR_RIGHT_BACKWARD, 255);
  delay(1000); 
  stopMotors();
}

void stopMotors() 
{
  // Stop both motors
  analogWrite(MOTOR_LEFT_FORWARD, 0);
  analogWrite(MOTOR_RIGHT_FORWARD, 0);
  analogWrite(MOTOR_RIGHT_BACKWARD, 0);
  analogWrite(MOTOR_LEFT_BACKWARD, 0);
}


void setServoAngle(int angle) 
{
    int pulseWidth = map(angle, 0, 180, 0, 255); 
    analogWrite(SERVO_PIN, pulseWidth);
    signalYellow_All(STRIP_NI);
    STRIP_NI.show();
    signalOff(STRIP_NI);
}



void followLine()
{
  if(IR3 > 500 && IR4 > 500)
  {
    analogWrite(MOTOR_LEFT_FORWARD, 255);
    analogWrite(MOTOR_RIGHT_FORWARD, 180); // 180
        signalGreen_FR(STRIP_NI);
        STRIP_NI.show();
        
  }else if(IR5 > 500 && IR6 > 500)
  {
    analogWrite(MOTOR_LEFT_FORWARD, 180); //180
    analogWrite(MOTOR_RIGHT_FORWARD, 255);
         signalGreen_FL(STRIP_NI);
         STRIP_NI.show();
          signalOff(STRIP_NI);
         
  }else if(IR2 > 500 && IR3 > 500)
  {
    analogWrite(MOTOR_LEFT_FORWARD, 255);
    analogWrite(MOTOR_RIGHT_FORWARD, 10); // 80
        signalGreen_FR(STRIP_NI);
        STRIP_NI.show();
        signalOff(STRIP_NI);
        
  }else if(IR6 > 500 && IR7 > 500)
  {
    analogWrite(MOTOR_LEFT_FORWARD, 10); //80
    analogWrite(MOTOR_RIGHT_FORWARD, 255);
        signalGreen_FL(STRIP_NI);
        STRIP_NI.show();
        signalOff(STRIP_NI);
        
  }else if(IR2 > 500 && IR1 > 500)
  {
    analogWrite(MOTOR_LEFT_FORWARD, 255);
    analogWrite(MOTOR_RIGHT_FORWARD, 10); //40
        signalGreen_FR(STRIP_NI);
        STRIP_NI.show();
    signalOff(STRIP_NI);
    
  }else if(IR7 > 500 && IR8 > 500)
  {
    analogWrite(MOTOR_LEFT_FORWARD, 10); //40
    analogWrite(MOTOR_RIGHT_FORWARD, 255);
        signalGreen_FL(STRIP_NI);
        STRIP_NI.show();
        signalOff(STRIP_NI);
        
  } else if (IR3 < 500 && IR4 < 500 && IR5 < 500 && IR6 < 500)
  {
    stopMotors();
    delay(1000);
    analogWrite(MOTOR_LEFT_FORWARD, 160);
    analogWrite(MOTOR_RIGHT_FORWARD, 220);
    signalWhite_FR(STRIP_NI);
    signalWhite_FL(STRIP_NI);
    STRIP_NI.show();
    signalOff(STRIP_NI);
  }
}


void distanceReader()
{
  digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH); 
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    

    duration = pulseIn(ECHO_PIN, HIGH); 

    distance = (duration / 2) * 0.0343; 
}

void distanceSensor(){
static unsigned long timer;
 if (millis() > timer) 
  {

    distanceReader();

    if (distance <= MAX_DISTANCE)
      {
        turn();   
      }
    else
      {
        Serial.print(distance);
        Serial.println(" cm");
      }
       timer = millis() + 200;
    }
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


// ------ signals -----
// green
void signalGreen_FL (Adafruit_NeoPixel &strip)
{
    strip.setPixelColor(3, strip.Color(GREEN));
}

void signalGreen_FR(Adafruit_NeoPixel &strip) 
{
    strip.setPixelColor(2, strip.Color(GREEN));
}

void signalGreen_BR(Adafruit_NeoPixel &strip)
{
    strip.setPixelColor(1, strip.Color(GREEN));
}

void signalGreen_BL(Adafruit_NeoPixel &strip) 
{
    strip.setPixelColor(0, strip.Color(GREEN));
}

// white
void signalWhite_FL(Adafruit_NeoPixel &strip)
{
    strip.setPixelColor(3, strip.Color(WHITE));
}

void signalWhite_FR(Adafruit_NeoPixel &strip) 
{
    strip.setPixelColor(2, strip.Color(WHITE));
}

void signalWhite_BR(Adafruit_NeoPixel &strip)
{
    strip.setPixelColor(1, strip.Color(WHITE));
}

void signalWhite_BL(Adafruit_NeoPixel &strip) 
{
    strip.setPixelColor(0, strip.Color(WHITE));
}


// yellow
void signalYellow_All(Adafruit_NeoPixel &strip) 
{
    strip.setPixelColor(0, strip.Color(YELLOW));
    strip.setPixelColor(1, strip.Color(YELLOW));
    strip.setPixelColor(2, strip.Color(YELLOW));
    strip.setPixelColor(3, strip.Color(YELLOW));
}

// clear signal

void signalOff(Adafruit_NeoPixel &strip) 
{
    strip.clear();  // Clear all NeoPixels
}
