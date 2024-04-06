//===================== LIBRARIES =====================//
#include <Adafruit_NeoPixel.h>

//===================== VARIABLES =====================//
// NeoPixels
#define NEOPIN_NO                  7
#define NEOPIN_NI                  8
#define NEOPIXEL_BRIGHTNESS_LEVEL 25
#define NUM_NEOPIXELS              4 

// Signal colors
#define WHITE             255, 255, 255
#define RED               0,   255,   0
#define GREEN             255, 0,   0
#define LIGHT_GREEN       155, 255,   0
#define BLUE              0,   0, 255
#define YELLOW            255, 255,   0
#define PURPLE            255,   0, 255

int speed = 50; // Speed of color transition (lower value = faster)
int brightness = 100; // Brightness of NeoPixels (0-255)


Adafruit_NeoPixel strip_NI(NUM_NEOPIXELS, NEOPIN_NI, NEO_GRB + NEO_KHZ800);

// Motors 
#define MOTOR_LEFT_BACKWARD   10   // A2
#define MOTOR_LEFT_FORWARD     9   // A1
#define MOTOR_RIGHT_BACKWARD   5   // B2
#define MOTOR_RIGHT_FORWARD    6   // B1

// Motor speed / power
#define MAX_POWER       255
#define HIGH_POWER      235
#define AVERAGE_POWER   200
#define LOW_POWER        50
#define NO_POWER          0

// Gripper
#define SERVO_PIN             11  

// Ultrasonic sensor
#define TRIGGER_PIN               12 
#define ECHO_PIN                   4 
#define MAX_DISTANCE_ULTRASONIC   20 // 20 cm maximum distance to track object
#define START_DISTANCE_FLAG       15 // 15 cm maximum distance from the flag
#define ECHO_READ_INTERVAL       245

unsigned long time;
float distance;
unsigned long duration;

unsigned long startTime = 0;
bool pickedUpObject = false;
bool raceFinished = false;
bool testing = false;

// Start race
bool onSquareStart = false; // Flag if it's on the square
unsigned long squareStartTime = 0; // Store the time when it detects the 1st square

// End race
unsigned long squareEndTime = 0; // Store the time when it detects the 2nd square
bool onSquare = false; // Flag if it's on the square

bool danceFinal = false;

// Infrared sensor thresholds
#define IR_THRESHOLD_HIGH     800
#define IR_THRESHOLD_AVERAGE  600
#define IR_THRESHOLD_LOW      500
#define SENSORS                 8

int IR1;
int IR2;
int IR3;
int IR4;
int IR5;
int IR6;
int IR7;
int IR8;

//===================== SETUP =====================//
void setup()
{
  Serial.begin(9600); 
  strip_NI.clear();
  // Setup ultrasonic sensor
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  
  // Setup motors
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);

  // Set gripper to 130 degrees
  setServoAngle(130);

  // Get distance
  distanceReader();

  strip_NI.begin();
}

//===================== LOOP =====================//

void loop() 
{
  while (distance < MAX_DISTANCE_ULTRASONIC && !pickedUpObject) 
  { 
    lightUpAll(strip_NI, WHITE, NEOPIXEL_BRIGHTNESS_LEVEL);
    distanceReader(); 
  }
   if (!pickedUpObject) 
   {
      startForward();
      start();  
   }
  else 
  {      
    if (!raceFinished) 
      {
      followLine();
      distanceSensor();
      endRace();
      } 
      if (raceFinished && !danceFinal)
      {
        dance();
      }
  }
}


//===================== FUNCTION =====================//

// Gripper
void setServoAngle(int angle) 
{
    int pulseWidth = map(angle, 0, 180, 0, 255); 
    analogWrite(SERVO_PIN, pulseWidth);
    strip_NI.clear();
    lightUpAll(strip_NI, YEL§LOW, NEOPIXEL_BRIGHTNESS_LEVEL);
}

// Wait
void wait(int waitingTime) 
{
  time = millis();
  while(millis() < time + waitingTime)
  {}
}

// Movements
void startForward() 
{
  analogWrite(MOTOR_LEFT_FORWARD, 225); 
  analogWrite(MOTOR_RIGHT_FORWARD, MAX_POWER); 
  
  strip_NI.clear();
  lightUpPixel(strip_NI, 2, GREEN, NEOPIXEL_BRIGHTNESS_LEVEL);
  lightUpPixel(strip_NI, 3, GREEN, NEOPIXEL_BRIGHTNESS_LEVEL);
}

void left()
{
  analogWrite(MOTOR_LEFT_FORWARD, 40);
  analogWrite(MOTOR_RIGHT_FORWARD, 230);

  strip_NI.clear();
  lightUpPixel(strip_NI, 2, GREEN, NEOPIXEL_BRIGHTNESS_LEVEL);
  wait(800); 
}

void go1Sec() 
{
  analogWrite(MOTOR_LEFT_FORWARD, 220); // 220
  analogWrite(MOTOR_RIGHT_FORWARD, 235); // 200
  
  strip_NI.clear();
  lightUpPixel(strip_NI, 2, GREEN, NEOPIXEL_BRIGHTNESS_LEVEL);
  lightUpPixel(strip_NI, 3, GREEN, NEOPIXEL_BRIGHTNESS_LEVEL);
  wait(400); 
}

void goSec() 
{
  analogWrite(MOTOR_LEFT_FORWARD, 220);
  analogWrite(MOTOR_RIGHT_FORWARD, 235);
  
  strip_NI.clear();
  lightUpPixel(strip_NI, 2, GREEN, NEOPIXEL_BRIGHTNESS_LEVEL);
  lightUpPixel(strip_NI, 3, GREEN, NEOPIXEL_BRIGHTNESS_LEVEL);
  wait(400);
}

void leftAvoid()
{
  analogWrite(MOTOR_LEFT_FORWARD, 40);
  analogWrite(MOTOR_RIGHT_FORWARD, 230);

  strip_NI.clear();
  lightUpPixel(strip_NI, 2, GREEN, NEOPIXEL_BRIGHTNESS_LEVEL);
  wait(750); 
}

void left1Sec()
{
  analogWrite(MOTOR_LEFT_FORWARD, 40);
  analogWrite(MOTOR_RIGHT_FORWARD, 230);

  strip_NI.clear();
  lightUpPixel(strip_NI, 2, GREEN, NEOPIXEL_BRIGHTNESS_LEVEL);
  wait(500); 
}

void rightAvoid()
{
  analogWrite(MOTOR_LEFT_FORWARD, 210);
  analogWrite(MOTOR_RIGHT_FORWARD, 40);

  strip_NI.clear();
  lightUpPixel(strip_NI, 3, GREEN, NEOPIXEL_BRIGHTNESS_LEVEL);
  wait(450);
}

void moveBackward() 
{
  // Move both motors backward
  analogWrite(MOTOR_LEFT_BACKWARD, MAX_POWER);
  analogWrite(MOTOR_RIGHT_BACKWARD, MAX_POWER);

  strip_NI.clear();
  lightUpPixel(strip_NI, 0, LIGHT_GREEN, NEOPIXEL_BRIGHTNESS_LEVEL);
  lightUpPixel(strip_NI, 1, LIGHT_GREEN, NEOPIXEL_BRIGHTNESS_LEVEL);
  wait(1500); 
  stopMotors();
}

void dance() 
{
  strip_NI.clear();
  rainbow(strip_NI, 20); 
  analogWrite(MOTOR_LEFT_FORWARD, MAX_POWER);
  analogWrite(MOTOR_RIGHT_FORWARD, NO_POWER );
  wait(350);
  analogWrite(MOTOR_LEFT_FORWARD, NO_POWER );
  analogWrite(MOTOR_RIGHT_FORWARD, MAX_POWER);
  wait(350);
  analogWrite(MOTOR_LEFT_FORWARD, MAX_POWER);
  analogWrite(MOTOR_RIGHT_FORWARD, NO_POWER);
  wait(350);
  analogWrite(MOTOR_LEFT_FORWARD, NO_POWER);
  analogWrite(MOTOR_RIGHT_FORWARD, MAX_POWER);
  wait(350);
  analogWrite(MOTOR_LEFT_FORWARD, MAX_POWER);
  analogWrite(MOTOR_RIGHT_FORWARD, NO_POWER);
  wait(350);
  analogWrite(MOTOR_LEFT_FORWARD, NO_POWER);
  analogWrite(MOTOR_RIGHT_FORWARD, MAX_POWER);
  wait(350);
  analogWrite(MOTOR_LEFT_BACKWARD, MAX_POWER);
  analogWrite(MOTOR_RIGHT_FORWARD, NO_POWER);
  wait(350);
  analogWrite(MOTOR_LEFT_FORWARD, NO_POWER);
  analogWrite(MOTOR_RIGHT_BACKWARD, MAX_POWER);
  stopMotors();
  danceFinal = true;
}

void stopMotors() 
{
  // Stop both motors
  analogWrite(MOTOR_LEFT_FORWARD, NO_POWER);
  analogWrite(MOTOR_RIGHT_FORWARD, NO_POWER);
  analogWrite(MOTOR_RIGHT_BACKWARD, NO_POWER);
  analogWrite(MOTOR_LEFT_BACKWARD, NO_POWER);
  strip_NI.clear();
  lightUpAll(strip_NI, RED, NEOPIXEL_BRIGHTNESS_LEVEL);
}

// Tasks
void followLine()
{
  readSensors();
  if(IR3 > IR_THRESHOLD_AVERAGE && IR4 > IR_THRESHOLD_AVERAGE)
  {
    analogWrite(MOTOR_LEFT_FORWARD, MAX_POWER);
    analogWrite(MOTOR_RIGHT_FORWARD, 240);
    strip_NI.clear();
    lightUpPixel(strip_NI, 3, GREEN, NEOPIXEL_BRIGHTNESS_LEVEL);
    
  }else if(IR5 > IR_THRESHOLD_AVERAGE && IR6 > IR_THRESHOLD_AVERAGE)
  {
    analogWrite(MOTOR_LEFT_FORWARD, 240);
    analogWrite(MOTOR_RIGHT_FORWARD, MAX_POWER);
    strip_NI.clear();
    lightUpPixel(strip_NI, 2, GREEN, NEOPIXEL_BRIGHTNESS_LEVEL);

    
  }else if(IR2 > IR_THRESHOLD_AVERAGE && IR3 > IR_THRESHOLD_AVERAGE)
  {
    analogWrite(MOTOR_LEFT_FORWARD, MAX_POWER);
    analogWrite(MOTOR_RIGHT_FORWARD, LOW_POWER);
    strip_NI.clear();
    lightUpPixel(strip_NI, 3, GREEN, NEOPIXEL_BRIGHTNESS_LEVEL);
    
  }else if(IR6 > IR_THRESHOLD_AVERAGE && IR7 > IR_THRESHOLD_AVERAGE)
  {
    analogWrite(MOTOR_LEFT_FORWARD, LOW_POWER);
    analogWrite(MOTOR_RIGHT_FORWARD, MAX_POWER);
    strip_NI.clear();
    lightUpPixel(strip_NI, 2, GREEN, NEOPIXEL_BRIGHTNESS_LEVEL);
    
  }else if(IR2 > IR_THRESHOLD_AVERAGE && IR1 > IR_THRESHOLD_AVERAGE)
  {
    analogWrite(MOTOR_LEFT_FORWARD, MAX_POWER);
    analogWrite(MOTOR_RIGHT_FORWARD, LOW_POWER);
    strip_NI.clear();
    lightUpPixel(strip_NI, 3, GREEN, NEOPIXEL_BRIGHTNESS_LEVEL);
  
  }else if(IR7 > IR_THRESHOLD_AVERAGE && IR8 > IR_THRESHOLD_AVERAGE)
  {
    analogWrite(MOTOR_LEFT_FORWARD, LOW_POWER );
    analogWrite(MOTOR_RIGHT_FORWARD, MAX_POWER);
    strip_NI.clear();
    lightUpPixel(strip_NI, 2, GREEN, NEOPIXEL_BRIGHTNESS_LEVEL);
  }
}

void findLine()
{
   readSensors();
   if (IR3 < IR_THRESHOLD_AVERAGE && IR4 < IR_THRESHOLD_AVERAGE && IR5 < IR_THRESHOLD_AVERAGE && IR6 < IR_THRESHOLD_AVERAGE)
   {
    analogWrite(MOTOR_LEFT_FORWARD, 200);
    analogWrite(MOTOR_RIGHT_FORWARD, 225); 
    strip_NI.clear();
    lightUpAll(strip_NI, BLUE, NEOPIXEL_BRIGHTNESS_LEVEL); 
   }
}

void start()
{
  readSensors();
  if (IR1 > IR_THRESHOLD_HIGH && IR2 > IR_THRESHOLD_HIGH && IR3 > IR_THRESHOLD_HIGH && IR6 > IR_THRESHOLD_HIGH && IR7 > IR_THRESHOLD_HIGH && IR8 > IR_THRESHOLD_HIGH) 
  {
        if (!onSquareStart) 
        { 
          onSquareStart = true; 
          squareStartTime = millis(); 
    } else if (millis() - squareStartTime > 200) { 
      stopMotors();
      setServoAngle(85);
      wait(800); 
      left();
      pickedUpObject = true; 
    }
  } else 
  {
    onSquareStart = false;
  }
}

void endRace()
{
  readSensors();
  if (IR1 > IR_THRESHOLD_HIGH && IR2 > IR_THRESHOLD_HIGH && IR3 > IR_THRESHOLD_HIGH && IR6 > IR_THRESHOLD_HIGH && IR7 > IR_THRESHOLD_HIGH && IR8 > IR_THRESHOLD_HIGH) 
  {
    if (!onSquare) 
    { 
      onSquare = true; 
      squareEndTime = millis();
    } else if (millis() - squareEndTime > 75) 
    { 
      stopMotors();
      setServoAngle(130); 
      wait(300);
      moveBackward(); 
      raceFinished = true; 
    }
  } else 
        {
    // If the robot is not on the square, reset the flag
    onSquare = false;
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
    distance = (duration / 2) * 0.0343; // 343 m/s per second as speed of sound
}

void distanceSensor()
{
static unsigned long timer;
 if (millis() > timer) 
  {
    distanceReader();

    if (distance <= MAX_DISTANCE_ULTRASONIC)
      {
        rightAvoid();
        goSec();
        left1Sec();
        go1Sec();
        leftAvoid();
        findLine(); 
      }
       timer = millis() + 100;
    }
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

// signals

// NeoPixel color functions
void lightUpPixel(Adafruit_NeoPixel &strip, int pixelNum, uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness) {
  strip.setPixelColor(pixelNum, strip.Color(red, green, blue));
  strip.setBrightness(brightness);
  strip.show();
}

void lightUpAll(Adafruit_NeoPixel &strip, uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness) {
  for (int i = 0; i < NUM_NEOPIXELS; i++) {
    strip.setPixelColor(i, strip.Color(red, green, blue));
  }
  strip.setBrightness(brightness);
  strip.show();
}

// rainbow color based on input position
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return strip_NI.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return strip_NI.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
    WheelPos -= 170;
    return strip_NI.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}


// display rainbow effect on NeoPixels
void rainbow(Adafruit_NeoPixel &strip, int speed) {
  for (int i = 0; i < 256; i++) {
    // Set each NeoPixel to a color in the rainbow
    for (int j = 0; j < strip.numPixels(); j++) {
      strip.setPixelColor(j, Wheel((i + j * 64) % 256));
    }
    strip.show(); 
    delay(speed); 
  }
}
