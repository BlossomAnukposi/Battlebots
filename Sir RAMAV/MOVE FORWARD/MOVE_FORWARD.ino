#include <Adafruit_NeoPixel.h>

#define PIN_NI 7 //connected to NI
#define NUM_PIXELS 4

Adafruit_NeoPixel strip_NI(NUM_PIXELS, PIN_NI, NEO_GRB + NEO_KHZ800);

//DEFINING THE PIN CONSTANTS
// Ultrasonic Sensor
const int trig = 2;
const int echo = 4;
long duration;
int distance;

// Motors
const int lForward = 5; // connected to B2
const int rForward = 3; // connected to A1
const int lBackward = 6; // connected to B1
const int rBackward = 9; // connected to A2

// Analog line Sensors
const int S1 = A0;
const int S2 = A1;
const int S3 = A2;
const int S4 = A3;
const int S5 = A4;
const int S6 = A5;
const int S7 = A6;
const int S8 = A7;

void setup() {
  // Motor
  pinMode(lForward, OUTPUT);
  pinMode(rForward, OUTPUT);
  pinMode(lBackward, OUTPUT);
  pinMode(rBackward, OUTPUT);

  // UltraSonic Sensor
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  Serial.begin(9600);

  // Analog Line Sensor
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
  pinMode(S6, INPUT);
  pinMode(S7, INPUT);
  pinMode(S8, INPUT);
}

void loop() {
 // Ultrasonic Sensor
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  duration = pulseIn(echo, HIGH);
  distance = duration * 0.034 / 2;

  //Analog Line Sensor
  int sensor1 = analogRead(S1);
  int sensor2 = analogRead(S2);
  int sensor3 = analogRead(S3);
  int sensor4 = analogRead(S4);
  int sensor5 = analogRead(S5);
  int sensor6 = analogRead(S6);
  int sensor7 = analogRead(S7);
  int sensor8 = analogRead(S8);

  //Calibrate the line sensor
  int blackLineSum = 0;
  int emptySpaceSum = 0;
  int samples = 100;
  
  for (int i = 0; i < samples; i++) {
    blackLineSum += analogRead(S4);
    emptySpaceSum += analogRead(S4);
    delay(10);
  }
  
  int blackLineAverage = blackLineSum / samples;
  int emptySpaceAverage = emptySpaceSum / samples;
  int threshold = (blackLineAverage + emptySpaceAverage) / 2;

  //Turn left or right if middle sensors do not sense the line
  if (sensor4 < threshold && sensor5 < threshold)
  {
    analogWrite(lForward, 255);
    analogWrite(rForward, 0); //turn left

    //if line has been found, go forward
    if (sensor4 > threshold && sensor5 > threshold)
    {
      analogWrite(lForward, 255);
      analogWrite(rForward, 250);
    }
  }

  // LED Signals Setup
  strip_NI.begin();

  if (analogRead(lForward) == 0 && analogRead(rForward) == 0 && analogRead(lBackward) == 0 && analogRead(rBackward) == 0)
  {
    for (int i = 0; i < NUM_PIXELS; i++) {
      strip_NI.setPixelColor(i, strip_NI.Color(0, 0, 0));
    }
  }
  else if (analogRead(lForward) > 200 && analogRead(rForward) > 0)
  {
    for (int i = 0; i < NUM_PIXELS; i++) {
      strip_NI.setPixelColor(i, strip_NI.Color(0, 0, 0)); //GRB not RGB
    }
  }
  strip_NI.show();

  //Threshold test
  Serial.print("threshold: ");
  Serial.println(threshold);
}
