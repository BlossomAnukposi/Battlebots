#include <Adafruit_NeoPixel.h>

#define PIN_NI 7 //connected to NI
#define NUM_PIXELS 4
Adafruit_NeoPixel strip_NI(NUM_PIXELS, PIN_NI, NEO_GRB + NEO_KHZ800);

//calibration stuff
int valuesS1[4];
int counter = 5;

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

  int sensor1 = analogRead(S1);
  int sensor2 = analogRead(S2);
  int sensor3 = analogRead(S3);
  int sensor4 = analogRead(S4);
  int sensor5 = analogRead(S5);
  int sensor6 = analogRead(S6);
  int sensor7 = analogRead(S7);
  int sensor8 = analogRead(S8);

void setup() {
  // Motor
  pinMode(lForward, OUTPUT);
  pinMode(rForward, OUTPUT);
  pinMode(lBackward, OUTPUT);
  pinMode(rBackward, OUTPUT);

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
  readSensors();
  calibration();

  Serial.print("S1: ");
  Serial.println(sensor1);
}
