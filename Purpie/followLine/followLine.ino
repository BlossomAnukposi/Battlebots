// ============= LIBRARIES ===============//
#include <Adafruit_NeoPixel.h>

// neo lights
#define NEOPIN_NO 7
#define NEOPIN_NI 8
#define NEO_PIXNUMBER 4
#define BRIGHTNESS_LEVEL 35

// signal colors
#define WHITE           255, 255, 255
#define RED               0, 255,   0
#define GREEN           255,   0,   0
#define LIGHT_GREEN     155, 255,   0
#define BLUE              0,   0, 255
#define YELLOW          255, 255,   0

// motor
#define MOTOR_LEFT_B 10
#define MOTOR_LEFT_F 11
#define MOTOR_RIGHT_B 5
#define MOTOR_RIGHT_F 6

int rightSensors;
int leftSensors;

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


Adafruit_NeoPixel strip_NO(NEO_PIXNUMBER, NEOPIN_NO, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_NI(NEO_PIXNUMBER, NEOPIN_NI, NEO_GRB + NEO_KHZ800);

//===============================
//*SETUP*************
//===============================


//===============================
//*FUNCTIONS***********
//===============================
void moveBackward(){
    analogWrite(MOTOR_LEFT_B, 255);
    analogWrite(MOTOR_RIGHT_B, 255);
}



void moveForward(){
    analogWrite(MOTOR_LEFT_F, 255);
    analogWrite(MOTOR_RIGHT_F, 240);
}

void turnLeft(){
    analogWrite(MOTOR_LEFT_F, 255);
    analogWrite(MOTOR_RIGHT_F, 230);
}


void turnRight(){
    analogWrite(MOTOR_LEFT_F, 180);
    analogWrite(MOTOR_RIGHT_F, 255);
}

void followLine() {
    readSensors();
    // Turn off all LEDs initially
    strip_NI.setPixelColor(2, strip_NI.Color(0, 0, 0)); // Turn off left LED
    strip_NI.setPixelColor(3, strip_NI.Color(0, 0, 0)); // Turn off right LED
    strip_NI.show(); // Show the changes

    if(IR3 > 500 && IR4 > 500) {
        analogWrite(MOTOR_LEFT_F, 255);
        analogWrite(MOTOR_RIGHT_F, 120);
        signalGreenForwardLeft(strip_NI);
        strip_NI.setBrightness(BRIGHTNESS_LEVEL);
        strip_NI.show();
        delay(200);
        
    } else if(IR5 > 500 && IR6 > 500) {
        analogWrite(MOTOR_LEFT_F, 120);
        analogWrite(MOTOR_RIGHT_F, 255);
        signalGreenForwardRight(strip_NI);
         strip_NI.setBrightness(BRIGHTNESS_LEVEL);
        strip_NI.show();
        delay(200);
        
    } else if(IR2 > 500 && IR3 > 500) {
        analogWrite(MOTOR_LEFT_F, 255); 
        analogWrite(MOTOR_RIGHT_F, 0);
        signalGreenForwardLeft(strip_NI);
         strip_NI.setBrightness(BRIGHTNESS_LEVEL);
        strip_NI.show();
        delay(200);
        
    } else if(IR6 > 500 && IR7 > 500) {
        analogWrite(MOTOR_LEFT_F, 0);
        analogWrite(MOTOR_RIGHT_F, 255);
        signalGreenForwardRight(strip_NI);
         strip_NI.setBrightness(BRIGHTNESS_LEVEL);
        strip_NI.show();
        delay(200);
        
    } else if(IR2 > 500 && IR1 > 500) {
        analogWrite(MOTOR_LEFT_F, 255);
        analogWrite(MOTOR_RIGHT_F, 0);
        signalGreenForwardLeft(strip_NI);
         strip_NI.setBrightness(BRIGHTNESS_LEVEL);
        strip_NI.show();
        delay(200);
        
    } else if(IR7 > 500 && IR8 > 500) {
        analogWrite(MOTOR_LEFT_F, 0);
        analogWrite(MOTOR_RIGHT_F, 255);
        signalGreenForwardRight(strip_NI);
         strip_NI.setBrightness(BRIGHTNESS_LEVEL);
        strip_NI.show();
        delay(200);
    }
}

bool squareDetected(){
  if(IR1 > 800 && IR2 > 800 && IR3 > 800 && IR4 > 800 && IR5 > 800 && IR6 > 800 && IR7 > 800 && IR8 > 800){
    return true;
  }
}

void setupMotors() {
    pinMode(MOTOR_LEFT_B, OUTPUT);
    pinMode(MOTOR_LEFT_F, OUTPUT);
    pinMode(MOTOR_RIGHT_B, OUTPUT);
    pinMode(MOTOR_RIGHT_F, OUTPUT);
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


void signalGreenForwardLeft(Adafruit_NeoPixel &strip) {
    strip.setPixelColor(2, strip.Color(GREEN));
}

void signalGreenForwardRight(Adafruit_NeoPixel &strip) {
    strip.setPixelColor(3, strip.Color(GREEN));
}

void setup(){
  strip_NO.begin();
  strip_NI.begin();
  Serial.begin(9600); // Initializes the serial communication
  setupIRSensors();
  setupMotors();
}

//===============================
//*LOOP**************
//===============================
void loop() {
 followLine();
}
