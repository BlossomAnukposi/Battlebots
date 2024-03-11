// ============= LIBRARIES ===============//
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel strip_NO(NUM_PIXELS, PIN_NO, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_NI(NUM_PIXELS, PIN_NI, NEO_GRB + NEO_KHZ800);

#define NEOPIN_NO 7
#define NEOPIN_NI 8
#define NEO_PIXNUMBER 4
#define BRIGHTNESS_LEVEL 25

#define GRIPPER_PIN 12
#define GRIPPER_OPEN 120
#define GRIPPER_CLOSE 60

#define TRIGGER_PIN 4
#define ECHO_PIN 9

#define MOTOR_LEFT_B 10
#define MOTOR_LEFT_F 11
#define MOTOR_RIGHT_B 5
#define MOTOR_RIGHT_F 6

//LineSensors
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


// ============= SETUP ===============//
void setup()
{
    pinMode(GRIPPER_PIN, OUTPUT);
    setupMotors();
    setupIRSensors();
}

// ============= LOOP ===============//
void loop ()
{
    followLine();
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(90);
    delay(2000);
    setGripperAngle(60);
    delay(1000);
}

// ============= FUNCTIONS ===============//

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
    if(IR3 > 500 && IR4 > 500) {
        analogWrite(MOTOR_LEFT_F, 255);
        analogWrite(MOTOR_RIGHT_F, 120);
    } else if(IR5 > 500 && IR6 > 500) {
        analogWrite(MOTOR_LEFT_F, 120);
        analogWrite(MOTOR_RIGHT_F, 255);
    } else if(IR2 > 500 && IR3 > 500) {
        analogWrite(MOTOR_LEFT_F, 255);
        analogWrite(MOTOR_RIGHT_F, 0);
    } else if(IR6 > 500 && IR7 > 500) {
        analogWrite(MOTOR_LEFT_F, 0);
        analogWrite(MOTOR_RIGHT_F, 255);
    } else if(IR2 > 500 && IR1 > 500) {
        analogWrite(MOTOR_LEFT_F, 255);
        analogWrite(MOTOR_RIGHT_F, 0);
    } else if(IR7 > 500 && IR8 > 500) {
        analogWrite(MOTOR_LEFT_F, 0);
        analogWrite(MOTOR_RIGHT_F, 255);
    }
}

bool squareDetected() {
    if(IR1 > 800 && IR2 > 800 && IR3 > 800 && IR4 > 800 && IR5 > 800 && IR6 > 800 && IR7 > 800 && IR8 > 800) {
        return true;
    }
}

void setupMotors() {
    pinMode(MOTOR_LEFT_B, OUTPUT);
    pinMode(MOTOR_LEFT_F, OUTPUT);
    pinMode(MOTOR_RIGHT_B, OUTPUT);
    pinMode(MOTOR_RIGHT_F, OUTPUT);
}

void setupIRSensors() {
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
    pinMode(A2, INPUT);
    pinMode(A3, INPUT);
    pinMode(A4, INPUT);
    pinMode(A5, INPUT);
    pinMode(A6, INPUT);
    pinMode(A7, INPUT);
}

void readSensors() {
    IR1 = analogRead(A0);
    IR2 = analogRead(A1);
    IR3 = analogRead(A3);
    IR4 = analogRead(A4);
    IR5 = analogRead(A2);
    IR6 = analogRead(A5);
    IR7 = analogRead(A6);
    IR8 = analogRead(A7);
}

void setGripperAngle(int angle) {
    int pulseWidth = map(angle, 0, 180, 1000, 2000);
    digitalWrite(GRIPPER_PIN, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(GRIPPER_PIN, LOW);
}

