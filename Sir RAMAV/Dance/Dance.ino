#include <Adafruit_NeoPixel.h>

//===================== NEOPIXEL SETUP =====================//
#define WHITE           255, 255, 255
#define RED             0  , 255, 0
#define GREEN           255, 0  , 0
#define BLUE            0  , 0  , 255
#define LIGHT_GREEN     255, 155, 0
#define YELLOW          255, 255, 0
#define OFF             0  , 0  , 0
#define CALIBRATE       0  , 200, 190
#define NEOPIN_INPUT    7
#define NEO_PIXNUMBER   4

Adafruit_NeoPixel strip_NI(NEO_PIXNUMBER, NEOPIN_INPUT, NEO_GRB + NEO_KHZ800);
//=============== ANALOG LINE SENSORS ===============//
#define AVERAGE 700
#define SENSORS 8

int S[SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};
int IR[SENSORS];

void readSensors() {
    // IR[0] = leftmost, IR[7] = rightmost
    for (int i = 0; i < SENSORS; i++) {
        IR[i] = analogRead(S[i]);
    }
}

//===================== MOTORS =====================//
#define MOTOR_LEFT_FORWARD    9 // connected to A1
#define MOTOR_RIGHT_FORWARD   6 // connected to B2
#define MOTOR_LEFT_BACKWARD   5 // connected to A2
#define MOTOR_RIGHT_BACKWARD  10 // connected to B1

void slightLeft() {
    analogWrite(MOTOR_LEFT_FORWARD, 255);
    analogWrite(MOTOR_RIGHT_FORWARD, 250);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
    delay(10);
}

void slightRight() {
    analogWrite(MOTOR_LEFT_FORWARD, 235);
    analogWrite(MOTOR_RIGHT_FORWARD, 255);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
    delay(10);
}

void turnRight() {
    analogWrite(MOTOR_LEFT_FORWARD, 255);
    analogWrite(MOTOR_RIGHT_FORWARD, 0);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 255);
    delay(10);
}

void turnLeft() {
    analogWrite(MOTOR_LEFT_FORWARD, 0);
    analogWrite(MOTOR_RIGHT_FORWARD, 255);
    analogWrite(MOTOR_LEFT_BACKWARD, 255);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
    delay(10);

}

void turnAround() {
    analogWrite(MOTOR_LEFT_FORWARD, 255);
    analogWrite(MOTOR_RIGHT_FORWARD, 0);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 255);
}

void stopMotors() {
    analogWrite(MOTOR_LEFT_FORWARD, 0);
    analogWrite(MOTOR_RIGHT_FORWARD, 0);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
}

void moveForward() {
    analogWrite(MOTOR_LEFT_FORWARD, 210);
    analogWrite(MOTOR_RIGHT_FORWARD, 204);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
}

//===================== GENERIC STUFF =====================//
void setup() {
    // LED Signals Setup
    strip_NI.begin();
    
    pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
    pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
    pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
    pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);

    for (int i = 0; i < SENSORS; i++) {
        pinMode(S[i], INPUT);
    }

    Serial.begin(9600);
}

void loop() {
    readSensors();

     for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(0, 255, 100));
    }
    strip_NI.show();
    turnLeft();
    delay(200);
    turnRight();
    delay(200);
         for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(255, 0, 100));
    }
    strip_NI.show(); 
    turnLeft();
    delay(200);
    turnRight();
    delay(200);
    turnLeft();
    delay(200);
         for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(0, 255, 0));
    }
    strip_NI.show();
    turnRight();
    delay(200);
    turnLeft();
    delay(1500);
    turnLeft();
    delay(200);
    turnRight();
    delay(200);
         for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(0, 100, 255));
    }
    strip_NI.show();
    turnLeft();
    delay(200);
    turnRight();
    delay(200);
         for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(0, 0, 255));
    }
    strip_NI.show();
    turnRight();
    delay(1500);
}
