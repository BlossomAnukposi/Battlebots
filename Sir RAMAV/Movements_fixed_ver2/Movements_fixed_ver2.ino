#include <Adafruit_NeoPixel.h>

//===================== NEOPIXELS =====================//
#define WHITE           255, 255, 255
#define RED             0  , 255, 0
#define GREEN           255, 0  , 0
#define BLUE            0  , 0  , 255
#define LIGHT_GREEN     255, 155, 0
#define YELLOW          255, 255, 0
#define OFF             0  , 0  , 0
#define NEOPIN_INPUT    7
#define NEO_PIXNUMBER   4

Adafruit_NeoPixel strip_NI(NEO_PIXNUMBER, NEOPIN_INPUT, NEO_GRB + NEO_KHZ800);

void signalOff()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(OFF));
    }
    strip_NI.show();
}

void signalForward()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(GREEN));
    }
    strip_NI.show();
}

void signalTurnAround()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(WHITE));
    }
    strip_NI.show();
}

void signalLeft()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(GREEN));
    }
    strip_NI.setPixelColor(2, strip_NI.Color(BLUE));
    strip_NI.show();
}

void signalRight()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(GREEN));
    }
    strip_NI.setPixelColor(3, strip_NI.Color(BLUE));
    strip_NI.show();
}

//=============== ANALOG LINE SENSORS ===============//
#define AVERAGE       600
#define SENSORS       8
bool    chooseLeft =  true;

int S[SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};
int IR[SENSORS];

void readSensors()
{
    // IR[0] = leftmost, IR[7] = rightmost
    for (int i = 0; i < SENSORS; i++)
    {
        IR[i] = analogRead(S[i]);
    }
}

//===================== ROTATION SENSORS =====================//
#define     ROTOR_RIGHT             2
#define     ROTOR_LEFT              3
#define     PULSES_PER_90DEG_LEFT   40
#define     PULSES_PER_90DEG_RIGHT  38
int         rightCount  =           0;
int         leftCount   =           0;

void rightPulse()
{
    rightCount++;
}

void leftPulse()
{
    leftCount++;
}

//===================== MOTORS =====================//
#define MOTOR_LEFT_FORWARD    6 // connected to A1
#define MOTOR_RIGHT_FORWARD   10 // connected to B2
#define MOTOR_LEFT_BACKWARD   5 // connected to A2
#define MOTOR_RIGHT_BACKWARD  9 // connected to B1

void slightLeft() 
{
    analogWrite(MOTOR_LEFT_FORWARD, 240);
    analogWrite(MOTOR_RIGHT_FORWARD, 255);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
}

void slightRight() 
{
    analogWrite(MOTOR_LEFT_FORWARD, 255);
    analogWrite(MOTOR_RIGHT_FORWARD, 225);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
}

void turnRight() 
{
    analogWrite(MOTOR_LEFT_FORWARD, 255);
    analogWrite(MOTOR_RIGHT_FORWARD, 0);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 255);
    chooseLeft = false;
    signalLeft();
}

void turnLeft() 
{
    analogWrite(MOTOR_LEFT_FORWARD, 0);
    analogWrite(MOTOR_RIGHT_FORWARD, 255);
    analogWrite(MOTOR_LEFT_BACKWARD, 255);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
    chooseLeft = true;
    signalRight();
}

void turnAround() 
{
    if (chooseLeft)
    {
        analogWrite(MOTOR_LEFT_FORWARD, 0);
        analogWrite(MOTOR_RIGHT_FORWARD, 255);
        analogWrite(MOTOR_LEFT_BACKWARD, 255);
        analogWrite(MOTOR_RIGHT_BACKWARD, 0);
    }

    else 
    {
        analogWrite(MOTOR_LEFT_FORWARD, 255);
        analogWrite(MOTOR_RIGHT_FORWARD, 0);
        analogWrite(MOTOR_LEFT_BACKWARD, 0);
        analogWrite(MOTOR_RIGHT_BACKWARD, 255);
    }
//    signalTurnAround();
}

void stopMotors() 
{
    analogWrite(MOTOR_LEFT_FORWARD, 0);
    analogWrite(MOTOR_RIGHT_FORWARD, 0);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
}

void adjustPath()
{
    if ((IR[2] > AVERAGE || IR[3] > AVERAGE) && IR[5] < AVERAGE) 
    {
        Serial.println("slightRight");
        slightRight();
    } 
    
    else if ((IR[5] > AVERAGE || IR[4] > AVERAGE) && IR[2] < AVERAGE) 
    {
        Serial.println("slightLeft");
        slightLeft();
    } 
}

void moveForward() 
{
    analogWrite(MOTOR_LEFT_FORWARD, 255);
    analogWrite(MOTOR_RIGHT_FORWARD, 255);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
    signalForward();
}

//===================== GENERIC STUFF =====================//
void setup() 
{
    pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
    pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
    pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
    pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);

    for (int i = 0; i < SENSORS; i++) 
    {
        pinMode(S[i], INPUT);
    }

    attachInterrupt(digitalPinToInterrupt(ROTOR_RIGHT), rightPulse, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTOR_LEFT), leftPulse, CHANGE);
    
    Serial.begin(9600);
}

void loop() {
    readSensors();

    if ((IR[0] > AVERAGE || IR[1] > AVERAGE) && (IR[6] > AVERAGE || IR[7] > AVERAGE)) 
    {
        Serial.println("Turn Right");
        turnRight();
    }

    else if (IR[6] > AVERAGE || IR[7] > AVERAGE) 
    {
        Serial.println("Left Turn");
        turnLeft();
    } 

    else if (IR[0] > AVERAGE || IR[1] > AVERAGE) 
    {
        Serial.println("Right Turn");
        turnRight();
    }
    
    else if (IR[0] < AVERAGE && IR[1] < AVERAGE && IR[2] < AVERAGE && IR[3] < AVERAGE && IR[4] < AVERAGE && IR[5] < AVERAGE && IR[6] < AVERAGE && IR[7] < AVERAGE) 
    {
        Serial.println("Turn Around");
        turnAround();
    }

    else if (IR[3] > AVERAGE || IR[4] > AVERAGE) 
    {
        if ((IR[0] < AVERAGE && IR[1] < AVERAGE) && (IR[6] < AVERAGE && IR[7] < AVERAGE))
        {
            Serial.println("Forward");
            moveForward();
        }
    } 

    adjustPath();
}
