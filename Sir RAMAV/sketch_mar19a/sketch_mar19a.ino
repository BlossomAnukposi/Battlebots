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
        strip_NI.setPixelColor(j, strip_NI.Color(RED));
    }
    strip_NI.show();
}

//=============== ANALOG LINE SENSORSS ===============//
#define AVERAGE                    700
#define SENSORS                    8
int     S[SENSORS] =               {A0, A1, A2, A3, A4, A5, A6, A7};
int     IR[SENSORS];

void readSensors()
{
    //IR[0] = rightmost, IR[7] = leftmost
    for (int i = 0; i < SENSORS; i++)
    {
        IR[i] = analogRead(S[i]);
    }
}

//===================== CALIBRATION =====================//


//===================== MOTORS =====================//
#define MOTOR_LEFT_FORWARD    3 // connected to B2
#define MOTOR_RIGHT_FORWARD   5 // connected to A1
#define MOTOR_LEFT_BACKWARD   9 // connected to B1
#define MOTOR_RIGHT_BACKWARD  6 // connected to A2


void slightLeft()
{
    analogWrite(MOTOR_LEFT_FORWARD, 255);
    analogWrite(MOTOR_RIGHT_FORWARD, 250);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
}

void slightRight()
{
    analogWrite(MOTOR_LEFT_FORWARD, 255);
    analogWrite(MOTOR_RIGHT_FORWARD, 235);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
}

void turnRight()
{
    analogWrite(MOTOR_LEFT_FORWARD, 255);
    analogWrite(MOTOR_RIGHT_FORWARD, 0);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
}

void turnLeft()
{
    analogWrite(MOTOR_LEFT_FORWARD, 0);
    analogWrite(MOTOR_RIGHT_FORWARD, 255);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
}

void turnAround()
{
    analogWrite(MOTOR_LEFT_FORWARD, 0);
    analogWrite(MOTOR_RIGHT_FORWARD, 255);
    analogWrite(MOTOR_LEFT_BACKWARD, 255);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
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
    if (IR[3] > AVERAGE && IR[4] > AVERAGE && IR[6] < AVERAGE)
    {
        slightRight();
    }

    else if (IR[3] < AVERAGE && IR[5] > AVERAGE && IR[6] > AVERAGE)
    {
        slightLeft();
    }
}

void moveForward()
{
    adjustPath();
    analogWrite(MOTOR_LEFT_FORWARD, 255);
    analogWrite(MOTOR_RIGHT_FORWARD, 249);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
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
        pinMode(IR[i], INPUT);
    }

    Serial.begin(9600);
}

void loop() 
{
    readSensors();

    if (IR[1] > AVERAGE && IR[2] > AVERAGE && IR[4] > AVERAGE && IR[7] < AVERAGE && IR[8] < AVERAGE)
    {
        delay(100);
        turnRight();
    }
    
    else if (IR[1] < AVERAGE && IR[2] < AVERAGE && IR[5] > AVERAGE && IR[7] > AVERAGE && IR[8] > AVERAGE)
    {
        delay(100);
        turnLeft();
    }

    else if (IR[1] < AVERAGE && IR[2] < AVERAGE && IR[3] < AVERAGE && IR[4] < AVERAGE && IR[5] < AVERAGE && IR[6] < AVERAGE && IR[7] < AVERAGE && IR[8] < AVERAGE)
    {
        delay(100);
        turnAround();
    }

    else
    {
        adjustPath();
    }

    moveForward(); //------ Change (A): Removed move forward from the loop to make sure the turns execute by all means ------//
    signalOff();
}
