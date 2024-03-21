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

//=============== ANALOG LINE SENSORSS ===============//
#define AVERAGE                    600
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


//===================== ROTATION SENSORS =====================//
#define ROTOR_RIGHT           2
#define ROTOR_LEFT            3

int readPosition = 0;
int lastRead = 0;
long lastReadTime = 0;

void rotorSensor() {
    int right =   digitalRead(ROTOR_RIGHT);
    int left =    digitalRead(ROTOR_LEFT);
    int reading = (right << 1) | left;
    int sum  =    (lastRead << 2) | reading;
    
    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) 
    {
        readPosition++;
    } 
    
    else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) 
    {
        readPosition--;
    }
    
    lastRead = reading;
    lastReadTime = millis();
}

//===================== MOTORS =====================//
#define MOTOR_LEFT_FORWARD    5 // connected to B2
#define MOTOR_RIGHT_FORWARD   10 // connected to A1
#define MOTOR_LEFT_BACKWARD   9 // connected to B1
#define MOTOR_RIGHT_BACKWARD  11 // connected to A2

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
    analogWrite(MOTOR_LEFT_FORWARD, 220);
    analogWrite(MOTOR_RIGHT_FORWARD, 0);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
}

void turnLeft()
{
    analogWrite(MOTOR_LEFT_FORWARD, 0);
    analogWrite(MOTOR_RIGHT_FORWARD, 210);
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
    if (IR[2] > AVERAGE && IR[3] > AVERAGE && IR[5] < AVERAGE)
    {
        slightRight();
    }

    else if (IR[2] < AVERAGE && IR[4] > AVERAGE && IR[5] > AVERAGE)
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
    pinMode(ROTOR_LEFT, INPUT);
    pinMode(ROTOR_RIGHT, INPUT);

    for (int i = 0; i < SENSORS; i++)
    {
        pinMode(IR[i], INPUT);
    }

    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);

    Serial.begin(9600);
}

void loop() 
{
      moveForward();
//    readSensors();

    if (IR[0] > AVERAGE && IR[1] > AVERAGE && IR[3] > AVERAGE && IR[6] < AVERAGE && IR[7] < AVERAGE)
    {
        turnRight();
        signalLeft();
//        delay(700);
    }

    readSensors();
    while (IR[0] < AVERAGE && IR[1] < AVERAGE && IR[4] > AVERAGE && IR[6] > AVERAGE && IR[7] > AVERAGE)
    {
        turnLeft();
        signalRight();
//        delay(700);
    }

    readSensors();
    if (IR[0] < AVERAGE && IR[1] < AVERAGE && IR[2] < AVERAGE && IR[3] < AVERAGE && IR[4] < AVERAGE && IR[5] < AVERAGE && IR[6] < AVERAGE && IR[7] < AVERAGE)
    {
        turnAround();
        signalTurnAround();
//        delay(1000);
    }

    readSensors();
    if (IR[0] > AVERAGE && IR[1] > AVERAGE && IR[3] > AVERAGE && IR[4] > AVERAGE && IR[6] > AVERAGE && IR[7] > AVERAGE)
    {
        turnRight();
        signalLeft();

        if (IR[0] < AVERAGE && IR[1] < AVERAGE && IR[3] > AVERAGE && IR[4] > AVERAGE && IR[6] < AVERAGE && IR[7] < AVERAGE)
        {
            moveForward();
            signalForward();
        }
    }

    readSensors();
    if (IR[0] < AVERAGE && IR[1] < AVERAGE && IR[3] > AVERAGE && IR[4] > AVERAGE && IR[6] < AVERAGE && IR[7] < AVERAGE)
    {
        moveForward();
        signalForward();
    }

    Serial.println(IR[0]);
    Serial.println(IR[1]);
    Serial.println(IR[6]);
    Serial.println(IR[7]);

}
