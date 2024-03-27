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

//=============== ANALOG LINE SENSOR SETUP ===============//
#define AVERAGE       600
#define SENSORS       8
bool    chooseLeft =  true;

int S[SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};
int IR[SENSORS];

//===================== CALIBRATION SETUP =====================//
bool calibrated = false;
int white[SENSORS];
int black[SENSORS];
int threshold[SENSORS];
bool whiteSeen = false;
bool blackSeen = false;

//===================== ULTRASONIC SENSOR SETUP =====================//
#define TRIGGER_PIN   12
#define ECHO_PIN      4
long duration;
int distance;

//===================== MOTORS =====================//
#define MOTOR_LEFT_FORWARD    6 // connected to A1
#define MOTOR_RIGHT_FORWARD   10 // connected to B2
#define MOTOR_LEFT_BACKWARD   5 // connected to A2
#define MOTOR_RIGHT_BACKWARD  9 // connected to B1

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

    pinMode(ECHO_PIN, INPUT);
    pinMode(TRIGGER_PIN, OUTPUT);

    for (int i = 0; i < SENSORS; i++) 
    {
        white[i] = 0;
        black[i] = 0;
    }
    
    Serial.begin(9600);
}

void loop() { 
    distanceReader();
    Serial.print(distance);
    Serial.println(" cm");

    if (!calibrated)
    {
        stopMotors();
        signalWait();
        distanceReader();
        
        if (distance < 27)
        {
            calibrate();
        }
    }

    else if (calibrated)
    {
        readSensors();
        
        if (IR[0] > AVERAGE && IR[1] > AVERAGE && IR[6] > AVERAGE && IR[7] > AVERAGE) 
        {
            turnRight();
            delay(150);
        }
        
        else if (IR[0] > AVERAGE || IR[1] > AVERAGE) 
        {
            turnRight();
            delay(150);
        }
        
        else if ((IR[1] < AVERAGE) && (IR[3] > AVERAGE || IR[4] > AVERAGE) && (IR[6] < AVERAGE)) 
        {
            moveForward();
            adjustPath();
        }
    
        else if ((IR[7] > AVERAGE || IR[6] > AVERAGE) && IR[0] < AVERAGE)
        {
            chooseLeft = true;
            if (IR[0] < AVERAGE && IR[2] < AVERAGE && IR[4] < AVERAGE && IR[6] < AVERAGE)
            {
                turnLeft();
            }
        }
    
        else if (IR[0] < AVERAGE && IR[1] < AVERAGE && IR[2] < AVERAGE && IR[3] < AVERAGE && IR[4] < AVERAGE && IR[5] < AVERAGE && IR[6] < AVERAGE && IR[7] < AVERAGE) 
        {
            turnAround();
        }
    
        readSensors();
        adjustPath();
    }
}

//===================== NEOPIXEL FUNCTIONS =====================//
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

void signalStop()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(RED));
    }
    strip_NI.show();
}

void signalWait()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(WHITE));
    }
    strip_NI.show();
}

void signalCalibrate()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(CALIBRATE));
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

//=============== ANALOG LINE SENSOR FUNCTION ===============//
void readSensors()
{
    // IR[0] = leftmost, IR[7] = rightmost
    for (int i = 0; i < SENSORS; i++)
    {
        IR[i] = analogRead(S[i]);
    }
}

void recordWhite()
{
    if (white[1] == 0 && white [7] == 0)
    {
        for (int i = 0; i < SENSORS; i++)
        {
            white[i] = IR[i];
        }
    }

    else
    {
        for (int i = 0; i < SENSORS; i++)
        {
            int whiteAdded = white[i] + IR[i];
            white[i] = whiteAdded / 2;
        }
    }
}

void recordBlack()
{
    if (black[1] == 0 && black[7] == 0)
    {
        for (int i = 0; i < SENSORS; i++)
        {
            black[i] = IR[i];
        }
    }

    else
    {
        for (int i = 0; i < SENSORS; i++)
        {
            int blackAdded = black[i] + IR[i];
            black[i] = blackAdded / 2;
        }
    }
}

void checkBlack()
{
    if (IR[1] > white[1] + 100 && IR[7] > white[7] + 100 && IR[3] > white[3] + 100)
    {
        blackSeen = true;
        whiteSeen = false;
    }
}

void checkWhite()
{
    if (IR[1] < black[1] + 100 && IR[7] < black[7] + 100 && IR[3] < black[3] + 100)
    {
        whiteSeen = true;
        blackSeen = false;
    }
}

void calibrate()
{
    readSensors();
    recordWhite();
        
    for (int i = 0; i < 3; i++)
    {
        readSensors();
        checkBlack();
        checkWhite();

        while (!blackSeen)
        {
            moveCalibrate();
            readSensors();
            checkBlack();
        }

        stopMotors();
        delay(200);

        if (blackSeen)
        {
            recordBlack();
        }

        while (!whiteSeen)
        {
            moveCalibrate();
            readSensors();
            checkWhite();
        }

        stopMotors();
        delay(200);
        
        if (whiteSeen)
        {
            recordWhite();
        }

        signalForward();
        delay(500);
    }

    signalStop();
    delay(1000);
    for (int i = 0; i < SENSORS; i++)
    {
        threshold[i] = white[i] + black[i] / 2;
        Serial.print("threshold");
        Serial.println(threshold[i]);
    }

    calibrated = true;
    startRace();
}

//===================== ULTRASONIC SENSOR FUNCTION =====================//
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

//===================== MOTOR FUNCTIONS =====================//
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
    readSensors();
    
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

void moveCalibrate() 
{
    analogWrite(MOTOR_LEFT_FORWARD, 255);
    analogWrite(MOTOR_RIGHT_FORWARD, 240);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
    signalCalibrate();
}

void startRace()
{
    moveForward();
    delay(600);
    turnLeft();
    delay(350);
}
