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

//=========================[ SPECIAL SEQUENCES ]=========================//
bool          raceStarted     = false;
bool          raceEnded       = false;
bool          chooseLeft      = false;

//===============[ ANALOG LINE SENSORS ]===============//
#define SENSORS 8
int S[SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};
int IR[SENSORS];

//=====================[ ULTRASONIC SENSOR SETUP ]=====================//
#define TRIGGER_PIN   12
#define ECHO_PIN      4
long    duration;
int     distance;

//=====================[ MOTORS ]=====================//
#define MOTOR_LEFT_FORWARD    9 // connected to A1
#define MOTOR_RIGHT_FORWARD   6 // connected to B2
#define MOTOR_LEFT_BACKWARD   5 // connected to A2
#define MOTOR_RIGHT_BACKWARD  10 // connected to B1

//===================== GRIPPER =====================//
#define GRIPPER_PIN   11

//===================== CALIBRATION SETUP =====================//
bool  calibrated =    false;
int   white[SENSORS];
int   black[SENSORS];
int   threshold[SENSORS];
bool  whiteSeen =     false;
bool  blackSeen =     false;

//=====================[ GENERIC STUFF ]=====================//
void setup()
{
    pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
    pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
    pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
    pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    for (int i = 0; i < SENSORS; i++)
    {
        pinMode(S[i], INPUT);
    }

    for (int i = 0; i < SENSORS; i++) 
    {
        white[i] = 0;
        black[i] = 0;
    }

    setServoAngle(150);
    
    Serial.begin(9600);
}

void loop()
{
    if (!calibrated)
    {
        stopMotors();
        signalWait();
        distanceReader();
        Serial.println(distance);
        
        if (distance <= 25)
        {
            calibrate();
        }
    }

    else if (!raceEnded)
    {
        readSensors();
        
        if (IR[0] > threshold[0] && IR[1] > threshold[1] && IR[2] > threshold[2] && IR[3] > threshold[3] && IR[4] > threshold[4] && IR[5] > threshold[5] && IR[6] > threshold[6] && IR[7] > threshold[7])
        {
          moveForward();
          delay(500);
          stopMotors();
          readSensors();
          if (IR[0] > threshold[0] && IR[1] > threshold[1] && IR[2] > threshold[2] && IR[3] > threshold[3] && IR[4] > threshold[4] && IR[5] > threshold[5] && IR[6] > threshold[6] && IR[7] > threshold[7])
          {
            Serial.println("end");
            endRace();
          }
          else
          {
            turnLeft();
            delay(550);
          }
        } 
        else if (IR[6] > threshold[6] || IR[7] > threshold[7])
        {
            Serial.println("Left Turn");
            turnLeft();
        }
        
        else if (IR[0] > threshold[0] || IR[1] > threshold[1])
        {
            Serial.println("Right Turn");
            turnRight();
        } 
        
        else if (IR[3] > threshold[3] || IR[4] > threshold[4])
        {
            Serial.println("Forward");
            moveForward();
        } 
        
        else if ((IR[2] > threshold[2] || IR[3] > threshold[3]) && IR[5] < threshold[5])
        {
            Serial.println("slightRight");
            slightRight();
        } 
        
        else if ((IR[5] > threshold[5] || IR[4] > threshold[4]) && IR[2] < threshold[2])
        {
            Serial.println("slightLeft");
            slightLeft();
        } 
        
        else if (IR[0] < threshold[0] && IR[1] < threshold[1] && IR[2] < threshold[2] && IR[3] < threshold[3] && IR[4] < threshold[4] && IR[5] < threshold[5] && IR[6] < threshold[6] && IR[7] < threshold[7])
        {
            Serial.println("Turn Around");
            turnAround();
        } 
        
        else
        {
            Serial.println("IDK");
        }
    }
}
//=====================[ ULTRASONIC SENSOR FUNCTION ]=====================//
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

//===============[ ANALOG LINE SENSOR FUNCTIONS ]===============//
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
    if (threshold[1] == 0 && white [7] == 0)
    {
        for (int i = 0; i < SENSORS; i++)
        {
            white[i] = IR[i];
            Serial.print("white: ");
            Serial.println(white[i]);
        }
    }

    else
    {
        for (int i = 0; i < SENSORS; i++)
        {
            int whiteAdded = white[i] + IR[i];
            white[i] = whiteAdded / 2;
            Serial.print("white: ");
            Serial.println(white[i]);
        }
    }
}

void recordBlack()
{
    if (threshold[1] == 0 && threshold[7] == 0)
    {
        for (int i = 0; i < SENSORS; i++)
        {
            black[i] = IR[i];
            Serial.print("black: ");
            Serial.println(black[i]);
        }
    }

    else
    {
        for (int i = 0; i < SENSORS; i++)
        {
            int blackAdded = black[i] + IR[i];
            black[i] = blackAdded / 2;
            Serial.print("black: ");
            Serial.println(black[i]);
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
    }

    signalStarting();
    delay(150);
    
    for (int i = 0; i < SENSORS; i++)
    {
        threshold[i] = (white[i] + black[i]) / 2;
        Serial.print("threshold");
        Serial.println(threshold[i]);
    }

    calibrated = true;
    startRace();
}

//=====================[ GRIPPER FUNCTION ]=====================//
void setServoAngle(int angle) 
{
    int pulseWidth = map(angle, 0, 180, 0, 255); 
    analogWrite(GRIPPER_PIN, pulseWidth);
}

//=====================[ MOTOR FUNCTIONS ]=====================//
void slightLeft()
{
    analogWrite(MOTOR_LEFT_FORWARD, 255);
    analogWrite(MOTOR_RIGHT_FORWARD, 250);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
    delay(15);
}

void slightRight()
{
    analogWrite(MOTOR_LEFT_FORWARD, 235);
    analogWrite(MOTOR_RIGHT_FORWARD, 255);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
    delay(15);
}

void turnRight()
{
    analogWrite(MOTOR_LEFT_FORWARD, 255);
    analogWrite(MOTOR_RIGHT_FORWARD, 0);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 255);
    delay(20);
    signalRight();
}

void turnLeft()
{
    analogWrite(MOTOR_LEFT_FORWARD, 0);
    analogWrite(MOTOR_RIGHT_FORWARD, 255);
    analogWrite(MOTOR_LEFT_BACKWARD, 255);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
    delay(20);
    signalLeft();
}

void turnAround()
{
    analogWrite(MOTOR_LEFT_FORWARD, 255);
    analogWrite(MOTOR_RIGHT_FORWARD, 0);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 255);
}

void stopMotors()
{
    analogWrite(MOTOR_LEFT_FORWARD, 0);
    analogWrite(MOTOR_RIGHT_FORWARD, 0);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
}

void moveForward()
{
    analogWrite(MOTOR_LEFT_FORWARD, 210);
    analogWrite(MOTOR_RIGHT_FORWARD, 225);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
    signalForward();
}

void moveCalibrate() 
{
    analogWrite(MOTOR_LEFT_FORWARD, 230);
    analogWrite(MOTOR_RIGHT_FORWARD, 226);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
    signalCalibrate();
}

void reverse() 
{
    analogWrite(MOTOR_LEFT_FORWARD, 0);
    analogWrite(MOTOR_RIGHT_FORWARD, 0);
    analogWrite(MOTOR_LEFT_BACKWARD, 255);
    analogWrite(MOTOR_RIGHT_BACKWARD, 245);
    signalReverse();
}

//=========================[ SPECIAL SEQUENCES ]=========================//
void startRace()
{
    moveForward();
    delay(600);
    setServoAngle(100);
    turnLeft();
    delay(350);
    moveForward();
    delay(350);
    raceStarted = true;
}

void endRace()
{
    reverse();
    delay(500);
    stopMotors();
    signalStop();
    delay(500);
    raceEnded = true;
}
