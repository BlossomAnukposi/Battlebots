// NeoPixels
const int neoPin_NO = 7; 
const int neoPin_NI = 8; 
const int numNeoPix = 4; 

int brightnessLevel = 25; 

// Gripper
const int servoPin = 12;

//Ultrasonic sensor
const int trigPin = 4; 
const int echoPin = 9; 

// Motor
int leftFmp1 = 11;
int leftBmp2 = 10; 
int rightFmp3 = 6;
int rightBmp4 = 5; 


//Gripper
const int closedGrip = 60; 
const int openGrip = 120;


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


// ==========[ SETUP ]=====================

void setup() {
    // Set the servo pin as an output
    pinMode(servoPin, OUTPUT);
    // Initialize motors and sensors
    setupMotors();
    setupIRSensors();
}

// ==========[ LOOP ]=====================

void loop() {
    followLine();
    
    // Trigger the ultrasonic sensor
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Open the grip (90 degrees)
    setServoAngle(90);
    delay(2000); // Wait for 2 seconds

    // Close the grip (60 degrees)
    setServoAngle(60);
    delay(1000); // Wait for 1 second
}

// ==========[ FUNCTIONS ]=====================

void moveBackward () {
    analogWrite(leftBmp2, 255);
    analogWrite(rightBmp4, 255);
}

void moveForward() {
    analogWrite(leftFmp1, 255);
    analogWrite(rightFmp3 , 240);
}

void turnLeft(){
    analogWrite(leftFmp1, 255);
    analogWrite(rightFmp3 , 180);
}

void turnRight() {
    analogWrite(leftFmp1, 180);
    analogWrite(rightFmp3 , 255);
}

// Auto follow line
void followLine() {
    readSensors();
    if(IR3 > 500 && IR4 > 500) {
        analogWrite(leftFmp1, 255);
        analogWrite(rightFmp3, 120);
    } else if(IR5 > 500 && IR6 > 500) {
        analogWrite(leftFmp1, 120);
        analogWrite(rightFmp3, 255);
    } else if(IR2 > 500 && IR3 > 500) {
        analogWrite(leftFmp1, 255);
        analogWrite(rightFmp3, 0);
    } else if(IR6 > 500 && IR7 > 500) {
        analogWrite(leftFmp1, 0);
        analogWrite(rightFmp3, 255);
    } else if(IR2 > 500 && IR1 > 500) {
        analogWrite(leftFmp1, 255);
        analogWrite(rightFmp3, 0);
    } else if(IR7 > 500 && IR8 > 500) {
        analogWrite(leftFmp1, 0);
        analogWrite(rightFmp3, 255);
    }
}

bool squareDetected() {
    if(IR1 > 800 && IR2 > 800 && IR3 > 800 && IR4 > 800 && IR5 > 800 && IR6 > 800 && IR7 > 800 && IR8 > 800) {
        return true;
    }
}

void setupMotors() {
    pinMode(leftBmp2, OUTPUT);
    pinMode(leftFmp1, OUTPUT);
    pinMode(rightBmp4, OUTPUT);
    pinMode(rightFmp3, OUTPUT);
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

void setServoAngle(int angle) {
    // Convert angle to pulse width
    int pulseWidth = map(angle, 0, 180, 1000, 2000); 

    // Send the pulse to the servo pin
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(servoPin, LOW);
}
