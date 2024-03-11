const int trigPin = 4;
const int echoPin = 9;
const int servoPin = 12;

const int neoPin_NO = 7;
const int neoPin_NI = 8;
const int numNeoPix = 4;


int leftFmp1 = 11;
int leftBmp2 = 10;
int rightFmp3 = 6;
int rightBmp4 = 5;

int brightnessLevel = 25;

const int closedGrip = 60;
const int openGrip = 120;

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

int leftValue = 0;
int rightValue = 0;
int adjustSpeed = 0;

void setup() {

   Serial.begin(9600); // Initializes the serial communication
    setupIRSensors();
    setupMotors();
}

// ============= loop ===============
void loop() {
  followLine();
  
  
  long duration;
  digitalWrite(trigPin, LOW);
  digitalWrite(trigPin, HIGH);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  float theSpeedOfSound = 0.034;
  int distance = duration * theSpeedOfSound / 2;
  Serial.println();
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm");
  delay(500);
  if (distance < 7) { // If an object is detected within 10cm
    openGripper(); // Open the gripper
    delay(500); // Wait for 2 seconds
    closeGripper(); // Close the gripper
    delay(500); // Wait for 1 second
  }
}

//====== functions ==========

void followLine(){
  readSensors();
  if(IR3 > 500 && IR4 > 500){
    analogWrite(leftFmp1, 255);
    analogWrite(rightFmp3, 120);
  }else if(IR5 > 500 && IR6 > 500){
    analogWrite(leftFmp1, 120);
    analogWrite(rightFmp3, 255);
  }else if(IR2 > 500 && IR3 > 500){
    analogWrite(leftFmp1, 255);
    analogWrite(rightFmp3, 0);
  }else if(IR6 > 500 && IR7 > 500){
    analogWrite(leftFmp1, 0);
    analogWrite(rightFmp3, 255);
  }else if(IR2 > 500 && IR1 > 500){
    analogWrite(leftFmp1, 255);
    analogWrite(rightFmp3, 0);
  }else if(IR7 > 500 && IR8 > 500){
    analogWrite(leftFmp1, 0);
    analogWrite(rightFmp3, 255);
  }
}




bool squareDetected() {
  if (IR1 > 800 && IR2 > 800 && IR3 > 800 && IR4 > 800 && IR5 > 800 && IR6 > 800 && IR7 > 800 && IR8 > 800) {
    return true;
  }
}


void moveBackward() {
  analogWrite(leftBmp2, 255);
  analogWrite(rightBmp4, 255);
}

void moveForward() {
  analogWrite(leftFmp1, 255);
  analogWrite(rightFmp3, 240);
}

void turnLeft() {
  analogWrite(leftFmp1, 255);
  analogWrite(rightFmp3, 180);
}

void turnRight() {
  analogWrite(leftFmp1, 180);
  analogWrite(rightFmp3, 255);
}

void openGripper() {
  setServoAngle(openGrip);
}

void closeGripper() {
  setServoAngle(closedGrip);
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
  int pulseWidth = map(angle, 0, 180, 1000, 2000);
  digitalWrite(servoPin, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(servoPin, LOW);
}
