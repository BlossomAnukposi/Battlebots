const int leftB = 10; //motor left - going backwards
const int leftF = 11; // motor left - going forward
const int rightB = 5; //motor right - going backwards
const int rightF = 6; //motor right - going forward

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

//===============================
//*SETUP*************
//===============================


//===============================
//*FUNCTIONS***********
//===============================
void goForward() {
  analogWrite(leftF, 255);
  analogWrite(rightF, 240);
}

void left(){
  analogWrite(leftF, 255);
  analogWrite(rightF, 180);
}

void right(){
  analogWrite(leftF, 180);
  analogWrite(rightF, 255);
}

void followLine(){
  readSensors();
  if(IR3 > 500 && IR4 > 500){
    analogWrite(leftF, 255);
    analogWrite(rightF, 120);
  }else if(IR5 > 500 && IR6 > 500){
    analogWrite(leftF, 120);
    analogWrite(rightF, 255);
  }else if(IR2 > 500 && IR3 > 500){
    analogWrite(leftF, 255);
    analogWrite(rightF, 0);
  }else if(IR6 > 500 && IR7 > 500){
    analogWrite(leftF, 0);
    analogWrite(rightF, 255);
  }else if(IR2 > 500 && IR1 > 500){
    analogWrite(leftF, 255);
    analogWrite(rightF, 0);
  }else if(IR7 > 500 && IR8 > 500){
    analogWrite(leftF, 0);
    analogWrite(rightF, 255);
  }
}

bool squareDetected(){
  if(IR1 > 800 && IR2 > 800 && IR3 > 800 && IR4 > 800 && IR5 > 800 && IR6 > 800 && IR7 > 800 && IR8 > 800){
    return true;
  }
}

void setupMotors(){
  //initializing motor pins
  pinMode(leftB, OUTPUT);
  pinMode(leftF, OUTPUT);
  pinMode(rightB, OUTPUT);
  pinMode(rightF, OUTPUT);
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
void setup(){
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
