// Motor A
int motorPin1 = 11; // Connected to pin 2 on Arduino Nano
int motorPin2 = 10; // Connected to pin 3 on Arduino Nano

// Motor B
int motorPin3 = 6; // Connected to pin 4 on Arduino Nano
int motorPin4 = 5; // Connected to pin 5 on Arduino Nano

void setup() {
    // Set motor pins as output
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(motorPin3, OUTPUT);
    pinMode(motorPin4, OUTPUT);
}

void loop() {
    // Move both motors forward
      analogWrite(motorPin1, 235); //forword, left
      analogWrite(motorPin2, 0); //back
      analogWrite(motorPin3, 220); //forword, right
      analogWrite(motorPin4, 0); //back
}
