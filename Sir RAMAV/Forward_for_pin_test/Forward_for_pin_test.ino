#define MOTOR_LEFT_FORWARD 10   // connected to B2
#define MOTOR_RIGHT_FORWARD 5  // connected to A1
#define MOTOR_LEFT_BACKWARD 9  // connected to B1
#define MOTOR_RIGHT_BACKWARD 6 // connected to A2
void setup() {
    pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
    pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
    pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
    pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);

}

void loop() {
    analogWrite(MOTOR_LEFT_FORWARD, 255);
    analogWrite(MOTOR_RIGHT_FORWARD, 249);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
}
