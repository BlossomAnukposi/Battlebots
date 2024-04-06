#define MOTOR_LEFT_FORWARD    9 // connected to A1
#define MOTOR_RIGHT_FORWARD   6 // connected to B2
#define MOTOR_LEFT_BACKWARD   5 // connected to A2
#define MOTOR_RIGHT_BACKWARD  10 // connected to B1
void setup() {
    pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
    pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
    pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
    pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);

}

void loop() {
    analogWrite(MOTOR_LEFT_FORWARD, 210);
    analogWrite(MOTOR_RIGHT_FORWARD, 225);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
}
