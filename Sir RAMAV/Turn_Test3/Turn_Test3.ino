//=============== ANALOG LINE SENSORS ===============//
#define AVERAGE 700
#define SENSORS 8

int S[SENSORS] = {A0, A1, A2, A3, A4, A5, A6, A7};
int IR[SENSORS];

void readSensors() {
    // IR[0] = leftmost, IR[7] = rightmost
    for (int i = 0; i < SENSORS; i++) {
        IR[i] = analogRead(S[i]);
    }
}

//===================== MOTORS =====================//
#define MOTOR_LEFT_FORWARD    6 // connected to A1
#define MOTOR_RIGHT_FORWARD   10 // connected to B2
#define MOTOR_LEFT_BACKWARD   5 // connected to A2
#define MOTOR_RIGHT_BACKWARD  9 // connected to B1

void slightLeft() {
    analogWrite(MOTOR_LEFT_FORWARD, 255);
    analogWrite(MOTOR_RIGHT_FORWARD, 250);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
    delay(10);
}

void slightRight() {
    analogWrite(MOTOR_LEFT_FORWARD, 235);
    analogWrite(MOTOR_RIGHT_FORWARD, 255);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
    delay(10);
}

void turnRight() {
    analogWrite(MOTOR_LEFT_FORWARD, 255);
    analogWrite(MOTOR_RIGHT_FORWARD, 0);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 255);
    delay(10);
}

void turnLeft() {
    analogWrite(MOTOR_LEFT_FORWARD, 0);
    analogWrite(MOTOR_RIGHT_FORWARD, 255);
    analogWrite(MOTOR_LEFT_BACKWARD, 255);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
    delay(10);

}

void turnAround() {
    analogWrite(MOTOR_LEFT_FORWARD, 255);
    analogWrite(MOTOR_RIGHT_FORWARD, 0);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 255);
}

void stopMotors() {
    analogWrite(MOTOR_LEFT_FORWARD, 0);
    analogWrite(MOTOR_RIGHT_FORWARD, 0);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
}

void moveForward() {
    analogWrite(MOTOR_LEFT_FORWARD, 210);
    analogWrite(MOTOR_RIGHT_FORWARD, 204);
    analogWrite(MOTOR_LEFT_BACKWARD, 0);
    analogWrite(MOTOR_RIGHT_BACKWARD, 0);
}

//===================== GENERIC STUFF =====================//
void setup() {
    pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
    pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
    pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
    pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);

    for (int i = 0; i < SENSORS; i++) {
        pinMode(S[i], INPUT);
    }

    Serial.begin(9600);
}

void loop() {
    readSensors();

    if (IR[6] > AVERAGE || IR[7] > AVERAGE) {
        Serial.println("Left Turn");
        turnLeft();
    } else if (IR[0] > AVERAGE || IR[1] > AVERAGE) {
        Serial.println("Right Turn");
        turnRight();
    } else if (IR[3] > AVERAGE || IR[4] > AVERAGE) {
        Serial.println("Forward");
        moveForward();
    } else if ((IR[2] > AVERAGE || IR[3] > AVERAGE) && IR[5] < AVERAGE) {
        Serial.println("slightRight");
        slightRight();
    } else if ((IR[5] > AVERAGE || IR[4] > AVERAGE) && IR[2] < AVERAGE) {
        Serial.println("slightLeft");
        slightLeft();
    } else if (IR[0] < AVERAGE && IR[1] < AVERAGE && IR[2] < AVERAGE && IR[3] < AVERAGE && IR[4] < AVERAGE && IR[5] < AVERAGE && IR[6] < AVERAGE && IR[7] < AVERAGE) {
        Serial.println("Turn Around");
        turnAround();
    } else {
        Serial.println("IDK");
    }
}
