// Motors
const int lForward = 5; // connected to B2
const int rForward = 3; // connected to A1
const int lBackward = 6; // connected to B1
const int rBackward = 9; // connected to A2


void setup() {
  // put your setup code here, to run once:
  pinMode(lForward, OUTPUT);
  pinMode(rForward, OUTPUT);
  pinMode(lBackward, OUTPUT);
  pinMode(rBackward, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
      analogWrite(lForward, 255);
      analogWrite(rForward, 250);
      delay(1000);
      analogWrite(lForward, 0);
      analogWrite(rForward, 0);
}
