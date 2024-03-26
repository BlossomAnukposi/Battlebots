int left = 5;
int leftBack = 6;
int right = 3;

void setup() {
  pinMode(left, OUTPUT);
  pinMode(right, OUTPUT);
}

void loop() {
  analogWrite(left, 255);
  analogWrite(right, 250);
  delay(2000);
  analogWrite(left, 0);
    analogWrite(leftBack, 255);
  delay(900);
      analogWrite(leftBack, 0);

}
