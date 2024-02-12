int left = 5;
int right = 3;

void setup() {
  pinMode(left, OUTPUT);
  pinMode(right, OUTPUT);
}

void loop() {
  analogWrite(left, 255);
  analogWrite(right, 250);
}
