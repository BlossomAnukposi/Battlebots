int ledRed = 11;
int ledGreen = 13;
int ledYellow = 12;

void setup() {
  // led modes output
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledYellow, OUTPUT);
}

void loop() {
  // turn on the red light for 3 seconds
  digitalWrite(ledRed, LOW);
  delay(1000);
  digitalWrite(ledRed, HIGH);
  digitalWrite(ledGreen, LOW);
  delay(1000);
  digitalWrite(ledGreen, HIGH);
  digitalWrite(ledYellow, LOW);
  delay(1000);
  digitalWrite(ledYellow, HIGH);

  //add a random delay after each sequence
  delay(1000);
}
