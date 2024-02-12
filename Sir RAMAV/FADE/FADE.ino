int led = 11;
int brightness = 0;       

void setup() {
  pinMode(led, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  for(brightness = 0; brightness <= 255; brightness += 5) {
    analogWrite(led, brightness);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }
  for(brightness = 255; brightness >= 0; brightness -= 5) {
    analogWrite(led, brightness);
    // wait for 30 milliseconds to see the dimming effect
    delay(30);
  }
}
