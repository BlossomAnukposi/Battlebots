  
int ledRed = 9; // only working with ~ pin

void setup() {
  pinMode(ledRed, OUTPUT);

}

void loop() {
  for (int brightness = 0; brightness <= 255; brightness += 5) {
    analogWrite(ledRed, brightness);
    delay(30);
  }

  for (int brightness = 255; brightness >= 0; brightness -= 5) {
    analogWrite(ledRed, brightness);
    delay(30);
  }
}
