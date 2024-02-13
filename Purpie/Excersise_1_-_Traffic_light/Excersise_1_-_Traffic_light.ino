/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/
int ledRed = 7;
int ledYellow = 8;
int ledGreen = 9;
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(ledRed, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(ledGreen, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
    digitalWrite(ledRed, LOW);
    delay(3000); 
    digitalWrite(ledRed, HIGH);
    delay(500);

    digitalWrite(ledGreen, LOW);
    delay(4000); 
    digitalWrite(ledGreen, HIGH);
    delay(500);

    digitalWrite(ledYellow, LOW);
    delay(1000); 
    digitalWrite(ledYellow, HIGH);
    delay(500);
}
