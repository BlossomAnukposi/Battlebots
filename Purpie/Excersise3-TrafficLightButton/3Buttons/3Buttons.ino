/*
  Button

  Turns on and off a light emitting diode(LED) connected to digital pin 13,
  when pressing a pushbutton attached to pin 2.

  The circuit:
  - LED attached from pin 13 to ground through 220 ohm resistor
  - pushbutton attached to pin 2 from +5V
  - 10K resistor attached to pin 2 from ground

  - Note: on most Arduinos there is already an LED on the board
    attached to pin 13.

  created 2005
  by DojoDave <http://www.0j0.org>
  modified 30 Aug 2011
  by Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Button
*/

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 2;     
const int ledPin =  7;      

const int buttonYellow = 12;     
const int ledYellow = 8;   

const int buttonGreen = 11;     
const int ledGreen =  9;   

// variables will change:
int buttonState = 0;
int buttonState2 = 0;
int buttonState3 = 0;

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  pinMode(buttonYellow, INPUT);
  pinMode(buttonGreen, INPUT);
}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);
  buttonState2 = digitalRead(buttonYellow);
  buttonState3 = digitalRead(buttonGreen);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(ledPin, HIGH);
  } else {
    // turn LED off:
    digitalWrite(ledPin, LOW);
  }

  if (buttonState2 == HIGH) {
    // turn LED on:
    digitalWrite(ledYellow, HIGH);
  } else {
    // turn LED off:
    digitalWrite(ledYellow, LOW);
  }

  if (buttonState3 == HIGH) {
    // turn LED on:
    digitalWrite(ledGreen, HIGH);
  } else {
    // turn LED off:
    digitalWrite(ledGreen, LOW);
  }
}
