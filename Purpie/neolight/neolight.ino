#include <Adafruit_NeoPixel.h>

#define PIN_NO 7 // Define the pin for NeoPixel strip 1
#define PIN_NI 8 // Define the pin for NeoPixel strip 2
#define NUM_PIXELS 4 // Number of pixels in each strip

// Create two instances of Adafruit_NeoPixel
Adafruit_NeoPixel strip_NO(NUM_PIXELS, PIN_NO, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_NI(NUM_PIXELS, PIN_NI, NEO_GRB + NEO_KHZ800);

void setup() {
  strip_NO.begin(); // Initialize NeoPixel strip 1
  strip_NI.begin(); // Initialize NeoPixel strip 2

  // Set all pixels to blue
  for (int i = 0; i < NUM_PIXELS; i++) {
    strip_NO.setPixelColor(i, strip_NO.Color(120, 0, 255));
    strip_NI.setPixelColor(i, strip_NI.Color(120, 0, 255));
  }

  strip_NO.show(); // Display the colors on strip 1
  strip_NI.show(); // Display the colors on strip 2
}

void loop() {
  // This example doesn't require any dynamic changes, so the loop is empty.
  // You can add your code here if needed.
}
