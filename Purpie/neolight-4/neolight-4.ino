#include <Adafruit_NeoPixel.h>

#define NEOPIXEL_PIN_NO 7
#define NEOPIXEL_PIN_NI 8
#define NEO_PIXNUMBER 4

Adafruit_NeoPixel strip_NO(NEO_PIXNUMBER, NEOPIXEL_PIN_NO, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_NI(NEO_PIXNUMBER, NEOPIXEL_PIN_NI, NEO_GRB + NEO_KHZ800);

void setup() {
    strip_NO.begin();
    strip_NI.begin();
}

void loop() {
    // Set colors for NO strip
    setNOColors();
    strip_NO.show();
    delay(1000);

    // Set colors for NI strip
    setNIColors();
    strip_NI.show();
    delay(1000);
}

void setNOColors() {
    // Set colors for NO strip individually
    setNOColorPixel(0, 0, 255, 0); // Pixel 0: Red
    setNOColorPixel(1, 255, 0, 0); // Pixel 1: Green
    setNOColorPixel(2, 0, 0, 255); // Pixel 2: Blue
    setNOColorPixel(3, 255, 255, 0); // Pixel 3: Yellow
}

void setNIColors() {
    // Set colors for NI strip individually
    setNIColorPixel(0, 0, 255, 0); // Pixel 0: Red
    setNIColorPixel(1, 255, 0, 0); // Pixel 1: Green
    setNIColorPixel(2, 0, 0, 255); // Pixel 2: Blue
    setNIColorPixel(3, 255, 255, 0); // Pixel 3: Yellow
}

void setNOColorPixel(int pixel, int red, int green, int blue) {
    strip_NO.setPixelColor(pixel, red, green, blue);
}

void setNIColorPixel(int pixel, int red, int green, int blue) {
    strip_NI.setPixelColor(pixel, red, green, blue);
}
