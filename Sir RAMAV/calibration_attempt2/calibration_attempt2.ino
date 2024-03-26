const int numSensors = 8;  // Number of analog sensors
int sensorPins[numSensors] = {A0, A1, A2, A3, A4, A5, A6, A7}; // Analog sensor pins
int sensorValues[numSensors];  // Array to store sensor values
int blackValues[numSensors];   // Array to store calibrated black values
int whiteValues[numSensors];   // Array to store calibrated white values
int thresholdValues[numSensors]; // Array to store threshold values

void setup() {
  Serial.begin(9600);  // Initialize serial communication for debugging
  // Initialize analog sensor pins as inputs
  for (int i = 0; i < numSensors; i++) {
    pinMode(sensorPins[i], INPUT);
  }
}

void loop() {
  // Calibrate black and white values
  calibrateSensors();

  // Print calibrated values for debugging
  printCalibratedValues();

  // Wait for a short time
  delay(1000);
}

void calibrateSensors() {
  Serial.println("Calibrating sensors...");
  Serial.println("Move the robot over black and white lines...");

  // Initialize min and max arrays
  int minValues[numSensors];
  int maxValues[numSensors];
  for (int i = 0; i < numSensors; i++) {
    minValues[i] = 1023;
    maxValues[i] = 0;
  }

  // Read sensor values while moving over lines
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {  // Calibration for 5 seconds
    readSensorValues();
    for (int i = 0; i < numSensors; i++) {
      // Update min and max values
      if (sensorValues[i] < minValues[i]) {
        minValues[i] = sensorValues[i];
      }
      if (sensorValues[i] > maxValues[i]) {
        maxValues[i] = sensorValues[i];
      }
    }
  }

  // Calculate black and white values based on min and max
  for (int i = 0; i < numSensors; i++) {
    blackValues[i] = minValues[i];
    whiteValues[i] = maxValues[i];
    // Calculate threshold value
    thresholdValues[i] = (blackValues[i] + whiteValues[i]) / 2;
  }

  Serial.println("Calibration complete.");
}

void readSensorValues() {
  // Read analog sensor values
  for (int i = 0; i < numSensors; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }
}

void printCalibratedValues() {
  // Print calibrated values for debugging
  Serial.println("Calibrated Values:");
  Serial.print("Black Values: ");
  for (int i = 0; i < numSensors; i++) {
    Serial.print(blackValues[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("White Values: ");
  for (int i = 0; i < numSensors; i++) {
    Serial.print(whiteValues[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("Threshold Values: ");
  for (int i = 0; i < numSensors; i++) {
    Serial.print(thresholdValues[i]);
    Serial.print(" ");
  }
  Serial.println();
}
