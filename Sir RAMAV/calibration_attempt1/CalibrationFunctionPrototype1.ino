void readSensors() {
  int sensor1 = analogRead(S1);
  int sensor2 = analogRead(S2);
  int sensor3 = analogRead(S3);
  int sensor4 = analogRead(S4);
  int sensor5 = analogRead(S5);
  int sensor6 = analogRead(S6);
  int sensor7 = analogRead(S7);
  int sensor8 = analogRead(S8);
}

void calibration() {
  valuesS1[0] = sensor1;
  for (int i = 1; i <= counter; i++){
    if (sensor1 > valuesS1[i-1] - 150 or sensor1 < valuesS1[i-1]-150){
      valuesS1[i] = sensor1;
    }
    else{
      counter += 1;
      Serial.println(counter);
    }
  }
  
  int sumOfArayS1 = 0;    //sum of array values
  for (int j = 0; j < 4; j++){
    sumOfArayS1 += valuesS1[j];
    }
  int averageValueS1 = sumOfArayS1 / 4;
  Serial.println(averageValueS1);
}
