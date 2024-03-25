void signalForward()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(GREEN));
    }
    strip_NI.show();
}

void signalWaiting()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(WHITE));
    }
    strip_NI.show();
}

void signalOff()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(OFF));
    }
    strip_NI.show();
}

void signalStop()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(RED));
    }
    strip_NI.show();
}

void signalCalibrate()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(CALIBRATE));
    }
    strip_NI.show();
}

void signalCalibrationBegin()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(OFF));
    }
    strip_NI.show();
    strip_NI.setPixelColor(1, strip_NI.Color(RED));
    strip_NI.show();
    delay(1000);
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(OFF));
    }
    strip_NI.show();
    strip_NI.setPixelColor(2, strip_NI.Color(YELLOW));
    strip_NI.show();
    delay(1000);
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(OFF));
    }
    strip_NI.show();
    strip_NI.setPixelColor(3, strip_NI.Color(GREEN));
    strip_NI.show();
    delay(1000);
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(OFF));
    }
    strip_NI.show();
    strip_NI.setPixelColor(0, strip_NI.Color(WHITE));
    strip_NI.show();
    delay(1000);
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(OFF));
    }
    strip_NI.show();
    delay(1000);
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(CALIBRATE));
    }
    strip_NI.show();
    delay(2000);
}

void signalCalibrationBegin2()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(OFF));
    }
    strip_NI.show();
    strip_NI.setPixelColor(0, strip_NI.Color(BLUE));
    strip_NI.show();
    delay(500);
    strip_NI.setPixelColor(0, strip_NI.Color(CALIBRATE));
    strip_NI.show();
    delay(300);
    strip_NI.setPixelColor(1, strip_NI.Color(BLUE));
    strip_NI.show();
    delay(500);
    strip_NI.setPixelColor(1, strip_NI.Color(CALIBRATE));
    strip_NI.show();
    delay(300);
    strip_NI.setPixelColor(2, strip_NI.Color(BLUE));
    strip_NI.show();
    delay(500);
    strip_NI.setPixelColor(2, strip_NI.Color(CALIBRATE));
    strip_NI.show();
    delay(300);
    strip_NI.setPixelColor(3, strip_NI.Color(BLUE));
    strip_NI.show();
    delay(500);
    strip_NI.setPixelColor(3, strip_NI.Color(CALIBRATE));
    strip_NI.show();
    delay(300);
}

void signalRight()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(GREEN));
    }
    strip_NI.setPixelColor(2, strip_NI.Color(BLUE));
    strip_NI.show();
}

void signalLeft()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(GREEN));
    }
    strip_NI.setPixelColor(3, strip_NI.Color(BLUE));
    strip_NI.show();
}

void calibrateSensors()
{
    int minValues[SENSORS];
    int maxValues[SENSORS];
    
    readSensors();
    signalCalibrationBegin2();
    
    for (int i = 0; i < SENSORS; i++)
    {
        minValues[i] = IR[i];
        maxValues[i] = 0;
    }

    delay(3000);
    readSensors();
    signalCalibrate();
    moveCalibrate();

    while (!calibrated)
    {
        moveCalibrate();
        const unsigned long timer = 1000;
        unsigned long loopTime = millis();
        unsigned long previousTime = 0;
        if (loopTime - previousTime >= timer)
        {
            calculateAvg(minValues, maxValues);
            delay(300);
            calibrated = true;
            stopMotors();
        }
        
        
        else if (IR[0] > minValues[0] + 100 && IR[7] > minValues[7] + 100)
        {
            calculateAvg(maxValues, IR);
        }
    
        else if (IR[0] < maxValues[0] + 100 && IR[7] < maxValues[7] - 100)
        {
            calculateAvg(minValues, IR);
        }
        previousTime = loopTime;
        objDistance++;
    }
}

void calculateAvg(int array1[SENSORS], int array2[SENSORS])
{
    for (int i = 0; i < SENSORS; i++)
    {
        readSensors();
        
        if (array1[i] != 0)
        {
            array1[i] += array2[i] / 2;
        }
    
        else
        {
            array1[i] = array2[i];
        }
    }
}
