//===================== NEOPIXEL FUNCTIONS =====================//
void signalOff()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(OFF));
    }
    strip_NI.show();
}

void signalStarting()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(GREEN));
    }
    
    strip_NI.show();
    delay(150);
    
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(OFF));
    }
    
    strip_NI.show();
    delay(150);
    
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(GREEN));
    }
    
    strip_NI.show();
    delay(150);
    
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(OFF));
    }
    strip_NI.show();
}

void signalForward()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(GREEN));
    }
    strip_NI.show();
}

void signalReverse()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(YELLOW));
    }
    strip_NI.show();
}

void signalWait()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(WHITE));
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

void signalStop()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(RED));
    }
    strip_NI.show();
}

void signalLeft()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(GREEN));
    }
    strip_NI.setPixelColor(2, strip_NI.Color(BLUE));
    strip_NI.show();
}

void signalRight()
{
    strip_NI.begin();
    for (int j = 0; j < NEO_PIXNUMBER; j++)
    {
        strip_NI.setPixelColor(j, strip_NI.Color(GREEN));
    }
    strip_NI.setPixelColor(3, strip_NI.Color(BLUE));
    strip_NI.show();
}
