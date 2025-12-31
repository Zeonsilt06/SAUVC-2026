#include <Arduino.h>
#include "ThrusterControl.h"

#define BAUD_RATE 9600

void setup()
{
    Serial.begin(BAUD_RATE);

    initMotor();
    stopAll();
}

void loop()
{
    forward(0);
    delay(3000);

    rotateRight(0);
    delay(3000);

    up(0);
    delay(3000);

    rotateLeftUp(0, 0);
    delay(3000);

    stopAll();
    delay(4000);
}