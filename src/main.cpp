#include "Arduino.h"
void setup()
{
    Serial.begin(115200);
    delay(2000);
    Serial.println("I am the new firmware");
}

void loop()
{
    delay(5000);
    Serial.println("New here");
}