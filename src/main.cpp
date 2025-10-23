#include "main.hpp"
#include "sensors/encoder.hpp"
#include <Arduino.h>

void setup() 
{
    Serial.begin(115200);
    sensors::encoder::initEncoder();
}

void loop() 
{
    static unsigned long start = millis();
    if (millis() - start >= 100) {
        Serial.println(sensors::encoder::enc_angle_rad.load());
        start = millis();
    }
}
