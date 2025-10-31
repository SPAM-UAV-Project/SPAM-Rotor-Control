#include "main.hpp"
#include "sensors/encoder.hpp"
#include "control/rotor_control.hpp"
#include <Arduino.h>


void setup() 
{
    Serial.begin(921600);

    // initialize encoder
    sensors::encoder::initEncoder();

    // initialize rotor control
    control::rotor::initRotor();
    control::rotor::stop();
}

void loop() 
{
    // static unsigned long start = millis();
    // if (millis() - start >= 100) {
    //     Serial.println(sensors::encoder::enc_angle_rad.load());
    //     start = millis();
    // }

    // send a pitch command at 25% throttle
    control::rotor::setControlInputs(0.0, 0.1, 0.0, 0.25);
}
