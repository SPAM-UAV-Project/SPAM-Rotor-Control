#ifndef ROTOR_CONTROL_HPP
#define ROTOR_CONTROL_HPP

#include "DShotRMT.h"

#define MOTOR1_PIN 10
#define AMP_OFFSET 0.0f //  amplitude offset to overcome static friction of hinge

namespace control::rotor
{
    void initRotor();
    void rotorControlTask(void *pvParameters);
    void setControlInputs(float roll, float pitch, float yaw, float thrust);
    void sendToDshot(float throttle_percent);
    void stop();
}

#endif // ROTOR_CONTROL_HPP