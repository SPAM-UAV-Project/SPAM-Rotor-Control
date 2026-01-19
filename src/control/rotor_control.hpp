#ifndef ROTOR_CONTROL_HPP
#define ROTOR_CONTROL_HPP

#define MOTOR1_PIN 18
#define ARMING_THROTTLE 0.10f // per motor, can be set to 0.075

#include <Arduino.h>

namespace control::rotor
{
    inline TaskHandle_t rotorTaskHandle = NULL;

    void initRotor();
    void rotorControlTask(void *pvParameters);
    void setControlInputs(float roll, float pitch, float yaw, float thrust, float phase_lag, float amp_cut_in);
    void sendToDshot(float throttle_percent);
}

#endif // ROTOR_CONTROL_HPP