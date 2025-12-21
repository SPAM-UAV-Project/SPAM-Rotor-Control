#ifndef LOAD_CELLS_HPP
#define LOAD_CELLS_HPP

#include <Arduino.h>
#include "hx711.hpp"

#define g 9.806f // m/s^2

namespace sensors
{

class ThrustStand
{
public:
    ThrustStand();
    ~ThrustStand() = default;

    void begin();
    void calibrate();

    float getTorqueX() { return torque_x; }
    float getTorqueZ() { return torque_z; }
    float getThrust() { return thrust; }

    // Setpoint accessors
    void setThrustSetpoint(float sp) { thrust_sp = sp; }
    void setTorqueXSetpoint(float sp) { torque_x_sp = sp; }
    float getThrustSetpoint() { return thrust_sp; }
    float getTorqueXSetpoint() { return torque_x_sp; }

    static void updateTaskEntry(void* instance) {
        static_cast<ThrustStand*>(instance)->updateTask();
    }   

    TaskHandle_t thrustStandTaskHandle;

private:
    const int lc_pin_dout[3] = {0, 1, 20};
    const int lc_pin_sck[3] = {4, 5, 21};
    float lc_calibration_factors[3] = {1.0f, 1.0f, 1.0f}; // S_h1, S_h2, S_b

    // depth=12 gives buffer for 12 readings, enough for tare(10)
    NBHX711 lc_fr = NBHX711(lc_pin_dout[0], lc_pin_sck[0], 12, 1);
    NBHX711 lc_fl = NBHX711(lc_pin_dout[1], lc_pin_sck[1], 12, 1);
    NBHX711 lc_b = NBHX711(lc_pin_dout[2], lc_pin_sck[2], 12, 1);

    float torque_x, torque_z, thrust;
    float thrust_sp = 0.0f;
    float torque_x_sp = 0.0f;
    float lc_values[3]; // fr, fl, b
    float calibration_weight_kg = 0.250f; // 250 grams

    // geometry of thrust stand
    float v = 0.1f; // vertical distance between load cells
    float x = 0.05f; // horizontal distance from calib stick to front load cells
    float l = 0.1f;

    void updateTask();
};

}

#endif // LOAD_CELLS_HPP