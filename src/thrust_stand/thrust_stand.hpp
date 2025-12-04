#ifndef LOAD_CELLS_HPP
#define LOAD_CELLS_HPP

#include <Arduino.h>
#include "hx711.hpp"

namespace sensors
{
class ThrustStand
{
public:
    ThrustStand();
    ~ThrustStand() = default();

    void begin();
    void calibrate();

    void getTorqueX();
    void getTorqueZ();
    void getThrust();

private:
    const int lc_pin_dout[3] = {0, 1, 2};
    const int lc_pin_sck[3] = {4, 5, 6};
    const float lc_calibration_factors[3] = {1.0f, 1.0f, 1.0f}; // to be calibrated

    NBHX711 lc_fr = NBHX711(lc_pin_dout[0], lc_pin_sck[0], 0, 1);
    NBHX711 lc_fl = NBHX711(lc_pin_dout[1], lc_pin_sck[1], 0, 1);
    NBHX711 lc_br = NBHX711(lc_pin_dout[2], lc_pin_sck[2], 0, 1);

    float torque_x, torque_z, thrust;
    float lc_values[3];

    void updateTask();
};

}

#endif // LOAD_CELLS_HPP