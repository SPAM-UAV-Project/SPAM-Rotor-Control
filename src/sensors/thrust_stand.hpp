#ifndef LOAD_CELLS_HPP
#define LOAD_CELLS_HPP

#include <Arduino.h>
#include "HX711.h"

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

    float getForceX() { return force_x; }
    float getForceY() { return force_y; }
    float getTorqueZ() { return torque_z; }
    float getThrust() { return thrust; }

    // Setpoint accessors
    void setThrustSetpoint(float sp) { thrust_sp = sp; }
    void setBladeAngleX(float sp) { blade_angle_x = sp; }
    void setBladeAngleY(float sp) { blade_angle_y = sp; }
    void setTorqueZSetpoint(float sp) { torque_z_sp = sp; }
    void setAmpCutIn(float amp) { amp_cut_in = amp; }
    void setPhaseLag(float phase) { phase_lag = phase; }
    void setAngularVelocity(float omega) { angular_velocity = omega; }
    float getThrustSetpoint() { return thrust_sp; }
    float getBladeAngleX() { return blade_angle_x; }
    float getBladeAngleY() { return blade_angle_y; }
    float getTorqueZSetpoint() { return torque_z_sp; }
    float getAmpCutIn() { return amp_cut_in; }
    float getPhaseLag() { return phase_lag * M_PI / 180.0f; }
    float getAngularVelocity() { return angular_velocity; }

    static void updateTaskEntry(void* instance) {
        static_cast<ThrustStand*>(instance)->updateTask();
    }   

    TaskHandle_t thrustStandTaskHandle;

private:
    const int lc_pin_dout[3] = {20, 4, 13 }; // fr, fl, b
    const int lc_pin_sck[3] = {21, 5, 14};
    float lc_calibration_factors[3] = {8.631114e-07f, 1.729917e-06f, -2.179983e-05}; // S_h1, S_h2, S_b

    HX711 lc_fr;
    HX711 lc_fl;
    HX711 lc_b;
    
    double force_x = 0.0f;
    double force_y = 0.0f;
    double torque_z = 0.0f;
    double thrust = 0.0f;
    float thrust_sp = 0.0f;
    float blade_angle_x = 0.0f;
    float blade_angle_y = 0.0f;
    float torque_z_sp = 0.0f;
    float amp_cut_in = 0.0f;
    float phase_lag = 0.0f;
    float angular_velocity = 0.0f;
    long lc_values[3]; // fr, fl, b
    float calibration_weight_kg = 0.200f; // 250 grams

    // geometry of thrust stand
    float v = 0.05f; // vertical distance between load cells
    float x = 0.045f; // horizontal distance from calib stick to front load cells (x axis)
    float l = 0.13f; // length of calibration stick for (z axis)

    void updateTask();
};

}

#endif // LOAD_CELLS_HPP