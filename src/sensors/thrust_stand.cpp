#include "thrust_stand.hpp"

namespace sensors
{

ThrustStand::ThrustStand()
{
}

void ThrustStand::begin()
{
    lc_fr.begin(lc_pin_dout[0], lc_pin_sck[0]);
    lc_fl.begin(lc_pin_dout[1], lc_pin_sck[1]);
    lc_b.begin(lc_pin_dout[2], lc_pin_sck[2]);
    delay(500); 

    lc_fr.tare(10);
    lc_fl.tare(10);
    lc_b.tare(10);

    xTaskCreate(updateTaskEntry, "ThrustStandTask", 4096, this, 2, &thrustStandTaskHandle);
    Serial.println("[Thrust Stand]: Thrust stand task started.");
}

void ThrustStand::updateTask()
{
    float torque_y = 0.0f;
    while(true)
    {
        lc_values[0] = lc_fr.get_value(1);
        lc_values[1] = lc_fl.get_value(1);
        lc_values[2] = lc_b.get_value(1);

        // compute force and thrust
        torque_y = force_x = (0.5 * (lc_values[0] + lc_values[1]) * lc_calibration_factors[0]) ;
        force_x = torque_y / x;
        torque_z = (0.5 * (lc_values[0] - lc_values[1]) * lc_calibration_factors[1]);
        thrust = (lc_values[2] * lc_calibration_factors[2]) - (torque_y / v);

        // print thrust with and without        
        // Serial.printf("[Thrust Stand] LC Values (raw): FR: %ld, FL: %ld, B: %ld\n", lc_values[0], lc_values[1], lc_values[2]);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

}