#include "thrust_stand.hpp"

namespace sensors
{

ThrustStand::ThrustStand()
{
}

void ThrustStand::begin()
{
    lc_fr.begin();
    lc_fl.begin();
    lc_b.begin();
    delay(100);

    Serial.println("[Thrust Stand]: Load cells initialized.");

    xTaskCreate(updateTaskEntry, "ThrustStandTask", 4096, NULL, 2, &thrustStandTaskHandle);

    Serial.println("[Thrust Stand]: Thrust stand task started.");
}

void ThrustStand::updateTask()
{
    while(true)
    {
        if (lc_fr.update()) {      
            lc_values[0] = lc_fr.getRaw();
        }
        if (lc_fl.update()) {      
            lc_values[1] = lc_fl.getRaw();
        }
        if (lc_b.update()) {      
            lc_values[2] = lc_b.getRaw();
        }

        // compute torque and thrust
        torque_x = 0.5 * (lc_values[0] + lc_values[1]) * lc_calibration_factors[0];
        torque_z = 0.5 * (lc_values[0] - lc_values[1]) * lc_calibration_factors[1];
        thrust = (lc_values[2] * lc_calibration_factors[2]) - (torque_x / v);
        
        vTaskDelay(pdMS_TO_TICKS(5)); // 200 hz update since load cells run at 80 hz, past nyquist 
    }
}

}