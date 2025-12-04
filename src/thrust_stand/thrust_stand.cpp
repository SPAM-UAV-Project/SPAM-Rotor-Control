#include "thrust_stand.hpp"

namespace sensors
{

ThrustStand::ThrustStand()
{
}

ThrustStand::begin()
{
    lc_fr.begin();
    lc_fl.begin();
    lc_br.begin();
    delay(100);

    Serial.println("[Thrust Stand]: Load cells initialized.");

    xTaskCreate(updateTask, "ThrustStandTask", 4096, NULL, 2, &thrustStandTaskHandle);

    Serial.println("[Thrust Stand]: Thrust stand task started.");
}

ThrustStand::updateTask()
{
    while(true)
    {
        if (lc_fr.update()) {      
            lc_values[0] = lc_fr.getRaw();
        }
        if (lc_fl.update()) {      
            lc_values[1] = lc_fl.getRaw();
        }
        if (lc_br.update()) {      
            lc_values[2] = lc_br.getRaw();
        }

        // compute torque and thrust here

        
    }
}

}