#include "encoder.hpp"
#include "AS5600.h"
#include <Arduino.h>

namespace sensors::encoder
{
    AS5600 magEnc;

    void initEncoder()
    {
        Wire.begin(PIN_ENC_SDN, PIN_ENC_SCL); // haha funny number
        Wire.setClock(400000);

        magEnc.begin(PIN_ENC_DIR); // direction on pin 4
        magEnc.setDirection(AS5600_CLOCK_WISE);
        
        // start freertos task
        xTaskCreate(encoderTask, "EncoderTask", 2048, NULL, 1, NULL);
    }

    void encoderTask(void *pvParameters)
    {
        // poll the encoder at 1000 hz (good luck freertos scheduler)
        while(1){
            enc_angle_rad.store((float)magEnc.rawAngle() * AS5600_RAW_TO_RADIANS, std::memory_order_relaxed);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    
}