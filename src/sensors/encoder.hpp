#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <atomic>

#define PIN_ENC_SDN 6
#define PIN_ENC_SCL 7 // haha funny number
#define PIN_ENC_DIR 4 // direction on pin 4

namespace sensors::encoder
{
    inline std::atomic<float> enc_angle_rad;

    void initEncoder();
    void encoderTask(void *pvParameters);
}

#endif // ENCODER_HPP