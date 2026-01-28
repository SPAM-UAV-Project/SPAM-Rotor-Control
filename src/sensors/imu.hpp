#ifndef IMU_HPP
#define IMU_HPP

#include <Arduino.h>
#include "ArduinoEigen/Eigen/Dense"

#define PIN_IMU_SDA 16
#define PIN_IMU_SCL 15
#define PIN_IMU_INT 17

namespace sensors::imu
{
    inline xTaskHandle imuTaskHandle = NULL;

    inline Eigen::Vector3f accel;

    void imuISR();
    void initIMU();
    void imuTask(void *pvParameters);
}

#endif // IMU_HPP