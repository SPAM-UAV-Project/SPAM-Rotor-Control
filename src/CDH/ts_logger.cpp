#include "ts_logger.hpp"
#include <Arduino.h>
#include "sensors/thrust_stand.hpp"
#include "sensors/imu.hpp"

namespace CDH {

Logger::Logger(sensors::ThrustStand& ts) : ts_(ts) {}

void Logger::start() {
    if (!currently_logging_) {
        currently_logging_ = true;
        Serial.println("time_ms, force_x, torque_z, thrust, B_x, B_y, torque_z_sp, thrust_sp, amp_cut_in, phase_lag, angular_velocity, accel_x, accel_y, accel_z");

    }
}

void Logger::stop() {
    if (currently_logging_) {
        currently_logging_ = false;
        Serial.println("[Logger] Logging stopped");
    }
}

void Logger::begin() {
    xTaskCreate(loggerTaskEntry, "LoggerTask", 4096, this, 1, &loggerTaskHandle);
    Serial.println("[Logger] Logger task started.");
}

void Logger::loggerTask() {

    while (true) {
        if (currently_logging_) {
            Serial.printf("%lu, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f\n",
                millis(),
                ts_.getForceX(),
                ts_.getTorqueZ(),   
                ts_.getThrust(),
                ts_.getBladeAngleX(),
                ts_.getBladeAngleY(),
                ts_.getTorqueZSetpoint(),
                ts_.getThrustSetpoint(),
                ts_.getAmpCutIn(),
                ts_.getPhaseLag(),
                ts_.getAngularVelocity(),
                sensors::imu::accel(0),
                sensors::imu::accel(1),
                sensors::imu::accel(2)
            );
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // log at 100 Hz
    }
}

} // namespace CDH