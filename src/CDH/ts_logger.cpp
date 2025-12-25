#include "ts_logger.hpp"
#include <Arduino.h>
#include "sensors/thrust_stand.hpp"

namespace CDH {

Logger::Logger(sensors::ThrustStand& ts) : ts_(ts) {}

void Logger::start() {
    if (!currently_logging_) {
        currently_logging_ = true;
        Serial.println("time_ms, torque_hor, torque_z, thrust, torque_x_sp, torque_y_sp, torque_z_sp, thrust_sp, amp_cut_in, phase_lag");

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
            Serial.printf("%lu, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f\n",
                millis(),
                ts_.getTorqueX(),
                ts_.getTorqueZ(),   
                ts_.getThrust(),
                ts_.getTorqueXSetpoint(),
                ts_.getTorqueYSetpoint(),
                ts_.getTorqueZSetpoint(),
                ts_.getThrustSetpoint(),
                ts_.getAmpCutIn(),
                ts_.getPhaseLag()
            );
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // log at 100 Hz
    }
}

} // namespace CDH