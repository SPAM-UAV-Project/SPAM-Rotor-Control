#include "thrust_stand.hpp"

namespace sensors
{

float ThrustStand::getAverage(int lc_index, int samples, int delay_ms) {
    float sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += lc_values[lc_index];
        Serial.println("lc val: " + String(lc_values[lc_index]));
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    return sum / samples;
}

void ThrustStand::calibrate()
{
    // calibrate thrust load cell //
    Serial.println("Put thrust stand vertical (motor mount facing up) with no load. Press any key to continue.");

    // tare load cell
    Serial.println("Taring load cells... do not touch thrust stand");
    lc_b.tare();

    Serial.println("Place the calibration weight on motor mount. Press any key to continue.");
    while (!Serial.available()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    Serial.read();
    
    // F_z = lc_v_val * s_v/v - T_x / v -> T_x = 0
    // s_v = F_z * v / lc_val
    Serial.println("Calibrating, wait 1 second");
    lc_calibration_factors[2] = (calibration_weight_kg * g * v) / getAverage(2, 10, 100); 

    // calibration Torque X load cells //
    Serial.println("Put thrust stand horizontal (motor mount sideways) clamped down. Attach calibration stick onto the mount. Press any key to continue.");
    while (!Serial.available()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    Serial.read(); // clear input buffer

    // tare load cells
    Serial.println("Taring load cells... do not touch thrust stand");
    lc_fr.tare();
    lc_fl.tare();

    Serial.println("Place the calibration weight on top of the motor mount. Press any key to continue.");
    while (!Serial.available()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    Serial.read();

    // T_x = 0.5*(lc_fr_val + lc_fl_val) * s_h1
    // s_h1 = Tx / (0.5 * (lc_fr_val + lc_fl_val))
    Serial.println("Calibrating, wait 2 seconds");
    lc_calibration_factors[0] = (calibration_weight_kg * g * x) / (0.5f * (getAverage(0, 10, 100) + getAverage(1, 10, 100)));

    // calibrate Torque Z load cell //
    Serial.println("Move the calibration weight to the end of calibration stick. Press any key to continue.");
    while (!Serial.available()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    Serial.read();

    // T_z = 0.5*(lc_fr_val - lc_fl_val) * s_h2
    // s_h2 = Tz / (0.5 * (lc_fr_val - lc_fl_val))
    Serial.println("Calibrating, wait 2 seconds");
    lc_calibration_factors[1] = (calibration_weight_kg * g * l) / (0.5f * (getAverage(0, 10, 100) - getAverage(1, 10, 100)));

    Serial.println("Calibration complete!");
    Serial.println("Calibration factors:");
    Serial.println("S_h1, S_h1, S_b: " + String(lc_calibration_factors[0]) + ", " + String(lc_calibration_factors[1]) + ", " + String(lc_calibration_factors[2]));
}

}