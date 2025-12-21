#include "thrust_stand.hpp"

namespace sensors
{

// Helper to wait for serial input and clear buffer
void waitForSerialInput() {
    // clear any existing buffer
    while (Serial.available()) {
        Serial.read();
    }
    // wait for new input
    while (!Serial.available()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    while (Serial.available()) {
        Serial.read();
    }
    delay(100);
}

void ThrustStand::calibrate()
{
    // calibrate thrust load cell //
    Serial.println("Put thrust stand vertical (motor mount facing up) with no load. Press any key to continue.");
    waitForSerialInput();

    // tare load cell
    Serial.println("Taring load cells... do not touch thrust stand");
    lc_b.tare(10);

    Serial.println("Place the calibration weight on motor mount. Press any key to continue.");
    waitForSerialInput();
    
    // F_z = lc_v_val * s_v - T_x / v -> T_x = 0
    // s_v = F_z / lc_val
    float lc_b_val = lc_b.getValue(10);
    Serial.println("lc_b value: " + String(lc_b_val));
    lc_calibration_factors[2] = (abs(lc_b_val) < 1e-1) ? 1.0f : (calibration_weight_kg * g) / lc_b_val;

    // calibration Torque X load cells //
    Serial.println("Put thrust stand horizontal (motor mount sideways) clamped down. Attach calibration stick onto the mount. Press any key to continue.");
    waitForSerialInput();

    // tare load cells
    Serial.println("Taring load cells... do not touch thrust stand");
    lc_fr.tare(10);
    lc_fl.tare(10);

    Serial.println("Place the calibration weight on top of the motor mount. Press any key to continue.");
    waitForSerialInput();

    // T_x = 0.5*(lc_fr_val + lc_fl_val) * s_h1
    // s_h1 = Tx / (0.5 * (lc_fr_val + lc_fl_val))
    Serial.println("Calibrating, wait 2 seconds");
    float avg_sum = 0.5f * (lc_fr.getValue(10) + lc_fl.getValue(10));
    Serial.println("avg_sum: " + String(avg_sum));
    lc_calibration_factors[0] = (abs(avg_sum) < 1e-1) ? 1.0f : (calibration_weight_kg * g * x) / avg_sum;

    // calibrate Torque Z load cell //
    Serial.println("Move the calibration weight to the end of calibration stick. Press any key to continue.");
    waitForSerialInput();

    // T_z = 0.5*(lc_fr_val - lc_fl_val) * s_h2
    // s_h2 = Tz / (0.5 * (lc_fr_val - lc_fl_val))
    Serial.println("Calibrating, wait 2 seconds");
    float avg_diff = 0.5f * (lc_fr.getValue(10) - lc_fl.getValue(10));
    Serial.println("avg_diff: " + String(avg_diff));
    lc_calibration_factors[1] = (abs(avg_diff) < 1e-1) ? 1.0f : (calibration_weight_kg * g * l) / avg_diff;

    Serial.println("Calibration complete!");
    Serial.printf("Calibration factors:\n S_h1: %.6e\n S_h2: %.6e\n S_b: %.6e\n", 
        lc_calibration_factors[0], lc_calibration_factors[1], lc_calibration_factors[2]);
    Serial.printf("Offset factors:\n lc_fr: %ld\n lc_fl: %ld\n lc_b: %ld\n", 
        lc_fr.getOffset(), lc_fl.getOffset(), lc_b.getOffset());
    }
}