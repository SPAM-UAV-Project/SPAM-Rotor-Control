#include "rotor_control.hpp"
#include "sensors/encoder.hpp"

// logic as described in "Flight Performance of a Swashplateless Micro Air Vehicle" by James Paulos and Mark Yim
// https://ieeexplore.ieee.org/document/7139936

namespace control::rotor
{
    DShotRMT motor1(MOTOR1_PIN); // 1 motor for testing purposes
    static float control_input[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // roll, pitch, yaw, thrust
    static SemaphoreHandle_t control_mutex = xSemaphoreCreateMutex();
    
    void initRotor()
    {
        motor1.begin(DSHOT600, ENABLE_BIDIRECTION, 14);
        xTaskCreate(rotorControlTask, "RotorControlTask", 2048, NULL, 1, NULL);
    }

    void rotorControlTask(void *pvParameters)
    {
        float amplitude = 0.0f;
        float phase = 0.0f;
        float local_control_input[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        float output_throttle_percent;

        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1000 Hz control loop

        while (true)
        {
            // copy control inputs atomically
            xSemaphoreTake(control_mutex, portMAX_DELAY);
            local_control_input[0] = control_input[0];
            local_control_input[1] = control_input[1];
            local_control_input[2] = control_input[2];
            local_control_input[3] = control_input[3];
            xSemaphoreGive(control_mutex);

            if (local_control_input[0] != 0.0f || local_control_input[1] != 0.0f)
            {
                // for swashplateless rotor control, we need to find an amplitude and a phase lag
                amplitude = AMP_OFFSET + sqrt(local_control_input[0] * local_control_input[0] + local_control_input[1] * local_control_input[1]);
                phase = atan2(local_control_input[1], local_control_input[0]);

                // convert to an oscillatory throttle response
                output_throttle_percent = local_control_input[3] + amplitude * cos(sensors::encoder::enc_angle_rad.load() - phase);
            } else {
                // no pitch or roll command, just set throttle directly
                output_throttle_percent = local_control_input[3];
            }
            sendToDshot(output_throttle_percent);

            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }

    void setControlInputs(float roll, float pitch, float yaw, float thrust)
    {
        xSemaphoreTake(control_mutex, portMAX_DELAY);
        control_input[0] = roll;
        control_input[1] = pitch;
        control_input[2] = yaw;
        control_input[3] = thrust;
        xSemaphoreGive(control_mutex);
    }

    void sendToDshot(float throttle_percent)
    {
        // ensure throttle_percent is within [0.0, 1.0]
        throttle_percent = std::max(0.0f, std::min(1.0f, throttle_percent));

        // convert to DShot value (48 to 2047 for throttle)
        uint16_t dshot_value = static_cast<uint16_t>(48 + throttle_percent * (2047 - 48));
        motor1.send_dshot_value(dshot_value);
    }

    void stop()
    {
        motor1.send_dshot_value(DSHOT_CMD_MOTOR_STOP);
    }
}