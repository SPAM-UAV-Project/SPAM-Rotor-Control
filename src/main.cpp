#include "main.hpp"
#include "sensors/encoder.hpp"
#include "control/rotor_control.hpp"
#include <Arduino.h>


// temp state for testing
enum class State {
    IDLE,
    CALIBRATE,
    ACTIVE,
};
State state = State::IDLE;

// Control parameters
float roll_command = 0.0f;
float pitch_command = 0.03f;
float thrust_command = 0.10f;

void setup() 
{
    Serial.begin(921600);   

    // hacky way to add an extra ground
    pinMode(18, OUTPUT);
    digitalWrite(18, LOW); // connect GND to pin 18 for now (this is very hacky)

    // initialize encoder
    sensors::encoder::initEncoder();
    // initialize rotor control
    control::rotor::initRotor();

    delay(2000);
    
    // Print command help
    Serial.println("=== SPAM Rotor Control ===");
    Serial.println("Commands:");
    Serial.println("  s - Start motor (ACTIVE state)");
    Serial.println("  x - Stop motor (IDLE state)");
    Serial.println("  r<value> - Set roll command (e.g., r0.03)");
    Serial.println("  p<value> - Set pitch command (e.g., p0.05)");
    Serial.println("  t<value> - Set thrust command (e.g., t0.12)");
    Serial.println("  ? - Show current status");
    Serial.println("========================");
}

void processSerialInput() {
    static bool waiting_for_confirmation = false;
    static char pending_command = 0;
    static float pending_value = 0.0f;
    
    if (Serial.available()) {
        if (waiting_for_confirmation) {
            // Handle confirmation
            char response = Serial.read();
            if (response == 'y' || response == 'Y') {
                // Execute the pending command
                switch (pending_command) {
                    case 's':
                        state = State::ACTIVE;
                        Serial.println("CONFIRMED - Motor ACTIVE");
                        break;
                    case 'x':
                        state = State::IDLE;
                        Serial.println("CONFIRMED - Motor IDLE");
                        break;
                    case 'r':
                        roll_command = pending_value;
                        Serial.printf("CONFIRMED - Roll set to %.3f\n", roll_command);
                        break;
                    case 'p':
                        pitch_command = pending_value;
                        Serial.printf("CONFIRMED - Pitch set to %.3f\n", pitch_command);
                        break;
                    case 't':
                        thrust_command = pending_value;
                        Serial.printf("CONFIRMED - Thrust set to %.3f\n", thrust_command);
                        break;
                }
            } else {
                Serial.println("CANCELLED");
            }
            waiting_for_confirmation = false;
            pending_command = 0;
            // Clear any remaining characters
            while (Serial.available()) Serial.read();
            return;
        }
        
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        if (input.length() == 0) return;
        
        char command = input.charAt(0);
        
        switch (command) {
            case 's':
            case 'S':
                Serial.printf("Command: START motor - Pitch: %.3f, Thrust: %.3f\n", pitch_command, thrust_command);
                Serial.print("Confirm? (y/n): ");
                waiting_for_confirmation = true;
                pending_command = 's';
                break;
                
            case 'x':
            case 'X':
                Serial.println("Command: STOP motor");
                Serial.print("Confirm? (y/n): ");
                waiting_for_confirmation = true;
                pending_command = 'x';
                break;
                
            case 'r':
            case 'R':
                if (input.length() > 1) {
                    float new_roll = input.substring(1).toFloat();
                    Serial.printf("Command: Set roll to %.3f (current: %.3f)\n", new_roll, roll_command);
                    Serial.print("Confirm? (y/n): ");
                    waiting_for_confirmation = true;
                    pending_command = 'r';
                    pending_value = new_roll;
                } else {
                    Serial.println("Usage: r<value> (e.g., r0.03)");
                }
                break;
                
            case 'p':
            case 'P':
                if (input.length() > 1) {
                    float new_pitch = input.substring(1).toFloat();
                    Serial.printf("Command: Set pitch to %.3f (current: %.3f)\n", new_pitch, pitch_command);
                    Serial.print("Confirm? (y/n): ");
                    waiting_for_confirmation = true;
                    pending_command = 'p';
                    pending_value = new_pitch;
                } else {
                    Serial.println("Usage: p<value> (e.g., p0.05)");
                }
                break;
                
            case 't':
            case 'T':
                if (input.length() > 1) {
                    float new_thrust = input.substring(1).toFloat();
                    Serial.printf("Command: Set thrust to %.3f (current: %.3f)\n", new_thrust, thrust_command);
                    Serial.print("Confirm? (y/n): ");
                    waiting_for_confirmation = true;
                    pending_command = 't';
                    pending_value = new_thrust;
                } else {
                    Serial.println("Usage: t<value> (e.g., t0.12)");
                }
                break;
                
            case '?':
                Serial.println("=== Current Status ===");
                Serial.printf("State: %s\n", (state == State::ACTIVE) ? "ACTIVE" : "IDLE");
                Serial.printf("Roll Command: %.3f\n", roll_command);
                Serial.printf("Pitch Command: %.3f\n", pitch_command);
                Serial.printf("Thrust Command: %.3f\n", thrust_command);
                Serial.printf("Encoder Angle: %.3f rad\n", sensors::encoder::enc_angle_rad.load());
                Serial.println("====================");
                break;
                
            default:
                Serial.println("Unknown command. Type ? for help");
                break;
        }
    }
}

void loop() 
{
    // Process serial input
    processSerialInput();

    static uint32_t last_print_time = 0;
    if (millis() - last_print_time >= 1000) {
        last_print_time = millis();
        Serial.printf("Encoder Angle: %.3f rad\n", sensors::encoder::enc_angle_rad.load());
    }

    
    // Apply current state
    switch (state)
    {
        case State::IDLE:
            control::rotor::setControlInputs(0.0, 0.0, 0.0, 0.0);
            break;

        case State::ACTIVE:
            control::rotor::setControlInputs(roll_command, pitch_command, 0.0, thrust_command);
            break;


    }
    delay(10);
}
