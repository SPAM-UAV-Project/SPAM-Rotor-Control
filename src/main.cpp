#include "main.hpp"
#include "sensors/encoder.hpp"
#include "control/rotor_control.hpp"
#include "thrust_stand/thrust_stand.hpp"
#include <Arduino.h>


// temp state for testing
enum class State {
    IDLE,
    CALIBRATE,
    PROGRAMMING,
    VERBOSE,
    MANUAL_ACTIVE,
    PROGRAM_ACTIVE
};
State state = State::IDLE;

// Control parameters
float roll_command = 0.0f;
float pitch_command = 0.03f;
float thrust_command = 0.10f;

// Thrust Stand
sensors::ThrustStand ts;

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
    // intialize thrust stand load cells
    ts.begin();

    delay(2000);
    
    // Print command help
    Serial.println("=== SwashPlateless Attitude Manipulation (SPAM) Thrust Stand ===");
    Serial.println("Commands:");
    Serial.println("  c - Start Calibration Routine");
    Serial.println("  s - Start Programmed Test (motor ACTIVE)");
    Serial.println("  p - Enter Programming Mode");
    Serial.println("  d - Display Current Encoder and Load Cell Values");
    Serial.println("  m - Manual Control Mode");
    Serial.println("========================");
}

void processSerialInput() {
    if (Serial.available() > 0) {
        char command = Serial.read();

        switch (command) {
            case 'c':
                Serial.println("Starting Calibration Routine...");
                state = State::CALIBRATE;
                break;

            case 's':
                Serial.println("Starting Programmed Test...");
                state = State::PROGRAM_ACTIVE;
                break;

            case 'p':
                Serial.println("Entering Programming Mode...");
                state = State::PROGRAMMING;
                break;

            case 'd':
                Serial.println("Displaying Current Encoder and Load Cell Values...");
                state = State::VERBOSE;
                break;

            case 'm':
                Serial.println("Entering Manual Control Mode...");
                state = State::MANUAL_ACTIVE;
                break;

            case 'h':
                Serial.println("Commands:");
                Serial.println("  c - Start Calibration Routine");
                Serial.println("  s - Start Programmed Test (motor ACTIVE)");
                Serial.println("  p - Enter Programming Mode");
                Serial.println("  d - Display Current Encoder and Load Cell Values");
                Serial.println("  m - Manual Control Mode");
                break;

            default:
                Serial.println("Unknown command. Please try again.");
                break;
        }
    }

}    


void loop() 
{
    processSerialInput();

    // static uint32_t last_print_time = 0;
    // if (millis() - last_print_time >= 1000) {
    //     last_print_time = millis();
    //     Serial.printf("Encoder Angle: %.3f rad\n", sensors::encoder::enc_angle_rad.load());
    // }
    
    // Apply current state
    switch (state)
    {
        case State::IDLE:
            control::rotor::setControlInputs(0.0, 0.0, 0.0, 0.0);
            break;

        case State::CALIBRATE:
            // Call calibration routine - this is blocking; this FSM wont run
            ts.calibrate();
            state = State::IDLE;
            break;

        case State::PROGRAM_ACTIVE:
            break;

        case State::PROGRAMMING:
            break;

        case State::MANUAL_ACTIVE:
            control::rotor::setControlInputs(roll_command, pitch_command, 0.0, thrust_command);
            break;

        case State::VERBOSE:
            Serial.printf("Encoder Angle: %.3f rad\n", sensors::encoder::enc_angle_rad.load());
            Serial.printf("Thrust Stand: Torque X: %.3f Nm, Torque Z: %.3f Nm, Thrust: %.3f N\n", 
                          ts.getTorqueX(), ts.getTorqueZ(), ts.getThrust());
            delay(50);
            break;

    }
    delay(10);
}
