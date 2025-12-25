#include "main.hpp"
#include "sensors/encoder.hpp"
#include "control/rotor_control.hpp"
#include "sensors/thrust_stand.hpp"
#include "CDH/ts_ui.hpp"
#include "CDH/ts_logger.hpp"
#include <Arduino.h>

enum class State {
    IDLE,
    CALIBRATE,
    MOTORS_ACTIVE
};
State state = State::IDLE;

sensors::ThrustStand ts;
CDH::ThrustStandUI tsUI(ts);
CDH::Logger tsLogger(ts);

const char* WIFI_SSID = "tommy";
const char* WIFI_PASSWORD = "hilfiger";

void switchState(State newState) {
    state = newState;
    Serial.println("[Main] State changed to " + String(static_cast<int>(state)));
}

void setup() 
{
    Serial.begin(921600);   

    pinMode(18, OUTPUT);
    digitalWrite(18, LOW);

    sensors::encoder::initEncoder();
    control::rotor::initRotor();
    ts.begin();
    tsLogger.begin();

    tsUI.onCalibrationStart([]() {
        switchState(State::CALIBRATE);
    });
    
    tsUI.onArm([]() {
        switchState(State::MOTORS_ACTIVE);
    });
    
    tsUI.onDisarm([]() {
        switchState(State::IDLE);
    });
    
    tsUI.begin(WIFI_SSID, WIFI_PASSWORD);

    delay(1000);
    Serial.println("[Main] System ready - connect to web UI");
}

void loop() 
{
    tsUI.update();
    
    switch (state)
    {
        case State::IDLE:
            tsLogger.stop();
            control::rotor::setControlInputs(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            break;

        case State::CALIBRATE:
            tsLogger.stop();
            ts.calibrate();
            tsUI.setCalibrationComplete();
            state = State::IDLE;
            break;

        case State::MOTORS_ACTIVE:
            tsLogger.start();
            ts.setThrustSetpoint(tsUI.getCurrentThrustSetpoint());
            ts.setTorqueXSetpoint(tsUI.getCurrentTorqueXSetpoint());
            ts.setTorqueYSetpoint(tsUI.getCurrentTorqueYSetpoint());
            ts.setTorqueZSetpoint(tsUI.getCurrentTorqueZSetpoint());
            ts.setAmpCutIn(tsUI.getCurrentAmpCutIn());
            ts.setPhaseLag(tsUI.getCurrentPhaseLag());

            control::rotor::setControlInputs(
                ts.getTorqueXSetpoint(),
                ts.getTorqueYSetpoint(),
                ts.getTorqueZSetpoint(),
                ts.getThrustSetpoint(),
                ts.getPhaseLag(),
                ts.getAmpCutIn()
            );
            break;
    }
    
    delay(10);
}
