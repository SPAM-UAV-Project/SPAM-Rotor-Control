#ifndef TS_UI_HPP
#define TS_UI_HPP

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "sensors/thrust_stand.hpp"

namespace CDH {

struct RoutineStep {
    float thrust_setpoint;
    float torque_x_setpoint;
    float torque_y_setpoint;
    float torque_z_setpoint;
    float phase_lag;
    float amp_cut_in;
    uint32_t duration_ms;
};

struct DataPoint {
    uint32_t timestamp;
    float torque_x;
    float torque_z;
    float thrust;
    float torque_x_sp;
    float thrust_sp;
};

class ThrustStandUI {
public:
    ThrustStandUI(sensors::ThrustStand& ts);
    ~ThrustStandUI() = default;

    // Entry point - call in setup(). Connects to WiFi network
    void begin(const char* ssid, const char* password);
    // Exit point - call to stop server
    void stop();
    // Call in loop() to handle updates
    void update();

    // Callbacks for extending functionality
    using CalibrationCallback = void(*)();
    using ArmCallback = void(*)();
    
    void onCalibrationStart(CalibrationCallback cb) { calibrationCallback = cb; }
    void onArm(ArmCallback cb) { armCallback = cb; }
    void onDisarm(ArmCallback cb) { disarmCallback = cb; }

    // State accessors for external control
    bool isCalibrating() const { return calibrating; }
    bool isRoutineRunning() const { return routineRunning; }
    float getCurrentThrustSetpoint() const { return currentThrustSp; }
    float getCurrentTorqueXSetpoint() const { return currentTorqueXSp; }
    float getCurrentTorqueYSetpoint() const { return currentTorqueYSp; }
    float getCurrentTorqueZSetpoint() const { return currentTorqueZSp; }
    float getCurrentPhaseLag() const { return currentPhaseLag; }
    float getCurrentAmpCutIn() const { return currentAmpCutIn; }
    
    void setCalibrationStep(uint8_t step) { calibrationStep = step; }
    void setCalibrationComplete() { calibrating = false; calibrationStep = 0; }

    static constexpr size_t MAX_ROUTINE_STEPS = 32;
    static constexpr size_t DATA_BUFFER_SIZE = 200;

private:
    sensors::ThrustStand& thrustStand;
    AsyncWebServer server;
    AsyncWebSocket ws;
    
    bool calibrating = false;
    uint8_t calibrationStep = 0;
    bool routineRunning = false;
    
    RoutineStep routine[MAX_ROUTINE_STEPS];
    size_t routineStepCount = 0;
    size_t currentRoutineStep = 0;
    uint32_t stepStartTime = 0;
    
    float currentThrustSp = 0;
    float currentTorqueXSp = 0;
    float currentTorqueYSp = 0;
    float currentTorqueZSp = 0;
    float currentPhaseLag = 0;
    float currentAmpCutIn = 0;
    
    DataPoint dataBuffer[DATA_BUFFER_SIZE];
    size_t dataIndex = 0;
    uint32_t lastSampleTime = 0;
    
    CalibrationCallback calibrationCallback = nullptr;
    ArmCallback armCallback = nullptr;
    ArmCallback disarmCallback = nullptr;
    
    void setupRoutes();
    void setupWebSocket();
    void handleWebSocketMessage(void* arg, uint8_t* data, size_t len);
    void sendDataUpdate();
    void updateRoutine();
};

}

#endif
