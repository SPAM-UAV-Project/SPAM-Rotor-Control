#include "ts_ui.hpp"
#include <LittleFS.h>

namespace CDH {

ThrustStandUI::ThrustStandUI(sensors::ThrustStand& ts) 
    : thrustStand(ts), server(80), ws("/ws") {}

void ThrustStandUI::begin(const char* ssid, const char* password) {
    if (!LittleFS.begin(true)) {
        Serial.println("[UI] LittleFS mount failed");
        return;
    }
    Serial.println("[UI] LittleFS mounted");
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    
    Serial.print("[UI] Connecting to WiFi: ");
    Serial.println(ssid);
    
    int timeout = 20; // 10 second timeout
    while (WiFi.status() != WL_CONNECTED && timeout > 0) {
        delay(500);
        Serial.print(".");
        timeout--;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.print("[UI] Connected! IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println();
        Serial.println("[UI] WiFi connection failed!");
        return;
    }
    
    setupWebSocket();
    setupRoutes();
    server.begin();
    Serial.println("[UI] Web server started");
}

void ThrustStandUI::stop() {
    ws.closeAll();
    server.end();
}

void ThrustStandUI::setupWebSocket() {
    ws.onEvent([this](AsyncWebSocket* s, AsyncWebSocketClient* c, 
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
        if (type == WS_EVT_DATA) {
            handleWebSocketMessage(arg, data, len);
        }
    });
    server.addHandler(&ws);
}

void ThrustStandUI::handleWebSocketMessage(void* arg, uint8_t* data, size_t len) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (info->opcode != WS_TEXT) return;
    
    data[len] = 0;
    JsonDocument doc;
    if (deserializeJson(doc, (char*)data)) return;
    
    const char* cmd = doc["cmd"];
    if (!cmd) return;
    
    if (strcmp(cmd, "arm") == 0) {
        if (armCallback) armCallback();
    }
    else if (strcmp(cmd, "disarm") == 0) {
        routineRunning = false;
        currentThrustSp = 0;
        currentTorqueXSp = 0;
        if (disarmCallback) disarmCallback();
    }
    else if (strcmp(cmd, "manual") == 0) {
        if (!routineRunning) {
            currentThrustSp = doc["thrust"] | 0.0f;
            currentTorqueXSp = doc["torque_x"] | 0.0f;
        }
    }
    else if (strcmp(cmd, "calibrate") == 0) {
        calibrating = true;
        calibrationStep = 1;
        if (calibrationCallback) calibrationCallback();
    }
    else if (strcmp(cmd, "cal_next") == 0) {
        calibrationStep++;
    }
    else if (strcmp(cmd, "start_routine") == 0) {
        if (!routineRunning && routineStepCount > 0) {
            routineRunning = true;
            currentRoutineStep = 0;
            stepStartTime = millis();
            currentThrustSp = routine[0].thrust_setpoint;
            currentTorqueXSp = routine[0].torque_x_setpoint;
        }
    }
    else if (strcmp(cmd, "stop_routine") == 0) {
        routineRunning = false;
        currentThrustSp = 0;
        currentTorqueXSp = 0;
    }
    else if (strcmp(cmd, "upload_routine") == 0) {
        JsonArray steps = doc["steps"];
        routineStepCount = 0;
        for (JsonObject step : steps) {
            if (routineStepCount >= MAX_ROUTINE_STEPS) break;
            routine[routineStepCount].thrust_setpoint = step["thrust"] | 0.0f;
            routine[routineStepCount].torque_x_setpoint = step["torque_x"] | 0.0f;
            routine[routineStepCount].duration_ms = step["duration"] | 1000;
            routineStepCount++;
        }
    }
}

void ThrustStandUI::update() {
    ws.cleanupClients();
    
    if (millis() - lastSampleTime >= 50) {
        lastSampleTime = millis();
        
        dataBuffer[dataIndex].timestamp = millis();
        dataBuffer[dataIndex].torque_x = thrustStand.getTorqueX();
        dataBuffer[dataIndex].torque_z = thrustStand.getTorqueZ();
        dataBuffer[dataIndex].thrust = thrustStand.getThrust();
        dataBuffer[dataIndex].torque_x_sp = currentTorqueXSp;
        dataBuffer[dataIndex].thrust_sp = currentThrustSp;
        dataIndex = (dataIndex + 1) % DATA_BUFFER_SIZE;
        
        sendDataUpdate();
    }
    
    if (routineRunning) updateRoutine();
}

void ThrustStandUI::updateRoutine() {
    if (currentRoutineStep >= routineStepCount) {
        routineRunning = false;
        currentThrustSp = 0;
        currentTorqueXSp = 0;
        return;
    }
    
    if (millis() - stepStartTime >= routine[currentRoutineStep].duration_ms) {
        currentRoutineStep++;
        if (currentRoutineStep < routineStepCount) {
            stepStartTime = millis();
            currentThrustSp = routine[currentRoutineStep].thrust_setpoint;
            currentTorqueXSp = routine[currentRoutineStep].torque_x_setpoint;
        }
    }
}

void ThrustStandUI::sendDataUpdate() {
    // Skip if no clients
    if (ws.count() == 0) return;
    
    // skip if any client queue is close to full
    for (auto& client : ws.getClients()) {
        if (client.queueLen() >= 4) {
            return; 
        }
    }
    
    JsonDocument doc;
    doc["t"] = millis();
    doc["tx"] = thrustStand.getTorqueX();
    doc["tz"] = thrustStand.getTorqueZ();
    doc["th"] = thrustStand.getThrust();
    doc["tx_sp"] = currentTorqueXSp;
    doc["th_sp"] = currentThrustSp;
    doc["cal"] = calibrating;
    doc["cal_step"] = calibrationStep;
    doc["running"] = routineRunning;
    doc["step"] = currentRoutineStep;
    
    String json;
    serializeJson(doc, json);
    ws.textAll(json);
}

void ThrustStandUI::setupRoutes() {
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
    
    server.on("/api/status", HTTP_GET, [this](AsyncWebServerRequest* req) {
        JsonDocument doc;
        doc["calibrating"] = calibrating;
        doc["calibration_step"] = calibrationStep;
        doc["routine_running"] = routineRunning;
        doc["routine_step"] = currentRoutineStep;
        doc["routine_steps"] = routineStepCount;
        String json;
        serializeJson(doc, json);
        req->send(200, "application/json", json);
    });
}

}
