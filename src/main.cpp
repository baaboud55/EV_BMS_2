/*
 * Enhanced BMS Monitor - ESP32 WiFi Dashboard with Real Sensor Integration
 * 
 * Features:
 * - Real-time BMS data from daisy-chained slaves
 * - ACS712 current sensing with calibration
 * - Two-stage PID charging control (CC/CV)
 * - Web-based dashboard via WiFi
 * - WebSocket real-time streaming
 */

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "BMSConfig.h"
#include "PIDController.h"
#include "ChargeController.h"

// ------ WiFi Configuration ------
const char* ssid = "Galaxy S21+";        // Replace with your WiFi SSID
const char* password = "77897890"; // Replace with your WiFi password

// ------ Server Objects ------
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ------ BMS Data Structure ------
struct BMSData {
    float packVoltage;
    float current;
    float soc;  // State of Charge (%)
    float soh;  // State of Health (%)
    float cellVoltages[NUM_CELLS];
    float slaveTemperatures[NUM_SLAVES];
    String batteryState;  // "CHARGING", "DISCHARGING", "IDLE"
    bool prechargeActive;
    bool protectionPMOSActive;
    bool activeBalancingActive;
    bool overvoltage;
    bool undervoltage;
    bool overcurrent;
    float remainingRuntime;  // hours
    float cumulativeCapacity; // Ah
    unsigned long timestamp;
};

BMSData currentBMS;

// ------ Global Variables ------
float cells[8];
float temps[2];  // Only 2 real temperature sensors
int balanceStatus[2];
float totalPackVoltage = 0;
float maxCellVoltage = 0;
float minCellVoltage = 0;
float avgTemp = 0;

// Current sensing
float measuredCurrent = 0.0;
float currentOffsetVoltage = 0.0;
bool currentSensorCalibrated = false;

// Charging state
ChargeState currentChargeState = IDLE;
bool precharge_state = false;
unsigned long S_time = 0;
unsigned long E_time = 0;

// PID controllers
PIDController currentPID;  // For CC mode
PIDController voltagePID;  // For CV mode

// Timing variables
unsigned long lastBMSReadTime = 0;
unsigned long lastStartupCheckTime = 0;
unsigned long lastCurrentReadTime = 0;
unsigned long lastWebUpdateTime = 0;

// Cumulative capacity tracking
static float cumulativeAh = 0;
static unsigned long lastCapacityUpdate = 0;

// ----- BMS Functions -----
bool parseAllData(String data, float cells[], float temps[], int balance[]) {
  int idx[11];
  idx[0] = data.indexOf(',');
  if (idx[0] == -1) return false;
  for (int i = 1; i < 11; i++) {
    idx[i] = data.indexOf(',', idx[i-1] + 1);
    if (idx[i] == -1) return false;
  }
  cells[0] = data.substring(0, idx[0]).toFloat();
  for (int i = 1; i < 8; i++) {
    cells[i] = data.substring(idx[i-1] + 1, idx[i]).toFloat();
  }
  temps[0] = data.substring(idx[7] + 1, idx[8]).toFloat();
  temps[1] = data.substring(idx[8] + 1, idx[9]).toFloat();
  balance[0] = data.substring(idx[9] + 1, idx[10]).toInt();
  balance[1] = data.substring(idx[10] + 1).toInt();
  return true;
}
void calculatePackStatistics() {
  totalPackVoltage = 0;
  maxCellVoltage = cells[0];
  minCellVoltage = cells[0];
  for (int i = 0; i < 8; i++) {
    totalPackVoltage += cells[i];
    if (cells[i] > maxCellVoltage) maxCellVoltage = cells[i];
    if (cells[i] < minCellVoltage) minCellVoltage = cells[i];
  }
  avgTemp = (temps[0] + temps[1]) / 2.0;
}

void readBMSData() {
  Serial2.println("READ_VOLTAGES");
  unsigned long timeout = millis();
  while (!Serial2.available() && (millis() - timeout < 2000)) {
    delay(10);
  }
  if (Serial2.available()) {
    String voltageData = Serial2.readStringUntil('\n');
    if (parseAllData(voltageData, cells, temps, balanceStatus)) {
      calculatePackStatistics();
    } else {
      Serial.println("⚠ Error: Failed to parse BMS data");
      currentChargeState = ERROR;
    }
  } else {
    Serial.println("⚠ Error: No response from BMS chain");
    currentChargeState = ERROR;
  }
}


// ----- Current Sensor -----
void calibrateCurrentSensor() {
  Serial.println("Calibrating ACS712 sensor...");
  digitalWrite(PRECHARGE_RELAY, LOW);
  digitalWrite(CHARGE_ENABLE_PIN, LOW);
  delay(5000);
  long totalRawValue = 0;
  for (int i = 0; i < 1000; i++) {
    totalRawValue += analogRead(CURRENT_SENSOR_PIN);
    delay(1);
  }
  currentOffsetVoltage = ((float)totalRawValue / 1000.0 / 4096.0) * ACS_VCC_VOLTAGE;
  currentSensorCalibrated = true;
  Serial.println("ACS712 calibration complete.");
}

void readCurrent() {
  const int numSamples = 50;
  long totalRawValue = 0;
  for (int i = 0; i < numSamples; i++) {
    totalRawValue += analogRead(CURRENT_SENSOR_PIN);
  }
  float avgRawValue = (float)totalRawValue / numSamples;
  float voltage_mV = (avgRawValue / 4096.0) * ACS_VCC_VOLTAGE;
  measuredCurrent = (currentOffsetVoltage - voltage_mV) / (ACS_SENSITIVITY * 1000);
  if (measuredCurrent < 0.15 && measuredCurrent > -0.15) {
    measuredCurrent = 0.0;
  }
}

// ----- Voltage Reading -----
float readVoltage_FB() {
  const int samples = 50;
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(FEEDBACK_PIN);
  }
  int adcValue = sum / samples;
  return (adcValue * 3.3 / 4095.0) * ((FBR1 + FBR2) / FBR2);
}

float readVoltage_IN() {
  int adcValue = analogRead(STARTUP_SENSE_PIN);
  return (adcValue * 3.3 / 4095.0) * ((INR1 + INR2) / INR2);
}

// ----- Charging Control -----
void preCharging() {
  if (currentChargeState == CHARGING_CC || currentChargeState == CHARGING_CV) {
    if (!precharge_state) {
      digitalWrite(PRECHARGE_RELAY, LOW);
      delay(5000);
      digitalWrite(PRECHARGE_RELAY, HIGH);
      precharge_state = true;
      S_time = millis();
      E_time = millis();
      return;
    } 
    if (precharge_state) {
      digitalWrite(PRECHARGE_RELAY, HIGH);
      precharge_state = true;
      return;
    }
  }
  else {
    digitalWrite(PRECHARGE_RELAY, LOW);
    precharge_state = false;
    return;
  }
}

void stopCharging() {
  preCharging();
  digitalWrite(CHARGE_ENABLE_PIN, LOW);
  ledcWrite(PWM_PIN, 511 - 0);
}

void waitForStartupCondition() {
  float IN_VOLTAGE = readVoltage_IN();
  bool tempOK = (avgTemp >= MIN_TEMP && avgTemp <= MAX_TEMP);
  bool voltageOK = (minCellVoltage >= MIN_CHARGE_VOLTAGE && maxCellVoltage < MAX_CELL_VOLTAGE);
  bool sensorOK = currentSensorCalibrated;
  bool chargerOK = (IN_VOLTAGE >= 1.0);
  if (tempOK && voltageOK && sensorOK && chargerOK) {
    if (currentChargeState == IDLE || currentChargeState == WAITING_FOR_CONDITIONS) {
      currentChargeState = CHARGING_CC;
    }
  }
  else {
    if (currentChargeState == CHARGING_CC || currentChargeState == CHARGING_CV) {
      currentChargeState = WAITING_FOR_CONDITIONS;
      stopCharging();
    } else if (currentChargeState == IDLE) {
      currentChargeState = WAITING_FOR_CONDITIONS;
    }
  }
}

void performCCCharging() {
  if (totalPackVoltage >= CC_TO_CV_VOLTAGE) {
    currentChargeState = CHARGING_CV;
    voltagePID.integral = 0;
    voltagePID.prev_error = 0;
    S_time = millis();
    E_time = millis();
    return;
  }
  if (avgTemp > MAX_TEMP || avgTemp < MIN_TEMP) {
    currentChargeState = WAITING_FOR_CONDITIONS;
    return;
  }
  digitalWrite(CHARGE_ENABLE_PIN, HIGH);
  preCharging();
  readCurrent();
  E_time = millis();
  float dt = (E_time - S_time) / 1000.0;
  float pidOutput = computePID(&currentPID, measuredCurrent, dt);
  ledcWrite(0, 511 - (int)pidOutput);
  S_time = millis();
}

void performCVCharging() {
  if (maxCellVoltage >= MAX_CELL_VOLTAGE || measuredCurrent <= CHARGE_COMPLETE_CURRENT) {
    if (!balanceStatus[0] || !balanceStatus[1]) {
      currentChargeState = BALANCING;
    } else {
      currentChargeState = COMPLETE;
    }
    return;
  }
  if (avgTemp > MAX_TEMP || avgTemp < MIN_TEMP) {
    currentChargeState = WAITING_FOR_CONDITIONS;
    return;
  }
  digitalWrite(CHARGE_ENABLE_PIN, HIGH);
  preCharging();
  float FB_VOLTAGE = readVoltage_FB();
  E_time = millis();
  float dt = (E_time - S_time) / 1000.0;
  float pidOutput = computePID(&voltagePID, FB_VOLTAGE, dt);
  ledcWrite(0, 511 - (int)pidOutput);
  S_time = millis();
}

void executeChargeControl() {
  switch (currentChargeState) {
    case IDLE: break;
    case WAITING_FOR_CONDITIONS: stopCharging(); break;
    case CHARGING_CC: performCCCharging(); break;
    case CHARGING_CV: performCVCharging(); break;
    case BALANCING:
      if (balanceStatus[0] && balanceStatus[1]) currentChargeState = COMPLETE;
      stopCharging(); break;
    case COMPLETE: stopCharging(); break;
    case ERROR: stopCharging(); break;
  }
}

// ----- SOC/SOH Estimation -----
float calculateSOC() {
  float avgVoltage = totalPackVoltage / 8.0;
  float soc = ((avgVoltage - 3.0) / (4.2 - 3.0)) * 100.0;
  return constrain(soc, 0.0, 100.0);
}

float calculateSOH() {
  return 95.0; // Placeholder
}

// ----- GUI Packager -----
BMSData getBMSData() {
    BMSData data;
    data.packVoltage = totalPackVoltage;
    data.current = measuredCurrent;
    data.soc = calculateSOC();
    data.soh = calculateSOH();
    for (int i = 0; i < NUM_CELLS; i++) { data.cellVoltages[i] = cells[i]; }
    data.slaveTemperatures[0] = temps[0];
    data.slaveTemperatures[1] = temps[0];
    data.slaveTemperatures[2] = temps[1];
    data.slaveTemperatures[3] = temps[1];

    if (currentChargeState == CHARGING_CC) {
        data.batteryState = "CHARGING_CC";
    } else if (currentChargeState == CHARGING_CV) {
        data.batteryState = "CHARGING_CV";
    } else if (currentChargeState == IDLE) {
        data.batteryState = "IDLE";
    } else if (currentChargeState == ERROR) {
        data.batteryState = "ERROR";
    } else if (currentChargeState == WAITING_FOR_CONDITIONS) {
        data.batteryState = "WAITING_FOR_CONDITIONS";
    } else if (currentChargeState == BALANCING) {
        data.batteryState = "BALANCING";
    } else if (currentChargeState == COMPLETE) {
        data.batteryState = "COMPLETE";
    }
    //} else if (currentChargeState == DISCHARGING) {
    //    data.batteryState = DISCHARGING;
    //}

    data.prechargeActive = precharge_state;
    data.protectionPMOSActive = (currentChargeState == CHARGING_CC || currentChargeState == CHARGING_CV);
    data.activeBalancingActive = balanceStatus[0] && balanceStatus[1];
    data.overvoltage = (maxCellVoltage > MAX_CELL_VOLTAGE);
    data.undervoltage = (minCellVoltage < MIN_CELL_VOLTAGE);
    data.overcurrent = (abs(measuredCurrent) > MAX_CURRENT);
    if (abs(data.current) > 0.1) {
        data.remainingRuntime = (data.soc / 100.0) * BATTERY_CAPACITY / abs(data.current);
    } else {
        data.remainingRuntime = 999.9;
    }
    unsigned long now = millis();
    if (lastCapacityUpdate > 0) {
        float deltaTime = (now - lastCapacityUpdate) / 3600000.0;
        cumulativeAh += abs(data.current) * deltaTime;
    }
    lastCapacityUpdate = now;
    data.cumulativeCapacity = cumulativeAh;
    data.timestamp = millis();
    return data;
}

// ----- WebSocket Functions -----
void sendBMSData() {
    currentBMS = getBMSData();
    DynamicJsonDocument json(1024);
    json["packVoltage"] = currentBMS.packVoltage;
    json["current"] = currentBMS.current;
    json["soc"] = currentBMS.soc;
    json["soh"] = currentBMS.soh;
    json["batteryState"] = currentBMS.batteryState;
    JsonArray cellVoltages = json.createNestedArray("cellVoltages");
    for (int i = 0; i < NUM_CELLS; i++) cellVoltages.add(currentBMS.cellVoltages[i]);
    JsonArray slaveTemps = json.createNestedArray("slaveTemperatures");
    for (int i = 0; i < NUM_SLAVES; i++) slaveTemps.add(currentBMS.slaveTemperatures[i]);
    json["prechargeActive"] = currentBMS.prechargeActive;
    json["protectionPMOSActive"] = currentBMS.protectionPMOSActive;
    json["activeBalancingActive"] = currentBMS.activeBalancingActive;
    json["overvoltage"] = currentBMS.overvoltage;
    json["undervoltage"] = currentBMS.undervoltage;
    json["overcurrent"] = currentBMS.overcurrent;
    json["remainingRuntime"] = currentBMS.remainingRuntime;
    json["cumulativeCapacity"] = currentBMS.cumulativeCapacity;
    json["timestamp"] = currentBMS.timestamp;
    String jsonString;
    serializeJson(json, jsonString);
    ws.textAll(jsonString);
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if(type == WS_EVT_CONNECT) {
        sendBMSData();
    } else if(type == WS_EVT_DISCONNECT) {
        // Optional: handle disconnect
    }
}

// ----- Setup -----
void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial2.begin(BMS_SERIAL_BAUD, SERIAL_8N1, BMS_RX_PIN, BMS_TX_PIN);
    pinMode(STARTUP_SENSE_PIN, INPUT);
    pinMode(FEEDBACK_PIN, INPUT);
    pinMode(PRECHARGE_RELAY, OUTPUT);
    digitalWrite(PRECHARGE_RELAY, LOW);
    pinMode(CHARGE_ENABLE_PIN, OUTPUT);
    digitalWrite(CHARGE_ENABLE_PIN, LOW);
    ledcSetup(0, PWM_FREQ, PWM_RES);
    ledcAttachPin(PWM_PIN, 0);
    ledcWrite(0, 511 - 0);
    pinMode(CURRENT_SENSOR_PIN, INPUT);
    initPID(&currentPID, 50.0, 10.0, 2.0, 0.0, 511.0);
    currentPID.setpoint = CC_TARGET_CURRENT;
    initPID(&voltagePID, 30.0, 5.0, 1.5, 0.0, 511.0);
    voltagePID.setpoint = CV_TARGET_VOLTAGE;
    currentChargeState = CALIBRATING_CURRENT_SENSOR;
    calibrateCurrentSensor();
    currentChargeState = IDLE;
    if(!SPIFFS.begin(true)) { Serial.println("ERROR: SPIFFS Mount Failed!"); return; }
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("WiFi connection failed!");
        return;
    }
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html", "text/html");
    });
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/style.css", "text/css");
    });
    server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/script.js", "text/javascript");
    });
    server.on("/api/bms", HTTP_GET, [](AsyncWebServerRequest *request) {
        BMSData data = getBMSData();
        DynamicJsonDocument json(1024);
        json["packVoltage"] = data.packVoltage;
        json["current"] = data.current;
        json["soc"] = data.soc;
        json["soh"] = data.soh;
        json["batteryState"] = data.batteryState;
        JsonArray cellVoltages = json.createNestedArray("cellVoltages");
        for (int i = 0; i < NUM_CELLS; i++) cellVoltages.add(data.cellVoltages[i]);
        JsonArray slaveTemps = json.createNestedArray("slaveTemperatures");
        for (int i = 0; i < NUM_SLAVES; i++) slaveTemps.add(data.slaveTemperatures[i]);
        json["prechargeActive"] = data.prechargeActive;
        json["protectionPMOSActive"] = data.protectionPMOSActive;
        json["activeBalancingActive"] = data.activeBalancingActive;
        json["overvoltage"] = data.overvoltage;
        json["undervoltage"] = data.undervoltage;
        json["overcurrent"] = data.overcurrent;
        json["remainingRuntime"] = data.remainingRuntime;
        json["cumulativeCapacity"] = data.cumulativeCapacity;
        json["timestamp"] = data.timestamp;
        String response;
        serializeJson(json, response);
        request->send(200, "application/json", response);
    });
    server.onNotFound([](AsyncWebServerRequest *request) {
        request->send(404, "text/plain", "Not Found");
    });
    server.begin();
    Serial.println("HTTP server started");
}

// ----- Main Loop -----
void loop() {
    unsigned long currentTime = millis();
    if (currentTime - lastBMSReadTime >= BMS_READ_INTERVAL) {
        lastBMSReadTime = currentTime;
        readBMSData();
    }
    if (currentTime - lastCurrentReadTime >= CURRENT_READ_INTERVAL) {
        lastCurrentReadTime = currentTime;
        if (currentSensorCalibrated) readCurrent();
    }
    if (currentTime - lastStartupCheckTime >= STARTUP_CHECK_INTERVAL) {
        lastStartupCheckTime = currentTime;
        waitForStartupCondition();
    }
    executeChargeControl();
    if (currentTime - lastWebUpdateTime >= WEB_UPDATE_INTERVAL) {
        lastWebUpdateTime = currentTime;
        sendBMSData();
    }
    ws.cleanupClients();
    delay(10);
}