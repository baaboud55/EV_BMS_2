/*
 * Enhanced BMS Monitor - ESP32 WiFi Dashboard with Real Sensor Integration
 * 
 * Features:
 * - Real-time BMS data from daisy-chained slaves
 * - ACS712 current sensing with calibration
 * - Two-stage PID charging control (CC/CV)
 * - Web-based dashboard via WiFi
 * - WebSocket real-time streaming
 * - Over-The-Air (OTA) firmware updates
 * - Unified timing system (all in milliseconds)
 */

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "BMSConfig.h"
#include "PIDController.h"
#include "ChargeController.h"
#include "SDCardManager.h"
#include "KalmanFilter.h"
#include "OTAManager.h"  // OTA firmware update support

// ------ Global Variable Definitions (from BMSConfig.h) ------
bool fbVoltageChanged = false;
bool currentChanged = false;

// ------ WiFi Configuration ------
const char* ssid = "Galaxy";        // Replace with your WiFi SSID
const char* password = "mmmmmmmm"; // Replace with your WiFi password

// ------ Server Objects ------
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ------ OTA Manager ------
extern OTAManager otaManager;

// ------ Function Forward Declarations ------
unsigned long getEpochTime();

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

// SD Card and Kalman Filter objects
SDCardManager sdCard;
KalmanFilterSOC kfSOC;
KalmanFilterSOH kfSOH;

// SOC/SOH state tracking
float estimatedSOC = 50.0;
float estimatedSOH = 100.0;
int totalCycles = 0;
unsigned long lastSDSaveTime = 0;
bool lookupTablesInitialized = false;

// Current sensing
float measuredCurrent = 0.0;
float FB_VOLTAGE = 0.0;
float currentOffsetVoltage = 0.0;
bool currentSensorCalibrated = false;

// Charging state
ChargeState currentChargeState = IDLE;
bool precharge_state = false;

// PID controllers
PIDController currentPID;  // For CC mode
PIDController voltagePID;  // For CV mode

// Timing variables (ALL IN MILLISECONDS)
unsigned long lastBMSReadTime = 0;
unsigned long lastStartupCheckTime = 0;
unsigned long lastCurrentReadTime = 0;
unsigned long lastFBvoltageReadTime = 0;
unsigned long lastWebUpdateTime = 0;

// Cumulative capacity tracking
static float cumulativeAh = 0;
static unsigned long lastCapacityUpdate = 0;  // milliseconds

// Simple timestamp counter (starts from 0 at boot)
static unsigned long timestampCounter = 0;

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
  digitalWrite(LOAD_ENABLE_PIN, LOW);

  delay(5000);
  long totalRawValue = 0;
  for (int i = 0; i < 1000; i++) {
    totalRawValue += analogRead(CURRENT_SENSOR_PIN);
    delay(1);
  }
  currentOffsetVoltage = ((float)totalRawValue / 1000.0 / 4096.0) * ACS_VCC_VOLTAGE;
  currentSensorCalibrated = true;
  Serial.println("ACS712 calibration complete.");
  Serial.println(currentOffsetVoltage);
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
  //if (abs(measuredCurrent) < 0.15) {
  //  measuredCurrent = 0.0;
  //}
}

// ----- Voltage Reading -----
void readVoltage_FB() {
  const int samples = 50;
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(FEEDBACK_PIN);
  }
  int adcValue = sum / samples;
  FB_VOLTAGE = (adcValue * 3.3 / 4095.0) * ((FBR1 + FBR2) / FBR2);
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

  bool loadOK = digitalRead(LOAD_SWITCH_PIN);
  bool tempOK = (avgTemp >= MIN_TEMP && avgTemp <= MAX_TEMP);
  bool voltageOK = (minCellVoltage >= MIN_CHARGE_VOLTAGE && maxCellVoltage < MAX_CELL_VOLTAGE);
  bool sensorOK = currentSensorCalibrated;
  bool chargerOK = (IN_VOLTAGE >= 5.0);
  

  if (tempOK && voltageOK && sensorOK && chargerOK) {
    if (currentChargeState == IDLE || currentChargeState == WAITING_FOR_CONDITIONS || currentChargeState == DISCHARGING || currentChargeState == BALANCING) {
      currentChargeState = CHARGING_CC;
    }
  }

  else if (tempOK && voltageOK && sensorOK && !chargerOK && loadOK) { 
    if (currentChargeState == IDLE || currentChargeState == WAITING_FOR_CONDITIONS || currentChargeState == DISCHARGING || currentChargeState == BALANCING) {
      currentChargeState = DISCHARGING;
    }
  }

  else if (tempOK && voltageOK && sensorOK) { 
    if (!balanceStatus[0] || !balanceStatus[1]) {
      currentChargeState = BALANCING;
    }
  }

  else {
    if (currentChargeState == CHARGING_CC || currentChargeState == CHARGING_CV || currentChargeState == DISCHARGING || currentChargeState == BALANCING || currentChargeState == ERROR) {
      currentChargeState = WAITING_FOR_CONDITIONS;
      stopCharging();

    } else if (currentChargeState == IDLE|| currentChargeState == WAITING_FOR_CONDITIONS || currentChargeState == ERROR) {
      currentChargeState = WAITING_FOR_CONDITIONS;
    }
  }
}

void performCCCharging() {
  if (maxCellVoltage >= MAX_CELL_VOLTAGE) {
    currentChargeState = CHARGING_CV;
    voltagePID.integral = 0;
    voltagePID.prev_error = 0;
    return;
  }
  if (avgTemp > MAX_TEMP || avgTemp < MIN_TEMP) {
    currentChargeState = WAITING_FOR_CONDITIONS;
    return;
  }
  digitalWrite(LOAD_ENABLE_PIN, LOW);
  digitalWrite(CHARGE_ENABLE_PIN, HIGH);
  preCharging();
  if (currentChanged == true) {  
    float pidOutput = computePID(&currentPID, measuredCurrent, 0.1);
    ledcWrite(0, 511 - (int)pidOutput);
    currentChanged = false;
  }
}

void performCVCharging() {
  if (maxCellVoltage >= MAX_CELL_VOLTAGE || measuredCurrent <= CHARGE_COMPLETE_CURRENT) {
    if (!balanceStatus[0] || !balanceStatus[1]) {
      currentChargeState = BALANCING;
    } else {
      currentChargeState = WAITING_FOR_CONDITIONS;
    }
    return;
  }
  if (avgTemp > MAX_TEMP || avgTemp < MIN_TEMP) {
    currentChargeState = WAITING_FOR_CONDITIONS;
    return;
  }
  digitalWrite(LOAD_ENABLE_PIN, LOW);
  digitalWrite(CHARGE_ENABLE_PIN, HIGH);
  preCharging();
  if (fbVoltageChanged == true) {  
    float pidOutput = computePID(&voltagePID, FB_VOLTAGE, 0.1);
    ledcWrite(0, 511 - (int)pidOutput);
    fbVoltageChanged = false;
  }
}

void performDischarging() {
digitalWrite(LOAD_ENABLE_PIN, HIGH);
digitalWrite(CHARGE_ENABLE_PIN, LOW);
preCharging();
}

void executeChargeControl() {
  switch (currentChargeState) {
    case IDLE: break;
    case WAITING_FOR_CONDITIONS: stopCharging(); break;
    case CHARGING_CC: performCCCharging(); break;
    case CHARGING_CV: performCVCharging(); break;
    case DISCHARGING: performDischarging(); break;
    case BALANCING: preCharging(); if (balanceStatus[0] && balanceStatus[1]) stopCharging(); break;
    case ERROR: stopCharging(); break;
  }
}

// ----- SOC/SOH Estimation -----
float calculateSOC() {
    if (!lookupTablesInitialized) {
        // Fallback to simple voltage-based estimation
        float avgVoltage = totalPackVoltage / 8.0;
        float soc = ((avgVoltage - 3.0) / (4.2 - 3.0)) * 100.0;
        return constrain(soc, 0.0, 100.0);
    }
    
    // Use Kalman filter for SOC estimation
    unsigned long currentTime_ms = millis();  // UNIFIED: Always in ms
    static unsigned long lastUpdateTime_ms = 0;
    
    if (lastUpdateTime_ms == 0) {
        lastUpdateTime_ms = currentTime_ms;
        return estimatedSOC;
    }
    
    // Calculate time delta in ms, convert to seconds only for Kalman filter
    unsigned long dt_ms = currentTime_ms - lastUpdateTime_ms;
    float dt_seconds = dt_ms * MS_TO_SECONDS;  // UNIFIED: Use conversion constant
    lastUpdateTime_ms = currentTime_ms;
    
    // Update Kalman filter
    estimatedSOC = kfSOC.update(totalPackVoltage, measuredCurrent, avgTemp, dt_seconds);
    
    return estimatedSOC;
}

float calculateSOH() {
    // Update SOH based on capacity measurement (simplified)
    // In real implementation, measure actual capacity during full charge/discharge
    float measuredCapacity = BATTERY_CAPACITY * (estimatedSOH / 100.0);  // Placeholder
    
    estimatedSOH = kfSOH.update(measuredCapacity, totalCycles);
    
    return estimatedSOH;
}

// Get simple timestamp (counter starting from 0)
unsigned long getEpochTime() {
    return timestampCounter++;
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
    data.slaveTemperatures[1] = temps[1];

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
    } else if (currentChargeState == DISCHARGING) {
        data.batteryState = "DISCHARGING";
    }

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
    
    // UNIFIED: All timing in milliseconds, convert to hours only for capacity calculation
    unsigned long now_ms = millis();
    if (lastCapacityUpdate > 0) {
        unsigned long dt_ms = now_ms - lastCapacityUpdate;
        float dt_hours = dt_ms * MS_TO_HOURS;  // UNIFIED: Use conversion constant
        cumulativeAh += abs(data.current) * dt_hours;
    }
    lastCapacityUpdate = now_ms;
    
    data.cumulativeCapacity = cumulativeAh;
    data.timestamp = getEpochTime();
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

// Initialize lookup tables on SD card (first-time setup)
void initializeLookupTables() {
    Serial.println("Creating default lookup tables...");
    
    // Create SOC lookup table (simplified)
    SOCLookupEntry socTable[11];
    for (int i = 0; i <= 10; i++) {
        float soc = i * 10.0;
        float tempFactor_minus10 = 1.0 - 0.028;
        float tempFactor_0 = 1.0 - 0.020;
        float tempFactor_10 = 1.0 - 0.012;
        float tempFactor_20 = 1.0 - 0.004;
        float tempFactor_25 = 1.0;
        float tempFactor_30 = 1.0 + 0.004;
        float tempFactor_40 = 1.0 + 0.012;
        
        socTable[i].soc_percent = soc;
        socTable[i].ocv_minus10C = (3.0 + (4.2 - 3.0) * pow(soc/100.0, 0.9)) * tempFactor_minus10;
        socTable[i].ocv_0C = (3.0 + (4.2 - 3.0) * pow(soc/100.0, 0.9)) * tempFactor_0;
        socTable[i].ocv_10C = (3.0 + (4.2 - 3.0) * pow(soc/100.0, 0.9)) * tempFactor_10;
        socTable[i].ocv_20C = (3.0 + (4.2 - 3.0) * pow(soc/100.0, 0.9)) * tempFactor_20;
        socTable[i].ocv_25C = (3.0 + (4.2 - 3.0) * pow(soc/100.0, 0.9)) * tempFactor_25;
        socTable[i].ocv_30C = (3.0 + (4.2 - 3.0) * pow(soc/100.0, 0.9)) * tempFactor_30;
        socTable[i].ocv_40C = (3.0 + (4.2 - 3.0) * pow(soc/100.0, 0.9)) * tempFactor_40;
    }
    sdCard.saveSOCLookupTable(socTable, 11);
    
    // Create SOH lookup table
    SOHLookupEntry sohTable[11] = {
        {0, 100.0, 100.0, 0.0},
        {100, 98.0, 98.0, 2.0},
        {200, 96.0, 96.0, 5.0},
        {300, 94.0, 94.0, 8.0},
        {400, 92.0, 92.0, 12.0},
        {500, 89.0, 89.0, 16.0},
        {600, 86.0, 86.0, 21.0},
        {700, 83.0, 83.0, 27.0},
        {800, 79.0, 79.0, 34.0},
        {900, 75.0, 75.0, 42.0},
        {1000, 70.0, 70.0, 50.0}
    };
    sdCard.saveSOHLookupTable(sohTable, 11);
    
    Serial.println("Lookup tables created successfully!");
}

// Load initial SOC/SOH from SD card
void loadInitialState() {
    // Try to load from lookup tables
    if (sdCard.fileExists(SOC_LOOKUP_FILE) && sdCard.fileExists(SOH_LOOKUP_FILE)) {
        Serial.println("Loading lookup tables from SD card...");
        
        // Initialize SOC from voltage reading
        float initialOCV = totalPackVoltage;  // Use measured pack voltage
        estimatedSOC = sdCard.interpolateSOCFromOCV(initialOCV, avgTemp);
        kfSOC.initialize(estimatedSOC, KF_PROCESS_NOISE_COV, KF_MEASUREMENT_NOISE_COV);
        
        // Initialize SOH from cycle count (assuming 0 for new battery)
        estimatedSOH = sdCard.interpolateSOHFromCycles(totalCycles);
        kfSOH.initialize(estimatedSOH, KF_PROCESS_NOISE_COV * 10, KF_MEASUREMENT_NOISE_COV * 10);
        
        lookupTablesInitialized = true;
        Serial.printf("Initial state loaded: SOC=%.2f%%, SOH=%.2f%%\n", estimatedSOC, estimatedSOH);
    } else {
        Serial.println("Lookup tables not found. Creating defaults...");
        initializeLookupTables();
        loadInitialState();  // Retry after creating tables
    }
}

// Save current SOC/SOH to SD card periodically
void saveStateToSD() {
    unsigned long currentTime_ms = millis();  // UNIFIED: Always in ms
    
    if (currentTime_ms - lastSDSaveTime >= SD_SAVE_INTERVAL) {
        lastSDSaveTime = currentTime_ms;
        
        // Log SOC data
        sdCard.logSOCData(estimatedSOC, totalPackVoltage, avgTemp, currentTime_ms);
        
        // Log SOH data
        sdCard.logSOHData(estimatedSOH, totalCycles, BATTERY_CAPACITY * (estimatedSOH / 100.0), currentTime_ms);
        
        // Log complete BMS data
        sdCard.logBMSData(cells, NUM_CELLS, totalPackVoltage, avgTemp, measuredCurrent, currentTime_ms);
        
        Serial.println("State saved to SD card");
    }
}

// ----- Setup -----
void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial2.begin(BMS_SERIAL_BAUD, SERIAL_8N1, BMS_RX_PIN, BMS_TX_PIN);
    pinMode(STARTUP_SENSE_PIN, INPUT);
    pinMode(FEEDBACK_PIN, INPUT);
    pinMode(LOAD_SWITCH_PIN, INPUT);
    pinMode(PRECHARGE_RELAY, OUTPUT);
    pinMode(LOAD_ENABLE_PIN, OUTPUT);
    pinMode(CHARGE_ENABLE_PIN, OUTPUT);
    digitalWrite(PRECHARGE_RELAY, LOW);
    digitalWrite(LOAD_ENABLE_PIN, LOW);
    digitalWrite(CHARGE_ENABLE_PIN, LOW);
    ledcSetup(0, PWM_FREQ, PWM_RES);
    ledcAttachPin(PWM_PIN, 0);
    ledcWrite(0, 511 - 0);
    pinMode(CURRENT_SENSOR_PIN, INPUT);
    
    initPID(&currentPID, 20.0, 4.0, 6.0, 0.0, 511.0);
    currentPID.setpoint = CC_TARGET_CURRENT;
    initPID(&voltagePID, 30.0, 5.0, 1.5, 0.0, 511.0);
    voltagePID.setpoint = CV_TARGET_VOLTAGE;
    currentChargeState = CALIBRATING_CURRENT_SENSOR;
    calibrateCurrentSensor();
    currentChargeState = IDLE;

    if(!SPIFFS.begin(true)) { Serial.println("ERROR: SPIFFS Mount Failed!"); return; }

    // Initialize SD card
    if (!sdCard.begin()) {
        Serial.println("WARNING: SD Card initialization failed!");
        Serial.println("Continuing without SD card support...");
    } else {
        // List files on SD card
        sdCard.listFiles();
        
        // Load initial state from SD card
        loadInitialState();
    }
    
    // Connect to WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.println("WiFi connected!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        
        // Initialize OTA Manager
        if (otaManager.begin("ESP32_BMS", "bms2024")) {
            Serial.println("OTA Manager initialized successfully!");
            Serial.println("You can now update firmware via:");
            Serial.println("1. PlatformIO OTA upload (see platformio.ini)");
            Serial.println("2. Web interface at http://" + WiFi.localIP().toString() + "/update");
        } else {
            Serial.println("WARNING: OTA Manager initialization failed!");
        }
    } else {
        Serial.println();
        Serial.println("WiFi connection failed!");
        Serial.println("OTA updates will not be available.");
    }
    
    // Setup WebSocket
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    
    // Setup web routes
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
    
    // Setup OTA web interface
    otaManager.setupWebOTA(&server);
    
    server.onNotFound([](AsyncWebServerRequest *request) {
        request->send(404, "text/plain", "Not Found");
    });
    
    server.begin();
    Serial.println("HTTP server started");
}

// ----- Main Loop -----
void loop() {
    // Handle OTA updates
    otaManager.handle();
    
    // UNIFIED: All timing checks use milliseconds
    unsigned long currentTime = millis();
    
    if (currentTime - lastBMSReadTime >= BMS_READ_INTERVAL) {
      lastBMSReadTime = currentTime;
      readBMSData();
    }

    if (currentTime - lastCurrentReadTime >= CURRENT_READ_INTERVAL) {
      lastCurrentReadTime = currentTime;
      if (currentSensorCalibrated) readCurrent();
      currentChanged = true;
    }

    if (currentTime - lastFBvoltageReadTime >= FB_VOLTAGE_READ_INTERVAL) {
      lastFBvoltageReadTime = currentTime;
      readVoltage_FB();
      fbVoltageChanged = true;
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
    
    // Save state to SD card periodically
    if (sdCard.isInitialized()) {
        saveStateToSD();
    }
    
    ws.cleanupClients();
    delay(10);
}