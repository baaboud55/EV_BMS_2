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
#include "time.h"

/*
#include "KalmanFilter.h"
*/

#include "OTAManager.h"  // OTA firmware update support
#include "SOCEstimator.h"
#include "SOHEstimator.h"

// ------ Global Variable Definitions (from BMSConfig.h) ------
bool fbVoltageChanged = false;
bool currentChanged = false;
bool initialocv = false;

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

/*
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
*/

// SD Card object
SDCardManager sdCard;

// SOC/SOH Estimators (new modular approach)
SOCEstimator socEstimator;
SOHEstimator sohEstimator;

// Timing for SOC/SOH updates
unsigned long lastSOCUpdateTime = 0;
unsigned long lastSOHUpdateTime = 0;
unsigned long lastSDSaveTime = 0;
unsigned long lastCurrentMeasurementTime = 0;


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

// Timing variables
unsigned long lastBMSReadTime = 0;
unsigned long lastStartupCheckTime = 0;
unsigned long lastCurrentReadTime = 0;
unsigned long lastFBvoltageReadTime = 0;
unsigned long lastWebUpdateTime = 0;

// NTP Time Management for Saudi Arabia (UTC+3)
static bool timeInitialized = false;
static unsigned long lastNTPSync = 0;
const long gmtOffset_sec = 3 * 3600;     // Saudi Arabia is UTC+3
const int daylightOffset_sec = 0;        // No daylight saving time

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

/*
// ----- SOC/SOH Estimation -----
float calculateSOC() {
    if (!lookupTablesInitialized) {
        // Fallback to simple voltage-based estimation
        float avgVoltage = totalPackVoltage / 8.0;
        float soc = ((avgVoltage - 3.0) / (4.2 - 3.0)) * 100.0;
        return constrain(soc, 0.0, 100.0);
    }
    
    // Use Kalman filter for SOC estimation
    unsigned long currentTime = millis();
    static unsigned long lastUpdateTime = 0;
    
    if (lastUpdateTime == 0) {
        lastUpdateTime = currentTime;
        return estimatedSOC;
    }
    
    float dt = (currentTime - lastUpdateTime) / 1000.0;  // Convert to seconds
    lastUpdateTime = currentTime;
    
    // Update Kalman filter
    estimatedSOC = kfSOC.update(totalPackVoltage, measuredCurrent, avgTemp, dt);
    
    return estimatedSOC;
}

float calculateSOH() { 
    // Update SOH based on capacity measurement (simplified)
    // In real implementation, measure actual capacity during full charge/discharge
    float measuredCapacity = BATTERY_CAPACITY * (estimatedSOH / 100.0);  // Placeholder
    
    estimatedSOH = kfSOH.update(measuredCapacity, totalCycles);
    
    return estimatedSOH;
}
*/

// ----- SOC/SOH Estimation (New Modular Approach) -----
// Update SOC using coulomb counting
void updateSOC() {
    unsigned long currentTime = millis();
    
    // Calculate time delta
    if (lastCurrentMeasurementTime == 0) {
        lastCurrentMeasurementTime = currentTime;
        return;
    }
    
    float deltaTime_s = (currentTime - lastCurrentMeasurementTime) / 1000.0;
    lastCurrentMeasurementTime = currentTime;
    
    // Update SOC via coulomb counting
    socEstimator.update(measuredCurrent, deltaTime_s);
}

// Update SOH (placeholder - currently does nothing)
void updateSOH() {
    sohEstimator.update();
}

// Calibrate SOC from OCV (call at startup or after rest period)
void calibrateSOCFromOCV() {
    initialocv == true;
    // Calculate average cell voltage
    float avgCellVoltage = totalPackVoltage / 8.0;
    
    // Use average temperature
    float avgTemperature = (temps[0] + temps[1]) / 2.0;
    
    // Calibrate SOC
    socEstimator.calibrateFromOCV(avgCellVoltage, avgTemperature);
    
    Serial.printf("SOC calibrated: %.2f%% (Avg Cell V: %.3fV, Temp: %.1f°C)\n",
                socEstimator.getSOC(), avgCellVoltage, avgTemperature);
}

// Initialize NTP time synchronization for Saudi Arabia
void initializeNTPTime() {
    Serial.println("\n========================================");
    Serial.println("Initializing NTP Time Sync (Saudi Arabia UTC+3)...");
    Serial.println("========================================");
    
    // Configure NTP with Saudi Arabia timezone
    configTime(gmtOffset_sec, daylightOffset_sec, 
               "time.google.com", 
               "pool.ntp.org",
               "time.nist.gov");
    
    // Wait for time to be synchronized (timeout after 10 seconds)
    Serial.print("Waiting for NTP time sync");
    struct tm timeinfo;
    int retries = 0;
    
    while (!getLocalTime(&timeinfo) && retries < 20) {
        Serial.print(".");
        delay(500);
        retries++;
    }
    
    if (retries < 20) {
        timeInitialized = true;
        lastNTPSync = millis();
        
        Serial.println(" SUCCESS!");
        Serial.println("========================================");
        Serial.printf("📅 Current Date: %s, %02d %s %04d\n",
                     (timeinfo.tm_wday == 0) ? "Sunday" :
                     (timeinfo.tm_wday == 1) ? "Monday" :
                     (timeinfo.tm_wday == 2) ? "Tuesday" :
                     (timeinfo.tm_wday == 3) ? "Wednesday" :
                     (timeinfo.tm_wday == 4) ? "Thursday" :
                     (timeinfo.tm_wday == 5) ? "Friday" : "Saturday",
                     timeinfo.tm_mday,
                     (timeinfo.tm_mon == 0) ? "Jan" :
                     (timeinfo.tm_mon == 1) ? "Feb" :
                     (timeinfo.tm_mon == 2) ? "Mar" :
                     (timeinfo.tm_mon == 3) ? "Apr" :
                     (timeinfo.tm_mon == 4) ? "May" :
                     (timeinfo.tm_mon == 5) ? "Jun" :
                     (timeinfo.tm_mon == 6) ? "Jul" :
                     (timeinfo.tm_mon == 7) ? "Aug" :
                     (timeinfo.tm_mon == 8) ? "Sep" :
                     (timeinfo.tm_mon == 9) ? "Oct" :
                     (timeinfo.tm_mon == 10) ? "Nov" : "Dec",
                     timeinfo.tm_year + 1900);
        Serial.printf("🕐 Current Time: %02d:%02d:%02d (Saudi Arabia)\n",
                     timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        Serial.printf("📊 Unix Timestamp: %lu\n", (unsigned long)time(NULL));
        Serial.println("========================================\n");
        
        // Save initial time to SD card
        if (sdCard.isInitialized()) {
            sdCard.saveLastTimestamp((unsigned long)time(NULL));
            Serial.println("✓ Time saved to SD card for backup");
        }
    } else {
        Serial.println(" TIMEOUT!");
        Serial.println("========================================");
        Serial.println("⚠ NTP sync failed. Trying SD card backup...");
        
        // Try to load last known time from SD card
        if (sdCard.isInitialized()) {
            unsigned long savedTime = sdCard.loadLastTimestamp();
            if (savedTime > 1700000000) {  // Sanity check (after year 2023)
                // Set system time from SD card backup
                struct timeval tv;
                tv.tv_sec = savedTime;
                tv.tv_usec = 0;
                settimeofday(&tv, NULL);
                
                timeInitialized = true;
                Serial.printf("✓ Time restored from SD card: %lu\n", savedTime);
                Serial.println("⚠ Time will drift until NTP sync succeeds");
            } else {
                Serial.println("✗ No valid backup time found");
            }
        }
        Serial.println("========================================\n");
    }
}

// Periodic NTP resync (every 6 hours)
void resyncNTPTime() {
    if (!timeInitialized) return;
    
    unsigned long currentMillis = millis();
    
    // Resync every 6 hours
    if (currentMillis - lastNTPSync >= 6UL * 3600UL * 1000UL) {
        Serial.println("\n⏰ Resyncing NTP time (6 hour interval)...");
        
        struct tm timeinfo;
        if (getLocalTime(&timeinfo)) {
            lastNTPSync = currentMillis;
            
            Serial.printf("✓ Time resynced: %02d:%02d:%02d %02d/%02d/%04d\n",
                         timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
                         timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
            
            // Save to SD card
            if (sdCard.isInitialized()) {
                sdCard.saveLastTimestamp((unsigned long)time(NULL));
            }
        } else {
            Serial.println("⚠ Resync failed, continuing with internal clock");
        }
    }
    
    // Save time to SD card every 5 minutes for backup
    static unsigned long lastTimeSave = 0;
    if (sdCard.isInitialized() && (currentMillis - lastTimeSave >= 300000)) {
        lastTimeSave = currentMillis;
        sdCard.saveLastTimestamp((unsigned long)time(NULL));
    }
}

// Get current Unix epoch time in seconds (Saudi Arabia time)
unsigned long getEpochTime() {
    if (timeInitialized) {
        return (unsigned long)time(NULL);  // Return actual Unix timestamp
    } else {
        // Fallback: return millis as seconds (until NTP sync)
        return millis() / 1000;
    }
}

// ----- GUI Packager -----
BMSData getBMSData() {
    BMSData data;
    data.packVoltage = totalPackVoltage;
    data.current = measuredCurrent;

/*
    data.soc = calculateSOC();
    data.soh = calculateSOH();
*/

    data.soc = socEstimator.getSOC();
    data.soh = sohEstimator.getSOH();

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
        data.batteryState = "WAITING";
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

    /*
    // Get Remaining runtime (in hours)
    if (abs(data.current) > 0.1) {
        data.remainingRuntime = (data.soc / 100.0) * BATTERY_CAPACITY / abs(data.current);
    } else {
        data.remainingRuntime = 999.9;
    }
    */

    // Get cumulative capacity directly from SOC estimator
    data.cumulativeCapacity = socEstimator.getCumulativeAh();
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
//    json["remainingRuntime"] = currentBMS.remainingRuntime;
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

/*
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
*/

/*
// Save current SOC/SOH to SD card periodically
void saveStateToSD() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastSDSaveTime >= SD_SAVE_INTERVAL) {
        lastSDSaveTime = currentTime;
        
        // Log SOC data
        sdCard.logSOCData(estimatedSOC, totalPackVoltage, avgTemp, currentTime);
        
        // Log SOH data
        sdCard.logSOHData(estimatedSOH, totalCycles, BATTERY_CAPACITY * (estimatedSOH / 100.0), currentTime);
        
        // Log complete BMS data (NEW)
        sdCard.logBMSData(cells, NUM_CELLS, totalPackVoltage, avgTemp, measuredCurrent, currentTime);
        
        Serial.println("State saved to SD card");
    }
}
*/

// Save current SOC/SOH to SD card periodically
void saveStateToSD() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastSDSaveTime >= SD_SAVE_INTERVAL) {
        lastSDSaveTime = currentTime;
        
        // Log SOC data
        float currentSOC = socEstimator.getSOC();
        sdCard.logSOCData(currentSOC, totalPackVoltage, avgTemp, currentTime);
        
        // Log SOH data (placeholder)
        float currentSOH = sohEstimator.getSOH();
        int cycleCount = sohEstimator.getCycleCount();
        sdCard.logSOHData(currentSOH, cycleCount, BATTERY_CAPACITY * (currentSOH / 100.0), currentTime);
        
        // Log complete BMS data
        sdCard.logBMSData(cells, NUM_CELLS, totalPackVoltage, avgTemp, measuredCurrent, currentTime);
        
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

    /*
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
    */

    // Initialize SD card (optional - only use if needed)
    if (!sdCard.begin()) {
        Serial.println("WARNING: SD Card initialization failed!");
        Serial.println("Continuing without SD card support...");
    } else {
        Serial.println("SD Card initialized successfully!");
        sdCard.listFiles();
    }
    
    // Initialize SOC estimator
    socEstimator.begin(BATTERY_CAPACITY, 50.0);  // Start at 50% SOC
    
    // Initialize SOH estimator
    sohEstimator.begin(BATTERY_CAPACITY);
    
    // Perform initial BMS read to get voltage
    readBMSData();
    
    /*
    // Calibrate SOC from OCV at startup
    calibrateSOCFromOCV();
    */


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
        initializeNTPTime();              // Initialize NTP time synchronization
        
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
//        json["remainingRuntime"] = data.remainingRuntime;
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
    resyncNTPTime();          // Keep time synchronized

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

    /*
    executeChargeControl();
    
    if (currentTime - lastWebUpdateTime >= WEB_UPDATE_INTERVAL) {
      lastWebUpdateTime = currentTime;
      sendBMSData();
    }
    
    // Save state to SD card periodically
    if (sdCard.isInitialized()) {
        saveStateToSD();
    }
    */
    
    executeChargeControl();
    
    // Update SOC estimation
    if (currentTime - lastSOCUpdateTime >= SOC_UPDATE_INTERVAL) {
        lastSOCUpdateTime = currentTime;
        updateSOC();
    }
    
    // Update SOH estimation (placeholder)
    if (currentTime - lastSOHUpdateTime >= SOH_UPDATE_INTERVAL) {
        lastSOHUpdateTime = currentTime;
        updateSOH();
    }
    
    if (currentTime - lastWebUpdateTime >= WEB_UPDATE_INTERVAL) {
      lastWebUpdateTime = currentTime;
      sendBMSData();
    }
    
    // Save state to SD card periodically (optional)
    if (sdCard.isInitialized()) {
        saveStateToSD();
    }

    if (currentChargeState == WAITING_FOR_CONDITIONS || currentChargeState == BALANCING)
    {
      // Calibrate SOC from OCV at startup
      if (initialocv == true) {
        // Do nothing
      }
      else
        calibrateSOCFromOCV();
    }

    ws.cleanupClients();
    delay(10);
}