/*
 * Enhanced BMS Monitor - ESP32 WiFi Dashboard with Real Sensor Integration
 * 
 * Integrates real BMS sensor reading with web GUI dashboard
 * Features:
 * - Real-time BMS monitoring via daisy-chain (GPIO 16 RX, GPIO 17 TX)
 * - ACS712 current sensing (GPIO 1)
 * - Two-stage charging: CC @ 1A until 32.5V, CV @ 33.2V until current drops
 * - PID control for both CC and CV modes
 * - Automatic current sensor calibration
 * - WiFi web dashboard for monitoring
 * - Safety interlocks (temperature, voltage, balance)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "BMSConfig.h"
#include "PIDController.h"
#include "ChargeController.h"

// ========== WiFi Configuration ==========
const char* ssid = "Galaxy S21+";        // Replace with your WiFi SSID
const char* password = "77897890"; // Replace with your WiFi password

// ==================== WEB SERVER ====================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ==================== TIMING VARIABLES ====================
unsigned long lastBMSReadTime = 0;
unsigned long lastStartupCheckTime = 0;
unsigned long lastCurrentReadTime = 0;
unsigned long lastWebUpdateTime = 0;
const unsigned long WEB_UPDATE_INTERVAL = 1000;    // 1 second for web updates
unsigned long S_time = 0;
unsigned long E_time = 0;

// ==================== BATTERY STATE VARIABLES ====================
float cells[8];
float temps[2];
int balanceStatus[2];
float totalPackVoltage = 0;
float maxCellVoltage = 0;
float minCellVoltage = 0;
float avgTemp = 0;
bool precharge_state = false;

// Current sensing
float measuredCurrent = 0.0;  // Measured current in Amps
float currentOffsetVoltage = 0.0;  // Will be auto-calibrated
bool currentSensorCalibrated = false;

// PID controllers
PIDController currentPID;  // For constant current control
PIDController voltagePID;  // For constant voltage control

// Charge state
ChargeState currentState = IDLE;

// ==================== FORWARD DECLARATIONS ====================
void calibrateCurrentSensor();
void readCurrent();
void readBMSData();
bool parseAllData(String data, float cells[], float temps[], int balance[]);
void displayBMSData();
void calculatePackStatistics();
void waitForStartupCondition();
void executeChargeControl();
void performCCCharging();
void performCVCharging();
void stopCharging();
void preCharging();
float readVoltage_FB();
float readVoltage_IN();
void sendWebSocketUpdate();
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
                     void *arg, uint8_t *data, size_t len);

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 17, 16);  // BMS communication (RX=17, TX=16)
  
  // Initialize charge control pins
  pinMode(STARTUP_SENSE_PIN, INPUT);
  pinMode(FEEDBACK_PIN, INPUT);
  pinMode(PRECHARGE_RELAY, OUTPUT);
  digitalWrite(PRECHARGE_RELAY, LOW);  
  pinMode(CHARGE_ENABLE_PIN, OUTPUT);
  digitalWrite(CHARGE_ENABLE_PIN, LOW);  // Charger disabled initially
  
  // Configure PWM for charge current control
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(PWM_PIN, 511 - 0);  // Start with 0 duty cycle (inverted PWM)
  
  // Initialize analog pin for current sensor
  pinMode(CURRENT_SENSOR_PIN, INPUT);
  
  // Initialize PID controllers
  // Current PID: For CC mode (target = 1A)
  initPID(&currentPID, 50.0, 10.0, 2.0, 0.0, 511.0);
  currentPID.setpoint = CC_TARGET_CURRENT;
  
  // Voltage PID: For CV mode (target = 33.2V)
  initPID(&voltagePID, 30.0, 5.0, 1.5, 0.0, 511.0);
  voltagePID.setpoint = CV_TARGET_VOLTAGE;
  
  Serial.println("╔════════════════════════════════════════════════════════╗");
  Serial.println("║  ESP32-S3 BMS + Two-Stage PID Charge Controller       ║");
  Serial.println("║  with ACS712 Current Sensing & WiFi Dashboard         ║");
  Serial.println("╚════════════════════════════════════════════════════════╝");
  Serial.println();
  
  // Initialize LittleFS
  if (!LittleFS.begin(true, "/littlefs", 10, "littlefs")) {
    Serial.println("LittleFS Mount Failed");
    return;
  }
  Serial.println("✓ LittleFS mounted");
  
  // Setup WiFi Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.println("\n✓ WiFi AP Started");
  Serial.print("  SSID: "); Serial.println(ssid);
  Serial.print("  IP address: "); Serial.println(WiFi.softAPIP());
  
  // Setup WebSocket
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  
  // Serve static files
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  
  // Start server
  server.begin();
  Serial.println("✓ HTTP server started");
  Serial.println();
  
  delay(1000);
  
  // Start current sensor calibration
  currentState = CALIBRATING_CURRENT_SENSOR;
}

// ==================== MAIN LOOP ====================
void loop() {
  unsigned long currentTime = millis();
  
  // Handle current sensor calibration state
  if (currentState == CALIBRATING_CURRENT_SENSOR) {
    calibrateCurrentSensor();
    currentState = IDLE;
    return;
  }
  
  // Read BMS data every 2 seconds
  if (currentTime - lastBMSReadTime >= BMS_READ_INTERVAL) {
    lastBMSReadTime = currentTime;
    readBMSData();
  }
  
  // Read current sensor every 100ms
  if (currentTime - lastCurrentReadTime >= CURRENT_READ_INTERVAL) {
    lastCurrentReadTime = currentTime;
    if (currentSensorCalibrated) {
      readCurrent();
    }
  }
  
  // Check startup conditions every 2 seconds
  if (currentTime - lastStartupCheckTime >= STARTUP_CHECK_INTERVAL) {
    lastStartupCheckTime = currentTime;
    waitForStartupCondition();
  }
  
  // Send web updates every 1 second
  if (currentTime - lastWebUpdateTime >= WEB_UPDATE_INTERVAL) {
    lastWebUpdateTime = currentTime;
    sendWebSocketUpdate();
  }
  
  // Execute charging state machine
  executeChargeControl();
  
  // Clean up WebSocket clients
  ws.cleanupClients();
}

// ==================== CURRENT SENSOR CALIBRATION ====================
void calibrateCurrentSensor() {
  Serial.println("╔════════════════════════════════════════════════════════╗");
  Serial.println("║           ACS712 AUTO-CALIBRATION                      ║");
  Serial.println("╚════════════════════════════════════════════════════════╝");
  Serial.println();
  Serial.println("⚠ IMPORTANT: Ensure NO current is flowing through sensor!");
  Serial.println("Starting calibration in 5 seconds...");
  digitalWrite(PRECHARGE_RELAY, LOW);
  digitalWrite(CHARGE_ENABLE_PIN, LOW);
  delay(5000);
  Serial.println("Calibrating...");
  
  long totalRawValue = 0;
  for (int i = 0; i < 1000; i++) {
    totalRawValue += analogRead(CURRENT_SENSOR_PIN);
    delay(1);
  }
  
  // Calculate offset voltage
  currentOffsetVoltage = ((float)totalRawValue / 1000.0 / 4096.0) * ACS_VCC_VOLTAGE;
  currentSensorCalibrated = true;
  
  Serial.print("✓ Calibration complete! Offset voltage: ");
  Serial.print(currentOffsetVoltage, 2);
  Serial.println(" mV");
  Serial.println("════════════════════════════════════════════════════════");
  Serial.println();
  delay(1000);
}

// ==================== CURRENT READING ====================
void readCurrent() {
  // Average multiple samples for stability
  const int numSamples = 50;  // Reduced from 100 for faster response
  long totalRawValue = 0;
  
  for (int i = 0; i < numSamples; i++) {
    totalRawValue += analogRead(CURRENT_SENSOR_PIN);
  }
  
  float avgRawValue = (float)totalRawValue / numSamples;
  
  // Convert to voltage
  float voltage_mV = (avgRawValue / 4096.0) * ACS_VCC_VOLTAGE;
  
  // Calculate current (using calibrated formula)
  measuredCurrent = (currentOffsetVoltage - voltage_mV) / (ACS_SENSITIVITY * 1000);
}

// ==================== BMS DATA READING ====================
void readBMSData() {
  // Send request down the chain
  Serial2.println("READ_VOLTAGES");
  
  // Wait for response from Slave 2 (end of chain)
  unsigned long timeout = millis();
  while (!Serial2.available() && (millis() - timeout < 2000)) {
    delay(10);
  }
  
  if (Serial2.available()) {
    String voltageData = Serial2.readStringUntil('\n');
    
    if (parseAllData(voltageData, cells, temps, balanceStatus)) {
      displayBMSData();
      calculatePackStatistics();
    } else {
      Serial.println("⚠ Error: Failed to parse BMS data");
      Serial.print("Raw data: "); Serial.println(voltageData);
      currentState = ERROR;
    }
  } else {
    Serial.println("⚠ Error: No response from BMS chain");
    currentState = ERROR;
  }
}

// ==================== DATA PARSING ====================
bool parseAllData(String data, float cells[], float temps[], int balance[]) {
  int idx[11];
  idx[0] = data.indexOf(',');
  if (idx[0] == -1) return false;
  
  for (int i = 1; i < 11; i++) {
    idx[i] = data.indexOf(',', idx[i-1] + 1);
    if (idx[i] == -1) return false;
  }
  
  // Parse 8 cell voltages
  cells[0] = data.substring(0, idx[0]).toFloat();
  for (int i = 1; i < 8; i++) {
    cells[i] = data.substring(idx[i-1] + 1, idx[i]).toFloat();
  }
  
  // Parse temperatures
  temps[0] = data.substring(idx[7] + 1, idx[8]).toFloat();
  temps[1] = data.substring(idx[8] + 1, idx[9]).toFloat();
  
  // Parse balance status
  balance[0] = data.substring(idx[9] + 1, idx[10]).toInt();
  balance[1] = data.substring(idx[10] + 1).toInt();
  
  return true;
}

// ==================== BMS DATA DISPLAY ====================
void displayBMSData() {
  Serial.println("\n╔════════════════════════════════════════════════════════╗");
  Serial.println("║                    BMS DATA                            ║");
  Serial.println("╚════════════════════════════════════════════════════════╝");

  Serial.println("\n┌─── Slave 1 (Cells 1-4) ───┐");
  for (int i = 0; i < 4; i++) {
    Serial.print("│ Cell "); Serial.print(i+1); 
    Serial.print(": "); Serial.print(cells[i], 3); Serial.println(" V");
  }
  Serial.print("│ Temperature: "); Serial.print(temps[0], 2); Serial.println(" °C");
  Serial.print("│ Balance: "); 
  Serial.println(balanceStatus[0] ? "✓ OK" : "⚠ Imbalanced");
  Serial.println("└───────────────────────────┘");
  
  Serial.println("\n┌─── Slave 2 (Cells 5-8) ───┐");
  for (int i = 4; i < 8; i++) {
    Serial.print("│ Cell "); Serial.print(i+1); 
    Serial.print(": "); Serial.print(cells[i], 3); Serial.println(" V");
  }
  Serial.print("│ Temperature: "); Serial.print(temps[1], 2); Serial.println(" °C");
  Serial.print("│ Balance: "); 
  Serial.println(balanceStatus[1] ? "✓ OK" : "⚠ Imbalanced");
  Serial.println("└───────────────────────────┘");
  
  Serial.print("\n📊 Total Pack Voltage: "); 
  Serial.print(totalPackVoltage, 3); Serial.println(" V");
}

// ==================== PACK STATISTICS ====================
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

// ==================== STARTUP CONDITION CHECK ====================
void waitForStartupCondition() {
  Serial.println("\n╔════════════════════════════════════════════════════════╗");
  Serial.println("║            STARTUP CONDITIONS CHECK                    ║");
  Serial.println("╚════════════════════════════════════════════════════════╝");
  float IN_VOLTAGE = readVoltage_IN();

  bool tempOK = (avgTemp >= MIN_TEMP && avgTemp <= MAX_TEMP);
  bool voltageOK = (minCellVoltage >= MIN_CHARGE_VOLTAGE && maxCellVoltage < MAX_CELL_VOLTAGE);
  bool sensorOK = currentSensorCalibrated;
  bool chargerOK = (IN_VOLTAGE >= 1.0);
  
  Serial.print("🌡️ Temperature: "); 
  Serial.print(avgTemp, 2); Serial.print(" °C ");
  Serial.println(tempOK ? "✓ OK" : "✗ FAIL");
  
  Serial.print("⚡ Pack Voltage: "); 
  Serial.print(totalPackVoltage, 2); Serial.print(" V ");
  Serial.println(voltageOK ? "✓ OK" : "✗ FAIL");
  
  Serial.print("📡 Current Sensor: ");
  Serial.print(measuredCurrent); Serial.print(" A ");
  Serial.println(sensorOK ? "✓ Calibrated" : "✗ Not Ready");

  Serial.print("   Charger input: ");
  Serial.print(IN_VOLTAGE); Serial.print(" V ");
  Serial.println(chargerOK ? "✓ Plugged" : "✗ Not plugged");
  
  if (tempOK && voltageOK && sensorOK && chargerOK) {
    if (currentState == IDLE || currentState == WAITING_FOR_CONDITIONS) {
      Serial.println("\n✓✓✓ ALL CONDITIONS MET - READY TO CHARGE! ✓✓✓");
      currentState = CHARGING_CC;  // Start with Constant Current mode
    }
  }
  else {
    if (currentState == CHARGING_CC || currentState == CHARGING_CV) {
      Serial.println("\n⚠⚠⚠ CONDITIONS NOT MET - STOPPING CHARGING ⚠⚠⚠");
      currentState = WAITING_FOR_CONDITIONS;
      stopCharging();
    } else if (currentState == IDLE) {
      currentState = WAITING_FOR_CONDITIONS;
    }
  }
  Serial.println("════════════════════════════════════════════════════════");
}

// ==================== CHARGE CONTROL STATE MACHINE ====================
void executeChargeControl() {
  switch (currentState) {
    case IDLE:
      break;
      
    case WAITING_FOR_CONDITIONS:
      stopCharging();
      break;
      
    case CHARGING_CC:
      performCCCharging();
      break;
      
    case CHARGING_CV:
      performCVCharging();
      break;
      
    case BALANCING:
      if (balanceStatus[0] && balanceStatus[1]) {
        Serial.println("✓ Balancing complete");
        currentState = COMPLETE;
      }
      stopCharging();
      break;
      
    case COMPLETE:
      stopCharging();
      break;
      
    case ERROR:
      stopCharging();
      break;
  }
}

// ==================== CONSTANT CURRENT CHARGING ====================
void performCCCharging() {
  // Check if we should transition to CV mode
  if (totalPackVoltage >= CC_TO_CV_VOLTAGE) {
    Serial.println("\n╔════════════════════════════════════════════════════════╗");
    Serial.println("║   SWITCHING TO CONSTANT VOLTAGE (CV) MODE              ║");
    Serial.println("╚════════════════════════════════════════════════════════╝");
    currentState = CHARGING_CV;
    voltagePID.integral = 0;
    voltagePID.prev_error = 0;
    S_time = millis();
    E_time = millis();
    return;
  }
  
  // Check temperature safety
  if (avgTemp > MAX_TEMP || avgTemp < MIN_TEMP) {
    Serial.println("\n⚠ Temperature out of range!");
    currentState = WAITING_FOR_CONDITIONS;
    return;
  }
  
  digitalWrite(CHARGE_ENABLE_PIN, HIGH);
  preCharging();

  readCurrent();
  E_time = millis();
  float dt = (E_time - S_time) / 1000.0;
  if (dt < 0.01) dt = 0.1; // Prevent division by zero
  float pidOutput = computePID(&currentPID, measuredCurrent, dt);
  
  ledcWrite(PWM_PIN, 511 - (int)pidOutput);
  S_time = millis();
  
  static unsigned long lastDisplay = 0;
  if (millis() - lastDisplay > 1000) {
    lastDisplay = millis();
    
    Serial.println("\n┌─────────────────────────────────────────────────┐");
    Serial.println("│      CONSTANT CURRENT (CC) MODE                 │");
    Serial.println("└─────────────────────────────────────────────────┘");
    Serial.print("🔋 Pack Voltage: "); Serial.print(totalPackVoltage, 3); Serial.println(" V");
    Serial.print("⚡ Target Current: "); Serial.print(CC_TARGET_CURRENT, 2); Serial.println(" A");
    Serial.print("📊 Measured Current: "); Serial.print(measuredCurrent, 3); Serial.println(" A");
    Serial.print("🎛️  PID Output: "); Serial.print(pidOutput, 1); Serial.print("/511");
    Serial.print(" ("); Serial.print((pidOutput/511.0)*100, 1); Serial.println("%)");
    Serial.print("🌡️  Temperature: "); Serial.print(avgTemp, 2); Serial.println(" °C");
  }
}

// ==================== CONSTANT VOLTAGE CHARGING ====================
void performCVCharging() {
  if (maxCellVoltage >= MAX_CELL_VOLTAGE || measuredCurrent <= CHARGE_COMPLETE_CURRENT) {
    Serial.println("\n╔════════════════════════════════════════════════════════╗");
    Serial.println("║           ✓✓✓ CHARGING COMPLETE ✓✓✓                   ║");
    Serial.println("╚════════════════════════════════════════════════════════╝");
    
    if (!balanceStatus[0] || !balanceStatus[1]) {
      currentState = BALANCING;
      Serial.println("→ Entering balancing state");
    } else {
      currentState = COMPLETE;
      Serial.println("→ Battery fully charged and balanced!");
    }
    return;
  }
  
  if (avgTemp > MAX_TEMP || avgTemp < MIN_TEMP) {
    Serial.println("\n⚠ Temperature out of range!");
    currentState = WAITING_FOR_CONDITIONS;
    return;
  }

  digitalWrite(CHARGE_ENABLE_PIN, HIGH);
  preCharging();

  float FB_VOLTAGE = readVoltage_FB();
  E_time = millis();
  float dt = (E_time - S_time) / 1000.0;
  if (dt < 0.01) dt = 0.1; // Prevent division by zero
  float pidOutput = computePID(&voltagePID, FB_VOLTAGE, dt);

  ledcWrite(PWM_PIN, 511 - (int)pidOutput);
  S_time = millis();

  static unsigned long lastDisplay = 0;
  if (millis() - lastDisplay > 1000) {
    lastDisplay = millis();
    
    Serial.println("\n┌─────────────────────────────────────────────────┐");
    Serial.println("│      CONSTANT VOLTAGE (CV) MODE                 │");
    Serial.println("└─────────────────────────────────────────────────┘");
    Serial.print("🔋 Pack Voltage: "); Serial.print(totalPackVoltage, 3); Serial.println(" V");
    Serial.print("🎯 Target Voltage: "); Serial.print(CV_TARGET_VOLTAGE, 2); Serial.println(" V");
    Serial.print("⚡ Measured Current: "); Serial.print(measuredCurrent, 3); Serial.println(" A");
    Serial.print("🎛️  PID Output: "); Serial.print(pidOutput, 1); Serial.print("/511");
    Serial.print(" ("); Serial.print((pidOutput/511.0)*100, 1); Serial.println("%)");
    Serial.print("🌡️  Temperature: "); Serial.print(avgTemp, 2); Serial.println(" °C");
  }
}

// ==================== STOP CHARGING ====================
void stopCharging() {
  preCharging();
  digitalWrite(CHARGE_ENABLE_PIN, LOW);
  ledcWrite(PWM_PIN, 511 - 0);
}

// ==================== PRE CHARGING ====================
void preCharging() {
  if (currentState == CHARGING_CC || currentState == CHARGING_CV) {
    if (!precharge_state) {
      digitalWrite(PRECHARGE_RELAY, LOW);
      Serial.println("PRE CHARGING");
      delay(5000);
      digitalWrite(PRECHARGE_RELAY, HIGH);
      Serial.println("DONE PRE CHARGING");
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

// ==================== VOLTAGE READING ====================
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

// ==================== WEBSOCKET UPDATE ====================
void sendWebSocketUpdate() {
  StaticJsonDocument<1024> doc;
  
  // Pack data
  doc["packVoltage"] = totalPackVoltage;
  doc["current"] = measuredCurrent;
  
  // Calculate SOC (simplified - you can replace with your algorithm)
  float soc = ((totalPackVoltage - (MIN_CELL_VOLTAGE * 8)) / ((MAX_CELL_VOLTAGE * 8) - (MIN_CELL_VOLTAGE * 8))) * 100.0;
  if (soc > 100) soc = 100;
  if (soc < 0) soc = 0;
  doc["soc"] = soc;
  
  doc["soh"] = 93.5; // Calculate from your SOH algorithm
  doc["runtime"] = 3.4; // Calculate from remaining capacity / current
  doc["capacity"] = 2.1; // Cumulative Ah
  
  // Battery state
  const char* stateStr = "IDLE";
  switch(currentState) {
    case CHARGING_CC: stateStr = "CHARGING"; break;
    case CHARGING_CV: stateStr = "CHARGING"; break;
    case COMPLETE: stateStr = "COMPLETE"; break;
    case BALANCING: stateStr = "BALANCING"; break;
    case ERROR: stateStr = "ERROR"; break;
    case WAITING_FOR_CONDITIONS: stateStr = "WAITING"; break;
    default: stateStr = "IDLE"; break;
  }
  doc["state"] = stateStr;
  
  // Temperatures
  JsonArray tempArray = doc.createNestedArray("temperatures");
  tempArray.add(temps[0]);
  tempArray.add(temps[1]);
  
  // Cell voltages
  JsonArray cellArray = doc.createNestedArray("cells");
  for (int i = 0; i < 8; i++) {
    cellArray.add(cells[i]);
  }
  
  // Faults
  doc["overvoltage"] = (maxCellVoltage > MAX_CELL_VOLTAGE);
  doc["undervoltage"] = (minCellVoltage < MIN_CELL_VOLTAGE);
  doc["overcurrent"] = (measuredCurrent > MAX_CURRENT);
  
  // Control status
  doc["precharge"] = !precharge_state;
  doc["protection"] = !precharge_state;
  doc["balancing"] = (balanceStatus[0] && balanceStatus[1]);
  
  String output;
  serializeJson(doc, output);
  ws.textAll(output);
}

// ==================== WEBSOCKET EVENT HANDLER ====================
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
                     void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected\n", client->id());
    sendWebSocketUpdate(); // Send initial data
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  }
}
