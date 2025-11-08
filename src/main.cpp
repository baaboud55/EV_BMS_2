/*
 * Enhanced BMS Monitor - ESP32 WiFi Dashboard
 * 
 * This code provides a web-based dashboard for monitoring BMS parameters
 * including cell voltages, temperatures, faults, and system status.
 * 
 * Features:
 * - Real-time data streaming via WebSocket
 * - 8 individual cell voltage monitoring
 * - 4 slave temperature sensors
 * - Battery state (Charging/Discharging/Idle)
 * - System status indicators (Precharge, Protection PMOS, Active Balancing)
 * - Fault detection (Overvoltage, Undervoltage, Overcurrent)
 * - Runtime estimation and cumulative capacity tracking
 */

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

// ========== WiFi Configuration ==========
const char* ssid = "YOUR_WIFI_SSID";        // Replace with your WiFi SSID
const char* password = "YOUR_WIFI_PASSWORD"; // Replace with your WiFi password

// ========== BMS Configuration ==========
#define NUM_CELLS 8
#define NUM_SLAVES 4
#define BATTERY_CAPACITY 100.0  // Battery capacity in Ah
#define MAX_CELL_VOLTAGE 4.2    // Maximum safe cell voltage
#define MIN_CELL_VOLTAGE 3.0    // Minimum safe cell voltage
#define MAX_CURRENT 50.0        // Maximum safe current

// ========== Server Objects ==========
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ========== BMS Data Structure ==========
struct BMSData {
    float packVoltage;
    float current;
    float soc;  // State of Charge (%)
    float soh;  // State of Health (%)

    // Cell voltages (8 cells)
    float cellVoltages[NUM_CELLS];

    // Temperature sensors (one per slave)
    float slaveTemperatures[NUM_SLAVES];

    // Battery state
    String batteryState;  // "CHARGING", "DISCHARGING", "IDLE"

    // Protection and control indicators
    bool prechargeActive;
    bool protectionPMOSActive;
    bool activeBalancingActive;

    // Fault indicators
    bool overvoltage;
    bool undervoltage;
    bool overcurrent;

    // Advanced monitoring
    float remainingRuntime;  // hours
    float cumulativeCapacity; // Ah

    unsigned long timestamp;
};

BMSData currentBMS;

// Static variables for cumulative capacity calculation
static float cumulativeAh = 0;
static unsigned long lastCapacityUpdate = 0;

// ========== Function Declarations ==========
BMSData getBMSData();
void sendBMSData();
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
               AwsEventType type, void *arg, uint8_t *data, size_t len);

// ========== BMS Data Acquisition Functions ==========
// Replace these simulation functions with your actual BMS functions

float readPackVoltage() {
    // TODO: Replace with your actual pack voltage reading
    // Example: return ads.readADC_SingleEnded(PACK_VOLTAGE_CHANNEL) * VOLTAGE_SCALE;
    return 48.5 + (random(-100, 100) / 100.0);
}

float readCurrentSensor() {
    // TODO: Replace with your actual current sensor reading
    // Example: return (ads.readADC_SingleEnded(CURRENT_CHANNEL) - CURRENT_OFFSET) * CURRENT_SCALE;
    return 25.0 + (random(-500, 500) / 100.0);
}

float calculateSOC() {
    // TODO: Replace with your actual SOC calculation
    // This could be based on voltage, coulomb counting, or a combination
    return 85.0 + (random(-50, 50) / 10.0);
}

float calculateSOH() {
    // TODO: Replace with your actual SOH calculation
    // This is typically based on capacity fade over cycles
    return 95.0 + (random(-20, 20) / 10.0);
}

float readCellVoltage(int cellIndex) {
    // TODO: Replace with your actual cell voltage reading
    // Example: return cellMonitorIC.getCellVoltage(cellIndex);
    return 3.6 + (random(-50, 50) / 1000.0);
}

float readSlaveTemperature(int slaveIndex) {
    // TODO: Replace with your actual slave temperature reading
    // Example: return slaves[slaveIndex].getTemperature();
    return 25.0 + (random(-50, 50) / 10.0);
}

bool isPrechargeActive() {
    // TODO: Replace with your actual precharge status check
    // Example: return digitalRead(PRECHARGE_STATUS_PIN);
    return random(0, 10) > 7;
}

bool isProtectionPMOSActive() {
    // TODO: Replace with your actual protection PMOS status check
    // Example: return digitalRead(PMOS_STATUS_PIN);
    return random(0, 10) > 2;
}

bool isActiveBalancingActive() {
    // TODO: Replace with your actual active balancing status check
    // Example: return balancingController.isActive();
    return random(0, 10) > 5;
}

bool checkOvervoltage() {
    // TODO: Replace with your actual overvoltage detection
    // Check if any cell exceeds maximum voltage
    for (int i = 0; i < NUM_CELLS; i++) {
        if (currentBMS.cellVoltages[i] > MAX_CELL_VOLTAGE) {
            return true;
        }
    }
    return random(0, 100) > 95;  // Simulation
}

bool checkUndervoltage() {
    // TODO: Replace with your actual undervoltage detection
    // Check if any cell is below minimum voltage
    for (int i = 0; i < NUM_CELLS; i++) {
        if (currentBMS.cellVoltages[i] < MIN_CELL_VOLTAGE) {
            return true;
        }
    }
    return random(0, 100) > 98;  // Simulation
}

bool checkOvercurrent() {
    // TODO: Replace with your actual overcurrent detection
    // Check if current exceeds safe limits
    if (abs(currentBMS.current) > MAX_CURRENT) {
        return true;
    }
    return random(0, 100) > 97;  // Simulation
}

// ========== BMS Data Collection ==========
BMSData getBMSData() {
    BMSData data;

    // Read basic parameters
    data.packVoltage = readPackVoltage();
    data.current = readCurrentSensor();
    data.soc = calculateSOC();
    data.soh = calculateSOH();

    // Read individual cell voltages
    for (int i = 0; i < NUM_CELLS; i++) {
        data.cellVoltages[i] = readCellVoltage(i);
    }

    // Read slave temperatures
    for (int i = 0; i < NUM_SLAVES; i++) {
        data.slaveTemperatures[i] = readSlaveTemperature(i);
    }

    // Determine battery state based on current
    if (data.current > 1.0) {
        data.batteryState = "CHARGING";
    } else if (data.current < -1.0) {
        data.batteryState = "DISCHARGING";
    } else {
        data.batteryState = "IDLE";
    }

    // Read protection and control status
    data.prechargeActive = isPrechargeActive();
    data.protectionPMOSActive = isProtectionPMOSActive();
    data.activeBalancingActive = isActiveBalancingActive();

    // Check fault conditions
    data.overvoltage = checkOvervoltage();
    data.undervoltage = checkUndervoltage();
    data.overcurrent = checkOvercurrent();

    // Calculate remaining runtime
    // Runtime = (SOC/100) * BatteryCapacity / abs(Current)
    if (abs(data.current) > 0.1) {
        data.remainingRuntime = (data.soc / 100.0) * BATTERY_CAPACITY / abs(data.current);
    } else {
        data.remainingRuntime = 999.9; // Very high when current is near zero
    }

    // Calculate cumulative capacity (integrate current over time)
    unsigned long now = millis();
    if (lastCapacityUpdate > 0) {
        float deltaTime = (now - lastCapacityUpdate) / 3600000.0; // Convert to hours
        cumulativeAh += abs(data.current) * deltaTime;
    }
    lastCapacityUpdate = now;
    data.cumulativeCapacity = cumulativeAh;

    data.timestamp = millis();

    return data;
}

// ========== WebSocket Functions ==========
void sendBMSData() {
    currentBMS = getBMSData();

    // Create JSON object with appropriate capacity
    DynamicJsonDocument json(1024);

    json["packVoltage"] = currentBMS.packVoltage;
    json["current"] = currentBMS.current;
    json["soc"] = currentBMS.soc;
    json["soh"] = currentBMS.soh;
    json["batteryState"] = currentBMS.batteryState;

    // Cell voltages array
    JsonArray cellVoltages = json.createNestedArray("cellVoltages");
    for (int i = 0; i < NUM_CELLS; i++) {
        cellVoltages.add(currentBMS.cellVoltages[i]);
    }

    // Slave temperatures array
    JsonArray slaveTemps = json.createNestedArray("slaveTemperatures");
    for (int i = 0; i < NUM_SLAVES; i++) {
        slaveTemps.add(currentBMS.slaveTemperatures[i]);
    }

    // Protection and control status
    json["prechargeActive"] = currentBMS.prechargeActive;
    json["protectionPMOSActive"] = currentBMS.protectionPMOSActive;
    json["activeBalancingActive"] = currentBMS.activeBalancingActive;

    // Fault status
    json["overvoltage"] = currentBMS.overvoltage;
    json["undervoltage"] = currentBMS.undervoltage;
    json["overcurrent"] = currentBMS.overcurrent;

    // Advanced monitoring
    json["remainingRuntime"] = currentBMS.remainingRuntime;
    json["cumulativeCapacity"] = currentBMS.cumulativeCapacity;

    json["timestamp"] = currentBMS.timestamp;

    String jsonString;
    serializeJson(json, jsonString);

    // Send to all connected WebSocket clients
    ws.textAll(jsonString);
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if(type == WS_EVT_CONNECT) {
        Serial.printf("WebSocket client #%u connected from %",client->id(), client->remoteIP().toString().c_str());
        // Send initial data to new client
        sendBMSData();
    } else if(type == WS_EVT_DISCONNECT) {
        Serial.printf("WebSocket client #%u disconnected", client->id());
    } else if(type == WS_EVT_ERROR) {
        Serial.printf("WebSocket client #%u error(%u): %s",client->id(), *((uint16_t*)arg), (char*)data);
    }
}

// ========== Setup Function ==========
void setup() {
    Serial.begin(115200);
    Serial.println("=== Enhanced BMS Monitor Starting ===");

    // Initialize random seed for simulation (remove when using real sensors)
    randomSeed(analogRead(0));

    // Initialize LittleFS for web files
    if(!LittleFS.begin(true)) {
        Serial.println("ERROR: LittleFS Mount Failed!");
        Serial.println("Make sure to upload filesystem with 'pio run --target uploadfs'");
        return;
    }
    Serial.println("✓ LittleFS mounted successfully");

    // TODO: Initialize your BMS hardware here
    // Example:
    // Wire.begin();
    // currentSensor.begin();
    // cellMonitor.begin();
    // etc.

    // Connect to WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("✓ WiFi connected!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        Serial.print("Signal strength (RSSI): ");
        Serial.println(WiFi.RSSI());
    } else {
        Serial.println("✗ WiFi connection failed!");
        Serial.println("Please check your SSID and password");
        return;
    }

    // Initialize WebSocket
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    Serial.println("✓ WebSocket handler initialized");

    // Serve static files from LittleFS
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(LittleFS, "/index.html", "text/html");
    });

    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(LittleFS, "/style.css", "text/css");
    });

    server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(LittleFS, "/script.js", "text/javascript");
    });

    // API endpoint for HTTP polling (backup to WebSocket)
    server.on("/api/bms", HTTP_GET, [](AsyncWebServerRequest *request) {
        BMSData data = getBMSData();

        DynamicJsonDocument json(1024);
        json["packVoltage"] = data.packVoltage;
        json["current"] = data.current;
        json["soc"] = data.soc;
        json["soh"] = data.soh;
        json["batteryState"] = data.batteryState;

        JsonArray cellVoltages = json.createNestedArray("cellVoltages");
        for (int i = 0; i < NUM_CELLS; i++) {
            cellVoltages.add(data.cellVoltages[i]);
        }

        JsonArray slaveTemps = json.createNestedArray("slaveTemperatures");
        for (int i = 0; i < NUM_SLAVES; i++) {
            slaveTemps.add(data.slaveTemperatures[i]);
        }

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

    // Handle 404 errors
    server.onNotFound([](AsyncWebServerRequest *request) {
        Serial.printf("404 Not Found: %s", request->url().c_str());
        request->send(404, "text/plain", "Not Found");
    });

    // Start the server
    server.begin();
    Serial.println("✓ HTTP server started");
    Serial.println("=== BMS Monitor Ready ===");
    Serial.println("Open your tablet browser and navigate to:");
    Serial.print("http://");
    Serial.println(WiFi.localIP());
    Serial.println("===========================");
}

// ========== Main Loop ==========
void loop() {
    // Send BMS data every 2 seconds
    static unsigned long lastUpdate = 0;
    unsigned long now = millis();

    if (now - lastUpdate >= 2000) {  // Update every 2 seconds
        sendBMSData();
        lastUpdate = now;
    }

    // Cleanup WebSocket connections
    ws.cleanupClients();

    // Small delay to prevent watchdog timer issues
    delay(10);
}