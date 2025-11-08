#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

// ========================================================================================
// WiFi Configuration - UPDATE THESE WITH YOUR NETWORK DETAILS
// ========================================================================================
const char* ssid = "YOUR_WIFI_SSID";        // Replace with your WiFi network name
const char* password = "YOUR_WIFI_PASSWORD"; // Replace with your WiFi password

// ========================================================================================
// BMS Configuration - UPDATE THESE BASED ON YOUR SYSTEM
// ========================================================================================
#define NUM_CELLS 8              // Number of battery cells in series
#define NUM_SLAVES 4             // Number of slave controllers with temperature sensors
#define BATTERY_CAPACITY 100.0   // Battery capacity in Ah
#define MAX_CELL_VOLTAGE 4.2     // Maximum safe cell voltage
#define MIN_CELL_VOLTAGE 3.0     // Minimum safe cell voltage
#define MAX_CURRENT 50.0         // Maximum safe current in Amps

// GPIO Pins for System Status - UPDATE THESE TO MATCH YOUR HARDWARE
#define PRECHARGE_STATUS_PIN 12  // Pin to read precharge circuit status
#define PMOS_STATUS_PIN 13       // Pin to read protection PMOS status
#define BALANCING_STATUS_PIN 14  // Pin to read active balancing status

// ========================================================================================
// Web Server Setup
// ========================================================================================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ========================================================================================
// BMS Data Structure
// ========================================================================================
struct BMSData {
    // Basic parameters
    float packVoltage;           // Total pack voltage in V
    float current;               // Pack current in A (positive = charging, negative = discharging)
    float soc;                   // State of Charge in %
    float soh;                   // State of Health in %

    // Individual cell voltages (8 cells)
    float cellVoltages[NUM_CELLS];

    // Temperature sensors (one per slave)
    float slaveTemperatures[NUM_SLAVES];

    // Battery state
    String batteryState;         // "CHARGING", "DISCHARGING", "IDLE"

    // Protection and control indicators
    bool prechargeActive;        // True if precharge circuit is active
    bool protectionPMOSActive;   // True if protection PMOS is active
    bool activeBalancingActive;  // True if active balancing is running

    // Fault indicators
    bool overvoltage;           // True if any cell has overvoltage
    bool undervoltage;          // True if any cell has undervoltage
    bool overcurrent;           // True if current exceeds safe limits

    // Advanced monitoring
    float remainingRuntime;     // Estimated remaining runtime in hours
    float cumulativeCapacity;   // Cumulative capacity processed in Ah

    unsigned long timestamp;    // Timestamp for data logging
};

BMSData currentBMS;

// Global variables for cumulative capacity calculation
static float totalCumulativeCapacity = 0.0;
static unsigned long lastCapacityUpdateTime = 0;

// ========================================================================================
// BMS DATA ACQUISITION FUNCTIONS
// REPLACE THESE FUNCTIONS WITH YOUR ACTUAL BMS CODE
// ========================================================================================

/**
 * Read pack voltage from your BMS
 * Replace this function with your actual pack voltage reading code
 */
float readPackVoltage() {
    // EXAMPLE: If you're using voltage dividers or ADC
    // float adcReading = analogRead(PACK_VOLTAGE_PIN);
    // float voltage = (adcReading / 4095.0) * 3.3 * VOLTAGE_DIVIDER_RATIO;
    // return voltage;

    // PLACEHOLDER - Replace with your actual implementation
    return 48.5 + (random(-100, 100) / 100.0); // Simulated: 47.5V to 49.5V
}

/**
 * Read current from your current sensor
 * Replace this function with your actual current sensor reading code
 */
float readCurrent() {
    // EXAMPLE: If you're using ACS712 or similar current sensor
    // float adcReading = analogRead(CURRENT_SENSOR_PIN);
    // float voltage = (adcReading / 4095.0) * 3.3;
    // float current = (voltage - 2.5) / CURRENT_SENSOR_SENSITIVITY;
    // return current;

    // PLACEHOLDER - Replace with your actual implementation
    return 25.0 + (random(-500, 500) / 100.0); // Simulated: 20A to 30A
}

/**
 * Calculate State of Charge
 * Replace this function with your actual SOC calculation
 */
float calculateSOC() {
    // EXAMPLE: Voltage-based SOC estimation or coulomb counting
    // float voltage = readPackVoltage();
    // return voltageToSOC(voltage);

    // PLACEHOLDER - Replace with your actual implementation
    return 85.0 + (random(-50, 50) / 10.0); // Simulated: 80% to 90%
}

/**
 * Calculate State of Health
 * Replace this function with your actual SOH calculation
 */
float calculateSOH() {
    // EXAMPLE: Based on internal resistance or capacity fade
    // return (currentCapacity / originalCapacity) * 100.0;

    // PLACEHOLDER - Replace with your actual implementation
    return 95.0 + (random(-20, 20) / 10.0); // Simulated: 93% to 97%
}

/**
 * Read individual cell voltage
 * Replace this function with your actual cell monitoring IC communication
 */
float readCellVoltage(int cellIndex) {
    // EXAMPLE: I2C communication with cell monitoring IC (LTC6811, BQ76920, etc.)
    // if (cellIndex < 0 || cellIndex >= NUM_CELLS) return 0.0;
    // return cellMonitoringIC.readCellVoltage(cellIndex);

    // PLACEHOLDER - Replace with your actual implementation
    return 3.6 + (random(-50, 50) / 1000.0); // Simulated: 3.55V to 3.65V per cell
}

/**
 * Read temperature from slave controller
 * Replace this function with your actual slave communication
 */
float readSlaveTemperature(int slaveIndex) {
    // EXAMPLE: I2C or CAN communication with slave controllers
    // if (slaveIndex < 0 || slaveIndex >= NUM_SLAVES) return 0.0;
    // return i2c_readTemperatureFromSlave(slaveIndex);

    // PLACEHOLDER - Replace with your actual implementation
    return 25.0 + (random(-50, 50) / 10.0); // Simulated: 20°C to 30°C
}

/**
 * Check if precharge circuit is active
 * Replace this function with your actual precharge status reading
 */
bool isPrechargeActive() {
    // EXAMPLE: Read digital pin or register
    // return digitalRead(PRECHARGE_STATUS_PIN);

    // PLACEHOLDER - Replace with your actual implementation
    return random(0, 10) > 7; // Simulated: 30% chance active
}

/**
 * Check if protection PMOS is active
 * Replace this function with your actual PMOS status reading
 */
bool isProtectionPMOSActive() {
    // EXAMPLE: Read digital pin or register
    // return digitalRead(PMOS_STATUS_PIN);

    // PLACEHOLDER - Replace with your actual implementation
    return random(0, 10) > 2; // Simulated: 80% chance active
}

/**
 * Check if active balancing is running
 * Replace this function with your actual balancing status
 */
bool isActiveBalancingActive() {
    // EXAMPLE: Check balancing controller status
    // return balancingController.isActive();

    // PLACEHOLDER - Replace with your actual implementation
    return random(0, 10) > 5; // Simulated: 50% chance active
}

/**
 * Check for overvoltage condition
 * Replace this function with your actual overvoltage detection
 */
bool checkOvervoltage() {
    // EXAMPLE: Check all cell voltages against maximum
    // for (int i = 0; i < NUM_CELLS; i++) {
    //     if (readCellVoltage(i) > MAX_CELL_VOLTAGE) {
    //         return true;
    //     }
    // }
    // return false;

    // PLACEHOLDER - Replace with your actual implementation
    return random(0, 100) > 95; // Simulated: 5% chance of fault
}

/**
 * Check for undervoltage condition
 * Replace this function with your actual undervoltage detection
 */
bool checkUndervoltage() {
    // EXAMPLE: Check all cell voltages against minimum
    // for (int i = 0; i < NUM_CELLS; i++) {
    //     if (readCellVoltage(i) < MIN_CELL_VOLTAGE) {
    //         return true;
    //     }
    // }
    // return false;

    // PLACEHOLDER - Replace with your actual implementation
    return random(0, 100) > 98; // Simulated: 2% chance of fault
}

/**
 * Check for overcurrent condition
 * Replace this function with your actual overcurrent detection
 */
bool checkOvercurrent() {
    // EXAMPLE: Check current against maximum limit
    // return abs(readCurrent()) > MAX_CURRENT;

    // PLACEHOLDER - Replace with your actual implementation
    return random(0, 100) > 97; // Simulated: 3% chance of fault
}

// ========================================================================================
// MAIN BMS DATA COLLECTION FUNCTION
// ========================================================================================
BMSData getBMSData() {
    BMSData data;

    // Read basic parameters using your BMS functions
    data.packVoltage = readPackVoltage();
    data.current = readCurrent();
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

    // Read system status
    data.prechargeActive = isPrechargeActive();
    data.protectionPMOSActive = isProtectionPMOSActive();
    data.activeBalancingActive = isActiveBalancingActive();

    // Check for faults
    data.overvoltage = checkOvervoltage();
    data.undervoltage = checkUndervoltage();
    data.overcurrent = checkOvercurrent();

    // Calculate remaining runtime
    if (abs(data.current) > 0.1) {
        data.remainingRuntime = (data.soc / 100.0) * BATTERY_CAPACITY / abs(data.current);
        // Cap the runtime at 999.9 hours for display purposes
        if (data.remainingRuntime > 999.9) {
            data.remainingRuntime = 999.9;
        }
    } else {
        data.remainingRuntime = 999.9; // Very high when current is near zero
    }

    // Update cumulative capacity (integrate current over time)
    unsigned long now = millis();
    if (lastCapacityUpdateTime != 0) {
        float deltaTime = (now - lastCapacityUpdateTime) / 3600000.0; // Convert to hours
        totalCumulativeCapacity += abs(data.current) * deltaTime;
    }
    data.cumulativeCapacity = totalCumulativeCapacity;
    lastCapacityUpdateTime = now;

    data.timestamp = now;

    return data;
}

// ========================================================================================
// WEB SERVER FUNCTIONS
// ========================================================================================

/**
 * WebSocket event handler
 */
void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, 
               AwsEventType type, void * arg, uint8_t *data, size_t len) {
    if(type == WS_EVT_CONNECT) {
        Serial.println("WebSocket client connected");
        Serial.printf("Client ID: %u\n", client->id());
        sendBMSData();
    } else if(type == WS_EVT_DISCONNECT) {
        Serial.printf("WebSocket client %u disconnected\n", client->id());
    } else if(type == WS_EVT_ERROR) {
        Serial.printf("WebSocket client %u error: %s\n", client->id(), (char*)data);
    }
}

/**
 * Send BMS data via WebSocket to all connected clients
 */
void sendBMSData() {
    currentBMS = getBMSData();

    // Create JSON object with all BMS data
    DynamicJsonDocument json(1536); // Increased size for all data

    // Basic parameters
    json["packVoltage"] = round(currentBMS.packVoltage * 100) / 100.0; // Round to 2 decimal places
    json["current"] = round(currentBMS.current * 100) / 100.0;
    json["soc"] = round(currentBMS.soc * 10) / 10.0; // Round to 1 decimal place
    json["soh"] = round(currentBMS.soh * 10) / 10.0;
    json["batteryState"] = currentBMS.batteryState;

    // Cell voltages array
    JsonArray cellVoltages = json.createNestedArray("cellVoltages");
    for (int i = 0; i < NUM_CELLS; i++) {
        cellVoltages.add(round(currentBMS.cellVoltages[i] * 1000) / 1000.0); // Round to 3 decimal places
    }

    // Slave temperatures array
    JsonArray slaveTemps = json.createNestedArray("slaveTemperatures");
    for (int i = 0; i < NUM_SLAVES; i++) {
        slaveTemps.add(round(currentBMS.slaveTemperatures[i] * 10) / 10.0);
    }

    // System status
    json["prechargeActive"] = currentBMS.prechargeActive;
    json["protectionPMOSActive"] = currentBMS.protectionPMOSActive;
    json["activeBalancingActive"] = currentBMS.activeBalancingActive;

    // Fault status
    json["overvoltage"] = currentBMS.overvoltage;
    json["undervoltage"] = currentBMS.undervoltage;
    json["overcurrent"] = currentBMS.overcurrent;

    // Advanced monitoring
    json["remainingRuntime"] = round(currentBMS.remainingRuntime * 10) / 10.0;
    json["cumulativeCapacity"] = round(currentBMS.cumulativeCapacity * 10) / 10.0;

    json["timestamp"] = currentBMS.timestamp;

    // Convert to string and send
    String jsonString;
    serializeJson(json, jsonString);

    // Send to all connected WebSocket clients
    ws.textAll(jsonString);

    // Print to serial for debugging
    Serial.printf("Sent BMS data: Pack=%.2fV, Current=%.2fA, SOC=%.1f%%, State=%s\n", 
                  currentBMS.packVoltage, currentBMS.current, currentBMS.soc, 
                  currentBMS.batteryState.c_str());
}

// ========================================================================================
// MAIN SETUP FUNCTION
// ========================================================================================
void setup() {
    Serial.begin(115200);
    Serial.println();
    Serial.println("=================================");
    Serial.println("Enhanced BMS WiFi Dashboard");
    Serial.println("=================================");

    // Initialize GPIO pins for status monitoring
    pinMode(PRECHARGE_STATUS_PIN, INPUT_PULLUP);
    pinMode(PMOS_STATUS_PIN, INPUT_PULLUP);
    pinMode(BALANCING_STATUS_PIN, INPUT_PULLUP);

    // Initialize LittleFS filesystem
    if(!LittleFS.begin(true)) {
        Serial.println("ERROR: LittleFS Mount Failed!");
        Serial.println("Make sure to upload the data folder using 'pio run --target uploadfs'");
        return;
    }
    Serial.println("✓ LittleFS filesystem initialized");

    // Connect to WiFi
    Serial.printf("Connecting to WiFi network: %s\n", ssid);
    WiFi.begin(ssid, password);

    int wifiRetries = 0;
    while (WiFi.status() != WL_CONNECTED && wifiRetries < 20) {
        delay(1000);
        Serial.print(".");
        wifiRetries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.println("✓ WiFi connected successfully!");
        Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf("Open http://%s in your tablet browser\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println();
        Serial.println("ERROR: WiFi connection failed!");
        Serial.println("Check your WiFi credentials and try again.");
        return;
    }

    // Initialize WebSocket
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    Serial.println("✓ WebSocket server initialized");

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

    // API endpoint for current BMS data (fallback for WebSocket)
    server.on("/api/bms", HTTP_GET, [](AsyncWebServerRequest *request) {
        BMSData data = getBMSData();

        DynamicJsonDocument json(1536);
        json["packVoltage"] = round(data.packVoltage * 100) / 100.0;
        json["current"] = round(data.current * 100) / 100.0;
        json["soc"] = round(data.soc * 10) / 10.0;
        json["soh"] = round(data.soh * 10) / 10.0;
        json["batteryState"] = data.batteryState;

        JsonArray cellVoltages = json.createNestedArray("cellVoltages");
        for (int i = 0; i < NUM_CELLS; i++) {
            cellVoltages.add(round(data.cellVoltages[i] * 1000) / 1000.0);
        }

        JsonArray slaveTemps = json.createNestedArray("slaveTemperatures");
        for (int i = 0; i < NUM_SLAVES; i++) {
            slaveTemps.add(round(data.slaveTemperatures[i] * 10) / 10.0);
        }

        json["prechargeActive"] = data.prechargeActive;
        json["protectionPMOSActive"] = data.protectionPMOSActive;
        json["activeBalancingActive"] = data.activeBalancingActive;

        json["overvoltage"] = data.overvoltage;
        json["undervoltage"] = data.undervoltage;
        json["overcurrent"] = data.overcurrent;

        json["remainingRuntime"] = round(data.remainingRuntime * 10) / 10.0;
        json["cumulativeCapacity"] = round(data.cumulativeCapacity * 10) / 10.0;

        json["timestamp"] = data.timestamp;

        String response;
        serializeJson(json, response);

        request->send(200, "application/json", response);
    });

    // Handle 404 errors
    server.onNotFound([](AsyncWebServerRequest *request) {
        Serial.printf("404 Not Found: %s\n", request->url().c_str());
        request->send(404, "text/plain", "File not found");
    });

    // Start the web server
    server.begin();
    Serial.println("✓ HTTP server started on port 80");

    // Initialize random seed for simulation data
    randomSeed(analogRead(0));

    // Initialize timing variables
    lastCapacityUpdateTime = millis();

    Serial.println("=================================");
    Serial.println("BMS Dashboard Ready!");
    Serial.printf("Dashboard URL: http://%s\n", WiFi.localIP().toString().c_str());
    Serial.println("=================================");
}

// ========================================================================================
// MAIN LOOP FUNCTION
// ========================================================================================
void loop() {
    // Send BMS data every 2 seconds
    static unsigned long lastDataUpdate = 0;
    unsigned long now = millis();

    if (now - lastDataUpdate >= 2000) {  // Update every 2 seconds
        if (WiFi.status() == WL_CONNECTED && ws.count() > 0) {
            sendBMSData();
        }
        lastDataUpdate = now;
    }

    // Cleanup WebSocket connections
    ws.cleanupClients();

    // Handle WiFi reconnection if disconnected
    static unsigned long lastWiFiCheck = 0;
    if (now - lastWiFiCheck >= 10000) { // Check every 10 seconds
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi disconnected, attempting to reconnect...");
            WiFi.begin(ssid, password);
        }
        lastWiFiCheck = now;
    }

    // Small delay to prevent watchdog timer issues
    delay(10);
}