/*
 * OTAManager.h - Over-The-Air firmware update management for ESP32-S3
 * 
 * Features:
 * - ArduinoOTA support for PlatformIO uploads
 * - HTTP-based OTA updates via web interface
 * - Progress monitoring and error handling
 * - Password protection
 * - Rollback safety checks
 */

#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>

// OTA Configuration
#define OTA_HOSTNAME "ESP32_BMS"           // Hostname for OTA identification
#define OTA_PASSWORD "bms2024"             // Password for OTA security (CHANGE THIS!)
#define OTA_PORT 3232                      // Default ArduinoOTA port

class OTAManager {
public:
    OTAManager();
    
    // Initialize OTA functionality
    bool begin(const char* hostname = OTA_HOSTNAME, const char* password = OTA_PASSWORD);
    
    // Handle OTA in main loop
    void handle();
    
    // Setup HTTP OTA endpoints for web-based updates
    void setupWebOTA(AsyncWebServer* server);
    
    // Check if OTA update is in progress
    bool isUpdating() { return _updateInProgress; }
    
    // Get update progress (0-100)
    uint8_t getProgress() { return _updateProgress; }
    
    // Get last error message
    String getLastError() { return _lastError; }
    
    // Enable/disable OTA
    void enable() { _enabled = true; }
    void disable() { _enabled = false; }
    
    // Restart ESP32
    void restart();

private:
    bool _enabled;
    bool _updateInProgress;
    uint8_t _updateProgress;
    String _lastError;
    
    // Callbacks for ArduinoOTA events
    static void onStart();
    static void onEnd();
    static void onProgress(unsigned int progress, unsigned int total);
    static void onError(ota_error_t error);
    
    // HTTP OTA update handler
    static void handleOTAUpdate(AsyncWebServerRequest *request, 
                               String filename, 
                               size_t index, 
                               uint8_t *data, 
                               size_t len, 
                               bool final);
};

// Global instance (optional - can be created locally)
extern OTAManager otaManager;

#endif // OTA_MANAGER_H