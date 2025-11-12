/*
 * SDCardManager.h
 * 
 * SD Card management for storing and retrieving SOC/SOH lookup tables
 */

#ifndef SD_CARD_MANAGER_H
#define SD_CARD_MANAGER_H

#include <Arduino.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// SOC Lookup Table Structure (OCV-SOC-Temperature relationship)
struct SOCLookupEntry {
    float soc_percent;           // State of Charge (0-100%)
    float ocv_minus10C;         // Open Circuit Voltage at -10°C
    float ocv_0C;               // Open Circuit Voltage at 0°C
    float ocv_10C;              // Open Circuit Voltage at 10°C
    float ocv_20C;              // Open Circuit Voltage at 20°C
    float ocv_25C;              // Open Circuit Voltage at 25°C
    float ocv_30C;              // Open Circuit Voltage at 30°C
    float ocv_40C;              // Open Circuit Voltage at 40°C
};

// SOH Lookup Table Structure (Degradation model)
struct SOHLookupEntry {
    int cycle_count;            // Number of charge/discharge cycles
    float capacity_retention;   // Capacity retention percentage (0-100%)
    float soh_percent;          // State of Health (0-100%)
    float resistance_increase;  // Internal resistance increase percentage
};

class SDCardManager {
public:
    SDCardManager();
    
    // Initialization
    bool begin();
    void end();
    
    // SOC Lookup Table Operations
    bool saveSOCLookupTable(const SOCLookupEntry* entries, int numEntries);
    bool loadSOCLookupTable(SOCLookupEntry* entries, int maxEntries, int& numLoaded);
    float interpolateSOCFromOCV(float ocv, float temperature);
    
    // SOH Lookup Table Operations
    bool saveSOHLookupTable(const SOHLookupEntry* entries, int numEntries);
    bool loadSOHLookupTable(SOHLookupEntry* entries, int maxEntries, int& numLoaded);
    float interpolateSOHFromCycles(int cycleCount);
    
    // Data Logging
    bool logSOCData(float soc, float ocv, float temperature, unsigned long timestamp);
    bool logSOHData(float soh, int cycles, float capacity, unsigned long timestamp);
    
    // Utility Functions
    bool fileExists(const char* path);
    void listFiles();
    bool isInitialized() { return _initialized; }
    
private:
    bool _initialized;
    SPIClass _spi;
    
    // Helper functions
    String readLine(File& file);
    bool parseSOCLine(const String& line, SOCLookupEntry& entry);
    bool parseSOHLine(const String& line, SOHLookupEntry& entry);
    float linearInterpolate(float x, float x0, float x1, float y0, float y1);
};

#endif // SD_CARD_MANAGER_H
