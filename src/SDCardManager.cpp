/*
 * SDCardManager.cpp
 * 
 * Implementation of SD Card operations for SOC/SOH management
 */

#include "SDCardManager.h"
#include "BMSConfig.h"

SDCardManager::SDCardManager() : _initialized(false), _spi(HSPI) {
}

bool SDCardManager::begin() {
    Serial.println("Initializing SD card...");
    
    // Initialize SPI with custom pins for ESP32-S3
    _spi.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    
    // Initialize SD card with custom SPI and CS pin
    if (!SD.begin(SD_CS_PIN, _spi)) {
        Serial.println("SD Card initialization failed!");
        return false;
    }
    
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("No SD card attached!");
        return false;
    }
    
    // Print card information
    Serial.print("SD Card Type: ");
    if (cardType == CARD_MMC) {
        Serial.println("MMC");
    } else if (cardType == CARD_SD) {
        Serial.println("SDSC");
    } else if (cardType == CARD_SDHC) {
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }
    
    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
    
    _initialized = true;
    Serial.println("SD Card initialized successfully!");
    return true;
}

void SDCardManager::end() {
    SD.end();
    _initialized = false;
}

bool SDCardManager::saveSOCLookupTable(const SOCLookupEntry* entries, int numEntries) {
    if (!_initialized) {
        Serial.println("SD card not initialized!");
        return false;
    }
    
    File file = SD.open(SOC_LOOKUP_FILE, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open SOC lookup file for writing!");
        return false;
    }
    
    // Write CSV header
    file.println("SOC_%,OCV_at_-10C,OCV_at_0C,OCV_at_10C,OCV_at_20C,OCV_at_25C,OCV_at_30C,OCV_at_40C");
    
    // Write data
    for (int i = 0; i < numEntries; i++) {
        file.printf("%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                    entries[i].soc_percent,
                    entries[i].ocv_minus10C,
                    entries[i].ocv_0C,
                    entries[i].ocv_10C,
                    entries[i].ocv_20C,
                    entries[i].ocv_25C,
                    entries[i].ocv_30C,
                    entries[i].ocv_40C);
    }
    
    file.close();
    Serial.printf("Saved %d SOC lookup entries to SD card\n", numEntries);
    return true;
}

bool SDCardManager::loadSOCLookupTable(SOCLookupEntry* entries, int maxEntries, int& numLoaded) {
    if (!_initialized) {
        Serial.println("SD card not initialized!");
        return false;
    }
    
    File file = SD.open(SOC_LOOKUP_FILE, FILE_READ);
    if (!file) {
        Serial.println("Failed to open SOC lookup file for reading!");
        return false;
    }
    
    // Skip header line
    String header = readLine(file);
    
    numLoaded = 0;
    while (file.available() && numLoaded < maxEntries) {
        String line = readLine(file);
        if (line.length() > 0) {
            if (parseSOCLine(line, entries[numLoaded])) {
                numLoaded++;
            }
        }
    }
    
    file.close();
    Serial.printf("Loaded %d SOC lookup entries from SD card\n", numLoaded);
    return true;
}

float SDCardManager::interpolateSOCFromOCV(float ocv, float temperature) {
    // Load SOC lookup table
    SOCLookupEntry entries[20];
    int numLoaded = 0;
    
    if (!loadSOCLookupTable(entries, 20, numLoaded) || numLoaded == 0) {
        // Fallback: simple linear approximation
        return ((ocv - 3.0) / (4.2 - 3.0)) * 100.0;
    }
    
    // Determine which temperature columns to interpolate between
    float ocvAtTemp;
    
    if (temperature <= -10) {
        ocvAtTemp = entries[0].ocv_minus10C;
    } else if (temperature <= 0) {
        // Interpolate between -10°C and 0°C
        float t = (temperature + 10) / 10.0;
        ocvAtTemp = entries[0].ocv_minus10C * (1 - t) + entries[0].ocv_0C * t;
    } else if (temperature <= 10) {
        float t = temperature / 10.0;
        ocvAtTemp = entries[0].ocv_0C * (1 - t) + entries[0].ocv_10C * t;
    } else if (temperature <= 20) {
        float t = (temperature - 10) / 10.0;
        ocvAtTemp = entries[0].ocv_10C * (1 - t) + entries[0].ocv_20C * t;
    } else if (temperature <= 25) {
        float t = (temperature - 20) / 5.0;
        ocvAtTemp = entries[0].ocv_20C * (1 - t) + entries[0].ocv_25C * t;
    } else if (temperature <= 30) {
        float t = (temperature - 25) / 5.0;
        ocvAtTemp = entries[0].ocv_25C * (1 - t) + entries[0].ocv_30C * t;
    } else if (temperature <= 40) {
        float t = (temperature - 30) / 10.0;
        ocvAtTemp = entries[0].ocv_30C * (1 - t) + entries[0].ocv_40C * t;
    } else {
        ocvAtTemp = entries[0].ocv_40C;
    }
    
    // Find SOC by interpolating in the lookup table
    for (int i = 0; i < numLoaded - 1; i++) {
        float ocv1, ocv2;
        
        // Get OCV values at current temperature for adjacent entries
        if (temperature <= -10) {
            ocv1 = entries[i].ocv_minus10C;
            ocv2 = entries[i+1].ocv_minus10C;
        } else if (temperature <= 25) {
            ocv1 = entries[i].ocv_25C;
            ocv2 = entries[i+1].ocv_25C;
        } else {
            ocv1 = entries[i].ocv_40C;
            ocv2 = entries[i+1].ocv_40C;
        }
        
        if (ocv >= ocv1 && ocv <= ocv2) {
            return linearInterpolate(ocv, ocv1, ocv2, 
                                    entries[i].soc_percent, 
                                    entries[i+1].soc_percent);
        }
    }
    
    // Out of range
    return (ocv < 3.0) ? 0.0 : 100.0;
}

bool SDCardManager::saveSOHLookupTable(const SOHLookupEntry* entries, int numEntries) {
    if (!_initialized) {
        Serial.println("SD card not initialized!");
        return false;
    }
    
    File file = SD.open(SOH_LOOKUP_FILE, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open SOH lookup file for writing!");
        return false;
    }
    
    // Write CSV header
    file.println("Cycle_Count,Capacity_Retention_%,SOH_%,Resistance_Increase_%");
    
    // Write data
    for (int i = 0; i < numEntries; i++) {
        file.printf("%d,%.1f,%.1f,%.1f\n",
                    entries[i].cycle_count,
                    entries[i].capacity_retention,
                    entries[i].soh_percent,
                    entries[i].resistance_increase);
    }
    
    file.close();
    Serial.printf("Saved %d SOH lookup entries to SD card\n", numEntries);
    return true;
}

bool SDCardManager::loadSOHLookupTable(SOHLookupEntry* entries, int maxEntries, int& numLoaded) {
    if (!_initialized) {
        Serial.println("SD card not initialized!");
        return false;
    }
    
    File file = SD.open(SOH_LOOKUP_FILE, FILE_READ);
    if (!file) {
        Serial.println("Failed to open SOH lookup file for reading!");
        return false;
    }
    
    // Skip header line
    String header = readLine(file);
    
    numLoaded = 0;
    while (file.available() && numLoaded < maxEntries) {
        String line = readLine(file);
        if (line.length() > 0) {
            if (parseSOHLine(line, entries[numLoaded])) {
                numLoaded++;
            }
        }
    }
    
    file.close();
    Serial.printf("Loaded %d SOH lookup entries from SD card\n", numLoaded);
    return true;
}

float SDCardManager::interpolateSOHFromCycles(int cycleCount) {
    SOHLookupEntry entries[20];
    int numLoaded = 0;
    
    // Find cycle count in table and interpolate
    for (int i = 0; i < numLoaded - 1; i++) {
        if (cycleCount >= entries[i].cycle_count && 
            cycleCount <= entries[i+1].cycle_count) {
            return linearInterpolate(cycleCount, 
                                    entries[i].cycle_count, 
                                    entries[i+1].cycle_count,
                                    entries[i].soh_percent, 
                                    entries[i+1].soh_percent);
        }
    }
    
    // Out of range - use boundary values
    if (cycleCount < entries[0].cycle_count) {
        return entries[0].soh_percent;
    } else {
        return entries[numLoaded-1].soh_percent;
    }
}

bool SDCardManager::logSOCData(float soc, float ocv, float temperature, unsigned long timestamp) {
    if (!_initialized) return false;
    
    File file = SD.open(SOC_LOG_FILE, FILE_APPEND);
    if (!file) {
        Serial.println("Failed to open SOC log file!");
        return false;
    }
    
    // If file is new, write header
    if (file.size() == 0) {
        file.println("Timestamp_ms,SOC_%,OCV_V,Temperature_C");
    }
    
    file.printf("%lu,%.2f,%.3f,%.1f\n", timestamp, soc, ocv, temperature);
    file.close();
    return true;
}

bool SDCardManager::logSOHData(float soh, int cycles, float capacity, unsigned long timestamp) {
    if (!_initialized) return false;
    
    File file = SD.open(SOH_LOG_FILE, FILE_APPEND);
    if (!file) {
        Serial.println("Failed to open SOH log file!");
        return false;
    }
    
    // If file is new, write header
    if (file.size() == 0) {
        file.println("Timestamp_ms,SOH_%,Cycle_Count,Capacity_Ah");
    }
    
    file.printf("%lu,%.2f,%d,%.2f\n", timestamp, soh, cycles, capacity);
    file.close();
    return true;
}

bool SDCardManager::fileExists(const char* path) {
    return SD.exists(path);
}

void SDCardManager::listFiles() {
    if (!_initialized) return;
    
    File root = SD.open("/");
    if (!root) {
        Serial.println("Failed to open root directory");
        return;
    }
    
    Serial.println("Files on SD card:");
    File file = root.openNextFile();
    while (file) {
        Serial.print("  ");
        Serial.print(file.name());
        Serial.print(" (");
        Serial.print(file.size());
        Serial.println(" bytes)");
        file = root.openNextFile();
    }
    root.close();
}

// Helper functions
String SDCardManager::readLine(File& file) {
    String line = "";
    while (file.available()) {
        char c = file.read();
        if (c == '\n') break;
        if (c != '\r') line += c;
    }
    return line;
}

bool SDCardManager::parseSOCLine(const String& line, SOCLookupEntry& entry) {
    int idx[8];
    idx[0] = line.indexOf(',');
    if (idx[0] == -1) return false;
    
    for (int i = 1; i < 8; i++) {
        idx[i] = line.indexOf(',', idx[i-1] + 1);
        if (idx[i] == -1 && i < 7) return false;
    }
    
    entry.soc_percent = line.substring(0, idx[0]).toFloat();
    entry.ocv_minus10C = line.substring(idx[0] + 1, idx[1]).toFloat();
    entry.ocv_0C = line.substring(idx[1] + 1, idx[2]).toFloat();
    entry.ocv_10C = line.substring(idx[2] + 1, idx[3]).toFloat();
    entry.ocv_20C = line.substring(idx[3] + 1, idx[4]).toFloat();
    entry.ocv_25C = line.substring(idx[4] + 1, idx[5]).toFloat();
    entry.ocv_30C = line.substring(idx[5] + 1, idx[6]).toFloat();
    entry.ocv_40C = line.substring(idx[6] + 1).toFloat();
    
    return true;
}

bool SDCardManager::parseSOHLine(const String& line, SOHLookupEntry& entry) {
    int idx[4];
    idx[0] = line.indexOf(',');
    if (idx[0] == -1) return false;
    
    for (int i = 1; i < 4; i++) {
        idx[i] = line.indexOf(',', idx[i-1] + 1);
        if (idx[i] == -1 && i < 3) return false;
    }
    
    entry.cycle_count = line.substring(0, idx[0]).toInt();
    entry.capacity_retention = line.substring(idx[0] + 1, idx[1]).toFloat();
    entry.soh_percent = line.substring(idx[1] + 1, idx[2]).toFloat();
    entry.resistance_increase = line.substring(idx[2] + 1).toFloat();
    
    return true;
}

float SDCardManager::linearInterpolate(float x, float x0, float x1, float y0, float y1) {
    if (x1 == x0) return y0;
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

bool SDCardManager::logBMSData(float* cellVoltages, int numCells, float packVoltage, 
                               float temperature, float current, unsigned long timestamp) {
    if (!_initialized) return false;
    
    File file = SD.open(BMS_DATA_LOG_FILE, FILE_APPEND);
    if (!file) {
        Serial.println("Failed to open BMS data log file!");
        return false;
    }
    
    // If file is new, write header
    if (file.size() == 0) {
        file.print("Timestamp_ms,Cell1_V,Cell2_V,Cell3_V,Cell4_V,Cell5_V,Cell6_V,Cell7_V,Cell8_V,");
        file.println("Pack_Voltage_V,Temperature_C,Current_A");
    }
    
    // Write timestamp
    file.printf("%lu,", timestamp);
    
    // Write all cell voltages
    for (int i = 0; i < numCells; i++) {
        file.printf("%.3f", cellVoltages[i]);
        if (i < numCells - 1) {
            file.print(",");
        }
    }
    
    // Write pack voltage, temperature, and current
    file.printf(",%.3f,%.1f,%.3f\n", packVoltage, temperature, current);
    
    file.close();
    return true;
}
