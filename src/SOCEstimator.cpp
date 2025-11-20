/*
 * SOCEstimator.cpp
 * 
 * Implementation of SOC estimation using Coulomb Counting with OCV correction
 */

#include "SOCEstimator.h"

// Constructor
SOCEstimator::SOCEstimator() {
    _nominalCapacity = 100.0;   // Default 100Ah
    _soc = 50.0;                // Default 50%
    _cumulativeAh = 0.0;
    initializeOCVTable();
}

// Initialize SOC estimator
void SOCEstimator::begin(float nominalCapacity_Ah, float initialSOC) {
    _nominalCapacity = nominalCapacity_Ah;
    _soc = constrain(initialSOC, 0.0, 100.0);
    _cumulativeAh = 0.0;
    initializeOCVTable();
    
    Serial.printf("SOC Estimator initialized: Capacity=%.2f Ah, Initial SOC=%.2f%%\n", 
                  _nominalCapacity, _soc);
}

// Main update function - Coulomb counting
float SOCEstimator::update(float current_A, float deltaTime_s) {
    // Coulomb counting formula:
    // SOC(t+1) = SOC(t) - (I × Δt / Q_nominal) × 100
    // 
    // Convention:
    // - Positive current = DISCHARGE (SOC decreases)
    // - Negative current = CHARGE (SOC increases)
    
    // Calculate change in SOC (%)
    float deltaSOC = (current_A * deltaTime_s / 3600.0) / _nominalCapacity * 100.0;
    
    // Update SOC
    _soc -= deltaSOC;  // Subtract because positive current = discharge
    _soc = constrain(_soc, 0.0, 100.0);
    
    // Update cumulative Ah (for tracking total energy flow)
    _cumulativeAh += (current_A * deltaTime_s / 3600.0);
    
    return _soc;
}

// Calibrate SOC from Open Circuit Voltage
void SOCEstimator::calibrateFromOCV(float avgCellVoltage, float temperature) {
    // Apply temperature compensation
    float compensatedOCV = temperatureCompensation(avgCellVoltage, temperature);
    
    // Interpolate SOC from OCV
    float calibratedSOC = interpolateSOCFromOCV(compensatedOCV, temperature);
    
    // Update SOC with calibrated value
    _soc = calibratedSOC;
    
    Serial.printf("SOC calibrated from OCV: Cell V=%.3fV, Temp=%.1f°C → SOC=%.2f%%\n",
                  avgCellVoltage, temperature, _soc);
}

// Set SOC manually (useful for testing or external calibration)
void SOCEstimator::setSOC(float soc) {
    _soc = constrain(soc, 0.0, 100.0);
}

// Initialize OCV-SOC lookup table (Li-ion typical curve)
void SOCEstimator::initializeOCVTable() {
    // Typical Li-ion OCV-SOC relationship at 25°C
    // These values are per-cell voltages
    _ocvTable[0]  = {0.0,   3.00};   // Empty
    _ocvTable[1]  = {10.0,  3.45};
    _ocvTable[2]  = {20.0,  3.60};
    _ocvTable[3]  = {30.0,  3.70};
    _ocvTable[4]  = {40.0,  3.75};
    _ocvTable[5]  = {50.0,  3.80};   // Mid-range
    _ocvTable[6]  = {60.0,  3.85};
    _ocvTable[7]  = {70.0,  3.92};
    _ocvTable[8]  = {80.0,  4.00};
    _ocvTable[9]  = {90.0,  4.10};
    _ocvTable[10] = {100.0, 4.20};   // Full
}

// Temperature compensation for OCV
float SOCEstimator::temperatureCompensation(float ocv, float temperature) {
    // Li-ion temperature coefficient: approximately -0.4 mV/°C per cell
    // Reference temperature: 25°C
    float tempDifference = temperature - 25.0;
    float compensation = -0.0004 * tempDifference;  // V per °C
    
    return ocv * (1.0 + compensation);
}

// Interpolate SOC from OCV using lookup table
float SOCEstimator::interpolateSOCFromOCV(float ocv, float temperature) {
    // Apply temperature compensation first
    float compensatedOCV = temperatureCompensation(ocv, temperature);
    
    // Handle boundary cases
    if (compensatedOCV <= _ocvTable[0].ocv) {
        return 0.0;
    }
    if (compensatedOCV >= _ocvTable[OCV_TABLE_SIZE - 1].ocv) {
        return 100.0;
    }
    
    // Find the two points to interpolate between
    for (int i = 0; i < OCV_TABLE_SIZE - 1; i++) {
        if (compensatedOCV >= _ocvTable[i].ocv && compensatedOCV <= _ocvTable[i + 1].ocv) {
            // Linear interpolation
            float ocv0 = _ocvTable[i].ocv;
            float ocv1 = _ocvTable[i + 1].ocv;
            float soc0 = _ocvTable[i].soc;
            float soc1 = _ocvTable[i + 1].soc;
            
            float interpolatedSOC = soc0 + (compensatedOCV - ocv0) * (soc1 - soc0) / (ocv1 - ocv0);
            return constrain(interpolatedSOC, 0.0, 100.0);
        }
    }
    
    // Fallback (should not reach here)
    return 50.0;
}
