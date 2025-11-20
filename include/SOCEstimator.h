/*
 * SOCEstimator.h
 * 
 * State of Charge estimation using Coulomb Counting with OCV correction
 * - Primary method: Coulomb counting (integrates current over time)
 * - Calibration: OCV-based correction at startup and during rest periods
 */

#ifndef SOC_ESTIMATOR_H
#define SOC_ESTIMATOR_H

#include <Arduino.h>

// OCV-SOC lookup table entry (simplified temperature-compensated)
struct OCVPoint {
    float soc;      // State of Charge (0-100%)
    float ocv;      // Open Circuit Voltage per cell (V)
};

class SOCEstimator {
public:
    SOCEstimator();
    
    // Initialize with battery parameters
    void begin(float nominalCapacity_Ah, float initialSOC = 50.0);
    
    // Main update function - call this regularly with current measurements
    // Returns: updated SOC (%)
    float update(float current_A, float deltaTime_s);
    
    // Calibrate SOC using Open Circuit Voltage (call at startup or rest)
    // avgCellVoltage: average voltage per cell (V)
    // temperature: battery temperature (°C)
    void calibrateFromOCV(float avgCellVoltage, float temperature);
    
    // Getters
    float getSOC() const { return _soc; }
    float getCumulativeAh() const { return _cumulativeAh; }
    
    // Setters
    void setSOC(float soc);
    void resetCumulativeAh() { _cumulativeAh = 0.0; }
    
private:
    // Battery parameters
    float _nominalCapacity;     // Battery capacity (Ah)
    float _soc;                 // Current State of Charge (%)
    float _cumulativeAh;        // Cumulative amp-hours (for tracking)
    
    // OCV-SOC lookup table (simplified Li-ion curve)
    static const int OCV_TABLE_SIZE = 11;
    OCVPoint _ocvTable[OCV_TABLE_SIZE];
    
    // Helper functions
    void initializeOCVTable();
    float interpolateSOCFromOCV(float ocv, float temperature);
    float temperatureCompensation(float ocv, float temperature);
};

#endif // SOC_ESTIMATOR_H
