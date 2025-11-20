/*
 * SOHEstimator.h
 * 
 * State of Health estimation - PLACEHOLDER
 * Future implementation will track capacity fade and internal resistance
 */

#ifndef SOH_ESTIMATOR_H
#define SOH_ESTIMATOR_H

#include <Arduino.h>

class SOHEstimator {
public:
    SOHEstimator();
    
    // Initialize with nominal capacity
    void begin(float nominalCapacity_Ah);
    
    // Update SOH (placeholder - always returns 100% for now)
    float update();
    
    // Getters
    float getSOH() const { return _soh; }
    int getCycleCount() const { return _cycleCount; }
    
    // Setters (for future implementation)
    void setCycleCount(int cycles);
    void incrementCycleCount();
    
private:
    float _nominalCapacity;     // Battery capacity (Ah)
    float _soh;                 // State of Health (%)
    int _cycleCount;            // Charge/discharge cycle count
};

#endif // SOH_ESTIMATOR_H
