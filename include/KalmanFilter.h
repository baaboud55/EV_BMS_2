/*
 * KalmanFilter.h
 * 
 * Extended Kalman Filter for SOC/SOH estimation
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Arduino.h>

// Kalman Filter for SOC estimation
class KalmanFilterSOC {
public:
    KalmanFilterSOC();
    
    void initialize(float initialSOC, float processNoise, float measurementNoise);
    float update(float voltage, float current, float temperature, float dt);
    float getSOC() const { return _soc; }
    void setSOC(float soc) { _soc = constrain(soc, 0.0, 100.0); }
    float estimateSOCFromOCV(float ocv, float temperature);
private:
    // State variables
    float _soc;                 // State of Charge (%)
    float _errorCovariance;     // Error covariance (P)
    
    // Kalman filter parameters
    float _processNoise;        // Process noise covariance (Q)
    float _measurementNoise;    // Measurement noise covariance (R)
    
    // Battery model parameters
    float _batteryCapacity;     // Battery capacity in Ah
    float _nominalVoltage;      // Nominal voltage
    
    // Helper functions
    float predictSOC(float current, float dt);
    float estimateOCVFromSOC(float soc, float temperature);
    
};

// Kalman Filter for SOH estimation
class KalmanFilterSOH {
public:
    KalmanFilterSOH();
    
    void initialize(float initialSOH, float processNoise, float measurementNoise);
    float update(float measuredCapacity, int cycleCount);
    float getSOH() const { return _soh; }
    void setSOH(float soh) { _soh = constrain(soh, 0.0, 100.0); }
    int getCycleCount() const { return _cycleCount; }
    void setCycleCount(int cycles) { _cycleCount = cycles; }
    
private:
    // State variables
    float _soh;                 // State of Health (%)
    float _errorCovariance;     // Error covariance (P)
    int _cycleCount;            // Charge/discharge cycle count
    
    // Kalman filter parameters
    float _processNoise;        // Process noise covariance (Q)
    float _measurementNoise;    // Measurement noise covariance (R)
    
    // Battery model parameters
    float _nominalCapacity;     // Nominal capacity in Ah
    
    // Helper functions
    float predictSOH(int cycleCount);
};

#endif // KALMAN_FILTER_H
