/*
 * KalmanFilter.cpp
 * 
 * Implementation of Extended Kalman Filter for SOC/SOH estimation
 */

#include "KalmanFilter.h"
#include "BMSConfig.h"

// ==================== SOC Kalman Filter ====================

KalmanFilterSOC::KalmanFilterSOC() {
    _soc = 50.0;
    _errorCovariance = 1.0;
    _processNoise = KF_PROCESS_NOISE_COV;
    _measurementNoise = KF_MEASUREMENT_NOISE_COV;
    _batteryCapacity = BATTERY_CAPACITY;
    _nominalVoltage = 3.7 * NUM_CELLS;  // Nominal voltage for Li-ion
}

void KalmanFilterSOC::initialize(float initialSOC, float processNoise, float measurementNoise) {
    _soc = constrain(initialSOC, 0.0, 100.0);
    _errorCovariance = KF_INITIAL_ERROR_COV;
    _processNoise = processNoise;
    _measurementNoise = measurementNoise;
    
    Serial.printf("Kalman Filter SOC initialized: SOC=%.2f%%\n", _soc);
}

float KalmanFilterSOC::update(float voltage, float current, float temperature, float dt) {
    // 1. ALWAYS Predict (Coulomb Counting)
    float predictedSOC = predictSOC(current, dt);
    float predictedErrorCov = _errorCovariance + _processNoise;

    // 2. CONDITIONALLY Update (Measurement)
    // Only trust voltage if current is low (e.g., < 2 Amps)
    if (abs(current) < 2.0) { 
        float measuredSOC = estimateSOCFromOCV(voltage, temperature);
        
        float kalmanGain = predictedErrorCov / (predictedErrorCov + _measurementNoise);
        _soc = predictedSOC + kalmanGain * (measuredSOC - predictedSOC);
        _errorCovariance = (1.0 - kalmanGain) * predictedErrorCov;
    } else {
        // Under high load, trust the prediction 100%
        _soc = predictedSOC;
        _errorCovariance = predictedErrorCov;
    }
    
    _soc = constrain(_soc, 0.0, 100.0);
    return _soc;
}
float KalmanFilterSOC::predictSOC(float current, float dt) {
    // Coulomb counting: SOC(t+1) = SOC(t) - (I * dt / Q_nom) * 100
    // Positive current = discharge, negative current = charge
    float deltaSOC = (current * dt / 3600.0) / _batteryCapacity * 100.0;
    float predicted = _soc - deltaSOC;
    return constrain(predicted, 0.0, 100.0);
}

float KalmanFilterSOC::estimateOCVFromSOC(float soc, float temperature) {
    // Simplified OCV-SOC relationship for Li-ion
    // Temperature compensation: -0.0008 V/°C deviation from 25°C
    float tempFactor = 1.0 - (25.0 - temperature) * 0.0008 / 3.7;
    float baseOCV = 3.0 + (4.2 - 3.0) * pow(soc / 100.0, 0.9);
    return baseOCV * tempFactor * NUM_CELLS;
}

float KalmanFilterSOC::estimateSOCFromOCV(float ocv, float temperature) {
    // Inverse of OCV-SOC relationship
    // Compensate for temperature
    float tempFactor = 1.0 - (25.0 - temperature) * 0.0008 / 3.7;
    float avgCellVoltage = ocv / NUM_CELLS / tempFactor;
    
    // Inverse relationship (approximate)
    if (avgCellVoltage <= 3.0) return 0.0;
    if (avgCellVoltage >= 4.2) return 100.0;
    
    float socEstimate = pow((avgCellVoltage - 3.0) / (4.2 - 3.0), 1.0/0.9) * 100.0;
    return constrain(socEstimate, 0.0, 100.0);
}

// ==================== SOH Kalman Filter ====================

KalmanFilterSOH::KalmanFilterSOH() {
    _soh = 100.0;
    _errorCovariance = 1.0;
    _processNoise = KF_PROCESS_NOISE_COV;
    _measurementNoise = KF_MEASUREMENT_NOISE_COV * 10.0;  // Higher noise for SOH
    _nominalCapacity = BATTERY_CAPACITY;
    _cycleCount = 0;
}

void KalmanFilterSOH::initialize(float initialSOH, float processNoise, float measurementNoise) {
    _soh = constrain(initialSOH, 0.0, 100.0);
    _errorCovariance = KF_INITIAL_ERROR_COV;
    _processNoise = processNoise;
    _measurementNoise = measurementNoise;
    
    Serial.printf("Kalman Filter SOH initialized: SOH=%.2f%%\n", _soh);
}

float KalmanFilterSOH::update(float measuredCapacity, int cycleCount) {
    _cycleCount = cycleCount;
    
    // ===== PREDICTION STEP =====
    // Predict SOH based on cycle count (empirical degradation model)
    float predictedSOH = predictSOH(cycleCount);
    
    // Predict error covariance
    float predictedErrorCov = _errorCovariance + _processNoise;
    
    // ===== UPDATE STEP =====
    // Calculate measurement (SOH from capacity measurement)
    float measuredSOH = (measuredCapacity / _nominalCapacity) * 100.0;
    measuredSOH = constrain(measuredSOH, 0.0, 100.0);
    
    // Calculate Kalman gain
    float kalmanGain = predictedErrorCov / (predictedErrorCov + _measurementNoise);
    
    // Update SOH estimate
    _soh = predictedSOH + kalmanGain * (measuredSOH - predictedSOH);
    _soh = constrain(_soh, 0.0, 100.0);
    
    // Update error covariance
    _errorCovariance = (1.0 - kalmanGain) * predictedErrorCov;
    
    return _soh;
}

float KalmanFilterSOH::predictSOH(int cycleCount) {
    // Empirical degradation model for Li-ion batteries
    // Typical capacity fade: 20% loss over 1000 cycles
    // Non-linear degradation (faster in later life)
    
    if (cycleCount <= 0) return 100.0;
    if (cycleCount >= 1000) return 70.0;
    
    // Piece-wise linear approximation
    if (cycleCount <= 500) {
        // Gradual fade: 100% -> 89% over first 500 cycles
        return 100.0 - (11.0 * cycleCount / 500.0);
    } else {
        // Accelerated fade: 89% -> 70% over next 500 cycles
        return 89.0 - (19.0 * (cycleCount - 500) / 500.0);
    }
}
