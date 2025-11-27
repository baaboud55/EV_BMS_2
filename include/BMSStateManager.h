#ifndef BMS_STATE_MANAGER_H
#define BMS_STATE_MANAGER_H

#include <Arduino.h> // Will use our Mock on PC, real on ESP32
#include "BMSConfig.h"
#include "ChargeController.h" // For BMSState enum
#include "KalmanFilter.h"

class BMSStateManager {
public:
    BMSStateManager();
    
    // Inputs (Call these before update)
    void setInputs(float packVoltage, float minCellV, float maxCellV, float current, float temp, bool chargerPresent, bool switchState);
    
    // The Main Logic Loop
    void update(float dt);
    
    // Getters
    BMSState getState() { return _currentState; }
    float getSOH() { return _kfSOH.getSOH(); }
    int getCycleCount() { return _kfSOH.getCycleCount(); }
    bool isPrechargeActive() { return _currentState == STATE_PRECHARGING; }
    bool isContactorClosed() { return _currentState == STATE_DISCHARGING; }
    bool isCharging() { return _currentState == STATE_CHARGING; }

private:
    // State
    BMSState _currentState;
    unsigned long _prechargeStartTime;
    
    // SOH Tracking
    float _currentCycleCapacityAh;
    bool _isTrackingDischarge;
    KalmanFilterSOH _kfSOH;

    // Inputs
    float _packVolt;
    float _minCell;
    float _maxCell;
    float _current;
    float _temp;
    bool _chargerConnected;
    bool _loadSwitch;

    // Internal Helpers
    void runStateMachine();
    void updateDischargeTracking(float dt);
};

#endif