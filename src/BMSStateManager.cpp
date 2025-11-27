#include "BMSStateManager.h"

BMSStateManager::BMSStateManager() {
    _currentState = STATE_IDLE;
    _prechargeStartTime = 0;
    _currentCycleCapacityAh = 0.0;
    _isTrackingDischarge = false;
    // Initialize SOH filter
    _kfSOH.initialize(100.0, KF_PROCESS_NOISE_COV * 10, KF_MEASUREMENT_NOISE_COV * 10);
}

void BMSStateManager::setInputs(float packVoltage, float minCellV, float maxCellV, float current, float temp, bool chargerPresent, bool switchState) {
    _packVolt = packVoltage;
    _minCell = minCellV;
    _maxCell = maxCellV;
    _current = current;
    _temp = temp;
    _chargerConnected = chargerPresent;
    _loadSwitch = switchState;
}

void BMSStateManager::update(float dt) {
    runStateMachine();
    updateDischargeTracking(dt);
}

// --- PASTE YOUR STATE MACHINE LOGIC HERE (Adapted for class members) ---
void BMSStateManager::runStateMachine() {
    // Safety Checks
    bool isOverVoltage = (_maxCell >= MAX_CELL_VOLTAGE);
    bool isUnderVoltage = (_minCell <= MIN_CELL_VOLTAGE);
    bool isOverTemp = (_temp >= MAX_TEMP);

    // Fault Check
    if (isOverTemp || (_currentState != STATE_CHARGING && _temp <= MIN_TEMP)) {
        _currentState = STATE_FAULT;
    }

    switch (_currentState) {
        case STATE_STARTUP:
        case STATE_IDLE:
            digitalWrite(PRECHARGE_RELAY, LOW);
            digitalWrite(LOAD_ENABLE_PIN, LOW);
            digitalWrite(CHARGE_ENABLE_PIN, LOW);

            if (_chargerConnected && !isOverVoltage && !isOverTemp) {
                _currentState = STATE_CHARGING;
            } else if (_loadSwitch && !isUnderVoltage && !isOverTemp) {
                _currentState = STATE_PRECHARGING;
                _prechargeStartTime = millis();
            }
            break;

        case STATE_PRECHARGING:
            digitalWrite(PRECHARGE_RELAY, HIGH);
            
            if (millis() - _prechargeStartTime >= PRECHARGE_TIME_MS) {
                _currentState = STATE_DISCHARGING;
            }
            if (!_loadSwitch || isUnderVoltage) _currentState = STATE_IDLE;
            break;

        case STATE_DISCHARGING:
            digitalWrite(LOAD_ENABLE_PIN, HIGH);
            digitalWrite(PRECHARGE_RELAY, HIGH);
            
            if (!_loadSwitch) _currentState = STATE_IDLE;
            if (isUnderVoltage) _currentState = STATE_FAULT;
            break;

        case STATE_CHARGING:
            digitalWrite(LOAD_ENABLE_PIN, LOW);
            
            if (_maxCell >= CHARGE_CUTOFF_VOLTAGE) digitalWrite(CHARGE_ENABLE_PIN, LOW);
            else if (_maxCell < CHARGE_RESTART_VOLTAGE) digitalWrite(CHARGE_ENABLE_PIN, HIGH);
            
            if (!_chargerConnected) _currentState = STATE_IDLE;
            if (isOverTemp || isOverVoltage) digitalWrite(CHARGE_ENABLE_PIN, LOW);
            break;

        case STATE_FAULT:
            digitalWrite(PRECHARGE_RELAY, LOW);
            digitalWrite(LOAD_ENABLE_PIN, LOW);
            digitalWrite(CHARGE_ENABLE_PIN, LOW);
            
            if (!isOverTemp && !isUnderVoltage && !isOverVoltage) _currentState = STATE_IDLE;
            break;
    }
}

// --- PASTE YOUR SOH TRACKING LOGIC HERE ---
void BMSStateManager::updateDischargeTracking(float dt) {
    // Logic adapted to use class members (_packVolt, etc.)
    if (_packVolt >= (CHARGE_COMPLETE_VOLTAGE - 0.2) && _current < 0.5) {
        if (!_isTrackingDischarge) {
            _currentCycleCapacityAh = 0.0;
            _isTrackingDischarge = true;
            Serial.println("[SOH] Fully Charged. Tracking started.");
        }
    }

    if (_isTrackingDischarge && _current > 0.1) {
        _currentCycleCapacityAh += (_current * dt / 3600.0);
    }

    if (_isTrackingDischarge && _minCell <= MIN_CELL_VOLTAGE) {
        _isTrackingDischarge = false;
        if (_currentCycleCapacityAh > (BATTERY_CAPACITY * 0.4)) {
             _kfSOH.update(_currentCycleCapacityAh, _kfSOH.getCycleCount() + 1);
             _kfSOH.setCycleCount(_kfSOH.getCycleCount() + 1);
             Serial.printf("[SOH] Updated: %.2f%%\n", _kfSOH.getSOH());
        }
    }
}