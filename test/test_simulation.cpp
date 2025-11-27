// THIS IS YOUR TEST BENCH
#include "mocks/Arduino.h"
#include "BMSStateManager.h"
#include <iostream>

// Globals needed for the mock
MockSerial Serial;
unsigned long _mockMillis = 0;
int _pinStates[100] = {0};

void printStatus(BMSStateManager& bms, float voltage, float current) {
    std::cout << "T=" << (_mockMillis/1000.0) << "s | V=" << voltage << " | I=" << current 
              << " | State=" << bms.getState() 
              << " | LoadPin=" << digitalRead(LOAD_ENABLE_PIN)
              << " | SOH=" << bms.getSOH() << "%" << std::endl;
}

int main() {
    std::cout << "--- STARTING BMS SIMULATION ---" << std::endl;
    
    BMSStateManager bms;
    
    // --- SCENARIO 1: STARTUP & PRECHARGE ---
    std::cout << "\n[TEST 1] User turns ON switch" << std::endl;
    // Simulating: 33V pack, Switch ON, No Charger
    bms.setInputs(33.0, 4.1, 4.2, 0.0, 25.0, false, true); 
    
    // Run for 4 seconds (Precharge is 3s)
    for(int i=0; i<40; i++) {
        bms.update(0.1); // 100ms steps
        delay(100);      // Advance mock time
        if (i%10==0) printStatus(bms, 33.0, 0.0);
    }
    
    if (bms.isContactorClosed()) std::cout << "PASS: Contactor Closed after Precharge" << std::endl;
    else std::cout << "FAIL: Contactor did not close" << std::endl;


    // --- SCENARIO 2: SOH DISCHARGE CYCLE ---
    std::cout << "\n[TEST 2] Simulating Full Discharge for SOH" << std::endl;
    
    // 1. Reset to Full Charge state
    bms.setInputs(CHARGE_COMPLETE_VOLTAGE, 4.2, 4.2, 0.0, 25.0, false, false);
    bms.update(0.1); // Trigger "Full Charge" detection
    
    // 2. Discharge 4.0 Ah (Battery Capacity is 5.0Ah)
    // We simulate 4A load for 1 hour
    std::cout << "Discharging 4.0Ah..." << std::endl;
    float simulatedCapacity = 0;
    float current = 4.0;
    
    for(int i=0; i<36000; i++) { // 3600 seconds * 10 (0.1s steps)
        bms.setInputs(36.0, 3.6, 3.7, current, 25.0, false, true);
        bms.update(0.1); 
        delay(100);
        
        // Stop when we hit 4.0Ah simulated
        simulatedCapacity += (current * 0.1 / 3600.0);
        if (simulatedCapacity >= 4.0) break;
    }
    
    // 3. Hit Empty Voltage Trigger
    std::cout << "Triggering Empty Voltage..." << std::endl;
    bms.setInputs(24.0, 3.0, 3.1, 1.0, 25.0, false, true);
    bms.update(0.1);
    
    std::cout << "Final SOH: " << bms.getSOH() << "%" << std::endl;
    
    // Expected: 4.0Ah / 5.0Ah = 80% SOH
    if (bms.getSOH() < 85.0 && bms.getSOH() > 75.0) 
        std::cout << "PASS: SOH calculated correctly around 80%" << std::endl;
    else 
        std::cout << "FAIL: SOH calculation wrong" << std::endl;

    return 0;
}