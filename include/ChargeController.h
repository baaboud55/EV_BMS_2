#ifndef CHARGE_CONTROLLER_H
#define CHARGE_CONTROLLER_H

// ==================== BMS OPERATING STATES ====================
enum BMSState {
  STATE_STARTUP,
  STATE_IDLE,           // Switches Open, monitoring
  STATE_PRECHARGING,    // Resistor engaged, waiting for caps to fill
  STATE_DISCHARGING,    // Load Enabled (Driving)
  STATE_CHARGING,       // Charger Enabled (Charging)
  STATE_BALANCING,      // Passive balancing active
  STATE_FAULT           // Error condition (Over/Under volt/temp)
};

// Global State Variable
extern BMSState currentBMSState;

// Function Declarations
void updateBMSStateMachine();

#endif // CHARGE_CONTROLLER_H