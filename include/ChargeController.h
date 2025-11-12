/*
 * ChargeController.h
 * 
 * Charging state machine and control logic for BMS
 */

#ifndef CHARGE_CONTROLLER_H
#define CHARGE_CONTROLLER_H

// ==================== CHARGING STATES ====================
enum ChargeState {
  IDLE,
  CALIBRATING_CURRENT_SENSOR,
  WAITING_FOR_CONDITIONS,
  CHARGING_CC,          // Constant Current mode
  CHARGING_CV,          // Constant Voltage mode
  BALANCING,
  COMPLETE,
  ERROR,
  DISCHARGING
};

// ==================== GLOBAL CHARGE STATE ====================
extern ChargeState currentChargeState;
extern bool precharge_state;
extern unsigned long S_time;
extern unsigned long E_time;

// ==================== FUNCTION DECLARATIONS ====================

/*
 * Execute charging state machine
 */
void executeChargeControl();

/*
 * Perform Constant Current (CC) charging
 */
void performCCCharging();

/*
 * Perform Constant Voltage (CV) charging
 */
void performCVCharging();

/*
 * Stop charging and disable charger
 */
void stopCharging();

/*
 * Handle precharge sequence
 */
void preCharging();

/*
 * Check if startup conditions are met for charging
 */
void waitForStartupCondition();

#endif // CHARGE_CONTROLLER_H