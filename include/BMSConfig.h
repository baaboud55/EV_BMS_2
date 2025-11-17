/*
 * BMSConfig.h
 * 
 * Configuration file for BMS hardware pins, constants, and parameters
 */

#ifndef BMS_CONFIG_H
#define BMS_CONFIG_H

// ==================== PIN DEFINITIONS ====================
const int CURRENT_SENSOR_PIN = 1;       // ACS712 current sensor
const int FEEDBACK_PIN = 19;            // Voltage feedback for CV mode
const int STARTUP_SENSE_PIN = 20;       // Charger input voltage sense
const int PRECHARGE_RELAY = 45;         // Precharge relay control
const int CHARGE_ENABLE_PIN = 47;
const int LOAD_ENABLE_PIN = 40;
const int LOAD_SWITCH_PIN = 42;                 // Load switch control
const int PWM_PIN = 48;                 // PWM for charge current control
const int SD_MISO_PIN = 13;             // SD card MISO
const int SD_MOSI_PIN = 11;             // SD card MOSI
const int SD_SCK_PIN = 12;              // SD card SCK (Clock)
const int SD_CS_PIN = 10;               // SD card CS (Chip Select)

// ==================== PWM CONFIGURATION ====================
const int PWM_FREQ = 62500;             // 62.5 kHz
const int PWM_CHANNEL = 0;
const int PWM_RES = 9;                  // 9-bit resolution (0-511)

// ==================== ACS712 CURRENT SENSOR CONFIGURATION ====================
const float ACS_SENSITIVITY = 0.061;  // V/A (calibrated value)
const int ACS_VCC_VOLTAGE = 3300;       // mV (measured value)

// ==================== BATTERY PARAMETERS ====================
const int NUM_CELLS = 8;
const int NUM_SLAVES = 2;               // Changed from 2 to 4 for GUI
const float BATTERY_CAPACITY = 100.0;   // Battery capacity in Ah

// Voltage limits
const float MAX_CELL_VOLTAGE = 4.2;     // Maximum safe cell voltage
const float MIN_CELL_VOLTAGE = 3.0;     // Minimum safe cell voltage
const float MIN_CHARGE_VOLTAGE = 3.0;   // Minimum voltage to start charging
// Current limits
const float MAX_CURRENT = 50.0;         // Maximum safe current

// Temperature limits
const float MAX_TEMP = 45.0;            // Maximum temperature in Celsius
const float MIN_TEMP = 0.0;             // Minimum temperature for charging

// ==================== CHARGING PARAMETERS ====================
// Two-stage charging parameters
const float CC_TARGET_CURRENT = 1.0;        // Constant Current: 1A
const float CC_TO_CV_VOLTAGE = 32.5;        // Switch to CV at 32.5V pack voltage
const float CV_TARGET_VOLTAGE = 33.2;       // Constant Voltage: 33.2V (4.15V per cell)
const float CHARGE_COMPLETE_VOLTAGE = 33.2; // Stop charging at 33.2V
const float CHARGE_COMPLETE_CURRENT = 0.1;  // Stop when current drops to 0.1A

// Voltage divider resistors for feedback
const float FBR1 = 98600.0;             // Feedback resistor 1
const float FBR2 = 10040.0;             // Feedback resistor 2
const float INR1 = 98100.0;             // Input resistor 1
const float INR2 = 9950.0;              // Input resistor 2

// ==================== TIMING INTERVALS ====================
const unsigned long BMS_READ_INTERVAL = 2000;       // 2 seconds
const unsigned long STARTUP_CHECK_INTERVAL = 2000;  // 2 seconds
const unsigned long CURRENT_READ_INTERVAL = 100;    // 100ms
const unsigned long FB_VOLTAGE_READ_INTERVAL = 100; // 100ms
const unsigned long WEB_UPDATE_INTERVAL = 2000;     // 2 seconds for web GUI
const unsigned long SD_SAVE_INTERVAL = 5000;       // NEW: Save to SD every 60 seconds
extern bool fbVoltageChanged;
extern bool currentChanged;

// ==================== SERIAL CONFIGURATION ====================
const int SERIAL_BAUD = 115200;         // USB serial
const int BMS_SERIAL_BAUD = 9600;       // BMS daisy-chain
const int BMS_RX_PIN = 17;              // BMS RX pin
const int BMS_TX_PIN = 16;              // BMS TX pin

// ==================== KALMAN FILTER PARAMETERS (NEW) ====================
const float KF_PROCESS_NOISE_COV = 1e-5;    // Process noise covariance Q
const float KF_MEASUREMENT_NOISE_COV = 1e-2; // Measurement noise covariance R
const float KF_INITIAL_ERROR_COV = 1.0;      // Initial error covariance P

// ==================== SD CARD FILE PATHS (NEW) ====================
static const char* SOC_LOOKUP_FILE = "/soc_lookup.csv";
static const char* SOH_LOOKUP_FILE = "/soh_lookup.csv";
static const char* SOC_LOG_FILE = "/soc_log.csv";
static const char* SOH_LOG_FILE = "/soh_log.csv";
static const char* BMS_DATA_LOG_FILE = "/bms_data_log.csv";  // NEW: Complete BMS data log


#endif // BMS_CONFIG_H