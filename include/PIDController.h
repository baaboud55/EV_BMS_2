/*
 * PIDController.h
 * 
 * PID controller for BMS charging control (CC and CV modes)
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

// ==================== PID CONTROLLER STRUCTURE ====================
struct PIDController {
  float Kp;           // Proportional gain
  float Ki;           // Integral gain
  float Kd;           // Derivative gain
  float setpoint;     // Target value
  float integral;     // Integral accumulator
  float prev_error;   // Previous error for derivative
  float output_min;   // Minimum output limit
  float output_max;   // Maximum output limit
};

// ==================== PID FUNCTION DECLARATIONS ====================

/*
 * Initialize PID controller with gains and output limits
 */
void initPID(PIDController* pid, float Kp, float Ki, float Kd, float min_out, float max_out);

/*
 * Compute PID output based on current input and time step
 * Returns: PID output value (clamped to min/max limits)
 */
float computePID(PIDController* pid, float input, float dt);

// ==================== PID IMPLEMENTATIONS ====================

void initPID(PIDController* pid, float Kp, float Ki, float Kd, float min_out, float max_out) {
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  pid->integral = 0;
  pid->prev_error = 0;
  pid->output_min = min_out;
  pid->output_max = max_out;
}

float computePID(PIDController* pid, float input, float dt) {
  float error = pid->setpoint - input;
  
  // Proportional term
  float P = pid->Kp * error;
  
  // Integral term with anti-windup
  pid->integral += error * dt;
  float I = pid->Ki * pid->integral;
  
  // Derivative term
  float derivative = (error - pid->prev_error) / dt;
  float D = pid->Kd * derivative;
  
  // Compute total output
  float output = P + I + D;
  
  // Clamp output and apply anti-windup
  if (output > pid->output_max) {
    output = pid->output_max;
    pid->integral -= error * dt;  // Anti-windup
  }
  if (output < pid->output_min) {
    output = pid->output_min;
    pid->integral -= error * dt;  // Anti-windup
  }
  
  pid->prev_error = error;
  
  return output;
}

#endif // PID_CONTROLLER_H