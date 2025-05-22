#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

// Maximum measurable ticks per second (for full-scale PWM)
#define MAX_TICKS_SEC 6912.0f

// Core and peripheral dependencies
#include "main.h"
#include "hall_sensor.h"
#include "can.h"
#include "stdbool.h"
#include <stdlib.h>
#include <math.h>

// PID tuning parameters
typedef struct constants_PID {
    double Kp;    // Proportional gain
    double Td;    // Derivative time constant
    double Ti;    // Integral time constant
    double dt;    // Sampling interval (s)
} constants_PID;

// Runtime data for PID calculations
typedef struct datos_PID {
    double errorAnterior;   // Previous error (for derivative term)
    double errorAcumulado;  // Accumulated error (for integral term)
} datos_PID;

// Timer handles for PWM outputs
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

// PWM limits and step sizes
#define DUTY_MAX   1999U  // Max PWM compare value
#define DUTY_MIN   0U     // Min PWM compare value
#define DUTY_STEP  50U    // Duty adjustment increment
#define DELAY_STEP 4U     // Delay increment for ramping

// Update speeds for all four motors; resetPID clears integral history
void update_all_motors(double setpoint_fl,  // front-left
                       double setpoint_fr,  // front-right
                       double setpoint_rl,  // rear-left
                       double setpoint_rr,  // rear-right
                       bool resetPID);

#endif /* MOTOR_CONTROL_H_ */
