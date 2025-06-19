#include "motor_control.h"

#define FORWARD  1  // GPIO pin set for forward rotation
#define BACKWARD 0  // GPIO pin set for reverse rotation

// PID constants for each motor channel
static constants_PID pidConsts1 = { .Kp = 0.3620, .Td = 0.0, .Ti = 0.5902, .dt = 0.0001 };
static constants_PID pidConsts2 = { .Kp = 0.2540, .Td = 0.0, .Ti = 0.4720, .dt = 0.0001 };
<<<<<<< HEAD
static constants_PID pidConsts3 = { .Kp = 0.2933, .Td = 0.0, .Ti = 0.4493, .dt = 0.0001 };
=======
static constants_PID pidConsts3 = { .Kp = 0.1633, .Td = 0.0, .Ti = 0.4193, .dt = 0.0001 };
>>>>>>> 3c11c9864d18f234fb81bf437f364af2d23b27f3
static constants_PID pidConsts4 = { .Kp = 0.2120, .Td = 0.0, .Ti = 0.4193, .dt = 0.0001 };

// PID runtime data (errors) for each motor
static datos_PID motor1_pidData = {0};
static datos_PID motor2_pidData = {0};
static datos_PID motor3_pidData = {0};
static datos_PID motor4_pidData = {0};

/**
 * @brief  Convert ticks/sec to PWM duty value.
 * @param  ticksec: measured speed in ticks/sec
 * @return PWM duty cycle (0..DUTY_MAX)
 */
static inline uint16_t tick_to_duty(double ticksec)
{
    return (uint16_t)((ticksec / MAX_TICKS_SEC) * DUTY_MAX);
}

/**
 * @brief  Set motor direction via two GPIO pins.
 * @param  speed: positive for forward, negative for backward
 * @param  GPIOx_pin1, pin1: first direction pin
 * @param  GPIOx_pin2, pin2: second direction pin
 */
static inline void set_motor_direction(double speed,
                                       GPIO_TypeDef* GPIOx_pin1, uint16_t pin1,
                                       GPIO_TypeDef* GPIOx_pin2, uint16_t pin2)
{
    if (speed > 0.0)
    {
        // Forward
        HAL_GPIO_WritePin(GPIOx_pin1, pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOx_pin2, pin2, GPIO_PIN_RESET);
    }
    else if (speed < 0.0)
    {
        // Reverse
        HAL_GPIO_WritePin(GPIOx_pin1, pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOx_pin2, pin2, GPIO_PIN_SET);
    }
    else
    {
        // Brake
        HAL_GPIO_WritePin(GPIOx_pin1, pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOx_pin2, pin2, GPIO_PIN_SET);
    }
}

/**
 * @brief  Clamp a computed duty to valid range.
 * @param  duty: raw duty value
 * @return duty constrained between DUTY_MIN and DUTY_MAX
 */
static inline uint16_t clamp_duty(double duty)
{
    if (duty < DUTY_MIN) return DUTY_MIN;
    if (duty > DUTY_MAX) return DUTY_MAX;
    return (uint16_t)duty;
}

/**
 * @brief  PID controller implementation.
 * @param  setpoint: desired speed
 * @param  pv: process variable (measured speed)
 * @param  c: PID constants
 * @param  d: PID runtime data (errors)
 * @param  reset: if true, clears the integral term
 * @return control output (before direction logic)
 */
double controlador_PID(double setpoint, double pv,
                       const constants_PID *c,
                       datos_PID *d,
                       bool reset)
{
    const double dt = c->dt;
    const double kp = c->Kp;
    const double ti = c->Ti;
    const double td = c->Td;

    double error = setpoint - pv;
    double der_error = (error - d->errorAnterior) / dt;
    d->errorAnterior = error;

    // Integral accumulation or reset
    d->errorAcumulado = reset ? 0.0 : (d->errorAcumulado + error * dt);

    // PID formula: P + D + I
    return (kp * error) + (kp * (td * der_error)) + (kp * (d->errorAcumulado / ti));
}

/**
 * @brief  Update PWM and direction for all four motors.
 * @param  setpoint_fl: front-left target
 * @param  setpoint_fr: front-right target
 * @param  setpoint_rl: rear-left target
 * @param  setpoint_rr: rear-right target
 * @param  resetPID: clear integrators if true
 */
void update_all_motors(double setpoint_fl,
                       double setpoint_fr,
                       double setpoint_rl,
                       double setpoint_rr,
                       bool resetPID)
{
    // Motor 1: front-left on TIM1->CCR1
    {
        int16_t pv = (int16_t)round(motor1.filtered_ticks_per_sec);
        set_motor_direction(setpoint_fl, GPIOA, GPIO_PIN_0, GPIOA, GPIO_PIN_1);
        double out = controlador_PID(fabs(setpoint_fl), fabs(pv),
                                     &pidConsts1, &motor1_pidData, resetPID);
        TIM1->CCR1 = clamp_duty(tick_to_duty(out));
    }

    // Motor 2: rear-left on TIM17->CCR1
    {
        int16_t pv = (int16_t)round(motor2.filtered_ticks_per_sec);
        set_motor_direction(setpoint_rl, GPIOB, GPIO_PIN_8, GPIOB, GPIO_PIN_9);
        double out = controlador_PID(fabs(setpoint_rl), fabs(pv),
                                     &pidConsts2, &motor2_pidData, resetPID);
        TIM17->CCR1 = clamp_duty(tick_to_duty(out));
    }

    // Motor 3: front-right on TIM16->CCR1
    {
        int16_t pv = (int16_t)round(motor3.filtered_ticks_per_sec);
        set_motor_direction(setpoint_fr, GPIOC, GPIO_PIN_6, GPIOC, GPIO_PIN_7);
        double out = controlador_PID(fabs(setpoint_fr), fabs(pv),
                                     &pidConsts3, &motor3_pidData, resetPID);
        TIM16->CCR1 = clamp_duty(tick_to_duty(out));
    }

    // Motor 4: rear-right on TIM1->CCR2
    {
        int16_t pv = (int16_t)round(motor4.filtered_ticks_per_sec);
        set_motor_direction(setpoint_rr, GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_1);
        double out = controlador_PID(fabs(setpoint_rr), fabs(pv),
                                     &pidConsts4, &motor4_pidData, resetPID);
        TIM1->CCR2 = clamp_duty(tick_to_duty(out));
    }
}
