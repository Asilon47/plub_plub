#ifndef HALL_SENSOR_H_
#define HALL_SENSOR_H_


// Exponential filter coefficient for measured speed (0 < α < 1)
#define FILTER_ALPHA       0.7f

// How often (Hz) we compute speed from accumulated counts (uses TIM2 interrupt)
#define TIM2_FREQUENCY_HZ  10.0f

#include "main.h"
#include "motor_control.h"
#include "can.h"
#include "stdbool.h"

// Data structure to track one motor's Hall encoder state and measurements
typedef struct {
    uint32_t    last_tick;               // Last timestamp (ms) a pulse was seen
    int16_t     position;                // Cumulative pulse count (position)
    int8_t      direction;               // +1 or -1 for last movement
    uint8_t     prev_ab_state;           // Last A/B GPIO state (2-bit)
    GPIO_TypeDef *portA;                 // GPIO port for channel A
    uint16_t    pinA;                    // GPIO pin for channel A
    GPIO_TypeDef *portB;                 // GPIO port for channel B
    uint16_t    pinB;                    // GPIO pin for channel B
    int16_t     accum_counts;            // Pulses counted since last speed compute
    int16_t     ticks_per_sec_computed;  // Raw speed in ticks/sec before filtering
    double      filtered_ticks_per_sec;  // Low-pass filtered speed
    bool        reverse_wiring;          // Flip sign if encoder wiring is inverted
} MotorHallSensor;

// One sensor instance per motor (defined in hall_sensor.c)
extern MotorHallSensor motor1;  // front-left
extern MotorHallSensor motor2;  // rear-left
extern MotorHallSensor motor3;  // front-right
extern MotorHallSensor motor4;  // rear-right

// Read all four Hall inputs and update positions/directions
void     update_motor_position(void);

// Calculate and low-pass filter speed for each motor
void     compute_speed_for_all_motors(void);

// Accessors by motor ID (1–4)
int16_t  get_motor_position(uint8_t id);
int8_t   get_motor_direction(uint8_t id);
float    get_motor_speed_ticks_per_sec(uint8_t id);

// Initialize GPIO pins, counters, and filter state
void     init_hall_sensors(void);

#endif /* HALL_SENSOR_H_ */
