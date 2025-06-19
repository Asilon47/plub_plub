#include "hall_sensor.h"

// Global sensor objects for each motor
MotorHallSensor motor1;
MotorHallSensor motor2;
MotorHallSensor motor3;
MotorHallSensor motor4;

// Lookup table for quadrature decoding: index = (prev_state<<2)|curr_state
static const int8_t quadrature_table[16] = {
     0, +1, -1,  0,   // prev 00 → curr 00,01,10,11
    -1,  0,  0, +1,   // prev 01 → …
    +1,  0,  0, -1,   // prev 10 → …
     0, -1, +1,  0    // prev 11 → …
};

// Poll each motor's Hall GPIOs and update position, direction, and accum_counts
void update_motor_position(void) {
    // Process motor1
    {
        MotorHallSensor *m = &motor1;
        uint8_t A = HAL_GPIO_ReadPin(m->portA, m->pinA);
        uint8_t B = HAL_GPIO_ReadPin(m->portB, m->pinB);
        uint8_t ab_state = (B << 1) | A;

        if (ab_state != m->prev_ab_state) {
            uint8_t idx = (m->prev_ab_state << 2) | ab_state;
            int8_t movement = quadrature_table[idx];

            if (movement != 0) {
                // Reverse sign if wiring flipped
                movement = m->reverse_wiring ? -movement : movement;
                m->position += movement;          // update position count
                m->direction = (movement > 0) ? +1 : -1;
                m->accum_counts += movement;      // for speed calc
            }
            m->prev_ab_state = ab_state;
        }
    }

    // Repeat identical logic for motor2
    {
        MotorHallSensor *m = &motor2;
        uint8_t A = HAL_GPIO_ReadPin(m->portA, m->pinA);
        uint8_t B = HAL_GPIO_ReadPin(m->portB, m->pinB);
        uint8_t ab_state = (B << 1) | A;
        if (ab_state != m->prev_ab_state) {
            uint8_t idx = (m->prev_ab_state << 2) | ab_state;
            int8_t movement = quadrature_table[idx];
            if (movement != 0) {
                movement = m->reverse_wiring ? -movement : movement;
                m->position += movement;
                m->direction = (movement > 0) ? +1 : -1;
                m->accum_counts += movement;
            }
            m->prev_ab_state = ab_state;
        }
    }

    // Repeat identical logic for motor3
    {
        MotorHallSensor *m = &motor3;
        uint8_t A = HAL_GPIO_ReadPin(m->portA, m->pinA);
        uint8_t B = HAL_GPIO_ReadPin(m->portB, m->pinB);
        uint8_t ab_state = (B << 1) | A;
        if (ab_state != m->prev_ab_state) {
            uint8_t idx = (m->prev_ab_state << 2) | ab_state;
            int8_t movement = quadrature_table[idx];
            if (movement != 0) {
                movement = m->reverse_wiring ? -movement : movement;
                m->position += movement;
                m->direction = (movement > 0) ? +1 : -1;
                m->accum_counts += movement;
            }
            m->prev_ab_state = ab_state;
        }
    }

    // Repeat identical logic for motor4
    {
        MotorHallSensor *m = &motor4;
        uint8_t A = HAL_GPIO_ReadPin(m->portA, m->pinA);
        uint8_t B = HAL_GPIO_ReadPin(m->portB, m->pinB);
        uint8_t ab_state = (B << 1) | A;
        if (ab_state != m->prev_ab_state) {
            uint8_t idx = (m->prev_ab_state << 2) | ab_state;
            int8_t movement = quadrature_table[idx];
            if (movement != 0) {
                movement = m->reverse_wiring ? -movement : movement;
                m->position += movement;
                m->direction = (movement > 0) ? +1 : -1;
                m->accum_counts += movement;
            }
            m->prev_ab_state = ab_state;
        }
    }
}

// Compute speed (ticks/sec) for each motor, reset accum_counts, and apply low-pass filter
void compute_speed_for_all_motors(void) {
    // Motor 1
    {
        MotorHallSensor *m = &motor1;
        m->ticks_per_sec_computed = m->accum_counts * TIM2_FREQUENCY_HZ;
        m->accum_counts = 0;
        m->filtered_ticks_per_sec =
            FILTER_ALPHA * m->filtered_ticks_per_sec +
            (1.0f - FILTER_ALPHA) * m->ticks_per_sec_computed;
    }
    // Motor 2
    {
        MotorHallSensor *m = &motor2;
        m->ticks_per_sec_computed = m->accum_counts * TIM2_FREQUENCY_HZ;
        m->accum_counts = 0;
        m->filtered_ticks_per_sec =
            FILTER_ALPHA * m->filtered_ticks_per_sec +
            (1.0f - FILTER_ALPHA) * m->ticks_per_sec_computed;
    }
    // Motor 3
    {
        MotorHallSensor *m = &motor3;
        m->ticks_per_sec_computed = m->accum_counts * TIM2_FREQUENCY_HZ;
        m->accum_counts = 0;
        m->filtered_ticks_per_sec =
            FILTER_ALPHA * m->filtered_ticks_per_sec +
            (1.0f - FILTER_ALPHA) * m->ticks_per_sec_computed;
    }
    // Motor 4
    {
        MotorHallSensor *m = &motor4;
        m->ticks_per_sec_computed = m->accum_counts * TIM2_FREQUENCY_HZ;
        m->accum_counts = 0;
        m->filtered_ticks_per_sec =
            FILTER_ALPHA * m->filtered_ticks_per_sec +
            (1.0f - FILTER_ALPHA) * m->ticks_per_sec_computed;
    }
}

// Return absolute position count for specified motor (1–4)
int16_t get_motor_position(uint8_t id) {
    switch (id) {
        case 1: return motor1.position;
        case 2: return motor2.position;
        case 3: return motor3.position;
        case 4: return motor4.position;
        default: return 0;
    }
}

// Return last movement direction (+1 or –1) for specified motor
int8_t get_motor_direction(uint8_t id) {
    switch (id) {
        case 1: return motor1.direction;
        case 2: return motor2.direction;
        case 3: return motor3.direction;
        case 4: return motor4.direction;
        default: return 0;
    }
}

// Return filtered speed (ticks/sec) for specified motor
float get_motor_speed_ticks_per_sec(uint8_t id) {
    switch (id) {
        case 1: return motor1.filtered_ticks_per_sec;
        case 2: return motor2.filtered_ticks_per_sec;
        case 3: return motor3.filtered_ticks_per_sec;
        case 4: return motor4.filtered_ticks_per_sec;
        default: return 0.0f;
    }
}

// Initialize each MotorHallSensor struct with GPIO pins and default state
void init_hall_sensors(void) {
    // Motor 1: front-left
    motor1.portA                = GPIOB;
    motor1.pinA                 = GPIO_PIN_6;
    motor1.portB                = GPIOB;
    motor1.pinB                 = GPIO_PIN_7;
    motor1.last_tick            = 0;
    motor1.accum_counts         = 0;
    motor1.position             = 0;
    motor1.direction            = 0;
    motor1.prev_ab_state        = 0;
    motor1.ticks_per_sec_computed = 0;
    motor1.filtered_ticks_per_sec = 0.0;
    motor1.reverse_wiring       = false;

    // Motor 2: rear-left
    motor2.portA                = GPIOA;
    motor2.pinA                 = GPIO_PIN_15;
    motor2.portB                = GPIOB;
    motor2.pinB                 = GPIO_PIN_3;
    motor2.last_tick            = 0;
    motor2.accum_counts         = 0;
    motor2.position             = 0;
    motor2.direction            = 0;
    motor2.prev_ab_state        = 0;
    motor2.ticks_per_sec_computed = 0;
    motor2.filtered_ticks_per_sec = 0.0;
    motor2.reverse_wiring       = false;

    // Motor 3: front-right (inverted wiring)
    motor3.portA                = GPIOC;
    motor3.pinA                 = GPIO_PIN_12;
    motor3.portB                = GPIOD;
    motor3.pinB                 = GPIO_PIN_2;
    motor3.last_tick            = 0;
    motor3.accum_counts         = 0;
    motor3.position             = 0;
    motor3.direction            = 0;
    motor3.prev_ab_state        = 0;
    motor3.ticks_per_sec_computed = 0;
    motor3.filtered_ticks_per_sec = 0.0;
    motor3.reverse_wiring       = true;

    // Motor 4: rear-right (inverted wiring)
    motor4.portA                = GPIOC;
    motor4.pinA                 = GPIO_PIN_10;
    motor4.portB                = GPIOC;
    motor4.pinB                 = GPIO_PIN_11;
    motor4.last_tick            = 0;
    motor4.accum_counts         = 0;
    motor4.position             = 0;
    motor4.direction            = 0;
    motor4.prev_ab_state        = 0;
    motor4.ticks_per_sec_computed = 0;
    motor4.filtered_ticks_per_sec = 0.0;
    motor4.reverse_wiring       = true;
}
