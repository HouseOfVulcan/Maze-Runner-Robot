#ifndef LOGIC_H
#define LOGIC_H

#include <stdint.h>

// Error code for bad sensor readings
#define SENSOR_ERR       0xFFFFu

// Motion thresholds and timings
#define SAFE_DISTANCE_CM 15

// Duration constants (ms)
#define TURN_LEFT_90   180    // in-place 90° turn duration (tune this)
#define TURN_RIGHT_90  100
#define TURN_RIGHT_180 360
#define BACKUP         50
#define IDLE           10

// PWM duty cycle presets (%)
#define TURN_DUTY        20
#define CRUISE_DUTY      20
#define BACKUP_DUTY      20

// Initializes the FSM
void logic_init(void);

// Executes one step of the obstacle-avoidance FSM
void obstacle_avoidance_fsm_step(void);

#endif // LOGIC_H
