#include "logic.h"
#include "motors.h"
#include "sensor.h"
#include "delay.h"
#include <stdint.h>
#include <stdio.h>

typedef enum {
    STATE_FORWARD,
    STATE_BACKWARD,   // reserved for future use
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT,
    STATE_ERROR
} ObstacleAvoidanceState;

// FSM state
static ObstacleAvoidanceState current_state = STATE_FORWARD;

// Obstacle escape sequencing:
// 0 = try left 90
// 1 = try right 180 (face opposite)
// 2 = dead-end escape: right 90
static uint8_t num_turns = 0;

void logic_init(void) {
    current_state = STATE_FORWARD;
    num_turns = 0;
}

uint32_t get_distance_cm(void);

void obstacle_avoidance_fsm_step(void)
{
    switch (current_state)
    {
    case STATE_FORWARD:
    {
        uint32_t distance = get_distance_cm();

        if (distance == SENSOR_ERR) {
            stop_motors();
            current_state = STATE_ERROR;
            return;
        }

        if (distance < SAFE_DISTANCE_CM) {
            stop_motors();
            delay_ms(200);

        if (num_turns ==0) {

         //   motors_set_all_percent(BACKUP_DUTY);
          //  move_backward();
          //  delay_ms(BACKUP);

            stop_motors();
            delay_ms(200);
        }

            num_turns++;

            if (num_turns == 1) {
                current_state = STATE_TURN_LEFT;   // first try: left 90
            }
            else {
                current_state = STATE_TURN_RIGHT;  // second: right 180, third: right 90
                return;
            }

        }
        else {
            // path clear: reset attempt sequence
            motors_set_all_percent(CRUISE_DUTY);
            move_forward();
            num_turns = 0;
        }

        break;
    }

    case STATE_TURN_LEFT:
        motors_set_all_percent(TURN_DUTY);
        turn_left();
        delay_ms(TURN_LEFT_90);
        stop_motors();
        delay_ms(100);
        current_state = STATE_FORWARD;
        break;

    case STATE_TURN_RIGHT:
        motors_set_all_percent(TURN_DUTY);
        turn_right();
        if (num_turns == 2) {
            delay_ms(TURN_RIGHT_180);
        }
        // Dead end scenario
        else {
            delay_ms(TURN_RIGHT_90);
            num_turns = 0;				// Dead-end escape: reset attempt counter
        }
        stop_motors();
        delay_ms(100);
        current_state = STATE_FORWARD;
        break;

    case STATE_ERROR:
        stop_motors();
        delay_ms(2000);
        current_state = STATE_FORWARD;
        break;

    default:
        stop_motors();
        current_state = STATE_FORWARD;
        break;
    }

    delay_ms(IDLE); // small pause between cycles
}
