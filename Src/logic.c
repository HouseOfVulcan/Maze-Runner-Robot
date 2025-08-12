#include "logic.h"
#include "motors.h"
#include "sensor.h"
#include "delay.h"

#include <stdint.h>
#include <stdio.h>

// Distance threshold and motion timing (in ms)
#define SAFE_DISTANCE_CM 15
#define TURN_90_MS       1000
#define BACKUP_MS        400
#define IDLE_MS          100

typedef enum {
    STATE_FORWARD,
    STATE_BACKWARD,   // reserved for future use
    STATE_TURN_LEFT,
    STATE_TURN_RIGHT,
    STATE_ERROR
} ObstacleAvoidanceState;

// FSM state
static ObstacleAvoidanceState current_state = STATE_FORWARD;
static int last_turn_was_left = 0; // 0 = right, 1 = left

void logic_init(void) {
    // keep for future seeding / resets
    current_state = STATE_FORWARD;
    last_turn_was_left = 0;
}

void obstacle_avoidance_fsm_step(void)
{
    switch (current_state)
    {
        case STATE_FORWARD:
        {
            uint32_t distance = get_distance_cm();

            if (distance == SENSOR_ERR) {
                stop_motors();
                printf("Sensor Error\n");
                current_state = STATE_ERROR;
                break;
            }

            if (distance < SAFE_DISTANCE_CM) {
                stop_motors();
                delay_ms(200);

                move_backward();
                delay_ms(BACKUP_MS);

                stop_motors();
                delay_ms(200);

                // cycling thru lefts and rights
                if (last_turn_was_left) {
                    current_state = STATE_TURN_RIGHT;
                    last_turn_was_left = 0;
                } else {
                    current_state = STATE_TURN_LEFT;
                    last_turn_was_left = 1;
                }
            } else {
                move_forward();
            }
            break;
        }

        case STATE_TURN_LEFT:
            turn_left();
            delay_ms(TURN_90_MS);
            current_state = STATE_FORWARD;
            break;

        case STATE_TURN_RIGHT:
            turn_right();
            delay_ms(TURN_90_MS);
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

    delay_ms(IDLE_MS); // small pause between cycles
}
