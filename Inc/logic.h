#ifndef LOGIC_H
#define LOGIC_H

#include <stdint.h>

// Initializes the FSM (if needed in future)
void logic_init(void);

// Executes one step of the obstacle-avoidance FSM
void obstacle_avoidance_fsm_step(void);

#endif // LOGIC_H
