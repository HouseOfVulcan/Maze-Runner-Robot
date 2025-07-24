#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>

// Motor control functions
void motors_init(void);
void move_forward(void);
void move_backward(void);
void turn_left(void);
void turn_right(void);
void stop_motors(void);

#endif // MOTORS_H
