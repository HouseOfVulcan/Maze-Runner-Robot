/**
 ******************************************************************************
 * @file           : main.c
 * @author         : James Gallegos
 * @brief          : Main entry point – Obstacle Avoiding Robot FSM
 *
 * Description:
 * This bare-metal STM32F4 project implements an obstacle-avoiding robot using:
 *  - Ultrasonic sensor (HC-SR04) for distance detection
 *  - TB6612FNG motor driver for controlling 4 DC motors
 *  - Finite State Machine (FSM) logic for autonomous decision-making
 *
 * Flow:
 *  1. Initialize sensor, motors, and logic state
 *  2. Enter infinite loop
 *     - Each iteration reads distance and updates FSM
 *     - FSM determines movement: forward, reverse, or turn
 ******************************************************************************
 */

#include "motors.h"
#include "sensor.h"
#include "logic.h"
#include "delay.h"
#include <stdio.h>
#include <stdint.h>

int main(void)
{
    // Order matters: hardware first, then logic
    sensor_init();   // config PA0/PA1 + TIM2 capture
    motors_init();   // sets PB pins, enables TIM4 PWM, STBY high
    logic_init();    // sets initial FSM state(s)

    delay_ms(5000);	 // 5 sec pause

    while (1) {
        obstacle_avoidance_fsm_step();  // delays handled internally
       
    }
}
