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
    // Initialize all system components
    sensor_init();     // Sets up GPIO + TIM2 for echo capture
    motors_init();     // Configures GPIOB + sets STBY high
    logic_init();      // Seeds RNG and sets default FSM state

    // Main robot control loop
    while (1)
    {
        obstacle_avoidance_fsm_step(); // Handles all movement logic
    }

    return 0;
}
