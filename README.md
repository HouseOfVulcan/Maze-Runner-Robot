# Maze Runner Robot

Bare-metal embedded C project to control a 4-motor robot using an ultrasonic sensor and utilizing the left-hand turn to solve a maze.

## 🧩 Maze Runner First Trial

Here’s a short demo of the robot attempting the maze:

[![](https://img.youtube.com/vi/PouMPIgu-xY/0.jpg)](https://youtube.com/shorts/PouMPIgu-xY?si=E1u7Ti7sdZmMK841)

*Click the image above to view on YouTube.*

**Note:** This is an early run — turning isn’t perfect yet, but you can see the navigation logic in action.


## Overview

This project implements an autonomous obstacle-avoiding robot using:
- Bare-metal C (no HAL or STM32CubeMX)
- Finite State Machine (FSM) for decision-making
- Ultrasonic sensing for distance measurement
- Modular source files for motors, sensors, and logic

Designed for educational purposes and as a foundational robotics build in the House of Vulcan project series.

## Hardware
- **Microcontroller:** STM32F407 Discovery
- **Motor Driver:** TB6612FNG (x2 for 4 motors)
- **Motors:** 48:1 DC gear motors
- **Sensor:** HC-SR04 Ultrasonic
- **Power:** External 5V motor supply 

**GPIO Usage:**
- `PA0` – Ultrasonic TRIG
- `PA1` – Ultrasonic ECHO (TIM2_CH2)
- `PB0–PB2`, `PB10`, `PB12–PB15` – Motor IN1/IN2 pins
- `PB11` – STBY pin for TB6612FNG

## Behavior

**Finite State Machine (FSM):**
- **STATE_FORWARD** – Moves forward if clear
- **STATE_BACKWARD** – Reverses if object detected
- **STATE_TURN_LEFT** – turns 90 degrees left upon first contact with wall
- **STATE_TURN_RIGHT** – Turns 180 degrees with second consecutive wall hit, then 90 degrees right again on third consecutive wall hit
- **STATE_ERROR** – Stops all motors on sensor timeout

**Timing Constants:**
- `SAFE_DISTANCE_CM = 7`
- `TURN_LEFT_90 = 90`
- `TURN_RIGHT_90 = 90`
- `TURN_RIGHT_180 = 180`
- `BACKUP = 50`
- `IDLE = 10`

**Duty Percentages :**
- `TURN_DUTY = 30`
- `CRUISE_DUTY = 20`
- `BACKUP_DUTY = 20`

## Project Structure
```
Inc/     – Header files (*.h)
delay.h   – Function prototypes for timing/delay utilities
motors.h  – Function prototypes and macros for motor control
sensor.h  – Function prototypes and constants for ultrasonic sensor handling
logic.h   – FSM states, function prototypes for maze logic / navigation

Src/     – Source files (*.c)
main.c    – System entry point; initializes peripherals and starts control loop
delay.c   – Implements microsecond/millisecond delays (busy-wait or timer-based)
motors.c  – Low-level H-bridge driver functions (forward, reverse, stop, brake)
sensor.c  – Ultrasonic trigger/echo measurement and distance calculation
logic.c   – Finite state machine for obstacle avoidance / maze-solving behavior
```

## Build
- Written using **bare-metal C** (no HAL or STM32CubeMX)
- Target clock: 168 MHz
- Built with STM32CubeIDE
- Flash to STM32F407 Discovery board

## Future Improvements
- LiDAR sensor
- Multiple Ultrasonic sensors

