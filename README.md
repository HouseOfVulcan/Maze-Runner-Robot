# Obstacle-Avoiding Robot

Bare-metal embedded C project to control a 4-motor robot using an ultrasonic sensor and utilizing the left-hand turn to solve a maze.


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
delay.h  –
motors.h – 
sensor.h –
logic.h  – 

Src/     – Source files (*.c)
main.c   – System entry point
delay.c  – 
motors.c – Motor control functions
sensor.c – Ultrasonic pulse/echo capture
logic.c  – FSM decision-making and state transitions
```

## Build
- Written using **bare-metal C** (no HAL or STM32CubeMX)
- Target clock: 168 MHz
- Built with STM32CubeIDE
- Flash to STM32F407 Discovery board

## Future Improvements
- LiDAR sensor
- Multiple Ultrasonic sensors

