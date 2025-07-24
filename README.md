# Obstacle-Avoiding Robot 🤖 (STM32F4 + HC-SR04 + TB6612FNG)

Bare-metal embedded C project to control a 4-motor robot using an ultrasonic sensor for real-time obstacle avoidance.

## 🔧 Hardware

- **Microcontroller:** STM32F407 Discovery
- **Motor Driver:** TB6612FNG (x2 for 4 motors)
- **Motors:** 48:1 DC gear motors
- **Sensor:** HC-SR04 Ultrasonic
- **Power:** External 5V motor supply (recommended)
- **GPIO Usage:**
  - `PA0` – Ultrasonic TRIG
  - `PA1` – Ultrasonic ECHO (TIM2_CH2)
  - `PB0–PB2`, `PB10`, `PB12–PB15` – Motor IN1/IN2 pins
  - `PB11` – STBY pin for TB6612FNG

## 🧠 Behavior

### Finite State Machine (FSM):
- **STATE_FORWARD** – Moves forward if clear
- **STATE_BACKWARD** – Reverses if object detected
- **STATE_TURN_LEFT / RIGHT** – Random 90° turns to avoid
- **STATE_ERROR** – Stops all motors on sensor timeout

### Timing Constants:
- `SAFE_DISTANCE_CM = 15`
- `TURN_90_MS = 1000` *(adjust for real-world 90°)*
- `BACKUP_MS = 400`
- `IDLE_MS = 100`

## 📂 Project Structure

- `Inc/` – Header files (`*.h`)
- `Src/` – Source files (`*.c`)
- `main.c` – System entry point
- `motors.c` – Motor control functions
- `sensor.c` – Ultrasonic pulse/echo capture
- `logic.c` – FSM decision-making and state transitions

## ⚙️ Build Notes

- Written using **bare-metal C** (no HAL or STM32CubeMX)
- Target clock: 168 MHz
- Delay functions are blocking (not accurate for real-time scheduling)

## 🛠️ Future Improvements

- PWM speed control
- Interrupt-driven echo capture
- UART debug output
- FSM expansion (Idle, Recovery, Patrol modes)




