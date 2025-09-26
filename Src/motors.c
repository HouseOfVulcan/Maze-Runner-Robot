#include "motors.h"
#include <stdio.h>
#include <stdint.h>

// RCC
#define RCC_AHB1ENR    (*(volatile uint32_t*) 0x40023830)
#define GPIOB_MODER    (*(volatile uint32_t*) 0x40020400) // sets the mode of the pins
#define GPIOB_OTYPER   (*(volatile uint32_t*) 0x40020404) // how the output signal behaves electrically
#define GPIOB_ODR      (*(volatile uint32_t*) 0x40020414) // How the control motor behaves, hold output values

// PWM - Pulse Width Modulation (change the width of ON portion each cycle)
// --- TIM4 & GPIOB AF (for PWM on PB6..PB9) ---
// Regular timer, 4 pins in a row

// Clock enable for APB1 bus peripherals (bit 2 in here enables TIM4)
#define RCC_APB1ENR     (*(volatile uint32_t*) 0x40023840)
// TIM4 base address
#define TIM4_BASE       (0x40000800UL)
// Control Register 1: starts/stops the counter, sets direction, auto-reload preload
#define TIM4_CR1        (*(volatile uint32_t*)(TIM4_BASE + 0x00))
// Event Generation Register: commit my new settings and restart cleanly
#define TIM4_EGR        (*(volatile uint32_t*)(TIM4_BASE + 0x14))
// Capture/Compare Mode Register 1: set behaviour for CH1 & CH2
#define TIM4_CCMR1      (*(volatile uint32_t*)(TIM4_BASE + 0x18))
// Capture/Compare Mode Register 2: set behaviour for CH3 & CH4
#define TIM4_CCMR2      (*(volatile uint32_t*)(TIM4_BASE + 0x1C))
// Capture/Compare Enable Register: the switch that lets the PWM signal drive the pin
#define TIM4_CCER       (*(volatile uint32_t*)(TIM4_BASE + 0x20))
// Prescaler: How long each tick is
#define TIM4_PSC        (*(volatile uint32_t*)(TIM4_BASE + 0x28))
// Auto-Reload Register: Max value we will hit before we reset
#define TIM4_ARR        (*(volatile uint32_t*)(TIM4_BASE + 0x2C))
// Capture/Compare Registers: cutoff point between ON and OFF
#define TIM4_CCR1       (*(volatile uint32_t*)(TIM4_BASE + 0x34)) // CH1 duty cycle
#define TIM4_CCR2       (*(volatile uint32_t*)(TIM4_BASE + 0x38)) // CH2 duty cycle
#define TIM4_CCR3       (*(volatile uint32_t*)(TIM4_BASE + 0x3C)) // CH3 duty cycle
#define TIM4_CCR4       (*(volatile uint32_t*)(TIM4_BASE + 0x40)) // CH4 duty cycle

#define GPIOB_AFRL      (*(volatile uint32_t*) 0x40020420)
#define GPIOB_AFRH      (*(volatile uint32_t*) 0x40020424)

// Defaulting speed to 30%
#ifndef DEFAULT_PWM_PERCENT
#define DEFAULT_PWM_PERCENT 30u
#endif

// Convert a 0–100% duty request into the CCR value for a given TIM4 channel.
// Duty cycle = percentage of each PWM period that the output is HIGH (ON).
// Notes:
//  - CCR = ((ARR + 1) * percent) / 100  because the counter runs 0..ARR (i.e., ARR+1 ticks).
//  - With PWM Mode 1, output is HIGH while CNT < CCR, then LOW until wrap.
//  - OCxPE (preload) is enabled, so CCR writes take effect on the next update event (rollover or EGR=UG).
static inline void pwm_set_percent_ch(uint8_t ch, uint8_t percent)
{
    if (percent > 100u) percent = 100u;	// handling overflow
    uint32_t arr = TIM4_ARR; // loading ARR value
    uint32_t ccr = ((arr + 1u) * percent) / 100u;	// formula to calculate CCR
    switch (ch) {
        case 1: TIM4_CCR1 = ccr; break; // PB6
        case 2: TIM4_CCR2 = ccr; break; // PB7
        case 3: TIM4_CCR3 = ccr; break; // PB8
        case 4: TIM4_CCR4 = ccr; break; // PB9
        default: break;
    }
}
// flipping all the switches, then tuning motors' speed
static void motors_pwm_init(void)
{
    // 1) Enable TIM4 clock on APB1 (bit 2)
    RCC_APB1ENR |= (0x1u << 2);

    // 2) setting PB6 - PB9 as Alternate Function AF2 (Timers need AF mode)
    // Clear MODER for pins 6..9 (set to 00)
    GPIOB_MODER &= ~((0x3u << (6*2)) | (0x3u << (7*2)) | (0x3u << (8*2)) | (0x3u << (9*2)));
    // Set to AF (10)
    GPIOB_MODER |=  ((0x2u << (6*2)) | (0x2u << (7*2)) | (0x2u << (8*2)) | (0x2u << (9*2)));

    // GPIO Alternate Function selection:
    //  - AFRL controls pins 0–7 (4 bits per pin)
    //  - AFRH controls pins 8–15 (4 bits per pin)
    // For PB6–PB9 we need AF2 to route TIM4_CH1..CH4 to the pins.
    // AFRL: PB6, PB7 -> AF2
    GPIOB_AFRL &= ~((0xFu << (6*4)) | (0xFu << (7*4)));   // Clear PB6 (bits [27:24]) and PB7 (bits [31:28])
    GPIOB_AFRL |=  ((0x2u << (6*4)) | (0x2u << (7*4)));   // Set AF2 for PB6/PB7 (TIM4_CH1/CH2)
    // AFRH: PB8, PB9 -> AF2
    GPIOB_AFRH &= ~((0xFu << (0*4)) | (0xFu << (1*4)));  // Clear PB8 (bits [3:0]) and PB9 (bits [7:4])
    GPIOB_AFRH |=  ((0x2u << (0*4)) | (0x2u << (1*4)));  // Set AF2 for PB8/PB9 (TIM4_CH3/CH4)

    // 3) Configure TIM4 for ~20 kHz PWM
    // Assume TIM4 timer clock is 84 MHz
    TIM4_CR1 = 0;                   // default mode: edge-aligned, upcounting
    TIM4_PSC = 41u;                 // Sets the tick speed, 84 MHz / (41+1) = 2 MHz
    TIM4_ARR = 99u;                 // Sets how many ticks per PWM period, 2 MHz / (99+1) = 20 kHz

	// Capture/Compare Mode Registers (personality per channel):
	//  - CCxS=00 selects OUTPUT mode (default after reset)
	//  - OCxM=110 selects PWM mode 1 (output HIGH while CNT < CCRx)
	//  - OCxPE=1 enables preload so CCR updates apply on update events
	// CCMR1 controls CH1/CH2
	// Output Compare Mode: OCxM = 110, PWM mode 1 → output HIGH while CNT < CCRx, then LOW
	// Output Compare Preload Enable: OCxPE=1, Preload on
	// preload the duty percentage??
    TIM4_CCMR1 &= ~((0xFFu << 0) | (0xFFu << 8));
    TIM4_CCMR1 |=  ((6u << 4) | (1u << 3))       // OC1M=110, OC1PE=1
                 | ((6u << 12) | (1u << 11));    // OC2M=110, OC2PE=1
	// CCMR2 controls CH3/4
    TIM4_CCMR2 &= ~((0xFFu << 0) | (0xFFu << 8));
    TIM4_CCMR2 |=  ((6u << 4) | (1u << 3))       // OC3M=110, OC3PE=1
                 | ((6u << 12) | (1u << 11));    // OC4M=110, OC4PE=1

	// Capture/Compare Enable Register
	// where we let the channel output drive the pin
    // Clear the enable bits (CC1E/CC2E/CC3E/CC4E) before setting them
    TIM4_CCER &= ~((1u << 0) | (1u << 4) | (1u << 8) | (1u << 12));
    TIM4_CCER |=  ((1u << 0) | (1u << 4) | (1u << 8) | (1u << 12));

    // Latch PSC/ARR and enable ARPE
    TIM4_EGR = 1u;                 // Update Generation bit. 1 = forces update event immediately
    TIM4_CR1 |= (1u << 7);         // ARPE - turns on ARR preload mechanism

    // Initial duty ~30%
    pwm_set_percent_ch(1, DEFAULT_PWM_PERCENT);
    pwm_set_percent_ch(2, DEFAULT_PWM_PERCENT);
    pwm_set_percent_ch(3, DEFAULT_PWM_PERCENT);
    pwm_set_percent_ch(4, DEFAULT_PWM_PERCENT);

    // Start counter
    TIM4_CR1 |= (1u << 0);         // CEN
}

void motors_set_all_percent(uint8_t percent)
{
    pwm_set_percent_ch(1, percent);
    pwm_set_percent_ch(2, percent);
    pwm_set_percent_ch(3, percent);
    pwm_set_percent_ch(4, percent);
}

void motors_set_each_percent(uint8_t lf, uint8_t lr, uint8_t rf, uint8_t rr)
{
    pwm_set_percent_ch(1, lf);
    pwm_set_percent_ch(2, lr);
    pwm_set_percent_ch(3, rf);
    pwm_set_percent_ch(4, rr);
}

void motors_init(void)
{
	RCC_AHB1ENR |= (1 << 1); //enable bit 1

	//MODER
		uint8_t motor_pins[] = {0, 1, 2, 10, 11, 12, 13, 14, 15}; // stating what pins we are working with

		// For loop to cycle thru all the pins we are using, clears them, then sets them
		for (int i = 0; i < sizeof(motor_pins)/sizeof(motor_pins[0]); i++) // (total size of the array in bytes)/( size of one element (also in bytes)
		{
			uint8_t pin = motor_pins[i];
			GPIOB_MODER &= ~(0x3 << (pin * 2));  // Clear mode bits
			GPIOB_MODER |=  (0x1 << (pin * 2));  // Set to output (01)
		}

	//OTYPER - setting each to push-pull
		for (int i = 0; i < sizeof(motor_pins)/sizeof(motor_pins[0]); i++)
		{
			uint8_t pin = motor_pins[i];
			GPIOB_OTYPER &= ~(1 << pin);  // Clear bits

		}

	//ODR
		GPIOB_ODR |= (1 << 11);  // Enable motor driver (STBY = HIGH)
		// Initializing control pins to LOW
		GPIOB_ODR &= ~( (1 << 0) | (1 << 1) | (1 << 2) |
			      (1 << 10) | (1 << 12) | (1 << 13) |
			      (1 << 14) | (1 << 15) );

    // --- Bare-metal PWM throttle on PB6..PB9 (TIM4 CH1..CH4) ---
    motors_pwm_init();

}

/*
Motor	        IN1	    IN2
Left Front------PB0     PB1
Left Rear-------PB2	    PB10
Right Front-----PB12	PB13
Right Rear------PB14	PB15

IN1	 IN2	Motor Behavior
 0	  0   	Brake
 0	  1     Reverse
 1	  0	    Forward
 1	  1	    Brake
*/
void move_forward(void)
{
	// Clear all control pins (set them all LOW)
	GPIOB_ODR &= ~( (1 << 0) | (1 << 1) | (1 << 2) |
	                (1 << 10) | (1 << 12) | (1 << 13) |
	                (1 << 14) | (1 << 15) );

	// Then set IN1 pins HIGH (for forward movement)
	GPIOB_ODR |= ( (1 << 0) | (1 << 2) | (1 << 12) | (1 << 14) );

}

void move_backward(void)
{
	// Clear all control pins (set them all LOW)
	GPIOB_ODR &= ~( (1 << 0) | (1 << 1) | (1 << 2) |
	                (1 << 10) | (1 << 12) | (1 << 13) |
	                (1 << 14) | (1 << 15) );

	// Then set IN2 pins HIGH (for backward movement)
	GPIOB_ODR |= ( (1 << 1) | (1 << 10) | (1 << 13) | (1 << 15) );
}

void turn_left(void)
{
	// Clear all control pins (set them all LOW)
	GPIOB_ODR &= ~( (1 << 0) | (1 << 1) | (1 << 2) |
	                (1 << 10) | (1 << 12) | (1 << 13) |
	                (1 << 14) | (1 << 15) );

	// Spin CCW, left motors go backward, right go forward
	GPIOB_ODR |= ( (1 << 1) | (1 << 10) | (1 << 12) | (1 << 14) );
}

void turn_right(void)
{
	// Clear all control pins (set them all LOW)
	GPIOB_ODR &= ~( (1 << 0) | (1 << 1) | (1 << 2) |
	                (1 << 10) | (1 << 12) | (1 << 13) |
	                (1 << 14) | (1 << 15) );

	// Spin CW, left motors go forward, right go backward
	GPIOB_ODR |= ( (1 << 0) | (1 << 2) | (1 << 13) | (1 << 15) );
}

void stop_motors(void)
{
	// Clear all control pins (set them all LOW)
	GPIOB_ODR &= ~( (1 << 0) | (1 << 1) | (1 << 2) |
	                (1 << 10) | (1 << 12) | (1 << 13) |
	                (1 << 14) | (1 << 15) );
}
