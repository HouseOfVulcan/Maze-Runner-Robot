#include "../inc/motors.h"
#include <stdio.h>

// RCC
#define RCC_AHB1ENR    (*(volatile uint32_t*) 0x40023830)
#define GPIOB_MODER    (*(volatile uint32_t*) 0x40020400) // sets the mode of the pins
#define GPIOB_OTYPER   (*(volatile uint32_t*) 0x40020404) // how the output signal behaves electrically
#define GPIOB_ODR      (*(volatile uint32_t*) 0x40020414) // How the control motor behaves, hold output values
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

}

/*
Motor	    IN1	   IN2
Left Front	PB0	    PB1
Left Rear	PB2	    PB10
Right Front	PB12	PB13
Right Rear	PB14	PB15

IN1	 IN2	Motor Behavior
0	 0   	Brake (fast stop)
0	 1	    Reverse
1	 0	    Forward ✅
1	 1	    Brake (fast stop again)
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
