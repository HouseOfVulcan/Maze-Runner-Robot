#include "sensor.h"
#include <stdio.h>
#include <stdint.h>

// RCC Registers
#define RCC_AHB1ENR    (*(volatile uint32_t*) 0x40023830)
#define RCC_APB1ENR    (*(volatile uint32_t*) 0x40023840)

// GPIOA Registers
#define GPIOA_MODER    (*(volatile uint32_t*) 0x40020000)
#define GPIOA_OTYPER   (*(volatile uint32_t*) 0x40020004)
#define GPIOA_AFRL     (*(volatile uint32_t*) 0x40020020)
#define GPIOA_ODR      (*(volatile uint32_t*) 0x40020014)

// TIM2 Registers
#define TIM2_CR1       (*(volatile uint32_t*) 0x40000000) // CR1: Start/stop timer
#define TIM2_CCMR1     (*(volatile uint32_t*) 0x40000018) // CCMR1: Configure input capture mode
#define TIM2_CCER      (*(volatile uint32_t*) 0x40000020) // CCER: Edge trigger (rising/falling), enable capture
#define TIM2_PSC       (*(volatile uint32_t*) 0x40000028) // PSC: Divides system clock to get 1 µs resolution
#define TIM2_ARR       (*(volatile uint32_t*) 0x4000002C) // ARR: Max value before timer overflows (set to 65535)
#define TIM2_SR        (*(volatile uint32_t*) 0x40000010) // SR: Status Register (flag when capture occurs)
#define TIM2_CCR2      (*(volatile uint32_t*) 0x40000038) // CCR2: Captured timer value for Channel 2
#define TIM2_CNT       (*(volatile uint32_t*) 0x40000024) // CNT: Timer count value

// Simple delay function (approximate)
static void delay_us(uint32_t us) {
    // Rough delay for 168MHz system clock
    volatile uint32_t count = us * 42; // ~4 cycles per loop at 168MHz
    while(count--) {
        __asm__("nop");
    }
}

static void delay_ms(uint32_t ms) {
    for(uint32_t i = 0; i < ms; i++) {
        delay_us(1000);
    }
}

void sensor_init(void) {

    // Enable GPIOA clock
    RCC_AHB1ENR |= (1 << 0);

    // Configure PA0 as output (TRIG)
    GPIOA_MODER &=  ~(0x3 << 0); // Clear PA0 mode bits
    GPIOA_MODER |=   (0x1 << 0); // PA0 = General purpose output
    GPIOA_OTYPER &= ~(0x1 << 0);  // PA0 = Push-pull output

    // Configure PA1 as alternate function (ECHO - TIM2_CH2)
    GPIOA_MODER &= ~(0x3 << 2); // Clear PA1 mode bits
    GPIOA_MODER |=  (0x2 << 2); // PA1 = Alternate function

    GPIOA_AFRL  &= ~(0xF << 4); // Clear PA1 AF bits
    GPIOA_AFRL  |=  (0x1 << 4); // PA1 = AF1 (TIM2_CH2)

    // Enable TIM2 clock
    RCC_APB1ENR |= (1 << 0);

    // Configure TIM2 for input capture
    TIM2_PSC = 83;              // Prescaler: 84MHz / (83 + 1) = 1MHz (1µs ticks)
    TIM2_ARR = 0xFFFF;          // Maximum auto-reload value

    // Configure CH2 as input capture
    TIM2_CCMR1 &= ~(0xFF << 8); // Clear CC2S and input filter bits
    TIM2_CCMR1 |= (1 << 8);     // CC2S = 01: IC2 mapped to TI2 (PA1)

    // Enable CH2 capture, start with rising edge
    TIM2_CCER &= ~(1 << 5);     // CC2P = 0: rising edge
    TIM2_CCER |=  (1 << 4);     // CC2E = 1: enable capture on CH2

    // Start timer
    TIM2_CR1 |= (1 << 0);       // CEN = 1: enable counter

    // Wait for sensor to stabilize
    delay_ms(100);
}

static uint32_t measure_distance_cm(void) {
    uint32_t start_time, end_time, pulse_width;
    uint32_t timeout;

    // Clear any pending capture flags to avoid old junk data
    TIM2_SR &= ~(1 << 2);     // Clear CC2IF

    // Reset timer counter to ensure accurate calculations later on
    TIM2_CNT = 0;

    // Set capture for rising edge
    // Done again cuz we set falling later down, want to reset when code runs again
    TIM2_CCER &= ~(1 << 5);   // CC2P = 0 (rising edge)

    // Generate trigger pulse (10us minimum)
    GPIOA_ODR |=  (1 << 0);   // Set PA0 high
    delay_us(100);            // Wait 100us (more than minimum 10us)
    GPIOA_ODR &= ~(1 << 0);   // Set PA0 low

    // Wait for rising edge with timeout
    while (!(TIM2_SR & (1 << 2)))
    {
        // Check for timer overflow
        if (TIM2_CNT > 50000)
        { // ~50ms timeout
            return 0xFFFF;   // Error: timeout
        }
    }

    // Capture start time and clear flag
    start_time = TIM2_CCR2; // CCR2 cuz it captures time event happens
    TIM2_SR &= ~(1 << 2);

    // Switch to falling edge
    TIM2_CCER |= (1 << 5); // CC2P = 1 (falling edge)

    // Wait for falling edge with timeout
    while (!(TIM2_SR & (1 << 2)))
    {
        // Check for timer overflow
        if (TIM2_CNT > 30000)
        { // ~30ms timeout, HC-SR04 max distance is 4m, ~23ms
            return 0xFFFF;   // Error: timeout
        }
    }

    // Capture end time and clear flag
    end_time = TIM2_CCR2;
    TIM2_SR &= ~(1 << 2);

    // Calculate pulse width (handle potential overflow)
    if (end_time >= start_time)
    {
        pulse_width = end_time - start_time;
    } else
    {
        // just in case we hit an overflow sitch
        pulse_width = (0xFFFF - start_time) + end_time + 1;
    }

    // Optional: debug pulse width
    printf("Pulse width: %lu us\r\n", pulse_width);

    // Convert to distance in cm using integer math
    // Distance = (pulse_width_us * speed_of_sound_cm_per_us) / 2
    // Speed of sound ≈ 0.0343 cm/µs
    // So: distance_cm = (pulse_width * 343) / 2000
    uint32_t distance_cm = (pulse_width * 343) / 4000; // /4000 cuase everything was reading twice the length, gotta figure out why still
    printf("Distance: %lu cm\r\n", distance_cm);

    return distance_cm;
}

uint32_t get_distance_cm(void) {
    return measure_distance_cm();
}
