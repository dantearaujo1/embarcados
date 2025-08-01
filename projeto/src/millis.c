#ifndef F_CPU
#define F_CPU 16000000UL // Default if it is not defined
#endif

#include <avr/io.h>
#include <avr/interrupt.h>

#include "millis.h" // Include our own header

static volatile uint32_t timer_overflow_count = 0;
static volatile uint8_t timer_fractional_ms = 0;

/*
 * Calculation for Timer0:
 * F_CPU = 16,000,000 Hz
 * Timer0 is 8-bit, so it overflows at 256 counts.
 * We want a frequency such that an overflow is as close to 1ms as possible.
 *
 * Prescaler options: 1, 8, 64, 256, 1024
 *
 * 1. Prescaler = 64:
 * Timer clock frequency = F_CPU / 64 = 16.000.000 / 64 = 250.000 Hz
 * Time per tick = 1 / 250.000 = 0,000004 seconds = 4 microseconds
 * Time per overflow = 256 ticks * 4 microseconds/tick = 1024 microseconds = 1,024 ms
 *
 * This means for every overflow, we increment by 1 millisecond, but we have an
 * accumulated error of 0,024 ms per overflow.
 * To correct this:
 * 1000 ms / 1.024 ms/overflow = 976.5625 overflows per second
 * So, after 1000 overflows, we have counted 1024 * 1000 = 1.024.000 microseconds
 * This is 24.000 microseconds (24 ms) too much per 1000 overflows.
 * We add 1 extra millisecond every time `timer_fractional_ms` crosses 1000.
 */

// Timer0 Overflow Interrupt Service Routine
ISR(TIMER0_OVF_vect) {
    timer_overflow_count++;
    timer_fractional_ms += 24;

    if (timer_fractional_ms >= 1000) {
        timer_overflow_count++;
        timer_fractional_ms -= 1000;
    }
}

// Function to initialize the millis timer
void millis_init(void) {
    // 1. Set Timer0 to Normal mode (WGM02:0 = 000)
    TCCR0A = 0;
    TCCR0B = 0;

    // 2. Set prescaler to 64
    // CS02 = 0, CS01 = 1, CS00 = 1
    TCCR0B |= (1 << CS01) | (1 << CS00);

    // 3. Enable Timer0 Overflow Interrupt
    TIMSK0 |= (1 << TOIE0);

    // 4. Enable global interrupts
    sei();
}

// Function to get the current milliseconds count
uint32_t millis(void) {
    uint32_t ms;

    cli();
    ms = timer_overflow_count;
    sei();

    return ms;
}
