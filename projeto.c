#define F_CPU 1000000UL // Define CPU frequency as 20 MHz (20,000,000 Hz)

#include <avr/io.h>
#include <util/delay.h>

// Pin definitions
#define IR_LEFT_PIN PD4
#define IR_RIGHT_PIN PB1

#define LED_PIN PB0   // LED on PB0
#define TOGGLE_PIN_LEFT PB4  // Pin to toggle for left IR sensor
#define TOGGLE_PIN_RIGHT PB5 // Pin to toggle for right IR sensor

// Function to initialize I/O pins
void io_init(void) {
    // Set IR sensor pins as input with pull-up resistors
    DDRD &= ~_BV(IR_LEFT_PIN);  // Set PD4 as input
    PORTD |= _BV(IR_LEFT_PIN);  // Enable pull-up for PD4

    DDRB &= ~_BV(IR_RIGHT_PIN); // Set PB1 as input
    PORTB |= _BV(IR_RIGHT_PIN); // Enable pull-up for PB1

    // Set LED pin as output
    DDRB |= _BV(LED_PIN); // Set PB0 as output

    // Set toggle pins as output
    DDRB |= _BV(TOGGLE_PIN_LEFT) | _BV(TOGGLE_PIN_RIGHT); // Set PB2 and PB3 as output

    // Initial state: LED OFF
    PORTB &= ~_BV(LED_PIN);

    // Initial state: Toggle pins OFF
    PORTB &= ~(_BV(TOGGLE_PIN_LEFT) | _BV(TOGGLE_PIN_RIGHT));
}

int main(void) {
    io_init();

    // Store previous states of IR sensors to detect changes
    uint8_t prev_ir_left_state = (PIND & _BV(IR_LEFT_PIN));
    uint8_t prev_ir_right_state = (PINB & _BV(IR_RIGHT_PIN));

    while (1) {
        // --- LED Blinking Logic ---
        // Toggle the LED state
        PORTB ^= _BV(LED_PIN); // XOR with _BV(LED_PIN) flips the bit

        _delay_ms(5); // Wait for 0.5 second (LED ON or OFF)
        // --- End LED Blinking Logic ---

        // Read current state of IR sensors
        uint8_t current_ir_left_state = (PIND & _BV(IR_LEFT_PIN));
        uint8_t current_ir_right_state = (PINB & _BV(IR_RIGHT_PIN));

        // Check if Left IR sensor state has changed
        if (current_ir_left_state != prev_ir_left_state) {
            // Toggle TOGGLE_PIN_LEFT (PB2)
            PORTB ^= _BV(TOGGLE_PIN_LEFT);
            prev_ir_left_state = current_ir_left_state; // Update previous state
            _delay_ms(1); // Debounce delay
        }

        // Check if Right IR sensor state has changed
        if (current_ir_right_state != prev_ir_right_state) {
            // Toggle TOGGLE_PIN_RIGHT (PB3)
            PORTB ^= _BV(TOGGLE_PIN_RIGHT);
            prev_ir_right_state = current_ir_right_state; // Update previous state
            _delay_ms(1); // Debounce delay
        }

        // A small delay at the end of the loop, but LED delay dominates
        // _delay_ms(10);
    }

    return 0;
}
