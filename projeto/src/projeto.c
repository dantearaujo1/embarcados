#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

#include "millis.h"

#define F_CPU 16000000UL

// --- MACROS ---
// i.e setting the bit 5
//    0010 1000 REG
// OR 0001 0000 Bit shifted by bit
//    0011 1000 Result
#define set_bit(REG,bit) REG |= (1<<bit)

// i.e clearing the bit 5
//     0001 0000 REG
// AND 1110 1111 Bit shifted by bit negated
//     0000 0000 Result
#define clear_bit(REG,bit) REG &= ~(1<<bit)

// i.e toggles a specific bit
//     0001 0000 REG
// XOR 0001 0000 Bit shifted by bit negated
//     0000 0000 Result
//
//     0000 0000 REG
// XOR 0001 0000 Bit shifted by bit negated
//     0001 0000 Result
#define toggle_bit(REG,bit) REG ^= (1<<bit)


// We are using DDRD, DDRB for input and output definition
#define TRAVAPIN PD4 // PIN 4 arduino
#define BUZZERPIN PD3 // PIN 3 arduino
#define LEDGPIN PB5 // PIN 13 arduino
#define LEDRPIN PB4 // PIN 12 arduino
#define C1 PD5
#define C2 PD6
#define C3 PD7
#define R1 PB0
#define R2 PB3
#define R3 PB2
#define R4 PB1
#define ROWS 4
#define COLS 3


// --- Global Variables ---
const char* correctPassword = "0000";
char enteredPassword[5];
uint8_t passwordLength = 0;

uint32_t solenoidActivateTime = 0;
const uint32_t activationDuration = 5000;
volatile uint8_t solenoidActive = 0;

char keypadButtons[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

// --- Functions declaration ---
void config(void);
void run(void);
char getKeypadKey(void);
void buzz(uint16_t frequency_hz, uint32_t duration_ms);
void noBuzz(void);

int main(void) {
  config();

  while (1) {
    run();
  }

  return 0;
}


// --- Setup Function ---
void config(void) {
  // Initialize custom millis function
  millis_init();

  // Configure LED, Solenoid, and Buzzer pins as outputs
  set_bit(DDRB, LEDGPIN);
  set_bit(DDRB, LEDRPIN);
  set_bit(DDRD, BUZZERPIN);
  set_bit(DDRD, TRAVAPIN);

  // Configure Columns as output
  set_bit(DDRD, C1);
  set_bit(DDRD, C2);
  set_bit(DDRD, C3);

  // Configure Rows as input
  clear_bit(DDRB, R1);
  clear_bit(DDRB, R2);
  clear_bit(DDRB, R3);
  clear_bit(DDRB, R4);

  // Configure Rows with internal pull-up enabled
  // PUD in MCUCR is 0 by default making it pull-up
  // enabled
  set_bit(PORTB, R1);
  set_bit(PORTB, R2);
  set_bit(PORTB, R3);
  set_bit(PORTB, R4);

  // Initial state of LEDs, Solenoid, and Buzzer
  set_bit(PORTB, LEDRPIN);
  clear_bit(PORTB, LEDGPIN);
  clear_bit(PORTD, TRAVAPIN);
  clear_bit(PORTD, BUZZERPIN);

  // Initial state of KEYPAD column
  clear_bit(PORTD, C1);
  clear_bit(PORTD, C2);
  clear_bit(PORTD, C3);

  enteredPassword[0] = '\0'; // Initialize enteredPassword as an empty string
}

// --- Main Loop Function ---
void run(void) {
    uint32_t currentMillis = millis();

    // --- Solenoid and LED Deactivation Logic ---
    if (solenoidActive && (currentMillis - solenoidActivateTime >= activationDuration)) {
        clear_bit(PORTB, LEDGPIN); // Turn Green LED OFF
        clear_bit(PORTD, TRAVAPIN);   // Turn Solenoid OFF
        solenoidActive = 0;                       // Reset flag
        set_bit(PORTB, LEDRPIN);       // Turn Red LED ON (return to initial state)
    }

    // --- Keypad Reading ---
    char customKey = getKeypadKey();

    if (customKey != '\0' && solenoidActive == 0) { // If a key was pressed and solenoid is not active
      buzz(2093, 200); // Key press sound
      toggle_bit(PORTB, LEDGPIN);    // Red LED OFF

      if (customKey == '#') {
        enteredPassword[passwordLength] = '\0'; // Null-terminate the string

        if (strcmp(enteredPassword, correctPassword) == 0) {
          // Password CORRECT!
          set_bit(PORTD, TRAVAPIN);   // Turn Solenoid ON
          solenoidActive = 1;                       // Set flag
          solenoidActivateTime = currentMillis;     // Record activation time

          clear_bit(PORTB, LEDRPIN);    // Red LED OFF
          set_bit(PORTB, LEDGPIN);  // Green LED ON

          // Correct password sound: 3 quick beeps
          buzz(523, 100);
          _delay_ms(100);
          buzz(784, 100);
          _delay_ms(100);
          buzz(1047, 100);
          _delay_ms(100);
        } else {
            // Password INCORRECT!
            buzz(1175, 100);
            _delay_ms(100);
            buzz(587, 100);
            _delay_ms(100);
        }
        passwordLength = 0; // Reset entered password
        enteredPassword[0] = '\0';
      } else if (customKey == '*') { // Clear input with '*'
        enteredPassword[0] = '\0';
        passwordLength = 0;
        buzz(1047, 100);
        _delay_ms(100);
        buzz(1047, 100);
        _delay_ms(100);
        buzz(1047, 100);
        _delay_ms(100);
      } else if (passwordLength < 4) { // Only store up to 4 digits for password
        enteredPassword[passwordLength++] = customKey;
      }
    }
}

// --- Keypad Scan Function ---
char getKeypadKey(void) {
    // Array of column ports/pins for easy iteration
    /* volatile uint8_t *col_ddr[] = {&DDRD, &DDRD, &DDRD}; */
    volatile uint8_t *col_port[] = {&PORTD, &PORTD, &PORTD};
    uint8_t col_bits[] = {C1, C2, C3};

    // Array of row input pins/ports for easy iteration
    volatile uint8_t *row_pin_reg[] = {&PINB, &PINB, &PINB, &PINB}; // Using PIN registers to read input
    uint8_t row_bits[] = {R1, R2, R3, R4};

    for (uint8_t c = 0; c < COLS; c++) {
        // Set current column LOW, others HIGH
        clear_bit(*col_port[c], col_bits[c]);
        for (uint8_t i = 0; i < COLS; i++) {
            if (i != c) {
                set_bit(*col_port[i], col_bits[i]);
            }
        }
        _delay_ms(10); // Small delay for pin state to settle

        for (uint8_t r = 0; r < ROWS; r++) {
            // Check if the current row is LOW (button pressed)
            if (!(*row_pin_reg[r] & (1 << row_bits[r]))) {
                // Reset columns to high for next scan
                for (uint8_t i = 0; i < COLS; i++) {
                    set_bit(*col_port[i], col_bits[i]);
                }
                return keypadButtons[r][c]; // Return the pressed key
            }
        }
    }
    // Reset columns to high for next scan (if no key was pressed)
    for (uint8_t i = 0; i < COLS; i++) {
        set_bit(*col_port[i], col_bits[i]);
    }
    return '\0'; // No key pressed
}


// --- Buzzer Functions (Simplified tone generation) ---
// Note: This is a very basic tone implementation using _delay_us.
// For more precise or simultaneous operations, consider using a timer/PWM.
/* void buzz(uint16_t frequency_hz, uint32_t duration_ms) { */
/*     if (frequency_hz == 0) { */
/*         noBuzz(); */
/*         return; */
/*     } */
/*     uint32_t period_us = 1000000UL / frequency_hz; */
/*     uint32_t num_cycles = (uint32_t)duration_ms * 1000UL / period_us; */
/**/
/*     for (uint32_t i = 0; i < num_cycles; i++) { */
/*         set_bit(DDRD, BUZZERPIN); */
/*         _delay_us(period_us / 2); */
/*         clear_bit(DDRD, BUZZERPIN); */
/*         _delay_us(period_us / 2); */
/*     } */
/* } */

void buzz(uint16_t frequency_hz, uint32_t duration_ms) {
    if (frequency_hz == 0) {
        noBuzz();
        return;
    }
    // Calculate the half-period in microseconds.
    // We'll use this to toggle the pin.
    uint32_t half_period_us = (1000000UL / frequency_hz) / 2;

    // Calculate the total number of toggles needed
    uint32_t total_toggles = (uint32_t)duration_ms * 1000UL / half_period_us;

    // Limit to prevent excessively long loops (optional, for safety)
    if (total_toggles > 1000000) total_toggles = 1000000;

    for (uint32_t i = 0; i < total_toggles; i++) {
        toggle_bit(PORTD, BUZZERPIN); // Toggle the bit
        // Use a loop with a small, constant delay inside
        // This will introduce some inaccuracy due to loop overhead
        for (uint32_t j = 0; j < half_period_us / 10; j++) { // Example: delay in 10us chunks
            _delay_us(10);
        }
    }
}

void noBuzz(void) {
    clear_bit(PORTD, BUZZERPIN);
}
