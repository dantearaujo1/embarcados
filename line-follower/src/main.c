#define F_CPU 16000000UL // Corrected: Using 20 MHz for external crystal
/* #define F_CPU 8000000UL // Corrected: Using 20 MHz for external crystal */
#define BAUD 9600 // Define desired Baud Rate for USART

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h> // Required for timer interrupts
#include <stdlib.h> // Required for itoa()

#include "usart.h"

// --- Proteus Simulation Speedup (Use with Caution!) ---
// If you uncomment PROTEUS_SIM_SPEEDUP, adjust delays accordingly.
// For example, delay_ms(1000) becomes delay_ms(1).
// This is for speeding up simulation only, not for real hardware.
#define PROTEUS_SIM_SPEEDUP

#ifdef PROTEUS_SIM_SPEEDUP
  #define SIM_DIVIDER 100 // Divide delays by 1000
#else
  #define SIM_DIVIDER 1 // No division for real hardware or normal simulation
#endif
// --- End Simulation Speedup ---

// User-defined Bit Manipulation Macros
#define setBit(REG,bit) REG |= (1<<bit)
#define clearBit(REG,bit) REG &= ~(1<<bit)
#define toggleBit(REG,bit) REG ^= (1<<bit)
#define readPin(PORT_REG, PIN_NUM) (PORT_REG & _BV(PIN_NUM)) // Reads the raw bit value
#define isOne(PORT_REG, PIN_NUM) ((PORT_REG & _BV(PIN_NUM)) != 0) // Returns 1 if pin is HIGH, 0 otherwise
#define isZero(PORT_REG, PIN_NUM) ((PORT_REG & _BV(PIN_NUM)) == 0) // Returns 1 if pin is LOW, 0 otherwise

// Pin definitions
#define IR_LEFT_PIN PD4
#define IR_RIGHT_PIN PB1

#define LED_PIN PB0     // Main Heartbeat LED on PB0
#define DEBUG_LED_IR_LEFT_PIN PB5   // Debug LED for Left IR sensor on PB5
#define DEBUG_LED_IR_RIGHT_PIN PB4  // Debug LED for Right IR sensor on PB4
#define DEBUG_LED_ULTRASSONIC PD7  // Debug LED for Right IR sensor on PB4

// Ultrasonic Sensor Pins & Registers
#define ULTRASONIC_TRIGGER_PIN PD3
#define ULTRASONIC_ECHO_PIN PD2
#define ULTRASONIC_ECHO_PIN PD2

#define ULTRASONIC_PORT PORTD
#define ULTRASONIC_PIN PIND
#define ULTRASONIC_DDR DDRD

// L298 Motor Driver Input Pins
#define IN1_PIN PD5     // Corresponds to the first pair (Left Motor control)
#define IN2_PIN PD6     // Corresponds to the first pair (Left Motor control)

#define IN3_PIN PB2     // Corresponds to the second pair (Right Motor control)
#define IN4_PIN PB3     // Corresponds to the second pair (Right Motor control)

// Distance threshold for stopping (in cm)
#define STOP_DISTANCE_CM 15 // Stop if distance is less than or equal to 15 cm

// --- Global variables for Input Capture ISR ---
volatile uint16_t timer1_ovf_count = 0; // Counts Timer1 overflows
volatile uint16_t capture_value_1 = 0;   // First capture (rising edge)
volatile uint16_t capture_value_2 = 0;   // Second capture (falling edge)
volatile uint8_t capture_state = 0;      // 0: waiting for rising, 1: waiting for falling, 2: complete

// Reference =======================
// https://github.com/hiibrarahmad/csr04-ultrasonic-sensor-with-atmega328p-2021-proteus-simulation/tree/main
void HCSR04Init();
void HCSR04Trigger();
uint16_t GetPulseWidth();
// =================================
void initIO();
void signalPulse();
void stopMotors();
void turnLeft();
void turnRight();
void forward();
void backward();

// Function to initialize I/O pins
void initIO(void) {
    // Set IR sensor pins as input with pull-up resistors
    clearBit(DDRD, IR_LEFT_PIN);  // Set PD4 as input
    setBit(PORTD, IR_LEFT_PIN);   // Enable pull-up for PD4; Set IR as 1 on the initial state

    clearBit(DDRB, IR_RIGHT_PIN); // Set PB1 as input
    setBit(PORTB, IR_RIGHT_PIN);  // Enable pull-up for PB1; Set IR as 1 on the initial state

    // Set LED pins as output
    setBit(DDRB, LED_PIN);              // Set PB0 as output
    setBit(DDRB, DEBUG_LED_IR_LEFT_PIN);  // Set PB5 as output
    setBit(DDRB, DEBUG_LED_IR_RIGHT_PIN); // Set PB4 as output
    setBit(DDRD, DEBUG_LED_ULTRASSONIC); // Set PD7 as output

    // Set L298 motor driver pins as output
    setBit(DDRD, IN1_PIN); clearBit(DDRD, IN2_PIN); // Configure PD5, PD6 as outputs (using combined macro usage)
    setBit(DDRD, IN2_PIN);

    setBit(DDRB, IN3_PIN); clearBit(DDRB, IN4_PIN); // Configure PB2, PB3 as outputs
    setBit(DDRB, IN4_PIN);

    // Set Ultrasonic Sensor Pins
    setBit(DDRD, ULTRASONIC_TRIGGER_PIN); // PD3 as output for Trigger
    clearBit(DDRD, ULTRASONIC_ECHO_PIN);  // PD2 as input for Echo
    // Initial state for Trigger pin: LOW
    clearBit(PORTD, ULTRASONIC_TRIGGER_PIN);

    // Initial state:
    // LED initially ON
    setBit(PORTB, LED_PIN);
    setBit(PORTB, DEBUG_LED_IR_LEFT_PIN);
    setBit(PORTB, DEBUG_LED_IR_RIGHT_PIN);
    setBit(PORTD, DEBUG_LED_ULTRASSONIC);

    // Initial state for L298 inputs:
    // IN1 (PD5) HIGH, IN2 (PD6) LOW
    // IN3 (PB2) HIGH, IN4 (PB3) LOW
    /* setBit(PORTD, IN1_PIN);     // Set PD5 HIGH */
    clearBit(PORTD, IN1_PIN);     // Set PD5 HIGH
    clearBit(PORTD, IN2_PIN);    // Set PD6 LOW

    setBit(PORTB, IN3_PIN);     // Set PB2 HIGH
    clearBit(PORTB, IN3_PIN);     // Set PB2 HIGH
    clearBit(PORTB, IN4_PIN);    // Set PB3 LOW
}


// ULTRASONIC UTILS
/*
We need total three functions to use the ultrasonic sound sensor. First we will initialize it, then generate ultrasonic sound wave and the
third step is measuring the time between generated wave and echo back wave after hitting an object. The complete reference to use the ultrasonic
sound sensor with math will be available on this link:  https://mendupmindcode.blogspot.com/2021/11/csr04-ultrasonic-sensor-with-atmega328p.html
Though the tutorial is for arduino but the concept is all the same. Please read the article throughly to have a clear understanding.

*/

void HCSR04Init() {

	// we're setting the trigger pin as output as it will generate ultrasonic sound wave
	setBit(ULTRASONIC_DDR, ULTRASONIC_TRIGGER_PIN);
}

void HCSR04Trigger() {   // this function will generate ultrasonic sound wave for 15 microseconds
	setBit(ULTRASONIC_PORT, ULTRASONIC_TRIGGER_PIN);
	_delay_us(15);				//wait 15uS
	clearBit(ULTRASONIC_PORT, ULTRASONIC_TRIGGER_PIN);
}

uint16_t GetPulseWidth() {
	// this function will be used to measure the pulse duration. When the ultra sound echo back after hitting an object
	// the microcontroller will read the pulse using the echo pin of the ultrasonic sensor connected to it.

	uint32_t i,result;

	// Section - 1: the following lines of code before the section - 2 is checking if the ultrasonic is working or not
	// it check the echo pin for a certain amount of time. If there is no signal it means the sensor is not working or not connect properly
	for(i=0;i<600000;i++)
	{
		if(!(ULTRASONIC_PIN & (1<<ULTRASONIC_ECHO_PIN)))
		continue;	//Line is still low, so wait
		else
		break;		//High edge detected, so break.
	}

	if(i==600000)
	return -1;	//Indicates time out

	//High Edge Found

	// Section -2 : This section is all about preparing the timer for counting the time of the pulse. Timers in microcontrllers is used for timimg operation
	//Setup Timer1
	TCCR1A=0X00;
	TCCR1B=(1<<CS11);	// This line sets the resolution of the timer. Maximum of how much value it should count.
	TCNT1=0x00;			// This line start the counter to start counting time

	// Section -3 : This section checks weather the there is any object or not
	for(i=0;i<600000;i++)                // the 600000 value is used randomly to denote a very small amount of time, almost 40 miliseconds
	{
		if(ULTRASONIC_PIN & (1<<ULTRASONIC_ECHO_PIN))
		{
			if(TCNT1 > 60000) break; else continue;   // if the TCNT1 value gets higher than 60000 it means there is not object in the range of the sensor
		}
		else
		break;
	}

	if(i==600000)
	return -2;

	//Falling edge found

	result=TCNT1;          // microcontroller stores the the value of the counted pulse time in the TCNT1 register. So, we're returning this value to the
	// main function for utilizing it later

	//Stop Timer
	TCCR1B=0x00;

	if(result > 60000)
	return -2;	//No obstacle
	else
	return (result>>1);
}
// Pulse 10uSecs to trigger input, this starts the ranging process
void signalPulse() {
	setBit(PORTD, ULTRASONIC_TRIGGER_PIN);
	/* delayFunction_Timer0(0xA4);	//0x9F = 159 = 15 uSecs */
  _delay_us(10);
	clearBit(PORTD, ULTRASONIC_TRIGGER_PIN);
}


// UTILS MOTOR FUNCTIONS ================================================
void stopMotors(void) {
    clearBit(PORTD, IN1_PIN);
    clearBit(PORTD, IN2_PIN);
    clearBit(PORTB, IN3_PIN);
    clearBit(PORTB, IN4_PIN);
}
void turnLeft(void) {
  // Left Motor: Backward (or stopped, depending on desired turn type)
    clearBit(PORTD, IN1_PIN);
    setBit(PORTD, IN2_PIN);
    // Right Motor: Forward
    setBit(PORTB, IN3_PIN);
    clearBit(PORTB, IN4_PIN);
}
void turnRight(void) {
  // Left Motor: Forward
  setBit(PORTD, IN1_PIN);
  clearBit(PORTD, IN2_PIN);
  // Right Motor: Backward (or stopped)
  clearBit(PORTB, IN3_PIN);
  setBit(PORTB, IN4_PIN);
}
void forward(void) {
  // Left Motor Forward
  setBit(PORTD, IN1_PIN);
  clearBit(PORTD, IN2_PIN);
  // Right Motor Forward
  setBit(PORTB, IN3_PIN);
  clearBit(PORTB, IN4_PIN);
}
void backward(void) {
  // Left Motor Backward
  clearBit(PORTD, IN1_PIN);
  setBit(PORTD, IN2_PIN);
  // Right Motor Backward
  clearBit(PORTB, IN3_PIN);
  setBit(PORTB, IN4_PIN);
}

// CHAT GPT SAVES
uint16_t get_ultrasonic_distance_cm(void) {
    // Reset state variables
    TCNT1 = 0; // Reset Timer1 counter
    timer1_ovf_count = 0;
    capture_state = 0; // Start waiting for rising edge

    // Clear any pending Timer1 Input Capture and Overflow flags
    setBit(TIFR1, ICF1);
    setBit(TIFR1, TOV1);

    // Send 10us pulse to Trigger pin
    signalPulse();

    // Wait for both edges to be captured by ISR (with a timeout)
    // Max echo for 400cm is ~23.32ms.
    // With F_CPU=16MHz, prescaler=8, 1 tick = 0.5us.
    // Max ticks = 23320 us / 0.5 us/tick = 46640 ticks.
    // This is less than 65536, so one overflow should be enough to cover it.
    // A generous timeout: e.g., 50ms (enough for ~8.5m range)
    uint32_t timeout_us = 50000;
    uint32_t start_time_us = (uint32_t)timer1_ovf_count * 65536 * 0.5 + TCNT1 * 0.5; // Current time in us

    // Disable interrupts temporarily to avoid race conditions with capture_state
    // or just rely on the volatile keyword if capture_state update is atomic.
    // For simple state, volatile is usually enough, but for critical sections, disable.
    // Here, it's a polling loop, so volatile should be fine.
    while (capture_state != 2) { // Wait until both edges are captured
        // Check for timeout. This calculation can be tricky, simple comparison is better.
        // Just check if too many overflows occurred or TCNT1 goes beyond expected without capture
        if (timer1_ovf_count > 1 || (timer1_ovf_count == 1 && TCNT1 > 10000)) { // Arbitrary overflow/tick count for timeout
            // Or, more robustly, calculate elapsed time
            uint32_t current_time_us = (uint32_t)timer1_ovf_count * 65536 * 0.5 + TCNT1 * 0.5;
            if ((current_time_us - start_time_us) > timeout_us) {
                return 0xFFFF; // Timeout occurred, return error value
            }
        }
    }

    // Calculate pulse duration in Timer1 ticks
    uint32_t pulse_duration_ticks;
    if (capture_value_2 >= capture_value_1) {
        pulse_duration_ticks = capture_value_2 - capture_value_1;
    } else {
        // This case *shouldn't* happen for HC-SR04 pulses if timer_ovf_count is accurate
        // because the max pulse is < 65536 ticks. But if it did, it means an overflow happened.
        pulse_duration_ticks = (uint32_t)(timer1_ovf_count * 65536 + capture_value_2) - capture_value_1;
    }

    // Convert duration from Timer1 ticks to microseconds
    // F_CPU = 16,000,000 Hz, prescaler = 8
    // Timer clock frequency = 16,000,000 / 8 = 2,000,000 Hz
    // Time per tick = 1 / 2,000,000 Hz = 0.0000005 seconds = 0.5 microseconds
    double pulse_duration_us = (double)pulse_duration_ticks * 0.5;

    // Calculate distance in centimeters
    // Speed of sound = 0.0343 cm/µs. Sound travels to target and back.
    // Distance = (duration_us * 0.0343) / 2
    // Or, simpler: distance = duration_us / 58.2 (approx)
    uint16_t distance_cm = (uint16_t)(pulse_duration_us / 58.0); // Using 58.0 for 1 tick = 0.5us

    return distance_cm;
}

int main(void) {
    initIO();
    HCSR04Init();
    const unsigned int ubrr_value = F_CPU / (16UL * BAUD) - 1;
    initUSART(ubrr_value);
    sei(); // Enable global interrupts (if not already enabled elsewhere)

    // Transmit a welcome message to the terminal (optional)
    writeln("System Started!");

    // Store previous states of IR sensors to detect changes
    // Using `readPin` will store the raw bit value (0 or _BV(PIN_NUM))
    uint8_t prev_ir_left_state = readPin(PIND, IR_LEFT_PIN);
    uint8_t prev_ir_right_state = readPin(PINB, IR_RIGHT_PIN);
    uint16_t distance; // Variable to store ultrasonic distance
    char distance_str[6];


    while (1) {
        uint16_t r;

        // --- Ultrasonic Sensor Logic
        /* distance = get_ultrasonic_distance_cm(); */

          // --- Main LED Blinking Logic ---
          toggleBit(PORTB, LED_PIN);
          _delay_ms(500 / SIM_DIVIDER);
          toggleBit(PORTB, LED_PIN);
          _delay_ms(500 / SIM_DIVIDER);
          // --- End LED Blinking Logic ---
          HCSR04Trigger();
          r = GetPulseWidth();
          distance=(r*0.034/2.0);	// This will give the distance in centimeters

          if (distance == 0xFFFF) {
              writeln("Dist: Timeout");
          } else {
              itoa(distance, distance_str, 10);
              write("Dist: ");
              write(distance_str);
              writeln(" cm");
          }

          if (distance <= STOP_DISTANCE_CM && distance != 0xFFFF) { // 0xFFFF is timeout/error
              stopMotors(); // Stop the robot if too close
              // Indicate stop with debug LEDs
              clearBit(PORTD, DEBUG_LED_ULTRASSONIC);
              _delay_ms(100 / SIM_DIVIDER); // Small delay while stopped, allowing for re-measurement
              continue; // Skip IR sensor checks and re-measure distance next loop
          } else {
            setBit(PORTD, DEBUG_LED_ULTRASSONIC);
          }
          // --- End Ultrasonic Sensor Logic ---

          // --- IR Sensor Logic and Motor Control (Priority 2: Line Following) ---
          // Read current state of IR sensors
          uint8_t current_ir_left_state = readPin(PIND, IR_LEFT_PIN);
          uint8_t current_ir_right_state = readPin(PINB, IR_RIGHT_PIN);

          // Define expected states for "on line" (LOW) and "off line" (HIGH) assuming active-low IR sensors
          #define ON_LINE 0x00
          #define OFF_LINE _BV(IR_LEFT_PIN)

          if (isOne(PIND, IR_LEFT_PIN) && isOne(PINB, IR_RIGHT_PIN)) {
              forward(); // Both off line, go straight (or lost line, implement search)
          } else if (isZero(PIND, IR_LEFT_PIN) && isOne(PINB, IR_RIGHT_PIN)) {
              turnRight(); // Left on line, right off line -> turn right
          } else if (isOne(PIND, IR_LEFT_PIN) && isZero(PINB, IR_RIGHT_PIN)) {
              turnLeft(); // Left off line, right on line -> turn left
          } else if (isZero(PIND, IR_LEFT_PIN) && isZero(PINB, IR_RIGHT_PIN)) {
              forward(); // Both on line, go straight
          }

          // Toggle debug LEDs based on *change* in IR sensor state (visual feedback)
          if (current_ir_left_state != prev_ir_left_state) {
              toggleBit(PORTB, DEBUG_LED_IR_LEFT_PIN);
              prev_ir_left_state = current_ir_left_state;
              _delay_ms(10 / SIM_DIVIDER); // Debounce
          }
          if (current_ir_right_state != prev_ir_right_state) {
              toggleBit(PORTB, DEBUG_LED_IR_RIGHT_PIN);
              prev_ir_right_state = current_ir_right_state;
              _delay_ms(1 / SIM_DIVIDER); // Debounce
          }
    }

    return 0;
}
