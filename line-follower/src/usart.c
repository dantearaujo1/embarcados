#include "usart.h"
#include <string.h> // For strlen()

void USART_Tx_char(char c) {
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    // Put data into buffer, sends the data
    UDR0 = c; // Directly assign char
}

void initUSART(unsigned int b_rate) {
    // Set baud rate
    UBRR0H = (uint8_t) (b_rate >> 8);
    UBRR0L = (uint8_t) b_rate;

    // Enable transmitter (TXEN0). Receiver (RXEN0) is not enabled for Tx-only.
    UCSR0B = (1 << TXEN0);

    // Set frame format: 8 data bits, 1 stop bit, no parity (default for Proteus Virtual Terminal)
    UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);
    /* UCSR0A |= (1 << U2X0); */
}

void USART_Tx_string(char* str) {
    for (int i = 0; str[i] != '\0'; i++) // Iterate until null terminator
        USART_Tx_char(str[i]);

    USART_Tx_char('\n'); // Send newline character
    USART_Tx_char('\r'); // Send carriage return character
}


//function to iterate over the message string
//cannot directly iterate over the string, so must use char pointer
void write(char* stringPtr)	//take pointer to char as parameter
{
	while (*stringPtr != 0) {	//while the data the pointer points to is not null
		while (!(UCSR0A & (1<<UDRE0)));	//wait to receive data
		UDR0 = *stringPtr;	//UDR0 = dereferenced stringPtr characters
		stringPtr++;	//advance stringPtr by 1
	}
}

void writeln(char* stringPtr)	//take pointer to char as parameter
{
	while (*stringPtr != 0) {	//while the data the pointer points to is not null
		while (!(UCSR0A & (1<<UDRE0)));	//wait to receive data
		UDR0 = *stringPtr;	//UDR0 = dereferenced stringPtr characters
		stringPtr++;	//advance stringPtr by 1
	}

  // Send Carriage Return (CR)
  while (!(UCSR0A & (1 << UDRE0))); //wait for UDR0 to be ready
  UDR0 = 0x0D; // Send CR

  // Send Line Feed (LF)
  while (!(UCSR0A & (1 << UDRE0))); //wait for UDR0 to be ready
  UDR0 = 0x0A; // Send LF
}
