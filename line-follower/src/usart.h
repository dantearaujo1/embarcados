#ifndef USART_H_
#define USART_H_
#include <avr/io.h>

// Function prototypes
void initUSART(unsigned int b_rate);
void USART_Tx_char(char c); // Changed to take a single char
void USART_Tx_string(char* str);
void write(char* str);
void writeln(char* str);

#endif /* USART_H_H_ */
