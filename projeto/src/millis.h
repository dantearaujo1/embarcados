#ifndef MILLIS_H
#define MILLIS_H

#include <stdint.h> // For uint32_t

// Function to initialize the millis timer
void millis_init(void);

// Function to get the current milliseconds since initialization
uint32_t millis(void);

#endif // MILLIS_H
