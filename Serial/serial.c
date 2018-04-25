/* serial.c : Capstone CAN Shifter
 * By John Gillis (jgillis@jgillis.com)
 */

#include <stdlib.h>
#include <avr/io.h>
#include <string.h>
#include "serial.h"

#ifndef F_CPU
#define F_CPU 16000000UL                  // Clock frequency
#endif

#ifndef OUTPUT_MODE_SERIAL_NOT_PWM
#define OUTPUT_MODE_SERIAL_NOT_PWM 1	// Default value: 1 (should be configured in main)
#endif

#define MAX_STRING_SIZE 100

// ********************************* Initialisation UART ********************************

void serial_init(unsigned long baud) {
#if OUTPUT_MODE_SERIAL_NOT_PWM

	LINCR = (1 << LSWRES);
	LINBRRH = (((F_CPU/baud)/32)-1)>>8;
	LINBRRL = (((F_CPU/baud)/32)-1);

	LINCR = (1<<LENA)|(1<<LCMD2)|(1<<LCMD1)|(1<<LCMD0);

#endif
	return;
}


 // ************************************** UART Tx **************************************
 void serial_transmit(char byte_data) {
#if OUTPUT_MODE_SERIAL_NOT_PWM

    while (LINSIR & (1 << LBUSY));          // Wait while the UART is busy.
    LINDAT = byte_data;

#endif
    return;
}

/* ----------------------------------------------------------------------------------- */

// Sends a string until hits a \0
void serial_transmit_string(char* string) {
#if OUTPUT_MODE_SERIAL_NOT_PWM

	int i = 0;
	char endChar[] = "\0";
	while(strcmp(&string[i], endChar)!=0 && i < MAX_STRING_SIZE) {
		serial_transmit(string[i]);
		i++;
	}

#endif
	return;
}
