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

#define MAX_STRING_SIZE 100

// ********************************* Initialisation UART ********************************

void serial_init(unsigned long baud) {
	LINCR = (1 << LSWRES);
	LINBRRH = (((F_CPU/baud)/32)-1)>>8;
	LINBRRL = (((F_CPU/baud)/32)-1);

	LINCR = (1<<LENA)|(1<<LCMD2)|(1<<LCMD1)|(1<<LCMD0);

	return;
}


 // ************************************** UART Tx **************************************
 void serial_transmit(char byte_data) {
    while (LINSIR & (1 << LBUSY));          // Wait while the UART is busy.
    LINDAT = byte_data;
    return;
}

/* ----------------------------------------------------------------------------------- */

// Sends a string until hits a \0
void serial_transmit_string(char* string) {
	int i = 0;
	char endChar[] = "\0";
	while(strcmp(&string[i], endChar)!=0 && i < MAX_STRING_SIZE) {
		serial_transmit(string[i]);
		i++;
	}
	return;
}
