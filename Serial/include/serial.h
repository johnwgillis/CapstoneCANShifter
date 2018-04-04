/* serial.h : Capstone CAN Shifter
 * By John Gillis (jgillis@jgillis.com)
 */

#ifndef SERIAL_H
#define SERIAL_H

void serial_init(unsigned long baud);
void serial_transmit(char byte_data);
void serial_transmit_string(char* string);

#endif

