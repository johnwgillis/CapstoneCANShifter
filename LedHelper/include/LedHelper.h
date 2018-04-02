/* LedHelper.h : Capstone CAN Shifter
 * By John Gillis (jgillis@jgillis.com)
 */

#ifndef LEDHELPER_H
#define LEDHELPER_H

#define partstDEFAULT_PORT_ADDRESS		( ( uint16_t ) 0x378 )

void vLedHelperInitialise( void );
void vLedHelperSetLED( UBaseType_t uxLED, BaseType_t xValue );
void vLedHelperToggleLED( UBaseType_t uxLED );

#endif