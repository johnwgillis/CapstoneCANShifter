/* LedHelper.h : Capstone CAN Shifter
 * By John Gillis (jgillis@jgillis.com)
 */

#ifndef LEDHELPER_H
#define LEDHELPER_H

void vLedHelperInitialiseLED( unsigned portBASE_TYPE uxLED );
void vLedHelperSetLED( UBaseType_t uxLED, BaseType_t xValue );
void vLedHelperToggleLED( UBaseType_t uxLED );

#endif