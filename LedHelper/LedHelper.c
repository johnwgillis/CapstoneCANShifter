/* LedHelper.c : Capstone CAN Shifter
 * By John Gillis (jgillis@jgillis.com)
 */

#include "FreeRTOS.h"
#include "task.h"
#include "LedHelper.h"

/*-----------------------------------------------------------
 * Simple parallel port IO routines.
 *-----------------------------------------------------------*/

#define MAX_OUTPUT_LED_NUM			( ( unsigned char ) 4 )

// 						LED #:		0		1		2		3		4
// 						Pin:		PD.0	PC.7	PB.5	PB.6	PB.7
volatile uint8_t* ledDDR[5]  = {	&DDRD, 	&DDRC, 	&DDRB, 	&DDRB, 	&DDRB	};
volatile uint8_t* ledPORT[5] = {	&PORTD, &PORTC, &PORTB, &PORTB, &PORTB	};
		 uint8_t  ledBit[5]  = {	0, 		7, 		5, 		6, 		7		};

/*-----------------------------------------------------------*/

void vLedHelperInitialiseLED( unsigned portBASE_TYPE uxLED ) {
	if( uxLED <= MAX_OUTPUT_LED_NUM ) {
		/* Set port direction to output.  Start with output off. */
		unsigned char ucBit = ( unsigned char ) 1;
		ucBit <<= ledBit[uxLED];
		*(ledDDR[uxLED]) |= ucBit;
		*(ledPORT[uxLED]) &= ~ucBit;
	}
}
/*-----------------------------------------------------------*/

void vLedHelperSetLED( unsigned portBASE_TYPE uxLED, signed portBASE_TYPE xValue ) {
	unsigned char ucBit = ( unsigned char ) 1;

	if( uxLED <= MAX_OUTPUT_LED_NUM ) {
		ucBit <<= ledBit[uxLED];	

		vTaskSuspendAll();
		{
			if( xValue == pdTRUE ) {
				*(ledPORT[uxLED]) &= ~ucBit;
			} else {
				*(ledPORT[uxLED]) |= ucBit;
			}
		}
		xTaskResumeAll();
	}
}
/*-----------------------------------------------------------*/

void vLedHelperToggleLED( unsigned portBASE_TYPE uxLED ) {
	unsigned char ucBit;

	if( uxLED <= MAX_OUTPUT_LED_NUM ) {
		ucBit = ( ( unsigned char ) 1 ) << ledBit[uxLED];

		vTaskSuspendAll();
		{
			if( *(ledPORT[uxLED]) & ucBit ) {
				*(ledPORT[uxLED]) &= ~ucBit;
			} else {
				*(ledPORT[uxLED]) |= ucBit;
			}
		}
		xTaskResumeAll();			
	}
}


