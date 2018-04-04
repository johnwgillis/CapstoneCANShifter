/* LedHelper.c : Capstone CAN Shifter
 * By John Gillis (jgillis@jgillis.com)
 */

#include "FreeRTOS.h"
#include "task.h"
#include "LedHelper.h"

/*-----------------------------------------------------------
 * Simple parallel port IO routines.
 *-----------------------------------------------------------*/

#define partstMAX_OUTPUT_LED			( ( unsigned char ) 7 )

#define DDR DDRC
#define PORT PORTC

/*-----------------------------------------------------------*/

void vLedHelperInitialiseLED( unsigned portBASE_TYPE uxLED ) {
	/* Set port B direction to output.  Start with output off. */
	unsigned char ucBit = ( unsigned char ) 1;
	ucBit <<= uxLED;
	DDR |= ucBit;
	PORT |= ucBit;
}
/*-----------------------------------------------------------*/

void vLedHelperSetLED( unsigned portBASE_TYPE uxLED, signed portBASE_TYPE xValue ) {
	unsigned char ucBit = ( unsigned char ) 1;

	if( uxLED <= partstMAX_OUTPUT_LED ) {
		ucBit <<= uxLED;	

		vTaskSuspendAll();
		{
			if( xValue == pdTRUE ) {
				PORT |= ucBit;
			} else {
				PORT &= ~ucBit;
			}
		}
		xTaskResumeAll();
	}
}
/*-----------------------------------------------------------*/

void vLedHelperToggleLED( unsigned portBASE_TYPE uxLED ) {
	unsigned char ucBit;

	if( uxLED <= partstMAX_OUTPUT_LED ) {
		ucBit = ( ( unsigned char ) 1 ) << uxLED;

		vTaskSuspendAll();
		{
			if( PORT & ucBit ) {
				PORT &= ~ucBit;
			} else {
				PORT |= ucBit;
			}
		}
		xTaskResumeAll();			
	}
}


