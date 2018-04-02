/* LedHelper.c : Capstone CAN Shifter
 * By John Gillis (jgillis@jgillis.com)
 */

#include "FreeRTOS.h"
#include "task.h"
#include "LedHelper.h"

/*-----------------------------------------------------------
 * Simple parallel port IO routines.
 *-----------------------------------------------------------*/

#define partstALL_BITS_OUTPUT			( ( unsigned char ) 0xff )
#define partstALL_OUTPUTS_OFF			( ( unsigned char ) 0xff )
#define partstMAX_OUTPUT_LED			( ( unsigned char ) 7 )

static volatile unsigned char ucCurrentOutputValue = partstALL_OUTPUTS_OFF; /*lint !e956 File scope parameters okay here. */

/*-----------------------------------------------------------*/

void vLedHelperInitialise( void ) {
	ucCurrentOutputValue = partstALL_OUTPUTS_OFF;

	/* Set port B direction to outputs.  Start with all output off. */
	DDRB = partstALL_BITS_OUTPUT;
	PORTB = ucCurrentOutputValue;
}
/*-----------------------------------------------------------*/

void vLedHelperSetLED( unsigned portBASE_TYPE uxLED, signed portBASE_TYPE xValue ) {
	unsigned char ucBit = ( unsigned char ) 1;

	if( uxLED <= partstMAX_OUTPUT_LED ) {
		ucBit <<= uxLED;	

		vTaskSuspendAll();
		{
			if( xValue == pdTRUE ) {
				ucBit ^= ( unsigned char ) 0xff;
				ucCurrentOutputValue &= ucBit;
			} else {
				ucCurrentOutputValue |= ucBit;
			}

			PORTB = ucCurrentOutputValue;
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
			if( ucCurrentOutputValue & ucBit ) {
				ucCurrentOutputValue &= ~ucBit;
			} else {
				ucCurrentOutputValue |= ucBit;
			}

			PORTB = ucCurrentOutputValue;
		}
		xTaskResumeAll();			
	}
}


