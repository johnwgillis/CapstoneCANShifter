/* ShifterManager.c : Capstone CAN Shifter
 * By John Gillis (jgillis@jgillis.com)
 */

#include "ShifterManager.h"

#include "FreeRTOS.h"
#include "task.h"

#include "serial.h"
#include "LdcSensor.h"

/* The stack size used for the LDC Monitor task. */
#define LDC_MONITOR_STACK_SIZE          1024
/* The period between executions of the LDC Monitor task. */
#define LDC_MONITOR_PERIOD				( ( TickType_t ) 100 / portTICK_PERIOD_MS  )
/* The initial wait after configuring the LDCs to allow them to stablize. */
#define LDC_MONITOR_INITIAL_WAIT		( ( TickType_t ) 500 / portTICK_PERIOD_MS  )

//                          POS:        1       2       3       4       5       6       7       8
uint32_t configPositionUpper[8] = {     1000,   1300,   1600,   1900,   2200,   2500,   2800,   3100    };
uint32_t configPositionLower[8] = {     1200,   1500,   1800,   2100,   2400,   2700,   3000,   3300    };


static volatile ShifterPosition currentShifterPosition = POS_UNKNOWN;
static volatile ShifterPosition lastShifterPosition = POS_UNKNOWN;
static volatile uint32_t dataReadsSinceLastCheck = 0; // Used for detecting stalls

static void vLDCMonit( void *pvParameters );
void vShifterManagerOutputPosition(ShifterPosition position);

void outputSerial(ShifterPosition currentPosition);
void outputCAN(ShifterPosition currentPosition);
void outputPWM(ShifterPosition currentPosition);
void outputInterlocks(ShifterPosition currentPosition);

/*-----------------------------------------------------------*/

void vShifterManagerStartTasks( UBaseType_t uxPriority ) {
	/* Initialise the LDC's config */
    vLdcSensorInitialise();

    /* Start monitoring task */
    xTaskCreate( vLDCMonit, "LDC_MONIT", LDC_MONITOR_STACK_SIZE, NULL, uxPriority, ( TaskHandle_t * ) NULL );
}
/*-----------------------------------------------------------*/

static void vLDCMonit( void *pvParameters ) {
	/* The parameters are not used. */
	( void ) pvParameters;

    /* Wait for the LDC's to data to stablize */
    vTaskDelay( LDC_MONITOR_INITIAL_WAIT );

	/* Cycle for ever, delaying monitoring cycle to a set rate. */
	for( ;; ) {
        uint32_t data[8];
        
        // Zero out data to start
        for(int i=0; i<8; i++) { data[i] = 0; }

        /* Recalculate current shifter postion */
        data[0] = vLdcSensorReadChannel(LDC_1_Addr, 0);
        // data[2] = vLdcSensorReadChannel(LDC_1_Addr, 1);
        // data[4] = vLdcSensorReadChannel(LDC_1_Addr, 2);
        // data[6] = vLdcSensorReadChannel(LDC_1_Addr, 3);

        // Old method: Sort to find max LDC value
        // int maxIndex = -1;
        // uint32_t maxValue = 0;
        // for(int i=0; i<8; i++) {
        //     // Mask out the error status bits (first 4 bits)
        //     data[i] &= ~(0xF0000000);
        //     if(data[i] > maxValue) {
        //         maxIndex = i;
        //         maxValue = data[i];
        //     }
        // }
        // currentShifterPosition = maxIndex;

        currentShifterPosition = POS_UNKNOWN;
        for(int i=0; i<8 && currentShifterPosition==POS_UNKNOWN; i++) {
            if( data[0] >= configPositionLower[i] && data[0] <= configPositionUpper[i] ) {
                currentShifterPosition = i;
            }
        }

        if( lastShifterPosition != currentShifterPosition ) {
            vShifterManagerOutputPosition(currentShifterPosition);
            lastShifterPosition = currentShifterPosition;
        }

        dataReadsSinceLastCheck++; // TODO: this should happen if valid data from LDC

        vTaskDelay( LDC_MONITOR_PERIOD );
	}
}
/*-----------------------------------------------------------*/

void vShifterManagerOutputPosition(ShifterPosition position) {

    outputSerial(position);

    outputCAN(position);

    outputPWM(position);

    outputInterlocks(position);

    return;
}
/*-----------------------------------------------------------*/

ShifterPosition vShifterManagerGetCurrentPosition(void) {
    return currentShifterPosition;
}
/*-----------------------------------------------------------*/

BaseType_t xAreShifterManagerTasksStillRunning( void ) {
    BaseType_t xReturn;

	if( dataReadsSinceLastCheck == 0 ) {
		xReturn = pdFALSE; // No reads so likely stalled
	} else {
		xReturn = pdTRUE;
	}

	dataReadsSinceLastCheck = 0;

	return xReturn;
}
/*-----------------------------------------------------------*/

void outputSerial(ShifterPosition currentPosition) {
	serial_transmit_string("Shifter Position Changed To: \0");
	switch(currentPosition) {
		case POS_1:
			serial_transmit_string("1\n\0");
			break;
		case POS_2:
			serial_transmit_string("2\n\0");
			break;
		case POS_3:
			serial_transmit_string("3\n\0");
			break;
		case POS_4:
			serial_transmit_string("4\n\0");
			break;
		case POS_5:
			serial_transmit_string("5\n\0");
			break;
		case POS_6:
			serial_transmit_string("6\n\0");
			break;
		case POS_7:
			serial_transmit_string("7\n\0");
			break;
		case POS_UNKNOWN:
		default:
			serial_transmit_string("Unknown\n\0");
			break;
	}
}
/*-----------------------------------------------------------*/

void outputCAN(ShifterPosition currentPosition) {
    // TODO
    return;
}
/*-----------------------------------------------------------*/

void outputPWM(ShifterPosition currentPosition) {
    // TODO
    return;
}
/*-----------------------------------------------------------*/

void outputInterlocks(ShifterPosition currentPosition) {
    // TODO
    return;
}
/*-----------------------------------------------------------*/

#define TO_HEX(i) (i <= 9 ? '0' + i : 'A' - 10 + i)
struct hexString numberToHexString(uint32_t num) {
	struct hexString result;
	result.str[10] = '\0';
	result.str[0] = '0';
	result.str[1] = 'x';

	result.str[2] = TO_HEX(((num & 0xF0000000) >>28));
	result.str[3] = TO_HEX(((num & 0xF000000) >>24));
	result.str[4] = TO_HEX(((num & 0xF00000) >>20));
	result.str[5] = TO_HEX(((num & 0xF0000) >>16));
	result.str[6] = TO_HEX(((num & 0xF000) >>12));
	result.str[7] = TO_HEX(((num & 0xF00) >>8));
	result.str[8] = TO_HEX(((num & 0xF0) >>4));
	result.str[9] = TO_HEX(((num & 0xF)));

	return result;
}