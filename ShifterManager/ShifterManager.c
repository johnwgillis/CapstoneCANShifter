/* ShifterManager.c : Capstone CAN Shifter
 * By John Gillis (jgillis@jgillis.com)
 */

#include "ShifterManager.h"

#include "FreeRTOS.h"
#include "task.h"

#include "LdcSensor.h"

/* The stack size used for the LDC Monitor task. */
#define LDC_MONITOR_STACK_SIZE          256
/* The period between executions of the LDC Monitor task. */
#define LDC_MONITOR_PERIOD				( ( TickType_t ) 100 / portTICK_PERIOD_MS  )

static volatile ShifterPosition currentShifterPosition = POS_UNKNOWN;
static volatile uint32_t dataReadsSinceLastCheck = 0; // Used for detecting stalls

static void vLDCMonit( void *pvParameters );
static LDC_Channel_Data vLDCMonit_ReadLDCData( void );
static ShifterPosition vLDCMonit_ProcessLDCData( LDC_Channel_Data data );

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
    // TODO

	/* Cycle for ever, delaying monitoring cycle to a set rate. */
	for( ;; ) {
		/* Read the LDC's to data */
        LDC_Channel_Data data = vLDCMonit_ReadLDCData();

        /* Recalculate current shifter postion */
        currentShifterPosition = vLDCMonit_ProcessLDCData(data);

        dataReadsSinceLastCheck++; // TODO: this should happen if valid data from LDC

        vTaskDelay( LDC_MONITOR_PERIOD );
	}
}
/*-----------------------------------------------------------*/

static LDC_Channel_Data vLDCMonit_ReadLDCData( void ) {
    LDC_Channel_Data result;

    // TODO
    // Zero out for now
    for(int i=0; i<8; i++) {
        result.data[i]=0;
    }

    return result;
}
/*-----------------------------------------------------------*/

static ShifterPosition vLDCMonit_ProcessLDCData( LDC_Channel_Data data ){
    // TODO
    return POS_UNKNOWN;
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