/* ShifterManager.c : Capstone CAN Shifter
 * By John Gillis (jgillis@jgillis.com)
 */

#include "ShifterManager.h"

#include "FreeRTOS.h"
#include "task.h"

#include "LdcSensor.h"

/* The stack size used for the LDC Monitor task. */
#define LDC_MONITOR_STACK_SIZE          1024
/* The period between executions of the LDC Monitor task. */
#define LDC_MONITOR_PERIOD				( ( TickType_t ) 100 / portTICK_PERIOD_MS  )

/* The initial wait after configuring the LDCs to allow them to stablize. */
#define LDC_MONITOR_INITIAL_WAIT		( ( TickType_t ) 500 / portTICK_PERIOD_MS  )

static volatile ShifterPosition currentShifterPosition = POS_1;
static volatile uint32_t dataReadsSinceLastCheck = 0; // Used for detecting stalls

static void vLDCMonit( void *pvParameters );

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
        data[2] = vLdcSensorReadChannel(LDC_1_Addr, 1);
        //data[4] = vLdcSensorReadChannel(LDC_1_Addr, 2); // Disabled due to hardware issue
        data[6] = vLdcSensorReadChannel(LDC_1_Addr, 3);

        int maxIndex = -1;
        uint32_t maxValue = 0;
        for(int i=0; i<8; i++) {
            // Mask out the error status bits (first 4 bits)
            data[i] &= ~(0xF0000000);
            if(data[i] > maxValue) {
                maxIndex = i;
                maxValue = data[i];
            }
        }
        currentShifterPosition = maxIndex;

        dataReadsSinceLastCheck++; // TODO: this should happen if valid data from LDC

        vTaskDelay( LDC_MONITOR_PERIOD );
	}
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