/* Main.c : Capstone CAN Shifter
 * By John Gillis (jgillis@jgillis.com)
 *
 * Creates all the demo application tasks, then starts the scheduler. 
 * Main. c handles the setup and management of each task.
 *
 */

#include <stdlib.h>
#include <string.h>

#include <avr/eeprom.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

/* Application file headers. */
#include "LedHelper.h"
#include "LdcSensor.h"
// #include "serial.h"
// #include "comtest.h"
// #include "print.h"

/* Priority definitions for most of the tasks in the application.  Some
tasks just use the idle priority. */
#define mainCOM_TEST_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY			( tskIDLE_PRIORITY + 4 )

/* Baud rate used by the serial port tasks. */
#define mainCOM_TEST_BAUD_RATE			( ( unsigned long ) 38400 )

/* LED used by the serial port tasks.  This is toggled on each character Tx,
and mainCOM_TEST_LED + 1 is toggles on each character Rx. */
#define mainCOM_TEST_LED				( 3 )

/* LED that is toggled by the check task.  The check task periodically checks
that all the other tasks are operating without error.  If no errors are found
the LED is toggled.  If an error is found at any time the LED is never toggles
again. */
#define mainCHECK_TASK_LED				( 7 )

/* The period between executions of the check task. */
#define mainCHECK_PERIOD				( ( TickType_t ) 3000 / portTICK_PERIOD_MS  )

/* An address in the EEPROM used to count resets.  This is used to check that
the application is not unexpectedly resetting. */
#define mainRESET_COUNT_ADDRESS			( ( void * ) 0x50 )

/*
 * The task function for the "Check" task.
 */
static void vErrorChecks( void *pvParameters );

/*
 * Checks the unique counts of other tasks to ensure they are still operational.
 * Flashes an LED if everything is okay.
 */
static void prvCheckOtherTasksAreStillRunning( void );

/*
 * Called on boot to increment a count stored in the EEPROM.  This is used to
 * ensure the CPU does not reset unexpectedly.
 */
static void prvIncrementResetCount( void );

// Application Idle Hook for FreeRTOS
void vApplicationIdleHook( void );

/*-----------------------------------------------------------*/

int main( void ) {
	//prvIncrementResetCount();

	/* Setup the LED's for output. */
	vLedHelperInitialise();

	/* Create the standard demo tasks. */
	//vAltStartComTestTasks( mainCOM_TEST_PRIORITY, mainCOM_TEST_BAUD_RATE, mainCOM_TEST_LED );

	/* Create the tasks defined within this file. */
	xTaskCreate( vErrorChecks, "Check", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL );

	/* In this port, to use preemptive scheduler define configUSE_PREEMPTION
	as 1 in portmacro.h.  To use the cooperative scheduler define
	configUSE_PREEMPTION as 0. */
	vTaskStartScheduler();

	return 0;
}
/*-----------------------------------------------------------*/

static void vErrorChecks( void *pvParameters ) {
	/* The parameters are not used. */
	( void ) pvParameters;

	/* Cycle for ever, delaying then checking all the other tasks are still
	operating without error. */
	for( ;; ) {
		vTaskDelay( mainCHECK_PERIOD );

		prvCheckOtherTasksAreStillRunning();
	}
}
/*-----------------------------------------------------------*/

static void prvCheckOtherTasksAreStillRunning( void ) {
	static portBASE_TYPE xErrorHasOccurred = pdFALSE;

	// if( xAreComTestTasksStillRunning() != pdTRUE ) {
    //    xErrorHasOccurred = pdTRUE;
	// }

	if( xErrorHasOccurred == pdFALSE ) {
		/* Toggle the LED if everything is okay so we know if an error occurs even if not
		using console IO. */
		vLedHelperToggleLED( mainCHECK_TASK_LED );
	}
}
/*-----------------------------------------------------------*/

static void prvIncrementResetCount( void ) {
	unsigned char ucCount;

	eeprom_read_block( &ucCount, mainRESET_COUNT_ADDRESS, sizeof( ucCount ) );
	ucCount++;
	eeprom_write_byte( mainRESET_COUNT_ADDRESS, ucCount );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void ) {
	//vCoRoutineSchedule();
}
