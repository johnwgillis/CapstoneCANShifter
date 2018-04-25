/* Main.c : Capstone CAN Shifter
 * By John Gillis (jgillis@jgillis.com)
 *
 * Creates all the demo application tasks, then starts the scheduler. 
 * Main. c handles the setup and management of each task.
 *
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include <util/delay.h>


#include <stdlib.h>
#include <string.h>

#include <avr/eeprom.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

/* Application file headers. */
#include "LedHelper.h"
#include "LdcSensor.h"
#include "ShifterManager.h"
#include "I2C.h"
#include "serial.h"

/* Priority definitions for most of the tasks in the application.  Some
tasks just use the idle priority. */
#define mainCOM_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY			( tskIDLE_PRIORITY + 3 )
#define mainShifterManager_PRIORITY		( tskIDLE_PRIORITY + 4 )

/* Baud rate used by the serial port tasks. */
#define mainCOM_BAUD_RATE				( ( unsigned long ) 38400 )

/* LED that is toggled by the check task.  The check task periodically checks
that all the other tasks are operating without error.  If no errors are found
the LED is toggled.  If an error is found at any time the LED is never toggles
again. */
#define mainCHECK_TASK_LED				( 5 )

/* The period between executions of the check task. */
#define mainCHECK_PERIOD				( ( TickType_t ) 3000 / portTICK_PERIOD_MS  )

/*
 * The task function for the "Check" task.
 */
static void vErrorChecks( void *pvParameters );

/*
 * Checks the unique counts of other tasks to ensure they are still operational.
 * Flashes an LED if everything is okay.
 */
static void prvCheckOtherTasksAreStillRunning( void );

// Application Idle Hook for FreeRTOS
void vApplicationIdleHook( void );

void printCurrentShifterPosition( void );
void printLDC1Register( char* registerName, uint8_t registerAddress );
void printLDC1FullDebug( void );

/*-----------------------------------------------------------*/

int main( void ) {
	/* Setup the LED's for output. */
	vLedHelperInitialiseLED(mainCHECK_TASK_LED);

	/* Setup UART */
	serial_init(mainCOM_BAUD_RATE);

	// Print welcome on UART
	serial_transmit_string("\nCapstone CAN Shifter\n\0");

	/* Start Shifter Manager Tasks (including LDC sensor setup and monitor) */
	vShifterManagerStartTasks(mainShifterManager_PRIORITY);

	/* Create the tasks defined within this file. */
	unsigned short additionalStackSize = 128; // Since printing a lot (temp until full calculation of bytes used)
	xTaskCreate( vErrorChecks, "Check", configMINIMAL_STACK_SIZE+additionalStackSize, NULL, mainCHECK_TASK_PRIORITY, NULL );

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

	if( xAreShifterManagerTasksStillRunning() != pdTRUE ) {
       xErrorHasOccurred = pdTRUE;
	}

	if( xErrorHasOccurred == pdFALSE ) {
		/* Toggle the LED if everything is okay so we know if an error occurs even if not
		using console IO. */
		vLedHelperToggleLED( mainCHECK_TASK_LED );
	}

	serial_transmit_string("\n\n\0");
	printLDC1FullDebug();
	printCurrentShifterPosition();
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void ) {
	//vCoRoutineSchedule();
}
/*-----------------------------------------------------------*/

void printCurrentShifterPosition( void ) {
	serial_transmit_string("Current Shifter Position: \0");
	ShifterPosition currentPosition = vShifterManagerGetCurrentPosition();
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

// To debug LDC register config
void printLDC1Register( char* registerName, uint8_t registerAddress ) {
	serial_transmit_string("LDC 1 Register: \0");

	// Print reg name
	serial_transmit_string(registerName);
	serial_transmit_string(" = \0");

	// Print reg value
	uint16_t ldcRegVal = vLdcSensorReadRegister(LDC_1_Addr, registerAddress);
	serial_transmit_string(numberToHexString(ldcRegVal).str);
	serial_transmit_string("\n\0");
}
/*-----------------------------------------------------------*/

// To debug LDC 1
void printLDC1FullDebug( void ) {
	// Read the LDC 1 Channel 0
	serial_transmit_string("LDC 1 Channel 0 Data: \0");
	uint32_t ldcData = vLdcSensorReadChannel(LDC_1_Addr, 0);
	serial_transmit_string(numberToHexString(ldcData).str);
	serial_transmit_string("\n\0");

	// // Read the LDC 1 Channel 1
	// serial_transmit_string("LDC 1 Channel 1 Data: \0");
	// ldcData = vLdcSensorReadChannel(LDC_1_Addr, 1);
	// serial_transmit_string(numberToHexString(ldcData).str);
	// serial_transmit_string("\n\0");

	// // Read the LDC 1 Channel 2
	// serial_transmit_string("LDC 1 Channel 2 Data: \0");
	// ldcData = vLdcSensorReadChannel(LDC_1_Addr, 2);
	// serial_transmit_string(numberToHexString(ldcData).str);
	// serial_transmit_string("\n\0");

	// // Read the LDC 1 Channel 3
	// serial_transmit_string("LDC 1 Channel 3 Data: \0");
	// ldcData = vLdcSensorReadChannel(LDC_1_Addr, 3);
	// serial_transmit_string(numberToHexString(ldcData).str);
	// serial_transmit_string("\n\0");

	// Print register(s) to debug
	printLDC1Register("Device Id\0", 0x7F); // Device Id
	printLDC1Register("Status\0", 0x18); // Status
	printLDC1Register("Config\0", 0x1A); // Config
	printLDC1Register("ERROR_CONFIG\0", 0x19); // ERROR_CONFIG
}
/*-----------------------------------------------------------*/