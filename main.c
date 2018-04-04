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
#include "I2C.h"
#include "serial.h"

/* Priority definitions for most of the tasks in the application.  Some
tasks just use the idle priority. */
#define mainCOM_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY			( tskIDLE_PRIORITY + 4 )

/* Baud rate used by the serial port tasks. */
#define mainCOM_BAUD_RATE				( ( unsigned long ) 38400 )

/* LED that is toggled by the check task.  The check task periodically checks
that all the other tasks are operating without error.  If no errors are found
the LED is toggled.  If an error is found at any time the LED is never toggles
again. */
#define mainCHECK_TASK_LED				( 5 )

/* The period between executions of the check task. */
#define mainCHECK_PERIOD				( ( TickType_t ) 3000 / portTICK_PERIOD_MS  )

// LDC 1 I2C Address
#define LDC_1_Addr 0x2A
// LDC 2 I2C Address
#define LDC_3_Addr 0x2C

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

/*-----------------------------------------------------------*/

int main( void ) {
	/* Setup I2C interface */
	SoftI2CInit();

	/* Setup the LED's for output. */
	vLedHelperInitialiseLED(mainCHECK_TASK_LED);
	//vLedHelperInitialiseLED(1);

	/* Setup the LDC sensor */
	vLdcSensorInitialise();

	/* Setup UART */
	serial_init(mainCOM_BAUD_RATE);

	// Print welcome on UART
	serial_transmit_string("\nCapstone CAN Shifter\n\0");

	#define Q_DEL _delay_us(4)

	// TEST I2C
	uint8_t ack;
	vTaskSuspendAll();
    {   
        // SoftI2CStart();
        // ack = SoftI2CWriteByte(0x58); // LDC address
        // //SoftI2CStop();

		DDRB &= ~(1 << SDA); Q_DEL; // Turn data on
		DDRB &= ~(1 << SCL); Q_DEL; // Turn clk on
		
		DDRB |= (1 << SDA); Q_DEL; // Turn data off
		

		// Sends 0
		DDRB |= (1 << SDA); Q_DEL; // Turn data off
		DDRB &= ~(1 << SCL); Q_DEL; // Turn clk on
		DDRB |= (1 << SCL); Q_DEL; // Turn clk off

		// Sends 1
		DDRB &= ~(1 << SDA); Q_DEL; // Turn data on
		DDRB &= ~(1 << SCL); Q_DEL; // Turn clk on
		DDRB |= (1 << SCL); Q_DEL; // Turn clk off

		// Sends 0
		DDRB |= (1 << SDA); Q_DEL; // Turn data off
		DDRB &= ~(1 << SCL); Q_DEL; // Turn clk on
		DDRB |= (1 << SCL); Q_DEL; // Turn clk off

		// Sends 1
		DDRB &= ~(1 << SDA); Q_DEL; // Turn data on
		DDRB &= ~(1 << SCL); Q_DEL; // Turn clk on
		DDRB |= (1 << SCL); Q_DEL; // Turn clk off

		// Sends 1
		DDRB &= ~(1 << SDA); Q_DEL; // Turn data on
		DDRB &= ~(1 << SCL); Q_DEL; // Turn clk on
		DDRB |= (1 << SCL); Q_DEL; // Turn clk off

		// Sends 0
		DDRB |= (1 << SDA); Q_DEL; // Turn data off
		DDRB &= ~(1 << SCL); Q_DEL; // Turn clk on
		DDRB |= (1 << SCL); Q_DEL; // Turn clk off

		// Sends 0
		DDRB |= (1 << SDA); Q_DEL; // Turn data off
		DDRB &= ~(1 << SCL); Q_DEL; // Turn clk on
		DDRB |= (1 << SCL); Q_DEL; // Turn clk off

		// Sends 0
		DDRB |= (1 << SDA); Q_DEL; // Turn data off
		DDRB &= ~(1 << SCL); Q_DEL; // Turn clk on
		DDRB |= (1 << SCL); Q_DEL; // Turn clk off


		// Check for ack (1 for NACK ; 0 for ACK)
		DDRB &= ~(1 << SDA); Q_DEL; // Turn data on
		DDRB &= ~(1 << SCL); Q_DEL; // Turn clk on

		ack = (SDAPIN & (1<<SDA));

		DDRB |= (1 << SCL); Q_DEL; // Turn clk off
    }
    xTaskResumeAll();
	if(ack == 0) {
		serial_transmit_string("\n\nTest Ack: 0\0");
	} else {
		serial_transmit_string("\n\nTest Ack: 1\0");
	}

	// Read the LDC 1 status
	// serial_transmit_string("\n\nLDC 1 Status: \0");
	// uint16_t ldcStatus = vLdcSensorReadStatus(LDC_3_Addr);
	// char ldcStatusString[17];
	// ldcStatusString[16] = '\0';
	// for (int i = 15; i >= 0; i--, ldcStatus>>=1) { ldcStatusString[i] = (ldcStatus & 1) + '0'; }
	// serial_transmit_string(ldcStatusString);
	// serial_transmit_string("\n\0");

	// // Read the LDC 1 Channel 3
	// serial_transmit_string("\n\nLDC 1 Channel 3: \0");
	// uint32_t ldcChannel = vLdcSensorReadChannel(LDC_3_Addr, 3);
	// char ldcChannelString[33];
	// ldcChannelString[32] = '\0';
	// for (int i = 31; i >= 0; i--, ldcChannel>>=1) { ldcChannelString[i] = (ldcChannel & 1) + '0'; }
	// serial_transmit_string(ldcChannelString);
	// serial_transmit_string("\n\0");

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

void vApplicationIdleHook( void ) {
	//vCoRoutineSchedule();
}
