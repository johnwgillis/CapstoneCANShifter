/* ShifterManager.c : Capstone CAN Shifter
 * By John Gillis (jgillis@jgillis.com)
 */

#include "ShifterManager.h"

#include "FreeRTOS.h"
#include "task.h"

#include "serial.h"
#include "LdcSensor.h"

#ifndef OUTPUT_MODE_SERIAL_NOT_PWM
#define OUTPUT_MODE_SERIAL_NOT_PWM 1	// Default value: 1 (should be configured in main)
#endif

/* The stack size used for the LDC Monitor task. */
#define LDC_MONITOR_STACK_SIZE          1024
/* The period between executions of the LDC Monitor task. */
#define LDC_MONITOR_PERIOD				( ( TickType_t ) 100 / portTICK_PERIOD_MS  )
/* The initial wait after configuring the LDCs to allow them to stablize. */
#define LDC_MONITOR_INITIAL_WAIT		( ( TickType_t ) 500 / portTICK_PERIOD_MS  )

#define MAX_CAN_MESSAGE_LENGTH 200

/* Hardware Config */
// 					Interlock #:	    1		2		3		4
// 					Pin:		        PD.7	PB.2	PC.4	PC.5
volatile uint8_t* interlockDDR[5]  = {	&DDRD, 	&DDRB, 	&DDRC, 	&DDRC   };
volatile uint8_t* interlockPORT[5] = {	&PORTD, &PORTB, &PORTC, &PORTC  };
		 uint8_t  interlockBit[5]  = {	7, 		2, 		4, 		5       };


/* Input Config */
//                          POS:        1        2        3        4        5        6        7        8
uint32_t configPositionUpper[8] = {     0xDD000, 0xDCAFF, 0xDC7AA, 0xDF1FF, 0xE3CFF, 0xE58FF, 20,      40    }; // Position 7 and 8 aren't used
uint32_t configPositionLower[8] = {     0xDCA01, 0xDC800, 0xDC400, 0xDE300, 0xE3200, 0xE4D00, 10,      30    }; // Position 7 and 8 aren't used

/* Output Config */
//                          POS:   1          2          3          4          5          6          7          8          Unknown
uint8_t configPWM[9]           = { 75,        100,       125,       150,       175,       200,       225,       250,       10          }; // from 0 to 255
uint8_t configInterlocks[9][4] = { {0,0,0,1}, {0,0,1,0}, {0,0,1,1}, {0,1,0,0}, {0,1,0,1}, {0,1,1,0}, {0,1,1,1}, {1,0,0,0}, {1,0,0,1}   }; // 0 or 1
uint8_t configCANAddress = 0x00;
char configCANMessages[9][MAX_CAN_MESSAGE_LENGTH] = {
    "Position_1", // POS: 1
    "Position_2", // POS: 2
    "Position_3", // POS: 3
    "Position_4", // POS: 4
    "Position_5", // POS: 5
    "Position_6", // POS: 6
    "Position_7", // POS: 7
    "Position_8", // POS: 8
    "Position_Unknown" // POS: Unknown
};


static volatile ShifterPosition currentShifterPosition = POS_UNKNOWN;
static volatile ShifterPosition lastShifterPosition = POS_UNKNOWN;
static volatile uint32_t dataReadsSinceLastCheck = 0; // Used for detecting stalls

static void vLDCMonit( void *pvParameters );

void vShifterManagerInitOutput( void );
void vShifterManagerOutputPosition(ShifterPosition position);

void outputSerial(ShifterPosition currentPosition);
void outputCAN(ShifterPosition currentPosition);
void outputPWM(ShifterPosition currentPosition);
void outputInterlocks(ShifterPosition currentPosition);

/*-----------------------------------------------------------*/

void vShifterManagerStartTasks( UBaseType_t uxPriority ) {
	/* Initialise the LDC's config */
    vLdcSensorInitialise();

    /* Initialise the outputs */
    vShifterManagerInitOutput();

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

        // Mask out the error status bits (first 4 bits)
        data[0] &= ~(0xF0000000); 

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

        // Check for LDC errors
        if( vLdcSensorGetErrorStaus() == pdFALSE ) {
            dataReadsSinceLastCheck++; // this should happen if valid data from LDC
        }

        vTaskDelay( LDC_MONITOR_PERIOD );
	}
}
/*-----------------------------------------------------------*/

void vShifterManagerInitOutput( void ) {

    // Init Interlocks with all off
    unsigned char ucBit = 1;
    for(int i=0; i<4; i++) {
        ucBit = (1 << interlockBit[i]);
        *(interlockDDR[i]) |= ucBit;
        *(interlockPORT[i]) &= ~ucBit;
    }

    // Init PWM by setting up intial timer with Position Unknown
    // Uses PD.3 for Timer 0 A (which is shared with UART Tx)
#if OUTPUT_MODE_SERIAL_NOT_PWM == 0
    DDRD |= (1 << 3); // Set PD.3 to output
    TCCR0A  = ((1 << COM1A1) | (1 << COM1A0)); // Set OC0A on compare match, clear OC0A at TOP
    TCCR0B = (1 << WGM00) | (1 << WGM01) | (1 << CS00) | (1 << CS02); // Fast PWM with 1/1024 prescaler
#endif

    return;
}
/*-----------------------------------------------------------*/

void vShifterManagerOutputPosition(ShifterPosition position) {

    outputCAN(position);

#if OUTPUT_MODE_SERIAL_NOT_PWM
    outputSerial(position);
#else
    outputPWM(position);
#endif

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
        case POS_8:
			serial_transmit_string("8\n\0");
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
#if OUTPUT_MODE_SERIAL_NOT_PWM == 0
    OCR0B = configPWM[currentPosition]; // Set the PWM duty
#endif
    return;
}
/*-----------------------------------------------------------*/

void outputInterlocks(ShifterPosition currentPosition) {
    unsigned char ucBit = 1;
    for(int i=0; i<4; i++) {
        ucBit = (1 << interlockBit[i]);
        *(interlockDDR[i]) |= ucBit;
        if(configInterlocks[currentPosition][i]) {
            *(interlockPORT[i]) |= ucBit;
        } else {
            *(interlockPORT[i]) &= ~ucBit;
        }
    }
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