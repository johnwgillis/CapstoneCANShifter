/* LdcSensor.c : Capstone CAN Shifter
 * By John Gillis (jgillis@jgillis.com)
 */

#include "FreeRTOS.h"
#include "task.h"
#include "LdcSensor.h"
#include "I2C.h"

/*-----------------------------------------------------------*/

void vLdcSensorInitialise( void ) {}
/*-----------------------------------------------------------*/

// Reads the given channel of the LDC and returns the full reading
uint32_t vLdcSensorReadChannel( uint8_t deviceAddress, uint8_t channel ) {
    uint32_t result = 0;

    uint8_t MSB_temp_result = 0;
    uint8_t LSB_temp_result = 0;

    uint8_t channelMSB = 0x00;
    uint8_t channelLSB = 0x00;

    // Assign register addresses based on channel from LDC 1614 datasheet
    switch(channel) {
        case 0:
            channelMSB = 0x00;
            channelLSB = 0x01;
            break;
        case 1:
            channelMSB = 0x02;
            channelLSB = 0x03;
            break;
        case 2:
            channelMSB = 0x04;
            channelLSB = 0x05;
            break;
        case 3:
            channelMSB = 0x06;
            channelLSB = 0x07;
            break;
        default:
            return 0; // error out since channel out of range
    }
    
    // Read the channel's MSB
	vTaskSuspendAll();
    {   
        SoftI2CStart();
        SoftI2CWriteByte(deviceAddress); // LDC address
        SoftI2CWriteByte(channelMSB); // Register for channel's MSB
        MSB_temp_result = SoftI2CReadByte(1); // Read with ACK the MSB
        LSB_temp_result = SoftI2CReadByte(0); // Read with NACK the LSB
        SoftI2CStop();
    }
    xTaskResumeAll();

    // Shift the MSB data into the final result
    result |= ( MSB_temp_result << 24);
    result |= ( LSB_temp_result << 16);

    // Read the channel's LSB
	vTaskSuspendAll();
    {   
        SoftI2CStart();
        SoftI2CWriteByte(deviceAddress); // LDC address
        SoftI2CWriteByte(channelLSB); // Register for channel's LSB
        MSB_temp_result = SoftI2CReadByte(1); // Read with ACK the MSB
        LSB_temp_result = SoftI2CReadByte(0); // Read with NACK the LSB
        SoftI2CStop();
    }
    xTaskResumeAll();

    // Shift the LSB data into the final result
    result |= ( MSB_temp_result << 8);
    result |= ( LSB_temp_result << 0);

    return result;
}
/*-----------------------------------------------------------*/

uint16_t vLdcSensorReadStatus( uint8_t deviceAddress ) {
    uint16_t result = 0;

    uint8_t MSB_temp_result = 0;
    uint8_t LSB_temp_result = 0;

    // Read the LDC status
	vTaskSuspendAll();
    {   
        SoftI2CStart();
        SoftI2CWriteByte(deviceAddress); // LDC address
        SoftI2CWriteByte(0x18); // Register for LDC status from datasheet
        MSB_temp_result = SoftI2CReadByte(1); // Read with ACK the MSB
        LSB_temp_result = SoftI2CReadByte(0); // Read with NACK the LSB
        SoftI2CStop();
    }
    xTaskResumeAll();

    // Shift the LSB data into the final result
    result |= ( MSB_temp_result << 8);
    result |= ( LSB_temp_result << 0);

    return result;
}