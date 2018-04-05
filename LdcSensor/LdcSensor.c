/* LdcSensor.c : Capstone CAN Shifter
 * By John Gillis (jgillis@jgillis.com)
 */

#include "FreeRTOS.h"
#include "task.h"
#include "LdcSensor.h"
#include "I2C.h"

/*-----------------------------------------------------------*/

void vLdcSensorInitialiseConfig( uint8_t deviceAddress );
void vLdcSensorWriteRegister( uint8_t deviceAddress, uint8_t registerAddress, uint16_t data );

// LDC Register Config
#define CONFIG_REG 0x1A
#define CONFIG_VAL_SLEEP 0xE681
#define CONFIG_VAL_WAKE 0xC681

#define MUX_CONFIG_REG 0x1B
#define MUX_CONFIG_VAL 0xC20F

#define ERROR_CONFIG_REG 0x19
#define ERROR_CONFIG_VAL 0xF800

#define CLOCK_DIVIDERS0_REG 0x14
#define CLOCK_DIVIDERS0_VAL 0x1001

#define CLOCK_DIVIDERS1_REG 0x15
#define CLOCK_DIVIDERS1_VAL CLOCK_DIVIDERS0_VAL

#define CLOCK_DIVIDERS2_REG 0x16
#define CLOCK_DIVIDERS2_VAL CLOCK_DIVIDERS0_VAL

#define CLOCK_DIVIDERS3_REG 0x17
#define CLOCK_DIVIDERS3_VAL CLOCK_DIVIDERS0_VAL

/*-----------------------------------------------------------*/

void vLdcSensorInitialise( void ) {
    /* Setup I2C interface */
	softwareWire_init(3, 4); // SDA = PortD.3  and  SCL = PortD.4
	begin();

    // Config the LDC's correctly
    vLdcSensorInitialiseConfig(LDC_1_Addr);
    //vLdcSensorInitialiseConfig(LDC_2_Addr);
}
/*-----------------------------------------------------------*/

void vLdcSensorInitialiseConfig( uint8_t deviceAddress ) {

    // Write the LDC config registers in sleep mode
	vTaskSuspendAll();
    {   
        vLdcSensorWriteRegister(deviceAddress, CONFIG_REG, CONFIG_VAL_SLEEP); // Write CONFIG (put to sleep)

        // Write all the config registers
        vLdcSensorWriteRegister(deviceAddress, MUX_CONFIG_REG, MUX_CONFIG_VAL); // Write MUX_CONFIG
        vLdcSensorWriteRegister(deviceAddress, ERROR_CONFIG_REG, ERROR_CONFIG_VAL); // Write ERROR_CONFIG

        vLdcSensorWriteRegister(deviceAddress, CLOCK_DIVIDERS0_REG, CLOCK_DIVIDERS0_VAL); // Write CLOCK_DIVIDERS0
        vLdcSensorWriteRegister(deviceAddress, CLOCK_DIVIDERS1_REG, CLOCK_DIVIDERS1_VAL); // Write CLOCK_DIVIDERS1
        vLdcSensorWriteRegister(deviceAddress, CLOCK_DIVIDERS2_REG, CLOCK_DIVIDERS2_VAL); // Write CLOCK_DIVIDERS2
        vLdcSensorWriteRegister(deviceAddress, CLOCK_DIVIDERS3_REG, CLOCK_DIVIDERS3_VAL); // Write CLOCK_DIVIDERS3


        vLdcSensorWriteRegister(deviceAddress, CONFIG_REG, CONFIG_VAL_WAKE); // Write CONFIG (wake from sleep)
    }
    xTaskResumeAll();
}
/*-----------------------------------------------------------*/

void vLdcSensorWriteRegister( uint8_t deviceAddress, uint8_t registerAddress, uint16_t data ) {

    // Write the LDC register
	vTaskSuspendAll();
    {   
        beginTransmission(deviceAddress);
        write(registerAddress);
        write((uint8_t)(data << 8)); // Send MSB
        write((uint8_t)(data)); // Send LSB
        endTransmission(pdTRUE); // send stop
    }
    xTaskResumeAll();
}
/*-----------------------------------------------------------*/

uint16_t vLdcSensorReadRegister( uint8_t deviceAddress, uint8_t registerAddress ) {
    uint16_t result = 0;

    uint16_t MSB_temp_result = 0;
    uint16_t LSB_temp_result = 0;

    // Read the LDC status
	vTaskSuspendAll();
    {   
        beginTransmission(deviceAddress);
        write(registerAddress);
        endTransmission(pdFALSE); // don't send stop
        requestFrom(deviceAddress, 2, pdTRUE); // send stop
    }
    xTaskResumeAll();

    // Read out of I2C buffer
    MSB_temp_result = read();
    LSB_temp_result = read();
    MSB_temp_result = (MSB_temp_result != (uint16_t)(-1)) ? MSB_temp_result : 0x00;
    LSB_temp_result = (LSB_temp_result != (uint16_t)(-1)) ? LSB_temp_result : 0x00;

    // Shift the status data into the final result
    result |= ( MSB_temp_result << 8);
    result |= ( LSB_temp_result << 0);

    return result;
}
/*-----------------------------------------------------------*/

// Reads the given channel of the LDC and returns the full reading
uint32_t vLdcSensorReadChannel( uint8_t deviceAddress, uint8_t channel ) {
    uint32_t result = 0;

    uint32_t MSB_temp_result = 0;
    uint32_t LSB_temp_result = 0;

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
        beginTransmission(deviceAddress);
        write(channelMSB);
        endTransmission(pdFALSE); // don't send stop
        requestFrom(deviceAddress, 2, pdTRUE); // send stop
    }
    xTaskResumeAll();

    // Read out of I2C buffer
    MSB_temp_result = read();
    LSB_temp_result = read();
    MSB_temp_result = (MSB_temp_result != (uint32_t)(-1)) ? MSB_temp_result : 0x00;
    LSB_temp_result = (LSB_temp_result != (uint32_t)(-1)) ? LSB_temp_result : 0x00;

    // Shift the MSB data into the final result
    result |= ( MSB_temp_result << 24);
    result |= ( LSB_temp_result << 16);

    // Read the channel's LSB
	vTaskSuspendAll();
    {   
        beginTransmission(deviceAddress);
        write(channelLSB);
        endTransmission(pdFALSE); // don't send stop
        requestFrom(deviceAddress, 2, pdTRUE); // send stop
    }
    xTaskResumeAll();

    // Read out of I2C buffer
    MSB_temp_result = read();
    LSB_temp_result = read();
    MSB_temp_result = (MSB_temp_result != (uint32_t)(-1)) ? MSB_temp_result : 0x00;
    LSB_temp_result = (LSB_temp_result != (uint32_t)(-1)) ? LSB_temp_result : 0x00;

    // Shift the LSB data into the final result
    result |= ( MSB_temp_result << 8);
    result |= ( LSB_temp_result << 0);

    return result;
}
/*-----------------------------------------------------------*/

uint16_t vLdcSensorReadStatus( uint8_t deviceAddress ) {
    uint16_t result = 0;

    uint16_t MSB_temp_result = 0;
    uint16_t LSB_temp_result = 0;

    // Read the LDC status
	vTaskSuspendAll();
    {   
        beginTransmission(deviceAddress);
        write(0x18); // Register for LDC status from datasheet
        endTransmission(pdFALSE); // don't send stop
        requestFrom(deviceAddress, 2, pdTRUE); // send stop
    }
    xTaskResumeAll();

    // Read out of I2C buffer
    MSB_temp_result = read();
    LSB_temp_result = read();
    MSB_temp_result = (MSB_temp_result != (uint16_t)(-1)) ? MSB_temp_result : 0x00;
    LSB_temp_result = (LSB_temp_result != (uint16_t)(-1)) ? LSB_temp_result : 0x00;

    // Shift the status data into the final result
    result |= ( MSB_temp_result << 8);
    result |= ( LSB_temp_result << 0);

    return result;
}
/*-----------------------------------------------------------*/

uint16_t vLdcSensorReadDeviceId( uint8_t deviceAddress ) {
    uint16_t result = 0;

    uint16_t MSB_temp_result = 0;
    uint16_t LSB_temp_result = 0;

    // Read the LDC status
	vTaskSuspendAll();
    {   
        beginTransmission(deviceAddress);
        write(0x7F); // Register for LDC device id from datasheet
        endTransmission(pdFALSE); // don't send stop
        requestFrom(deviceAddress, 2, pdTRUE); // send stop
    }
    xTaskResumeAll();

    // Read out of I2C buffer
    MSB_temp_result = read();
    LSB_temp_result = read();
    MSB_temp_result = (MSB_temp_result != (uint16_t)(-1)) ? MSB_temp_result : 0x00;
    LSB_temp_result = (LSB_temp_result != (uint16_t)(-1)) ? LSB_temp_result : 0x00;

    // Shift the status data into the final result
    result |= ( MSB_temp_result << 8);
    result |= ( LSB_temp_result << 0);

    return result;
}