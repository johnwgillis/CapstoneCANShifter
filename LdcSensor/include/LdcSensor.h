/* LdcSensor.h : Capstone CAN Shifter
 * By John Gillis (jgillis@jgillis.com)
 */

#ifndef LDCSENSOR_H
#define LDCSENSOR_H

// LDC 1 I2C Address
#define LDC_1_Addr 0x2A
// LDC 2 I2C Address
#define LDC_2_Addr 0x2B

void vLdcSensorInitialise( void );

void vLdcSensorWriteRegister( uint8_t deviceAddress, uint8_t registerAddress, uint16_t data );
uint16_t vLdcSensorReadRegister( uint8_t deviceAddress, uint8_t registerAddress );

uint32_t vLdcSensorReadChannel( uint8_t deviceAddress, uint8_t channel );
uint16_t vLdcSensorReadStatus( uint8_t deviceAddress );
uint16_t vLdcSensorReadDeviceId( uint8_t deviceAddress );

#endif