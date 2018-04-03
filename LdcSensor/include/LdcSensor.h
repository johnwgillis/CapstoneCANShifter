/* LdcSensor.h : Capstone CAN Shifter
 * By John Gillis (jgillis@jgillis.com)
 */

#ifndef LDCSENSOR_H
#define LDCSENSOR_H

void vLdcSensorInitialise( void );

uint32_t vLdcSensorReadChannel( uint8_t deviceAddress, uint8_t channel );

uint16_t vLdcSensorReadStatus( uint8_t deviceAddress );

#endif