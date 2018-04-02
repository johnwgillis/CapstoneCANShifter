/* comtest.h : Capstone CAN Shifter
 * By John Gillis (jgillis@jgillis.com)
 */

#ifndef COMTEST_H
#define COMTEST_H

void vAltStartComTestTasks( UBaseType_t uxPriority, uint32_t ulBaudRate, UBaseType_t uxLED );
void vStartComTestTasks( UBaseType_t uxPriority, eCOMPort ePort, eBaud eBaudRate );
BaseType_t xAreComTestTasksStillRunning( void );
void vComTestUnsuspendTask( void );

#endif

