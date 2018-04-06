/* ShifterManager.h : Capstone CAN Shifter
 * By John Gillis (jgillis@jgillis.com)
 */

#ifndef SHIFTERMANAGER_H
#define SHIFTERMANAGER_H

#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"

typedef enum ShifterPosition{POS_1=0, POS_2=1, POS_3=2, POS_4=3, POS_5=4, POS_6=5, POS_7=6, POS_UNKNOWN=-1} ShifterPosition;

void vShifterManagerStartTasks( UBaseType_t uxPriority );

ShifterPosition vShifterManagerGetCurrentPosition(void);

BaseType_t xAreShifterManagerTasksStillRunning( void );

#endif