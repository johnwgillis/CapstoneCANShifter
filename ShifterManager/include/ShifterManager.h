/* ShifterManager.h : Capstone CAN Shifter
 * By John Gillis (jgillis@jgillis.com)
 */

#ifndef SHIFTERMANAGER_H
#define SHIFTERMANAGER_H

#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"

typedef struct LDC_Channel_Data {
    uint32_t data[8];
} LDC_Channel_Data;
typedef enum ShifterPosition{POS_1, POS_2, POS_3, POS_4, POS_5, POS_6, POS_7, POS_UNKNOWN} ShifterPosition;

void vShifterManagerStartTasks( UBaseType_t uxPriority );

ShifterPosition vShifterManagerGetCurrentPosition(void);

BaseType_t xAreShifterManagerTasksStillRunning( void );

#endif