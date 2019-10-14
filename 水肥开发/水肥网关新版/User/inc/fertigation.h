#ifndef __FERTIGATION_H
#define __FERTIGATION_H

#include "stm32f10x.h"
#include "Stm32_Configuration.h"
#include "HandleTask.h"
#include "systemclock.h"
#include "string.h"
#include "DataFlash.h"
#include "Stm32_Configuration.h"
#include <stdlib.h>
#include "stm32f10x_it.h"




#define MAX_SFJ_I 12
#define SF_SlaveID_0 37
#define SF_SlaveID_1 38              //定义两个控制器子站用于控制阀门
#define SF_Para_Tran_time 500



void SF_Para_Trans(void);
void SF_Flow_Measu(void);
void SF_Flow_Trans(void);
void SF_WriteDataToDMA_BufferTX3(uint16_t size);

#endif 


