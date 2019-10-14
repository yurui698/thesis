#ifndef __MAX44009_H__
#define __MAX44009_H__

#include "stm32f10x.h"
#include "Stm32_Configuration.h"
#include "junglesys.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*Slave Address*/
#define MAX44009_WRITE              0x94
#define MAX44009_READ				0x95

/*Register Address*/
#define INTERRUPT_STATUS            0x00
#define INTERRUPT_ENABLE            0x01
#define CONFIGURATION	            0x02
#define LUX_HIGH_BYTE	            0x03
#define LUX_LOW_BYTE                0x04
#define UPPER_THRESHOLD_HIGH_BYTE   0x05
#define LOWER_THRESHOLD_HIGH_BYTE   0x06
#define THRESHOLD_TIMER             0x07

#define TIMEOUT       5
#define LIGHT_SCK_H   PCout(3)=1
#define LIGHT_SCK_L   PCout(3)=0

#define LIGHT_DTA_H   PCout(0)=1
#define LIGHT_DTA_L   PCout(0)=0

#define LIGHT_DTA_R   {GPIOC->CRL &= 0XFFFFFFF0;GPIOC->CRL |= 8<<0;}
#define LIGHT_DTA_W   {GPIOC->CRL &= 0XFFFFFFF0;GPIOC->CRL |= 3<<0;}

#define LIGHT_DTA     PCin(0)

extern void MAX44009_Transtart(void);
extern void MAX44009_Transtop(void);
extern u8  	MAX44009_Send(u8 val);
extern u8  	MAX44009_Read(u8 ack);
extern void MAX44009_INIT(void);
extern u16 	get_light(void);
extern u16 	Get_Illuminance(void);//光照度
extern u16  GET_PRESSUE0(void);//大气压力
extern u16  PRESSUE_level2(void);//LWP5050
extern u16  GET_level0(void);//液位压力
extern u16  level2_temperature;
#ifdef __cplusplus
}
#endif

#endif
