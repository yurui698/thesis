#ifndef __SI7021_H
#define __SI7021_H

#include "stm32f10x.h"
#include "Stm32_Configuration.h"
#include "junglesys.h"

/* Si7021 CMD Code */
#define Measure_RH_M      0xE5
#define Measure_RH_NM     0xF5
#define Measure_T_M       0xE3
#define Measure_T_NM      0xF3
#define Read_Temp_RH      0xE0
#define Reset             0xFE
#define Write_RH_T_REG    0xE6
#define Read_RH_T_REG     0xE7
#define Read_ID_1_1       0xFA
#define Read_ID_1_2       0x0F
#define Read_ID_2_1       0xFC
#define Read_ID_2_2       0xC9
#define Read_Rev_1        0x84
#define Read_Rev_2        0xB8

/* ID Register */
#define ID_SI7021         0x15

#define WAKE_UP_TIME      15
#define WAIT_TIME         5
#define SI7021_ADR        0x40

/* Coefficients */
#define TEMPERATURE_OFFSET    46.85
#define TEMPERATURE_MULTIPLE  175.72
#define TEMPERATURE_SLOPE     65536

#define HUMIDITY_OFFSET       6
#define HUMIDITY_MULTIPLE     125
#define HUMIDITY_SLOPE        65536

#define DATA        PAin(1)   //空气温湿度
#define DATA_H      PAout(1) = 1
#define DATA_L      PAout(1) = 0

#define SCK_H       PCout(3) = 1
#define SCK_L       PCout(3) = 0

#define DATA_INPUT  {GPIOA->CRL&=0XFFFFFF0F;GPIOA->CRL|=8<<4;}//上拉/下拉模式输入，由PxODR寄存器约定
#define DATA_OUTPUT {GPIOA->CRL&=0XFFFFFF0F;GPIOA->CRL|=3<<4;}
#define CPS_DTA_R     {GPIOA->CRL&=0XFFFFFFF0;GPIOA->CRL|=8<<0;}
#define CPS_DTA_W     {GPIOA->CRL&=0XFFFFFFF0;GPIOA->CRL|=3<<0;}//PA0 通用推挽输出，最大速度50MHz

#define ACK   1
#define NOACK 0

extern void SI7021_I2C_Init(void);
extern void SI7021_TransStart(void);
extern void SI7021_TransStop(void);

extern u16 SI7021_TempMeasurement(void);
extern u16 SI7021_HumiMeasurement(void);

extern u16  GET_PRESSUE1(void);//大气压力
extern u16  GET_PRESSUE4(void);//大气压力
extern u16  GET_level1(void);//液位压力
extern u16  PRESSUE_level1(void);//表压液位
extern u16  GET_level4(void);//液位压力
extern u16  level1_temperature;
extern u8   SI7021_WriteByte(u8 data); 
extern u8  SI7021_ReadByte(u8 ack); 
extern u16 	Get_Carbon(void) ;


void   CPS131_Transtart(void);
void   CPS131_Transtop(void);
u8  CPS131_Send(u8 val);
u8  CPS131_Read(u8 ack);

#endif 
