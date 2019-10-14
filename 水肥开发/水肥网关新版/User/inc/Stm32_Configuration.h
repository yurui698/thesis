#ifndef _Stm32_Configuration_H
#define _Stm32_Configuration_H

#include "stm32f10x.h"
#include "systemclock.h"
#include "si4463.h"

#define ADC1_DR_Address    ((u32)0x4001244C)

#define Boolean            u8

/* 有线通信指示灯 */
#define YX_LED_ON             GPIO_ResetBits(GPIOE, GPIO_Pin_0)	
#define YX_LED_OFF            GPIO_SetBits(GPIOE, GPIO_Pin_0)	
#define YX_LED_TOGGLE		   GPIO_WriteBit(GPIOE,GPIO_Pin_0, (BitAction)(1-GPIO_ReadOutputDataBit(GPIOE,GPIO_Pin_0)))

/* 无线通信指示灯 */
#define WX_LED_ON             GPIO_ResetBits(GPIOB, GPIO_Pin_6)	
#define WX_LED_OFF            GPIO_SetBits(GPIOB, GPIO_Pin_6)	
#define WX_LED_TOGGLE		   GPIO_WriteBit(GPIOB,GPIO_Pin_6, (BitAction)(1-GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_6)))

#ifdef __cplusplus
extern "C"
{
#endif

void RCC_Configuration(void);
void GPIO_Configuration(void);
void EXTI_Configuration(void);
void NVIC_Configuration(void);
void ADC1_Configuration(void);
void UART5_Configuration(void);
//void UART4_Configuration(void);
//void USART3_Configuration(void);
//void USART2_Configuration(void);
//void USART1_Configuration(void);
void SPI1_Configuration(void);
void IWDG_Configuration(void);
void EXIT0_Enable(Boolean bEnable);
void EXTI1_Enable(Boolean bEnable);//PA1,空气温湿度	
void SYS_Confgiuration(void);

//void PowerOnInteruptEnable(Boolean bEnable);
//void PowerOffInteruptEnable(Boolean bEnable);
    
#ifdef __cplusplus
}
#endif

#endif
