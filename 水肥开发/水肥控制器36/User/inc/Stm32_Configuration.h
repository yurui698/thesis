#ifndef _Stm32_Configuration_H
#define _Stm32_Configuration_H

#include "stm32f10x.h"
#include "systemclock.h"
#include "si4463.h"

#define ADC1_DR_Address    ((u32)0x4001244C)

#ifdef __cplusplus
extern "C"
{
#endif

void RCC_Configuration(void);
void GPIO_Configuration(void);
void EXTI_Configuration(void);
void NVIC_Configuration(void);
void ADC1_Configuration(void);
void USART3_Configuration(void);
void USART2_Configuration(void);
void USART1_Configuration(void);
void SPI1_Configuration(void);
void IWDG_Configuration(void);
void SYS_Confgiuration(void);
    
#ifdef __cplusplus
}
#endif

#endif
