
#ifndef _lcd_12864_H
#define _lcd_12864_H


#include "stm32f10x.h"


#define uint8     unsigned char
#define int32     long
#define uint32    unsigned long

#define Rom_IN_1  GPIOE->BSRR=GPIO_Pin_1                       //字库 IC 接口定义:Rom_IN 就是字库 IC 的 SI
#define Rom_IN_0  GPIOE->BRR =GPIO_Pin_1

#define Rom_OUT   GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)     //字库 IC 接口定义:Rom_OUT 就是字库 IC 的 SO

#define Rom_SCK_1 GPIOB->BSRR=GPIO_Pin_8                       //字库 IC 接口定义:Rom_SCK 就是字库 IC 的 SCK
#define Rom_SCK_0 GPIOB->BRR =GPIO_Pin_8

#define Rom_CS_1  GPIOB->BSRR=GPIO_Pin_7                       //字库 IC 接口定义 Rom_CS 就是字库 IC 的 CS#
#define Rom_CS_0  GPIOB->BRR =GPIO_Pin_7

#define LED_off   GPIOB->BSRR=GPIO_Pin_6
#define LED_on    GPIOB->BRR =GPIO_Pin_6

#define rs_1      GPIOE->BSRR=GPIO_Pin_6                       //A0
#define rs_0      GPIOE->BRR =GPIO_Pin_6

#define reset_1   GPIOE->BSRR=GPIO_Pin_4
#define reset_0   GPIOE->BRR =GPIO_Pin_4

#define cs1_1     GPIOE->BSRR=GPIO_Pin_5
#define cs1_0     GPIOE->BRR =GPIO_Pin_5

#define sid_1     GPIOE->BSRR=GPIO_Pin_2                       //d7
#define sid_0     GPIOE->BRR =GPIO_Pin_2

#define sclk_1    GPIOE->BSRR=GPIO_Pin_3                       //d6
#define sclk_0    GPIOE->BRR =GPIO_Pin_3


#ifdef __cplusplus
extern "C"
{
#endif

  	void LCD_GPIO_Configuration(void);
  	void Transfer_data(uint8 data1);
  	void Transfer_command(uint8 data1);
  	void Delay(int i);
  	void Delay1(int i);
  	void Initial_Lcd(void);
  	void clear_screen(void);
  
  	void display_graphic_8x16(uint8 page,uint8 column,const uint8 *dp) ;
  	void display_graphic_16x8(uint8 page,uint8 column,const uint8 *dp); 
  	void display_graphic_16x16(uint8 page,uint8 column,const uint8 *dp) ;
  	void display_GB2312_string(uint8 y,uint8 x,const uint8 *text) ;
  	void display_hex(const uint8 *data,uint8 count,uint8 y,uint8 x);
  	void display_ascii(const uint8 *dp,uint8 count,uint8 y,uint8 x); 
  	uint8 display_int(const int32,uint8 y,uint8 x,uint8 pt); 
  
  	void lcd_sleep(void);
  	void lcd_sleep_cancel(void); 
  
  	void get_n_bytes_data_from_ROM(uint8 addrHigh,uint8 addrMid,uint8 addrLow,uint8 *pBuff,uint8 DataLen ) ;
  	void send_command_to_ROM( uint8 datu ) ;
  	static uint8 get_data_from_ROM(void ) ;
  	void display_GB2312_char(uint8 y,uint8 x,uint8 code_h,uint8 code_l); 
  
  
  	void displayNum(s32 num,uint8 y,uint8 x);
  
#ifdef __cplusplus
}
#endif

#endif // _LCD_12864_H
