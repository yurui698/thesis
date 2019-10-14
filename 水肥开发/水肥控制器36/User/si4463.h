#ifndef __SI4463_H__
#define __SI4463_H__

#include "stm32f10x.h"
#include "radio_config_si4463.h"
#include "si446x_defs.h"

#define	 SI4463_NIRQ     GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4)

/*4463时钟脚，主要用于软件模拟SPI时使用*/
#define  SI_SCK_LOW      GPIO_ResetBits(GPIOA, GPIO_Pin_5)
#define  SI_SCK_HIGH     GPIO_SetBits(GPIOA, GPIO_Pin_5)

/*Stm32为主机，SI4463为从机，SDO为主机输出，从机输入*/
#define  SI_SDO_LOW      GPIO_ResetBits(GPIOA, GPIO_Pin_7)
#define  SI_SDO_HIGH     GPIO_SetBits(GPIOA, GPIO_Pin_7)

/*SDI为主机输入，从机输出*/
#define  SI4463_SDI      GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)

/* 无线数据接收指示灯PC.12，翻转表示有接收到数据*/
#define LED_ON      	 GPIO_ResetBits(GPIOC, GPIO_Pin_12)//PC6-PC12修改
#define LED_OFF          GPIO_SetBits(GPIOC, GPIO_Pin_12)//PC6-PC12修改	

/*SPI1片选*/
#define SPI1_NSS_LOW       GPIO_ResetBits(GPIOA, GPIO_Pin_4)
#define SPI1_NSS_HIGH      GPIO_SetBits(GPIOA, GPIO_Pin_4)

/*SI4463复位管脚PC.06*/
#define  SI4463_SDN_LOW    GPIO_ResetBits(GPIOC, GPIO_Pin_6)//PC6-PC12修改
#define  SI4463_SDN_HIGH   GPIO_SetBits(GPIOC, GPIO_Pin_6)//PC6-PC12修改

#define  PACKET_LENGTH   0

/*配置SI4463时，采用软件SPI，配置后改为硬件SPI
**软件SPI时，softspi = 1
**硬件SPI时，softspi = 0
*/
static u8 softspi = 1;

u8 SPI_WriteByte(u8 TxData);
void SI4463_GET_PROPERTY_1(u8 *buffer);
void SI4463_SET_PROPERTY_1( SI446X_PROPERTY GROUP_NUM, u8 proirity );
void SI4463_Init(void);
void SI4463_INT_STATUS( u8 *buffer );
void SI4463_RESET(void);
void SI4463_CMD(u8 *cmd, u8 cmdsize);
void SI4463_SPI_Active(void);
void SI4463_Rx_State(void);
void SI4463_CONFIG_INIT(void);
void SI4463_PART_INFO(u8 *buffer);
void SI4463_GET_INFO(u8 *buffer);
void SI4463_RX_FIFO_RESET(void);
void SI4463_TX_FIFO_RESET(void);
void SI4463_SEND_PACKET(u8 *txbuffer, u8 size, u8 channel, u8 condition);
void SI4463_START_RX( u8 channel, u8 condition, u16 rx_len, u8 n_state1, u8 n_state2, u8 n_state3 );
u8 SI4463_READ_PACKET( u8 *buffer );

void Delayms(u16 ms);
void Delayus(u32 us);

#endif
