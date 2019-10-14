#ifndef SST25VF016B_H
#define SST25VF016B_H

#include "stm32f10x.h"

#ifdef __cplusplus
extern "C"
{
#endif

void intial_sst(void);
void hold(void);
void active(void);
void Transfer_data(u8 data1);
u8   ReadByte(void);
void Poll_SO(void);
u8   Read1(u32 addr);
void Read(u32 addr,u32 num,u8 *pBuf);
u8   High_Speed_Read1(u32 addr);
void High_Speed_Read(u32 addr,u32 num,u8 *pBuf);
void Byte_Program(u32 addr,u8 data);
void AAI_SOE(void);
void AAI_SOD(void);
void AAIA(u32 addr, u8 byte1, u8 byte2);
void AAIB(u8 byte1, u8 byte2);
void AAIA_EBSY(u32 addr, u8 byte1, u8 byte2);
void AAIB_EBSY(u8 byte1, u8 byte2);
void Sector_Erase(u32 addr);
void Block_Erase_32K(u32 addr);
void Block_Erase_64K(u32 addr);
void Chip_Erase(void);
u8   Read_Status_Register(void);
void EWSR(void);
void Write_Enable(void);
void Write_Disable(void);
void Write_Status_Register(u8 data);
u32  JEDEC_Read_ID(void);
u8   Read_ID(u32 addr);
void Wait_Busy(void);
void Wait_Busy_AAI(void);
void WREN_Check(void);
void WREN_AAI_Check(void);
void Verify(u8 byte, u8 cor_byte);

void AutoAddressIncrement_WordProgramA(u8 Byte1, u8 Byte2, u32 Addr);
void AutoAddressIncrement_WordProgramB(u8 state, u8 Byte1, u8 Byte2);
void SPI_Flash_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite);
void Flash_WriteByte(u8* pBuffer,u32 WriteAddr);
void SPI_Flash_Erase_Chip(void);
void SPI_Flash_Erase_Sector(u32 Addr);

#ifdef __cplusplus
}
#endif

#endif
