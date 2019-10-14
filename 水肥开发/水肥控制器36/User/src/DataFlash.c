#include "DataFlash.h"
#include "string.h"


uint16_t Flash_Write_Without_check(uint32_t iAddress, uint8_t *buf, uint16_t iNumByteToWrite) 
{
    uint16_t i;
    volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
    i = 0;
    
//    FLASH_UnlockBank1();
    while((i < iNumByteToWrite) && (FLASHStatus == FLASH_COMPLETE))
    {
		FLASHStatus = FLASH_ProgramHalfWord(iAddress, *(uint16_t*)buf);
		i = i+2;
		iAddress = iAddress + 2;
		buf = buf + 2;
    }
    
    return iNumByteToWrite;
}



/**
  * @brief  Programs a half word at a specified Option Byte Data address.
  * @note   This function can be used for all STM32F10x devices.
  * @param  Address: specifies the address to be programmed.
  * @param  buf: specifies the data to be programmed.
  * @param  iNbrToWrite: the number to write into flash
  * @retval if success return the number to write, -1 if error
  *  
  */
int Flash_Write(uint32_t iAddress, uint8_t *buf, uint32_t iNbrToWrite) 
{
	/* Unlock the Flash Bank1 Program Erase controller */
	uint32_t secpos;
	uint16_t secoff;
	uint16_t secremain;  
	uint32_t i = 0;
	uint32_t temp=0;
	uint8_t  tmp[128];
	volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;	


	FLASH_UnlockBank1();
	secpos=iAddress & (~(FLASH_PAGE_SIZE -1 )) ;//扇区地址 
	secoff=iAddress & (FLASH_PAGE_SIZE -1);     //在扇区内的偏移
	secremain=FLASH_PAGE_SIZE-secoff;           //扇区剩余空间大小 

	if(iNbrToWrite<=secremain) 
		temp=1;
	else                                                             //剩余空间小于所存数据
	{
		i=iNbrToWrite-secremain;                                  //判断还占了几个扇区
		if(i%FLASH_PAGE_SIZE==0)
			temp=i/FLASH_PAGE_SIZE+1;
		else
			temp=i/FLASH_PAGE_SIZE+2;
	}


	for(i=0;i<temp;i++)
		FLASHStatus = FLASH_ErasePage(secpos+i*FLASH_PAGE_SIZE); //擦除扇区
	
	if(FLASHStatus != FLASH_COMPLETE)
	  	return -1;
	
	memcpy(tmp,buf,iNbrToWrite);

	if(iNbrToWrite%2)
		temp=iNbrToWrite+1;
	else
		temp=iNbrToWrite;


	Flash_Write_Without_check(iAddress,tmp,temp);

	FLASH_LockBank1();
	return iNbrToWrite; 
}






/**
  * @brief  Programs a half word at a specified Option Byte Data address.
  * @note   This function can be used for all STM32F10x devices.
  * @param  Address: specifies the address to be programmed.
  * @param  buf: specifies the data to be programmed.
  * @param  iNbrToWrite: the number to read from flash
  * @retval if success return the number to write, without error
  *  
  */
int Flash_Read(uint32_t iAddress, uint8_t *buf, int32_t iNbrToRead) 
{
    int i = 0;
    while(i < iNbrToRead ) 
	{
       *(buf + i) = *(__IO uint8_t*) iAddress++;
       i++;
    }
    return i;
}
