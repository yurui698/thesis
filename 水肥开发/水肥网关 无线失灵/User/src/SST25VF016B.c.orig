#include "SST25VF016B.H"


#define CE_ON        GPIO_SetBits(GPIOD, GPIO_Pin_0)
#define CE_OFF		 GPIO_ResetBits(GPIOD, GPIO_Pin_0)

#define	WP_ON        GPIO_SetBits(GPIOD, GPIO_Pin_7)
#define WP_OFF		 GPIO_ResetBits(GPIOD, GPIO_Pin_7)

#define HOLD_ON	     GPIO_SetBits(GPIOC, GPIO_Pin_12)
#define HOLD_OFF     GPIO_ResetBits(GPIOC, GPIO_Pin_12)

#define SCK_ON       GPIO_SetBits(GPIOC, GPIO_Pin_11)
#define SCK_OFF		 GPIO_ResetBits(GPIOC, GPIO_Pin_11)

#define SI_ON     	 GPIO_SetBits(GPIOD, GPIO_Pin_1)
#define SI_OFF       GPIO_ResetBits(GPIOD, GPIO_Pin_1)

#define SO			 GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2)



void intial_sst(void) //内存初始化
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_0|\
	                                   GPIO_Pin_1|\
									   GPIO_Pin_7;				        
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;		          
	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_11|\
	                                   GPIO_Pin_12;				        
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;		          
	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_2;			         
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_IN_FLOATING;	         
  	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
//	EWSR();
//	Write_Status_Register(0X02);
//	//AAI_SOE();
//	AAI_SOD();
//	WP=1;
}



void hold(void)
{
	//start();
	HOLD_ON;
	SCK_OFF;
	//MicroWait(5);
	HOLD_OFF; 
}

//void active()
//{
//  HOLD=0;
//  SCK=0;
//  MicroWait(5);
//  HOLD=1;
//}
//
//
//void Transfer_data(uint8 data)//向芯片中写数据
//{
//  uint8 i;
//  for(i=0;i<8;i++)
//  {
//    if(data&0x80) SI=1;
//    else SI=0;
//    SCK=1;
//    data<<=1; 	 
//    SCK=0;
//  }
//}
//
//uint8 ReadByte(void) //从芯片中读取字节
//{
//  uint8 SPICount;                     // Counter used to clock out the data
//  uint8 SPIData;                  
//  SPIData = 0;
//  for (SPICount = 0; SPICount < 8; SPICount++)// Prepare to clock in the data to be read
//  {
//    SPIData <<=1; asm("NOP");asm("NOP");asm("NOP");  // Rotate the data
//    SCK=1;// Raise the clock to clock the data out of the MAX7456
//    if(SO){
//      SPIData|=0x01;
//    }
//    SCK=0;asm("NOP");asm("NOP");// Raise the clock to clock the data out of the MAX7456
//  }// and loop back
//  return (SPIData);// Finally return the read data
//}
//
//void Poll_SO(void)
//{
//  uint8 temp = 0;
//  CE=0;
//  while (temp == 0x00)	/* waste time until not busy */
//    temp = SO;
//  CE=1;
//}
//
//uint8 Read1(uint32 addr) //读一个字节
//{
//  uint8 byte = 0;	 
//  CE=0;			/* enable device */
//  Transfer_data(0x03); 		/* read command */
//  Transfer_data(((addr & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  Transfer_data(((addr & 0xFFFF) >> 8));
//  Transfer_data(addr & 0xFF);
//  byte = ReadByte();
//  CE=1;			/* disable device */
//  return byte;			/* return one byte read */
//}
//
//
//void Read(uint32 addr,uint32 num,uint8 *pBuf)//读一串数据
//{
//  uint32 i=0;
//  CE=0;
//  Transfer_data(0x03);
//  Transfer_data((addr&0xFF0000)>>16);
//  Transfer_data((addr&0xFF00)>>8);
//  Transfer_data(addr&0xFF);
//  for(i=0;i<num;i++)
//  {
//    *(pBuf++)=ReadByte();
//  }
//  CE=1;
//}
//
//uint8 High_Speed_Read1(uint32 addr) //高速读一个字节
//{
//  uint8 byte = 0;	 
//  CE=0;			/* enable device */
//  Transfer_data(0x03); 		/* read command */
//  Transfer_data(((addr & 0xFFFFFF) >> 16));	/* send 3 address bytes */
//  Transfer_data(((addr & 0xFFFF) >> 8));
//  Transfer_data(addr & 0xFF);
//  Transfer_data(0xFF);
//  byte = ReadByte();
//  CE=1;			/* disable device */
//  return byte;			/* return one byte read */
//}
//
//
//
//void High_Speed_Read(uint32 addr,uint32 num,uint8 *pBuf)//高速读一串数据
//{
//  uint32 i=0;
//  CE=0;
//  Transfer_data(0x0B);
//  Transfer_data((addr&0xFF0000)>>16);
//  Transfer_data((addr&0xFF00)>>8);
//  Transfer_data(addr&0xFF);
//  Transfer_data(0xFF);
//  for(i=0;i<num;i++)
//  {
//    *(pBuf++)=ReadByte();
//  }
//  CE=1;
//}
//
//
//void Byte_Program(uint32 addr,uint8 data) //写字节
//{
//  CE=0;
//  Transfer_data(0x02);
//  Transfer_data((addr&0xFF0000)>>16);
//  Transfer_data((addr&0xFF00)>>8);
//  Transfer_data(addr&0xFF);
//  Transfer_data(data);
//  CE=1;
//}
//
//void AAI_SOE(void)
//{
//  CE=0;
//  Transfer_data(0x70);
//  CE=1;
//}
//
//void AAI_SOD(void)
//{
//  CE=0;
//  Transfer_data(0x80);
//  CE=1;
//}
//void AAIA(uint32 addr, uint8 byte1, uint8 byte2)
//{
//  CE=0;				/* enable device */
//  Transfer_data(0xAD);			/* send AAI command */
//  Transfer_data(((addr & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
//  Transfer_data(((addr & 0xFFFF) >> 8));
//  Transfer_data(addr & 0xFF);
//  Transfer_data(byte1);			/* send 1st byte to be programmed */	
//  Transfer_data(byte2);			/* send 2nd byte to be programmed */
//  CE=1;				/* disable device */
//}
//
//void AAIB(uint8 byte1, uint8 byte2)
//{
//  CE=0;				/* enable device */
//  Transfer_data(0xAD);			/* send AAI command */
//  Transfer_data(byte1);			/* send 1st byte to be programmed */
//  Transfer_data(byte2);			/* send 2nd byte to be programmed */
//  CE=1;				/* disable device */
//}
//
////地址自动增加的写数据A
//void AAIA_EBSY(uint32 addr, uint8 byte1, uint8 byte2)
//{
//  AAI_SOE();					/* enable RY/BY# status for SO in AAI */	
//  CE=0;				/* enable device */
//  Transfer_data(0xAD);			/* send AAI command */
//  Transfer_data(((addr & 0xFFFFFF) >> 16)); 	/* send 3 address bytes */
//  Transfer_data(((addr & 0xFFFF) >> 8));
//  Transfer_data(addr & 0xFF);
//  Transfer_data(byte1);			/* send 1st byte to be programmed */	
//  Transfer_data(byte2);			/* send 2nd byte to be programmed */
//  CE=1;				/* disable device */
//  Poll_SO();				/* polls RY/BY# using SO line */
//}
//
////地址自动增加的写数据B
//void AAIB_EBSY(uint8 byte1, uint8 byte2)
//{
//  CE=0;			/* enable device */
//  Transfer_data(0xAD);			/* send AAI command */
//  Transfer_data(byte1);			/* send 1st byte to be programmed */
//  Transfer_data(byte2);			/* send 2nd byte to be programmed */
//  CE=1;				/* disable device */
//  
//  Poll_SO();				/* polls RY/BY# using SO line */
//  
//  Write_Disable(); 				/* Exit AAI before executing DBSY */
//  AAI_SOD();			/* disable SO as RY/BY# output if in AAI */
//}
//
//
//void Sector_Erase(uint32 addr)
//{
//  CE=0;
//  Transfer_data(0x20);
//  Transfer_data((addr&0xFF0000)>>16);
//  Transfer_data((addr&0xFF00)>>8);
//  Transfer_data(addr&0xFF);
//  CE=1;
//}
//
//void Block_Erase_32K(uint32 addr)
//{
//  CE=0;
//  Transfer_data(0x52);
//  Transfer_data((addr&0xFF0000)>>16);
//  Transfer_data((addr&0xFF00)>>8);
//  Transfer_data(addr&0xFF);
//  CE=1;
//}
//
//void Block_Erase_64K(uint32 addr)
//{
//  CE=0;
//  Transfer_data(0xD8);
//  Transfer_data((addr&0xFF0000)>>16);
//  Transfer_data((addr&0xFF00)>>8);
//  Transfer_data(addr&0xFF);
//  CE=1;
//}
//
//
//void Chip_Erase(void)
//{
//  CE=0;
//  Transfer_data(0x60);
//  CE=1;
//}
//
//uint8 Read_Status_Register(void)
//{
//  uint8 byte=0;
//  CE=0;
//  Transfer_data(0x05);
//  byte=ReadByte();
//  CE=1;
//  return byte;
//}
//
//void EWSR(void)
//{
//  CE=0;		
//  Transfer_data(0x50);		
//  CE=1;		
//}
//
//
//void Write_Enable(void)
//{
//  CE=0;
//  Transfer_data(0x06);
//  CE=1;
//}
//
//void Write_Disable(void)
//{
//  CE=0;
//  Transfer_data(0x04);
//  CE=1;
//}
//
//void Write_Status_Register(uint8 data)
//{
//  CE=0;
//  Transfer_data(0x01);
//  Transfer_data(data);
//  CE=1;
//}
//
//uint32 JEDEC_Read_ID(void)
//{
//  uint32 temp=0;
//  CE=0;
//  Transfer_data(0x9F);
//  temp = (temp | ReadByte()) << 8;
//  temp = (temp | ReadByte()) << 8;
//  temp = (temp | ReadByte()) ;
//  CE=1;
//  return temp;
//}
//
//uint8 Read_ID(uint32 addr)
//{
//  uint8 byte;
//  CE=0;
//  Transfer_data(0x90);
//  Transfer_data(0x00);
//  Transfer_data(0x00);
//  Transfer_data(addr);
//  byte= ReadByte();
//  CE=1;
//  return byte;  
//}
//
//void Wait_Busy(void)
//{
//  while ((Read_Status_Register() & 0x03) == 0x03)	/* waste time until not busy */
//    Read_Status_Register();
//}
//
//void Wait_Busy_AAI(void)
//{
//  while ((Read_Status_Register() & 0x43) == 0x43)	/* waste time until not busy */
//    Read_Status_Register();
//}
//
//void WREN_Check(void)
//{
//  uint8 byte;
//  byte = Read_Status_Register();	/* read the status register */
//  if (byte != 0x02)		/* verify that WEL bit is set */
//  {
//    while(1);
//    /* add source code or statements for this file */
//    /* to compile   			       */
//    /* i.e. option: insert a display to view error on LED? */
//    /* option: insert a display to view error on LED? */
//  }
//}
//
//void WREN_AAI_Check(void)
//{
//  uint8 byte;
//  byte = Read_Status_Register();	/* read the status register */
//  if (byte != 0x42)		/* verify that AAI and WEL bit is set */
//  {
//    while(1);	
//    /* add source code or statements for this file */
//    /* to compile   			       */
//    /* i.e. option: insert a display to view error on LED? */
//    /* option: insert a display to view error on LED? */
//    
//  }
//}
//
//void Verify(uint8 byte, uint8 cor_byte)
//{
//  if (byte != cor_byte)
//  {
//    while(1);
//    /* add source code or statement for this file */
//    /* to compile   			       */
//    /* i.e. option: insert a display to view error on LED? */
//    
//    /* option: insert a display to view error on LED? */
//  }
//}
//
//
//
//
//
//
//
//
////地址自动增加的写数据A
//void AutoAddressIncrement_WordProgramA(uint8 Byte1, uint8 Byte2, uint32 Addr)
//{
//  Write_Enable();
//  CE=0;
//  Transfer_data(0xAD);
//  //输入所要写数据的起始地址
//  Transfer_data((Addr & 0xFF0000) >> 16);
//  Transfer_data((Addr & 0xFF00) >> 8);
//  Transfer_data(Addr & 0xFF);                                  //发送最初的两个数据
//  Transfer_data(Byte1);
//  Transfer_data(Byte2);
//  CE=1;
//  Wait_Busy();
//}
//
////地址自动增加的写数据B
//void AutoAddressIncrement_WordProgramB(uint8 state,uint8 Byte1, uint8 Byte2)
//{
//  Write_Enable();
//  CE=0;
//  Transfer_data(0xAD);
//  Transfer_data(Byte1);
//  Transfer_data(Byte2);
//  CE=1;
//  Wait_Busy();
//  if(state==1)
//  {
//    Write_Disable();
//  }
//  Wait_Busy();
//}
//
////结合AB构成的地址自动增加的连续数据的写入
////具有先擦除待写区域的功能
////pBuffer：为待写数据组
////WriteAddr：所写数据的起始地址
////NumByteToWrite：所要写的数据的长度
//void SPI_Flash_Write(uint8* pBuffer,uint32 WriteAddr,uint16 NumByteToWrite)   //在连续区域写入一串数据
//{
//  uint16 i,temp;
//  uint32 secpos;
//  uint16 secoff;
//  uint16 secremain;     
//  //以下代码为擦除待写区域的代码
//  secpos=WriteAddr/4096;                                           //扇区（4K）地址0~511 for     SST25VF016
//  secoff=WriteAddr%96 ;                                                          //在扇区内的偏移
//  secremain=4096-secoff;                                                           //扇区剩余空间大小
//  if(NumByteToWrite<secremain)                                                  //剩余空间大于所存数据
//  {
//    temp=1;
//  }
//  else                                                                 //剩余空间小于所存数据
//  {
//    i=NumByteToWrite-secremain;                           //判断还占了几个扇区
//    if(i%96==0)
//      temp=i/4096+1;
//    else
//      temp=i/4096+2;
//  }
//  for(i=0;i<temp;i++)
//  {
//    SPI_Flash_Erase_Sector((secpos+i)*4096);                   //擦除将要写入数据的扇区   
//  }
//
//  //以下代码为将数据写入指定地址的代码
//  if(NumByteToWrite%2==0)
//  {
//    temp=NumByteToWrite/2-1;
//  }
//  else
//  {
//    temp=NumByteToWrite/2;
//  }
//  AutoAddressIncrement_WordProgramA(pBuffer[0], pBuffer[1],WriteAddr );                                        //开始写数据
//  for(i=1;i<temp;i++)
//  {
//    AutoAddressIncrement_WordProgramB(0,pBuffer[2*i], pBuffer[2*i+1]);
//  }
//  if(NumByteToWrite%2==0)
//  {
//    AutoAddressIncrement_WordProgramB(1,pBuffer[NumByteToWrite-2], pBuffer[NumByteToWrite-1]);   //结束写数据
//  }
//  else
//  {
//    AutoAddressIncrement_WordProgramB(1,pBuffer[NumByteToWrite-1],0);                                         //结束写数据
//  }
//}
//
////写入1Byte数据
////pBuffer:待写的数据
////WriteAddr：待写数据的地址
//void Flash_WriteByte(uint8* pBuffer,uint32 WriteAddr)
//{
//  uint32 secpos;
//  secpos=WriteAddr/4096;                                   //扇区地址 0~511 for w25x16  4096=4k
//  SPI_Flash_Erase_Sector(secpos);                        //擦除这个扇区
//  Write_Enable();                              //SET WEL
//  CE=0;                                            //使能器件  
//  Transfer_data(0X02 );       //发送写页命令
//  
//  //发送24bit地址   
//  Transfer_data((uint8)((WriteAddr)>>16));  
//  Transfer_data((uint8)((WriteAddr)>>8));  
//  Transfer_data((uint8)WriteAddr);                  //发送待写的数据
//  Transfer_data(pBuffer[0]);
//  CE=1;
//  Wait_Busy();                                     //等待写完成
//}
//
////擦除整个芯片
////整片擦除时间:
////W25X16:25s
////W25X32:40s
////W25X64:40s
////等待时间超长...
//void SPI_Flash_Erase_Chip(void)  
//{                                            
//  Write_Enable();                          //SET WEL
//  Wait_Busy();  
//  CE=0;                                       //使能器件  
//  Transfer_data(0X60);       //发送片擦除命令
//  CE=1;                                     //取消片选               
//  Wait_Busy();                              //等待芯片擦除结束
//}
//
////擦除一个扇区
////Dst_Addr:扇区地址 0~511 for w25x16
////擦除一个山区的最少时间:150ms
//void SPI_Flash_Erase_Sector(uint32 Addr)  
//{  
//  Write_Enable();                                       //SET WEL           
//  Wait_Busy();  
//  CE=0;                                                   //使能器件  
//  Transfer_data(0X20);      //发送扇区擦除指令
//  Transfer_data((uint8)((Addr)>>16));            //发送24bit地址   
//  Transfer_data((uint8)((Addr)>>8));  
//  Transfer_data((uint8)Addr);
//  CE=1;                                            //取消片选               
//  Wait_Busy();                                    //等待擦除完成
//}
