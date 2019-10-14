#include "si4463.h"

u8 First[9]={0};

/*4463初始化参数配置数组*/
u8 config_table[] = RADIO_CONFIGURATION_DATA_ARRAY;
extern u8 close_433MHZ;
static u32 max_delay; 

/*

=================================================================================
SI4463_IOSET()
Function : 4463用IO口（不包括SPI）配置
		   PA4 = NSEL_CSN 输出， PC13 = SI_SDN 输出, PA0 = SI_NIRQ(输入)	
INTPUT   : NONE
OUTPUT   : NONE
=================================================================================
*/
void SI4463_IOSET(void)
{
	
} 
/*
=================================================================================
u8 SPI_ExchangeByte(u8 TxData)
Function : wait the device ready to response a command
INTPUT   : NONE
OUTPUT   : NONE
=================================================================================
*/
u8 SPI_WriteByte(u8 TxData)
{
	if(softspi){
		u8 i,ret=0;	
	    for(i=0;i<8;i++)
	    {          
	      if(TxData&0x80) 
	      	SI_SDO_HIGH; 
	      else
	      	SI_SDO_LOW;  
		  TxData<<=1; 	  
	      ret<<=1; 
	      SI_SCK_HIGH; //上升沿，采样
	      Delayus(20);
		  if(SI4463_SDI)
	      ret|=1; 
	      SI_SCK_LOW;  //下降沿，锁存         
	    }	
	    return ret;
	}
	else
	{			
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); 			  
		SPI_I2S_SendData(SPI1, TxData); 
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 	  						    
		return SPI_I2S_ReceiveData(SPI1); 
	}				    
}

/*
=================================================================================
SI446X_WAIT_CTS( );
Function : wait the device ready to response a command
INTPUT   : NONE
OUTPUT   : NONE
=================================================================================
*/
void SI4463_WAIT_CTS(void)
{
	u8 cts;	
	   max_delay=1000;//防止无线发送模块出问题或不存在造成死机
    do
    {
        SPI1_NSS_LOW;				
        SPI_WriteByte( READ_CMD_BUFF );       		
        cts = SPI_WriteByte( NOP );		
        SPI1_NSS_HIGH;
		  IWDG_ReloadCounter();
			max_delay--;
    }while( cts != 0xFF&&max_delay!=0);//&max_delay!=0 
    if(max_delay==0){	close_433MHZ=0;}	
}
/*
=================================================================================
SI446X_READ_RESPONSE( );
Function : read a array of command response
INTPUT   : buffer,  a buffer, stores the data responsed
           size,    How many bytes should be read
OUTPUT   : NONE
=================================================================================
*/
void SI4463_READ_RESPONSE(u8 *buffer, u8 size )
{
    SI4463_WAIT_CTS( );
    SPI1_NSS_LOW;
	SPI_WriteByte( READ_CMD_BUFF );
	while( size -- )
    {
        *buffer++ = SPI_WriteByte(NOP);
    }
    SPI1_NSS_HIGH;

}
/*
=================================================================================
SI446X_CMD( );
Function : Send a command to the device
INTPUT   : cmd, the buffer stores the command array
           cmdsize, the size of the command array
OUTPUT   : NONE
=================================================================================
*/
void SI4463_CMD(u8 *cmd, u8 cmdsize)
{	
    SI4463_WAIT_CTS( );	
    SPI1_NSS_LOW;
    while( cmdsize -- )
    {
        SPI_WriteByte( *cmd++ );
    }
    SPI1_NSS_HIGH;
}

void SI4463_SPI_Active(void)
{
	u8 cmd[2];
    cmd[0] = CHANGE_STATE;
    cmd[1] = 0x02;;
    SI4463_CMD( cmd, 2 );	
}

void SI4463_Rx_State(void)
{
	u8 cmd[2];
    cmd[0] = CHANGE_STATE;
    cmd[1] = 0x08;;
    SI4463_CMD( cmd, 2 );
}
/*
=================================================================================
SI4463_Init( );
Function : 
INTPUT   : 
           
OUTPUT   : NONE
=================================================================================
*/
void SI4463_Init(void)
{  	
	SI4463_RESET( );	
	SI4463_CONFIG_INIT( );
	SI4463_INT_STATUS(First);	
}
/*
=================================================================================
SI446X_RESET( );
Function : reset the SI446x device
INTPUT   : NONE
OUTPUT   : NONE
=================================================================================
*/
void SI4463_RESET(void)
{		
    SI4463_SDN_HIGH;
	Delayus(10);
    SI4463_SDN_LOW;
    SPI1_NSS_HIGH;
	Delayms(10);
}
/*
=================================================================================
SI446X_SET_PROPERTY_1( );
Function : Set the PROPERTY of the device, only 1 byte
INTPUT   : GROUP_NUM, the group and number index
           prioriry,  the value to be set
OUTPUT   : NONE
=================================================================================
*/
void SI4463_SET_PROPERTY_1( SI446X_PROPERTY GROUP_NUM, u8 proirity )
{
    u8 cmd[5];

    cmd[0] = SET_PROPERTY;
    cmd[1] = GROUP_NUM>>8;
    cmd[2] = 1;
    cmd[3] = GROUP_NUM;
    cmd[4] = proirity;
    SI4463_CMD( cmd, 5 );
}

void SI4463_GET_PROPERTY_1(u8 *buffer)
{
	u8 cmd[4];
    cmd[0] = GET_PROPERTY;
    cmd[1] = 0x12;
    cmd[2] = 2;
    cmd[3] = 0x0d;
    SI4463_CMD( cmd, 4 );
	SI4463_READ_RESPONSE( buffer, 3 );
	
}
/*
=================================================================================
SI446X_GPIO_CONFIG( );
Function : config the GPIOs, IRQ, SDO
INTPUT   :
OUTPUT   : NONE
=================================================================================
*/
void SI4463_GPIO_CONFIG( u8 G0, u8 G1, u8 G2, u8 G3,
                         u8 IRQ, u8 SDO, u8 GEN_CONFIG )
{
    u8 cmd[10];
    cmd[0] = GPIO_PIN_CFG;
    cmd[1] = G0;
    cmd[2] = G1;
    cmd[3] = G2;
    cmd[4] = G3;
    cmd[5] = IRQ;
    cmd[6] = SDO;
    cmd[7] = GEN_CONFIG;
    SI4463_CMD( cmd, 8 );
    SI4463_READ_RESPONSE( cmd, 8 );
}
/*
=================================================================================
SI446X_CONFIG_INIT( );
Function : configuration the device
INTPUT   : NONE
OUTPUT   : NONE
=================================================================================
*/
void SI4463_CONFIG_INIT(void)
{
	u8 i;
    u16 j = 0;
    while( ( i = config_table[j] ) != 0 )
    {
        j += 1;		
        SI4463_CMD( config_table + j, i );		
        j += i;
    } 	
//	  variable packet length
//    SI4463_SET_PROPERTY_1( PKT_CONFIG1, 0x00 );	
//    SI4463_SET_PROPERTY_1( PKT_CRC_CONFIG, 0x00 );
//    SI4463_SET_PROPERTY_1( PKT_LEN_FIELD_SOURCE, 0x01 );
//    SI4463_SET_PROPERTY_1( PKT_LEN, 0x2A );
//    SI4463_SET_PROPERTY_1( PKT_LEN_ADJUST, 0x00 );
//    SI4463_SET_PROPERTY_1( PKT_FIELD_1_LENGTH_12_8, 0x00 );
//    SI4463_SET_PROPERTY_1( PKT_FIELD_1_LENGTH_7_0, 0x01 );
//    SI4463_SET_PROPERTY_1( PKT_FIELD_1_CONFIG, 0x00 );
//    SI4463_SET_PROPERTY_1( PKT_FIELD_1_CRC_CONFIG, 0x00 );
//    SI4463_SET_PROPERTY_1( PKT_FIELD_2_LENGTH_12_8, 0x00 );
//    SI4463_SET_PROPERTY_1( PKT_FIELD_2_LENGTH_7_0, 0x3F );
//    SI4463_SET_PROPERTY_1( PKT_FIELD_2_CONFIG, 0x02 );
//    SI4463_SET_PROPERTY_1( PKT_FIELD_2_CRC_CONFIG, 0x2A );

    SI4463_GPIO_CONFIG( 0, 0, 0x61, 0x60, 0x67, 0, 0 );	
}
/*
=================================================================================
SI446X_RX_FIFO_RESET( );
Function : reset the RX FIFO of the device
INTPUT   : None
OUTPUT   : NONE
=================================================================================
*/
void SI4463_RX_FIFO_RESET( void )
{
    u8 cmd[2];

    cmd[0] = FIFO_INFO;
    cmd[1] = 0x02;
    SI4463_CMD( cmd, 2 );
}
/*
=================================================================================
SI446X_TX_FIFO_RESET( );
Function : reset the TX FIFO of the device
INTPUT   : None
OUTPUT   : NONE
=================================================================================
*/
void SI4463_TX_FIFO_RESET( void )
{
    u8 cmd[2];

    cmd[0] = FIFO_INFO;
    cmd[1] = 0x01;
    SI4463_CMD( cmd, 2 );
}
/*
=================================================================================
SI446X_PART_INFO( );
Function : Read the PART_INFO of the device, 8 bytes needed
INTPUT   : buffer, the buffer stores the part information
OUTPUT   : NONE
=================================================================================
*/
void SI4463_PART_INFO(u8 *buffer)
{
	u8 cmd = PART_INFO;

    SI4463_CMD( &cmd, 1 );
    SI4463_READ_RESPONSE( buffer, 8 );
}

void SI4463_GET_INFO(u8 *buffer)
{
	u8 cmd = REQUEST_DEVICE_STATE;

    SI4463_CMD( &cmd, 1 );
    SI4463_READ_RESPONSE( buffer, 3 );
}
/*
=================================================================================
SI446X_INT_STATUS( );
Function : Read the INT status of the device, 9 bytes needed
INTPUT   : buffer, the buffer stores the int status
OUTPUT   : NONE
=================================================================================
*/
void SI4463_INT_STATUS( u8 *buffer )
{
    u8 cmd[4];
    cmd[0] = GET_INT_STATUS;
    cmd[1] = 0;
    cmd[2] = 0;
    cmd[3] = 0;
    SI4463_CMD( cmd, 4 );
    SI4463_READ_RESPONSE( buffer, 9 );
}

void SI4463_SEND_PACKET( u8 *txbuffer, u8 size, u8 channel, u8 condition)
{
		
		u8 cmd[5];
	    u8 tx_len = size;
		SI4463_RX_FIFO_RESET();
	    SI4463_TX_FIFO_RESET();		        //清发送FIFO
		SI4463_SPI_Active();
	    SPI1_NSS_LOW;
	    SPI_WriteByte(WRITE_TX_FIFO);	    	//启动写TXFIFO
	#if PACKET_LENGTH == 0						//可变长度则将长度数据写入FIFO
	    tx_len++;
	    SPI_WriteByte(size);
	#endif
	    while( size -- )    					//将有效数据写入FIFO
		{ 
			SPI_WriteByte( *txbuffer++ ); 
		}
	    SPI1_NSS_HIGH;
	    cmd[0] = START_TX;						
	    cmd[1] = channel;
	    cmd[2] = condition;
	    cmd[3] = 0;
	    cmd[4] = tx_len;								//发送多长的数据
	    SI4463_CMD( cmd, 5 );					//启动发送		
}
																   
void Delayus(u32 us)
{
	u32 i;
  	i = us * 20;
  	for(; i != 0; i--);

}

void Delayms(u16 ms)
{
	u32 i;
  	i = ms * 8000;
  	for(; i != 0; i--);
}

void SI4463_START_RX( u8 channel, u8 condition, u16 rx_len,
                      u8 n_state1, u8 n_state2, u8 n_state3 )
{
    u8 cmd[8];
    SI4463_RX_FIFO_RESET( );
    SI4463_TX_FIFO_RESET( );
    cmd[0] = START_RX;
    cmd[1] = channel;
    cmd[2] = condition;
    cmd[3] = rx_len>>8;
    cmd[4] = rx_len;
    cmd[5] = n_state1;
    cmd[6] = n_state2;
    cmd[7] = n_state3;
    SI4463_CMD( cmd, 8 );
}

u8 SI4463_READ_PACKET( u8 *buffer )
{
    u8 length, i;
	SI4463_SPI_Active();
    SI4463_WAIT_CTS( );
    SPI1_NSS_LOW;
    SPI_WriteByte( READ_RX_FIFO );
#if PACKET_LENGTH == 0
    length = SPI_WriteByte(NOP);
#else
    length = PACKET_LENGTH;
#endif
    i = length;

    while( length -- )
    {
        *buffer++ = SPI_WriteByte(NOP);
    }
    SPI1_NSS_HIGH;
    return i;
}
