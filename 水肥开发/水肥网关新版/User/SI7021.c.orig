#include "SI7021.h"

#define TIMEOUT_CPS   500

#define CPS_SCK_H     PCout(3)=1
#define CPS_SCK_L     PCout(3)=0
#define CPS_DTA_H     PAout(0)=1 //二氧化碳
#define CPS_DTA_L     PAout(0)=0


#define CPS_DTA       GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)//PA0 二氧化碳
//u8 test_data[7];
void SI7021_I2C_Init(void)
{
  DATA_OUTPUT; //PA1通用推挽输出，最大速度50MHz
  SCK_H;
  Delayus(WAIT_TIME);
  DATA_H;
  Delayus(WAIT_TIME);
  DATA_INPUT;//安全使用	
}


void SI7021_TransStart(void) //测量空气温湿度
{
  DATA_OUTPUT;//PA1通用推挽输出，最大速度50MHz
  DATA_H;    //PA1=1 空气温湿度
  Delayus(WAIT_TIME);
  SCK_H;    //PC3=1
  Delayus(WAIT_TIME);
  DATA_L;  //PA1=0
  Delayus(WAIT_TIME);
  SCK_L;   //PC3=0
  Delayus(WAIT_TIME);
	DATA_INPUT;//安全使用
}

void SI7021_TransStop(void)
{
  DATA_OUTPUT;//PA1通用推挽输出，最大速度50MHz
  DATA_L;    //PA1=0
  Delayus(WAIT_TIME);
  SCK_H;     //PC3=1
  Delayus(WAIT_TIME);
  DATA_H;
  Delayus(WAIT_TIME);
  SCK_L;
  Delayus(WAIT_TIME);
	DATA_INPUT;//安全使用
}


u8 SI7021_WriteByte(u8 data)
{
  u8 i;
  u8 ack=0; 
  DATA_OUTPUT;//PA1通用推挽输出，最大速度50MHz
  for(i=0;i<8;i++)
  {
    if(data&0x80)
      DATA_H;
    else
      DATA_L;      
    Delayus(WAIT_TIME);
    SCK_H;  
    Delayus(WAIT_TIME);
    SCK_L; 
    data<<=1;
    Delayus(WAIT_TIME);
  }
  DATA_H;
  DATA_INPUT;//PA1通用推挽输入，最大速度50MHz	
  Delayus(WAIT_TIME); 
  SCK_H;
  Delayus(WAIT_TIME);
  if(DATA)
     ack = 0;
  else
     ack = 1;
  Delayus(WAIT_TIME);
  SCK_L;
  Delayus(WAIT_TIME);
  return ack;
}


u8  SI7021_ReadByte(u8 ack)
{
  u8 i; 
  u8 val=0;
  DATA_INPUT; 
  for(i=0;i<8;i++) 
  {     
    val<<=1;
    SCK_H;
    Delayus(WAIT_TIME);
    if(DATA) 
      val|=0x01; 
    SCK_L;
    Delayus(WAIT_TIME);
  }
  
  DATA_OUTPUT;
  if(ack!=0) 
    DATA_L; 
  else
    DATA_H;
  
  Delayus(WAIT_TIME);
  SCK_H; 
  Delayus(WAIT_TIME);
  SCK_L; 
  DATA_H;
  DATA_INPUT;//安全使用	
  return val; 
}



u16 SI7021_TempMeasurement(void)
{
    u16 value;
    SI7021_TransStart();
    SI7021_WriteByte(0x80);
    SI7021_WriteByte(0xE0);
    SI7021_TransStart();
    SI7021_WriteByte(0x81);
    value=SI7021_ReadByte(1);
   
    value=(value<<8)|SI7021_ReadByte(0);
  
    SI7021_TransStop();
    
    return value;
     
}
u16 SI7021_HumiMeasurement(void)
{
    u16 value,i;

    SI7021_TransStart();
    
    SI7021_WriteByte(0x80);
    SI7021_WriteByte(0xE5);
   
    SI7021_TransStart();
    SI7021_WriteByte(0x81);
     for(i=1200;i>0;i--)
	{
      Delayus(2*WAIT_TIME);
	}
         
    value=SI7021_ReadByte(1);
    value=(value<<8) | SI7021_ReadByte(0);

    SI7021_TransStop();   
    return value;
}

u16 GET_PRESSUE1(void) //大气压力;通道1空气温湿度PA1
{   
  u16 value;
  u16 data[5]={0x00};
  
  SI7021_TransStart();
  SI7021_WriteByte(0x50);
  SI7021_WriteByte(0xAC);
  SI7021_TransStop();
  Delayms(100);//这个延时很重要，时间要大于37ms
  
  SI7021_TransStart(); 
  SI7021_WriteByte(0x51);   //发送地址   
  data[0]=SI7021_ReadByte(1); 
  data[1]=SI7021_ReadByte(1); 
  data[2]=SI7021_ReadByte(1);
  data[3]=SI7021_ReadByte(1);
  data[4]=SI7021_ReadByte(0);
  SI7021_TransStop(); 
  
  if((data[0]&0x60)==0x40)
  {
  	value=(data[1]<<8)|data[2];
  }
  else
  {
   	value=0xffff;
  }
  return value; 
}

u16 GET_level1(void) //液位压力cps120;通道1-通道1空气温湿度PA1
{   
  u16 value;
  u16 data[5]={0x00};
  
  SI7021_TransStart();
  SI7021_WriteByte(0x50);
  SI7021_WriteByte(0xAC);
  SI7021_TransStop();
  Delayms(100);//这个延时很重要，时间要大于37ms
  
  SI7021_TransStart(); 
  SI7021_WriteByte(0x51);   //发送地址   
  data[0]=SI7021_ReadByte(1); 
  data[1]=SI7021_ReadByte(1); 
  data[2]=SI7021_ReadByte(1);
  data[3]=SI7021_ReadByte(0);  
  SI7021_TransStop(); 
  
  if((data[0]&0xC0)==0x00)
  {
  	value=(data[0]<<8)|data[1];
		level1_temperature=((data[2]<<8)|data[3])>>2;
  }
  else
  {
   	value=0xffff;
  }
  return value; 
}

u16 PRESSUE_level1(void) //液位压力LWP5050GD;通道1-通道1空气温湿度PA1
{  
	u16 value;
  u8 data[7]={0x00};
 
  SI7021_TransStart();
  SI7021_WriteByte(0x00);
//Delayms(5);
  SI7021_WriteByte(0xAA);
//Delayms(5);
	SI7021_WriteByte(0x00);
//Delayms(5);
  SI7021_WriteByte(0x80);
//Delayms(5);
  SI7021_TransStop();
  Delayms(30);//这个延时很重要，时间要大于5ms
  
  SI7021_TransStart(); //读状态byte
  SI7021_WriteByte(0x01);   //发送地址
//Delayms(5);
  data[6]=SI7021_ReadByte(1);	
  data[0]=SI7021_ReadByte(1); 
  data[1]=SI7021_ReadByte(1); 
  data[2]=SI7021_ReadByte(1);
  data[3]=SI7021_ReadByte(1);
  data[4]=SI7021_ReadByte(1);
  data[5]=SI7021_ReadByte(0);   
  SI7021_TransStop(); 
  value=((data[0]<<16)+(data[1]<<8)+data[2])>>8;
	level1_temperature=((data[3]<<16)+(data[4]<<8)+data[5])>>8;  
  return value; 
}


u16 Get_Carbon(void) 
{
  u16 value;
  u16 data[7];
  
  CPS131_Transtart();
  CPS131_Send(0x62);
  CPS131_Send(0x52);
  CPS131_Transtop();
  
  Delayus(5000); //这个延时很重要
  
  CPS131_Transtart(); 
  CPS131_Send(0x63);   //发送地址   
  Delayus(2000);
  data[0]=CPS131_Read(1); 
  Delayus(2000);
  data[1]=CPS131_Read(1); 
  Delayus(2000); 
  data[2]=CPS131_Read(0);
   
  CPS131_Transtop(); 
 
  value=(data[1]<<8)|data[2];

  return value; 
  
}

u8 CPS131_Read(u8 ack) 
{ 
  u8 i; 
  u8 val=0;
  
  CPS_DTA_R;
  
  for(i=0;i<8;i++) 
  { 
    val<<=1;
    CPS_SCK_H;    
    Delayus(TIMEOUT_CPS); 
    if(CPS_DTA) 
      val|=0x01; 
    CPS_SCK_L; 
    Delayus(TIMEOUT_CPS);
  }   
  CPS_DTA_W;

  if(ack!=0) 
    CPS_DTA_L; 
  else
    CPS_DTA_H;
  
  Delayus(TIMEOUT_CPS);
  CPS_SCK_H; 
  Delayus(TIMEOUT_CPS); 
  CPS_SCK_L; 
  CPS_DTA_H;
  CPS_DTA_R;//安全使用	
  return val; 
} 


void CPS131_Transtop(void) 
{ 
  CPS_DTA_W; 
  CPS_DTA_L; 
  CPS_SCK_H; 
  Delayus(TIMEOUT_CPS); 
  CPS_DTA_H; 
  Delayus(TIMEOUT_CPS);
  CPS_SCK_L; 
  Delayus(TIMEOUT_CPS);
	CPS_DTA_R;//安全使用
} 

u8 CPS131_Send(u8 val)                  
{ 
  u8 i;  
  u8 err=0;
  
  CPS_DTA_W;

  for(i=0;i<8;i++)
  {
    if(val&0x80)
      CPS_DTA_H;
    else
      CPS_DTA_L;
      
    Delayus(TIMEOUT_CPS); 
    CPS_SCK_H;  
    Delayus(TIMEOUT_CPS);
    CPS_SCK_L;
    val<<=1;
    Delayus(TIMEOUT_CPS);
  } 
  
  CPS_DTA_H; 
  CPS_DTA_R;
  Delayus(TIMEOUT_CPS);
  CPS_SCK_H;  
  Delayus(TIMEOUT_CPS);
  if(CPS_DTA) 
    err=0; 
  else
    err=1;
  
  Delayus(TIMEOUT_CPS);
  CPS_SCK_L;
  Delayus(TIMEOUT_CPS);
  return err; 
} 

void CPS131_Transtart(void) 
{ 
  CPS_DTA_W; //PA0
  CPS_DTA_H; 
  CPS_SCK_H; 
  Delayus(TIMEOUT_CPS);
  CPS_DTA_L; 
  Delayus(TIMEOUT_CPS);
  CPS_SCK_L; 
  Delayus(TIMEOUT_CPS);
	CPS_DTA_R;//安全使用
} 

u16 GET_PRESSUE4(void) //大气压力
{   
  u16 value;
  u16 data[5]={0x00};
  
  CPS131_Transtart();
  CPS131_Send(0x50);
  CPS131_Send(0xAC);
  CPS131_Transtop();
  Delayms(100);//这个延时很重要，时间要大于37ms
  
  CPS131_Transtart(); 
  CPS131_Send(0x51);   //发送地址   
  data[0]=CPS131_Read(1); 
  data[1]=CPS131_Read(1); 
  data[2]=CPS131_Read(1);
  data[3]=CPS131_Read(1);
  data[4]=CPS131_Read(0);
  CPS131_Transtop(); 
  
  if((data[0]&0x60)==0x40)
  {
  	value=(data[1]<<8)|data[2];
  }
  else
  {
   	value=0xffff;
  }
  return value; 
}

u16 GET_level4(void) //液位压力
{   
  u16 value;
  u16 data[5]={0x00};
  
  CPS131_Transtart();
  CPS131_Send(0x50);
  CPS131_Send(0xAC);
  CPS131_Transtop();
  Delayms(100);//这个延时很重要，时间要大于37ms
  
  CPS131_Transtart(); 
  CPS131_Send(0x51);   //发送地址   
  data[0]=CPS131_Read(1); 
  data[1]=CPS131_Read(1); 
  data[2]=CPS131_Read(1);
  data[3]=CPS131_Read(0);  
  CPS131_Transtop(); 
  
  if((data[0]&0xC0)==0x00)
  {
  	value=(data[0]<<8)|data[1];
  }
  else
  {
   	value=0xffff;
  }
  return value; 
}

