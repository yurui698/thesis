#include "Stm32_Configuration.h"
#include "string.h"
#include "DataFlash.h"
#define DBGMCU_CR (*((volatile unsigned long *)0xE0042004))//在顶部加上宏定义 释放PB3使用
ErrorStatus HSEStartUpStatus;

extern u8 SOFTSPI;
__IO uint16_t ADC_ConvertedValue[4]={0,0,0,0};

void RCC_Configuration(void)
{   
  	/* RCC system reset(for debug purpose) */
  	RCC_DeInit();
  	/* Enable HSE */
  	RCC_HSEConfig(RCC_HSE_ON);
  	/* Wait till HSE is ready */
  	HSEStartUpStatus = RCC_WaitForHSEStartUp();

  	if(HSEStartUpStatus == SUCCESS)
  	{
    	/* HCLK = SYSCLK */
    	RCC_HCLKConfig(RCC_SYSCLK_Div1);   
    	/* PCLK2 = HCLK */
    	RCC_PCLK2Config(RCC_HCLK_Div1); 
    	/* PCLK1 = HCLK/2 */
    	RCC_PCLK1Config(RCC_HCLK_Div2);
    	/* Flash 2 wait state */
    	FLASH_SetLatency(FLASH_Latency_2);
    	/* Enable Prefetch Buffer */
    	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    	/* PLLCLK = 8MHz * 9 = 72 MHz */
    	RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
    	/* Enable PLL */ 
    	RCC_PLLCmd(ENABLE);

    	/* Wait till PLL is ready */
    	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    	{
    	}

    	/* Select PLL as system clock source */
    	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    	/* Wait till PLL is used as system clock source */
    	while(RCC_GetSYSCLKSource() != 0x08)
    	{
    	}
  	}
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); //采集
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	//释放PB3使用	
}



void GPIO_Configuration(void)
{
  	GPIO_InitTypeDef GPIO_InitStructure;
  	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
							RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
							RCC_APB2Periph_AFIO
							, ENABLE  );
  GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);//USART3的pin部分重新映射（PartialRemap，PC10_TX3，PC11_RX3）切换-PA15；定义唯一的，不能自由组合
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);	
//	  GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST,ENABLE); 

	/*PC.12无线收发数据指示灯*/
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_12;				        
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;		          
	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
	GPIO_Init(GPIOC, &GPIO_InitStructure);
//	LED_ON;

	/*PC.05 RS485有线收发数据指示灯*/
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_5;				        
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;		          
	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/*设备识别拨码开关管脚配置 PA.00,PC.02,PB.10,PB.11PB.08,PB.09 */
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_0;				        
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_IPU;		          		         
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_2;				        		          		         
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_10 | GPIO_Pin_11| GPIO_Pin_8 | GPIO_Pin_9;				        		          
	GPIO_Init(GPIOB, &GPIO_InitStructure);
/*去除JTAG*/
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);//释放PB3使用
	DBGMCU_CR &= 0xFFFFFFDF;  //禁止异步跟踪，释放PB3
	
	/*SI4463信道识别拨码开关管脚配置 PD.02 PB.03~PB.06*/
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_2;				        
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_IPU;		          		         
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |GPIO_Pin_6 ;
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_IPU; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*四路控制信号输出管脚配置 PC.03,PB.12,PB,13,PB,14 */

	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_3;				        
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;		          
	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC, GPIO_Pin_3);

	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
 	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;		          
	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	GPIO_SetBits(GPIOB, GPIO_Pin_13);
	GPIO_SetBits(GPIOB, GPIO_Pin_14);

	/*四路电流检测管脚配置*/
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_0 | GPIO_Pin_1;	
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AIN;			        		          
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_0 | GPIO_Pin_1;	
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AIN;			        		          
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/*SPI1管脚配置：PA.04--NSS;PA.05--SCK;PA.06--MISO;PA.07--MOSI */
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_4;				        
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;		          
	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	SPI1_NSS_HIGH;

	/*SI4463中断输出管脚nIRQ--PC.04及电源管脚SDN配置PC.06*/
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_4;				        
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_IPU;		          		         
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_6;				        
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;		          
	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* 串口2 发射管脚配置 PA.02 */
  	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_2;			         
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF_PP;		        
  	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
  	GPIO_Init(GPIOA, &GPIO_InitStructure);				                 
  	/* 串口2 接收管脚配置 PA.03 */
  	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_3;			         
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_IN_FLOATING;	         
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/*串口2 RS485收发控制管脚 PA.01 */
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_1;				        
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;		          
	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 串口1 发射管脚配置 PA.09 */
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_9;			         
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF_PP;		        
  	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
  	GPIO_Init(GPIOA, &GPIO_InitStructure);				                 
	/* 串口1 接收管脚配置 PA.10 */
  	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_10;			         
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_IN_FLOATING;	         
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
		
			/* 串口3 发射管脚配置 PC10 */
  	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_10;			         
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF_PP;		        
  	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
  	GPIO_Init(GPIOC, &GPIO_InitStructure);				                 
  /* 串口3 接收管脚配置 PC11 */
  	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_11;			         
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_IN_FLOATING;	         
  	GPIO_Init(GPIOC, &GPIO_InitStructure);
	/*串口3 RS485收发控制管脚 PA15 */
		GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_15;				        
		GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;		          
		GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
		GPIO_Init(GPIOA, &GPIO_InitStructure);
}


void EXTI_Configuration(void)
{
	EXTI_InitTypeDef EXTI_InitStructure; 

	EXTI_ClearITPendingBit(EXTI_Line4);
    /*将EXTI线4连接到PC.4*/
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource4);
    /*配置EXTI线4上出现下降沿，则产生中断*/
    EXTI_InitStructure.EXTI_Line    =  EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode    =  EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger =  EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd =  ENABLE;
    EXTI_Init(&EXTI_InitStructure);  	
}

void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel                   = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
  NVIC_InitStructure.NVIC_IRQChannel                   = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel                   = EXTI4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


	#ifdef   VECT_TAB_RAM   //如果程序在ram中调试那么定义中断向量表在Ram中否则在Flash中
    /* Set the Vector Table base location at 0x20000000 */
    NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);

    #else   /* VECT_TAB_FLASH   */
    /* Set the Vector Table base location at 0x08000000 */
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   

    #endif
}


void ADC1_Configuration(void)
{
	DMA_InitTypeDef DMA_InitStructure;
  	ADC_InitTypeDef ADC_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE );
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE );

	/* DMA channel1 configuration */
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;	 //ADC地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)ADC_ConvertedValue;////内存地址,修改通道数就要修改ADC_ConvertedValue
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 4;//4通道
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址固定
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//半字
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		//循环传输
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);	 	

   	/* Time Base configuration */
  	ADC_InitStructure.ADC_Mode               =  ADC_Mode_Independent;
  	ADC_InitStructure.ADC_ScanConvMode       =  ENABLE;
  	ADC_InitStructure.ADC_ContinuousConvMode =  ENABLE;
  	ADC_InitStructure.ADC_ExternalTrigConv   =  ADC_ExternalTrigConv_None;       
  	ADC_InitStructure.ADC_DataAlign          =  ADC_DataAlign_Right;             
  	ADC_InitStructure.ADC_NbrOfChannel       =  4;                            
  	ADC_Init(ADC1, &ADC_InitStructure);

 	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 2, ADC_SampleTime_239Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 3, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 4, ADC_SampleTime_239Cycles5);	

	ADC_DMACmd(ADC1,ENABLE);
	ADC_Cmd(ADC1, ENABLE);  	

	ADC_ResetCalibration(ADC1);
  	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
  	while(ADC_GetCalibrationStatus(ADC1)); 
}


void USART1_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef  USART_ClockInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 , ENABLE  );

	USART_ClockInitStructure.USART_Clock   =  USART_Clock_Disable;			    // 时钟低电平活动
	USART_ClockInitStructure.USART_CPOL    =  USART_CPOL_Low;				    // 时钟低电平
	USART_ClockInitStructure.USART_CPHA    =  USART_CPHA_2Edge;				    // 时钟第二个边沿进行数据捕获
	USART_ClockInitStructure.USART_LastBit =  USART_LastBit_Disable;		    // 最后一位数据的时钟脉冲不从SCLK输出
	/* Configure the USART1 synchronous paramters */
	USART_ClockInit(USART1, &USART_ClockInitStructure);					        // 时钟参数初始化设置
																	 
	USART_InitStructure.USART_BaudRate     =  9600;						        // 波特率为：115200
	USART_InitStructure.USART_WordLength   =  USART_WordLength_8b;			    // 8位数据
	USART_InitStructure.USART_StopBits     =  USART_StopBits_1;				    // 在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity       =  USART_Parity_No ;				    // 奇偶失能
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	// 硬件流控制失能

	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		        // 发送使能+接收使能
	/* Configure USART1 basic and asynchronous paramters */
	USART_Init(USART1, &USART_InitStructure);
    
	/* Enable USART1 */
	USART_ClearFlag(USART1, USART_IT_RXNE); 			                        //清中断，以免一启用中断后立即产生中断
	USART_ITConfig(USART1,USART_IT_RXNE, ENABLE);		                        //使能USART1中断源
	USART_Cmd(USART1, ENABLE); 
}

void USART2_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 , ENABLE);
																	 
	USART_InitStructure.USART_BaudRate     =  9600;						        // 波特率为：9600
	USART_InitStructure.USART_WordLength   =  USART_WordLength_8b;			    // 9位数据；奇偶校验
	USART_InitStructure.USART_StopBits     =  USART_StopBits_1;				    // 在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity       =  USART_Parity_No ;				    // 偶校验USART_Parity_Even
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //USART_HardwareFlowControl_RTS_CTS;	// 硬件流控制

	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		        // 发送使能+接收使能
	/* Configure USART1 basic and asynchronous paramters */
	USART_Init(USART2, &USART_InitStructure);
    
	/* Enable USART1 */
	USART_ClearFlag(USART2, USART_IT_RXNE); 			                        //清中断，以免一启用中断后立即产生中断
	USART_ITConfig(USART2,USART_IT_RXNE, ENABLE);		                        //使能USART2中断源
	USART_Cmd(USART2, ENABLE); 
}



void USART3_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 , ENABLE  );
																	 
	USART_InitStructure.USART_BaudRate     =  9600;						        // 波特率为：9600
	USART_InitStructure.USART_WordLength   =  USART_WordLength_8b;			    // 8位数据
	USART_InitStructure.USART_StopBits     =  USART_StopBits_1;				    // 在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity       =  USART_Parity_No ;				    // 奇偶失能
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	// 硬件流控制失能

	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		        // 发送使能+接收使能
	/* Configure USART3 basic and asynchronous paramters */
	USART_Init(USART3, &USART_InitStructure);
    
	/* Enable USART3 */
	USART_ClearFlag(USART3, USART_IT_RXNE); 			                        //清中断，以免一启用中断后立即产生中断
	USART_ITConfig(USART3,USART_IT_RXNE, ENABLE);		                        //使能USART3中断源
	USART_Cmd(USART3, ENABLE); 
}

void IWDG_Configuration(void)
{
    RCC_LSICmd(ENABLE);                              //打开LSI
    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY)==RESET);
 
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_32);
    IWDG_SetReload(2000);      //1000ms ,max 0xFFF  0~4095 
    IWDG_ReloadCounter();
    IWDG_Enable();
}

 void SPI1_Configuration(void)
 {
 	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO, ENABLE);
	if(softspi)
		{
		GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_5 | GPIO_Pin_7;				        
		GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;		          
		GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_6;			         
		GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_IN_FLOATING;	         
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	}
	else
	{
		SPI_Cmd(SPI1, DISABLE);
		GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;				        
		GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF_PP;		          
		GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	  	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	  	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	  	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	  	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	  	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	  	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	  	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	  	SPI_InitStructure.SPI_CRCPolynomial = 7;
	  	SPI_Init(SPI1, &SPI_InitStructure);
	  	/* Enable SPI1  */
	  	SPI_Cmd(SPI1, ENABLE);
      SPI_SSOutputCmd(SPI1, ENABLE);
	}
	Delayms(100);
  
 }
void SYS_Confgiuration(void)
{
	RCC_Configuration();
	GPIO_Configuration();
	EXTI_Configuration();		   	 		
	NVIC_Configuration();
	ADC1_Configuration();
	USART1_Configuration();
	USART2_Configuration();
	SPI1_Configuration(); 	
	IWDG_Configuration();
//	SI4463_Init();
	 
	SysTick->LOAD  = 0x00FFFFFF- 1;    
  	SysTick->VAL   = 0;
  	SysTick->CTRL  = 0x05;

	TimerInit();
	Start_timerEx(SYS_INIT_EVT,5000);
}
