#include "Stm32_Configuration.h"
#include "string.h"
#include "DataFlash.h"

ErrorStatus HSEStartUpStatus;

extern u8 SOFTSPI;//采集
extern u8 RecDataBuffer1[];
extern u8 USART1SendTCB[];
extern u8 RecDataBuffer2[];
extern u8 USART2SendTCB[];
extern u8 RecDataBuffer3[];
extern u8 USART3SendTCB[];
extern u8 RecDataBuffer4[];
extern u8 UART4SendTCB[];
__IO uint16_t ADC_ConvertedValue[2]={0,0};//采集
extern u8 factory_gateway_set[];
#define FALSE   0
#define TRUE    1

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
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);//采集	
}

/*   STM32的四种输出模式:
1.普通推挽输出（GPIO_Mode_Out_PP）
2.普通开漏输出（GPIO_Mode_Out_OD）
3.复用推挽输出（GPIO_Mode_AF_PP）:用作串口的输出
4.复用开漏输出（GPIO_Mode_AF_OD）：用在IIC*/

void GPIO_Configuration(void)
{
  	GPIO_InitTypeDef GPIO_InitStructure;

  	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
							RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
							RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE  );

	/*USART2、USART3 Alternate Function mapping*/
	GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE); //USART2的pin重新映射（Remap）；定义唯一的，不能自由组合 GPIO_Remap_USART2
	GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);//USART3的pin全部重新映射（FullRemap）；定义唯一的，不能自由组合

	/* 设置PE0为推挽输出,用于有线通信运行指示（与小板通讯指示，即：串口1通讯指示） */
  	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_0;                           
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;				     
  	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		             
  	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOE, GPIO_Pin_0);  //说明Px端口，如：GPIOE为PE端口
	
	/* 设置PB6为推挽输出,用于无线通信运行指示（与M433HZ小板通讯指示，即：SPI1通讯指示） */
  	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_6;                           
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;				     
  	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		             
  	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);  //说明Px端口，如：GPIOB为PB端口	

	/*设置PD14为推挽输出，用于控制CDMA的开关机*/
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_14;                           
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;				     
  	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		             
  	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOD, GPIO_Pin_14);	 

	/*设置PD13为推挽输出，用于控制CDMA的重置*/
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_13;                           
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;				     
  	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		             
  	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOD, GPIO_Pin_13);	 

	/*设置PD15为推挽输出，用于控制CDMA注入休眠，本程序暂不使用*/
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_15;                           
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;				     
  	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		             
  	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOD, GPIO_Pin_15);	 

	/* Configure USART1 Tx (PA.09) as alternate function push-pull */
	  GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_9;			         
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF_PP;		        
  	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
  	GPIO_Init(GPIOA, &GPIO_InitStructure);				                 
	/* Configure USART1 Rx (PA.10) as input floating */
  	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_10;			         
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_IN_FLOATING;	         
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

  	/* Configure USART2 Tx (PD.05) as alternate function push-pull */
  	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_5;			         
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF_PP;		        
  	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
  	GPIO_Init(GPIOD, &GPIO_InitStructure);				                 
  	/* Configure USART2 Rx (PD.06) as input floating */
  	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_6;			         
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_IN_FLOATING;	         
  	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* 设置PD4为复用推挽输出,为串口2的RTS脚 */
  	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_4;                           
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF_PP;				   
  	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		             
  	GPIO_Init(GPIOD, &GPIO_InitStructure);
	  GPIO_ResetBits(GPIOD, GPIO_Pin_4);

	/*PD3为串口2的CTS脚*/
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_3;			         
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_IN_FLOATING;	         
  	GPIO_Init(GPIOD, &GPIO_InitStructure);
 
  	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_7;                           
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF_PP;				   
  	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		             
  	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_SetBits(GPIOD, GPIO_Pin_7);

	/* Configure USART3 Tx (PD.08) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_8;			         
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF_PP;		        
  	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
  	GPIO_Init(GPIOD, &GPIO_InitStructure);				                 
	/* Configure USART3 Rx (PD.09) as input floating */
  	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_9;			         
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_IN_FLOATING;	         
  	GPIO_Init(GPIOD, &GPIO_InitStructure);
	/*485收发切换*/
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_10;				        
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;		          
	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
		/* Configure UART4 Tx (PC.10) as alternate function push-pull */
	  GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_10;			         
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF_PP;	//根据GPS需要修改	        
  	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
  	GPIO_Init(GPIOC, &GPIO_InitStructure);				                 
	/* Configure UART4 Rx (PC.11) as input floating */
  	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_11;			         
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_IN_FLOATING;	         
  	GPIO_Init(GPIOC, &GPIO_InitStructure);
		/*485收发切换*/
		GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_9;				        
		GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;		          
		GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
		GPIO_Init(GPIOB, &GPIO_InitStructure);	
		
		/* Configure UART5 Tx (PC.12) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_12;			         
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF_PP;		        
  	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
  	GPIO_Init(GPIOC, &GPIO_InitStructure);				                 
	/* Configure UART5 Rx (PD.02) as input floating */
  	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_2;			         
  	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_IN_FLOATING;	         
  	GPIO_Init(GPIOD, &GPIO_InitStructure);
	/*485收发切换*/
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_8;				        
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;		          
	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
	GPIO_Init(GPIOB, &GPIO_InitStructure);	 
 
/*PB.6程序运行指示灯*/
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_6;				        
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;		          
	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	WX_LED_ON;

	/*SI4463信道识别拨码开关管脚配置 PD.02 PB.03~PB.09*/
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | 
									   GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_IPU;				        		          
	GPIO_Init(GPIOE, &GPIO_InitStructure);


	/*SPI1管脚配置：PA.04--NSS;PA.05--SCK;PA.06--MISO;PA.07--MOSI */
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_4;				        
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;		          
	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	SPI1_NSS_HIGH;

	/*SI4463中断输出管脚nIRQ--PC.04及电源管脚SDN配置PB.07*/
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_4;				        
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_IPU;		          		         
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_7;				        
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;		          
	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	
/*五路输入管脚配置 PC1土壤温度DATA2、PC2土壤水分DATA1;PC0光照度DATA3;PA0二氧化碳DATA5;PA1空气温湿度DATA4;PC3 为SCK;
  输入输出：PE724V电源输出控制 */
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_1 | GPIO_Pin_2;	
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AIN;	//模拟量输入（PC1：土壤温度；PC2土壤水分）		        		          
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_3;	//开关量输出PC3为SCK
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;			        		          
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_0;	//开关量输出PC0为光照
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_IN_FLOATING;//未定义，悬浮输入
	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;			        		          
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_0 |GPIO_Pin_1;	//开关量输出（PA0二氧化碳；PA1空气温湿度）
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_IN_FLOATING;//未定义，悬浮输入
	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;			        		          
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_7;	//开关量输出（PE7蜂鸣器）
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;			        		          
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	}

void BH1750_DATA_R(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC , ENABLE  );

	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_0;				        
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_IN_FLOATING;		          		         
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
}

void BH1750_DATA_W(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC , ENABLE  );

	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_0;				        
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;		          
	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void BH1750_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB , ENABLE  );

	GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_13;				        
	GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_Out_PP;		          
	GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_50MHz;		         
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
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
//采集定义中断结束
	
}

void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel                   = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	 
	NVIC_InitStructure.NVIC_IRQChannel                   = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
  NVIC_InitStructure.NVIC_IRQChannel                   = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
	NVIC_InitStructure.NVIC_IRQChannel                   = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
//采集中断优先级
  NVIC_InitStructure.NVIC_IRQChannel                   = EXTI4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;   // 发送DMA通道的中断配置
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 优先级设置
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	
		
		 /* Enable the TIM2 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                        = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority      = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority             = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                     = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

		 /* Enable the TIM5 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                        = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority      = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority             = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                     = ENABLE;
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
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)ADC_ConvertedValue;//内存地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 2;
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
  	ADC_InitStructure.ADC_NbrOfChannel       =  2;                            
  	ADC_Init(ADC1, &ADC_InitStructure);
  
 	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 2, ADC_SampleTime_239Cycles5); 

	ADC_DMACmd(ADC1,ENABLE);
	ADC_Cmd(ADC1, ENABLE);  	

	ADC_ResetCalibration(ADC1);
  	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
  	while(ADC_GetCalibrationStatus(ADC1)); 
}


//void USART1_Configuration(void)
//{
//	USART_InitTypeDef USART_InitStructure;
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 , ENABLE  );
//																	 
//	USART_InitStructure.USART_BaudRate     =  9600;						        // 下载程序为波特率为：115200，boot0管脚拉高，用户程序不执行
//	USART_InitStructure.USART_WordLength   =  USART_WordLength_8b;			    // 8位数据
//	USART_InitStructure.USART_StopBits     =  USART_StopBits_1;				    // 在帧结尾传输1个停止位
//	USART_InitStructure.USART_Parity       =  USART_Parity_No ;				    // 奇偶失能
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	// 硬件流控制失能

//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		        // 发送使能+接收使能
//	/* Configure USART1 basic and asynchronous paramters */
//	USART_Init(USART1, &USART_InitStructure);
//    
//	/* Enable USART1 */
//	  USART_ClearFlag(USART1, USART_IT_RXNE); 			                        //清中断，以免一启用中断后立即产生中断	
//    USART_ITConfig(USART1,USART_IT_RXNE, ENABLE);		                        //使能USART1接收中断

//	USART_Cmd(USART1, ENABLE); 
//}

void UART5_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5 , ENABLE  );
	
	USART_InitStructure.USART_BaudRate     =  9600;						        // 默认配置参数
	USART_InitStructure.USART_WordLength   =  USART_WordLength_8b;			    // 8位数据；有校验位为9位
	USART_InitStructure.USART_StopBits     =  USART_StopBits_1;				    // 在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity       =  USART_Parity_No ;				    // 奇偶失能；USART_Parity_No；流量计为偶校验
	if(factory_gateway_set[27]==13)         //超声波流量计（果红军）
  {		
		USART_InitStructure.USART_BaudRate     =  2400;						        // =13 超声波流量计；=16 超声波流量计+超声波液位
		USART_InitStructure.USART_WordLength   =  USART_WordLength_9b;			    // 8位数据；有校验位为9位
		USART_InitStructure.USART_StopBits     =  USART_StopBits_1;				    // 在帧结尾传输1个停止位
		USART_InitStructure.USART_Parity       =  USART_Parity_Even ;				    // 奇偶失能；USART_Parity_No；流量计为偶校验
	}
	if(factory_gateway_set[27]==14||factory_gateway_set[27]==16||factory_gateway_set[27]==17)         //电磁流量计上海帆杨、科霸流量计
  {		
		USART_InitStructure.USART_BaudRate     =  2400;						        // =14 电磁流量计；=17 电磁流量计+超声波液位
		USART_InitStructure.USART_WordLength   =  USART_WordLength_8b;			    // 8位数据；有校验位为9位
		USART_InitStructure.USART_StopBits     =  USART_StopBits_1;				    // 在帧结尾传输1个停止位
		USART_InitStructure.USART_Parity       =  USART_Parity_No ;				    // 奇偶失能；USART_Parity_No；流量计为偶校验
	}
	
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	// 硬件流控制失能

	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		        // 发送使能+接收使能
	/* Configure UART5 basic and asynchronous paramters */
	USART_Init(UART5, &USART_InitStructure);
    
	/* Enable UART5 */
	USART_ClearFlag(UART5, USART_IT_RXNE); 			                        //清中断，以免一启用中断后立即产生中断
//  USART_ClearFlag(UART5, USART_IT_TC);                                   //清除中断标志
	
//   USART_ITConfig(UART5,USART_IT_IDLE, ENABLE); 
//    USART_ITConfig(UART5,USART_FLAG_ORE, ENABLE);                               //使能UART5空闲中断 
	USART_ITConfig(UART5,USART_IT_RXNE, ENABLE);		                        //使能USART5中断源
	USART_Cmd(UART5, ENABLE); 
}


void IWDG_Configuration(void)
{
    RCC_LSICmd(ENABLE);                              //打开LSI
    while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY)==RESET); 
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_32);	  //32
    IWDG_SetReload(3000);      				
    IWDG_ReloadCounter();
    IWDG_Enable();//调试时关闭看门狗
}

void SPI1_Configuration(void)
 {
 	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO, ENABLE);
	if(softspi){
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
/*DMA1(7个)、DMA2（5个）定义：USART3接收（未用，采用中断方式接收），DMA通道与具体使用的外设是定义好的，不能随便使用；
 DMA1_Channel 1：A/D转换
 DMA1_Channel 2：USART3 发送
 DMA1_Channel 3：未用
 DMA1_Channel 4：USART1发送
 DMA1_Channel 5：USART1接收
 DMA1_Channel 6：USART2接收
 DMA1_Channel 7：USART2发送
 DMA2_Channel 1：未用
 DMA2_Channel 2：未用
 DMA2_Channel 3：UART4接收
 DMA2_Channel 4：未用
 DMA2_Channel 5：UART4发送
 */
/*--- Usart1 DMA发送+DMA接收 Config ---------------------------------------*/ 
void Uart1_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
 
    /* System Clocks Configuration */
//= System Clocks Configuration ====================================================================//
   
    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,  ENABLE ); // 开启串口所在IO端口的时钟
    /* Enable USART Clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // 开始串口时钟
   
   
//=NVIC_Configuration==============================================================================//
 
    /* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
 
    /* Enable the DMA Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;   // 发送DMA通道的中断配置
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 优先级设置
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
 
    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;     // 串口中断配置，接收
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
   
//=GPIO_Configuration==============================================================================//
 
//    GPIO_PinRemapConfig(GPIO_FullRemap_USART1, ENABLE);  // 我这里没有用默认IO口，所以进行了重新映射，这个可以根据自己的硬件情况配置选择
   
    /* Configure USART1 Rx as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   // 串口接收IO口的设置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//USART1--RX管脚
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
    /* Configure USART1 Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // 串口发送IO口的设置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // 这里设置成复用形式的推挽输出   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//USART1--TX管脚
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
   
 
    /* USART Format configuration ------------------------------------------------------*/
    /* Configure USART1 */
		//串口1默认设定
		USART_InitStructure.USART_BaudRate = 9600;		
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;    // 串口格式配置
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
		if(factory_gateway_set[0]==3)  //有线以太网设定
      { 
			  USART_InitStructure.USART_BaudRate = 115200;
			  USART_InitStructure.USART_WordLength = USART_WordLength_8b;    // 串口格式配置
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_No;
			} 

    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; 
    
    USART_Init(USART1, &USART_InitStructure);
 
    /* Enable USART1 Receive and Transmit interrupts */
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);  // 开启 串口空闲IDEL 中断
//		USART_ClearFlag(USART1, USART_IT_RXNE);             //清中断，以免一启用中断后立即产生中断  
//	  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);		   
   
    /* Enable the USART1 */
    USART_Cmd(USART1, ENABLE);  // 开启串口
    /* Enable USARTy DMA TX request */
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);  // 开启串口DMA发送
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE); // 开启串口DMA接收

}
 
 
void DMA_Uart1_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
   
    /* DMA clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); // 开启DMA1时钟
   
   
//=DMA_Configuration==============================================================================//
 
/*--- 串口1 UART_Tx_DMA_Channel DMA Config 使用DMA1_Channel4、DMA1_Channel5---*/    
    DMA_Cmd(DMA1_Channel4, DISABLE);                           // 关DMA通道
    DMA_DeInit(DMA1_Channel4);                                 // 恢复缺省值
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);// 设置串口发送数据寄存器
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART1SendTCB;         // 设置发送缓冲区首地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                      // 设置外设位目标，内存缓冲区 -> 外设寄存器
    DMA_InitStructure.DMA_BufferSize = 256;                     // 需要发送的字节数，这里其实可以设置为0，因为在实际要发送的时候，会重新设置次值
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // 外设地址不做增加调整，调整不调整是DMA自动实现的
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // 内存缓冲区地址增加调整
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据宽度8位，1个字节
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // 内存数据宽度8位，1个字节
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // 单次传输模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // 优先级设置
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // 关闭内存到内存的DMA模式
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);               // 写入配置
    DMA_ClearFlag(DMA1_FLAG_GL4);                                 // 清除DMA所有标志，DMA1_FLAG_TC4 | DMA1_FLAG_TE4 
    DMA_Cmd(DMA1_Channel4, DISABLE); // 关闭DMA
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);            // 开启发送DMA通道中断
   
/*--- DMA1_Channel5 DMA Config ---*/
 
    DMA_Cmd(DMA1_Channel5, DISABLE);                           // 关DMA通道，接收
    DMA_DeInit(DMA1_Channel5);                                 // 恢复缺省值
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);// 设置串口接收数据寄存器
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RecDataBuffer1;         // 设置接收缓冲区首地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                      // 设置外设为数据源，外设寄存器 -> 内存缓冲区
    DMA_InitStructure.DMA_BufferSize = 256;                     // 需要最大可能接收到的字节数
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // 外设地址不做增加调整，调整不调整是DMA自动实现的
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // 内存缓冲区地址增加调整
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据宽度8位，1个字节
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // 内存数据宽度8位，1个字节
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // 单次传输模式;循环模式DMA_Mode_Circular
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // 优先级设置
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // 关闭内存到内存的DMA模式
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);               // 写入配置
    DMA_ClearFlag(DMA1_FLAG_GL5);                                 // 清除DMA所有标志
    DMA_Cmd(DMA1_Channel5, ENABLE);                            // 开启接收DMA通道，等待接收数据
   
}

//串口1--DMA定义结束 
 
/*--- Usart2 DMA发送+DMA接收 Config ---------------------------------------*/ 
void Uart2_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
 
    /* System Clocks Configuration */
//= System Clocks Configuration ====================================================================//
   
    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO,  ENABLE ); // 开启串口所在IO端口的时钟
    /* Enable USART Clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // 开始串口时钟
   
   
//=NVIC_Configuration==============================================================================//
 
    /* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
 
    /* Enable the DMA Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;   // 发送DMA通道的中断配置
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     // 优先级设置
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
 
    /* Enable the USART Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;     // 串口中断配置，接收
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
   
//=GPIO_Configuration==============================================================================//
 
//    GPIO_PinRemapConfig(GPIO_FullRemap_USART2, ENABLE);  // 我这里没有用默认IO口，所以进行了重新映射，这个可以根据自己的硬件情况配置选择
   
    /* Configure USART2 Rx as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   // 串口接收IO口的设置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;//USART3--RX管脚
    GPIO_Init(GPIOD, &GPIO_InitStructure);
 
    /* Configure USART3 Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // 串口发送IO口的设置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // 这里设置成复用形式的推挽输出   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;//USART3--TX管脚
    GPIO_Init(GPIOD, &GPIO_InitStructure);
 
   
 
    /* USART Format configuration ------------------------------------------------------*/
 
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;    // 串口格式配置
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
    /* Configure USART2 */
    USART_InitStructure.USART_BaudRate = 115200;  //  波特率设置
    USART_Init(USART2, &USART_InitStructure);
 
    /* Enable USART3 Receive and Transmit interrupts */
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);  // 开启 串口空闲IDEL 中断
//		USART_ClearFlag(USART2, USART_IT_RXNE);             //清中断，以免一启用中断后立即产生中断  
//	  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);		   
   
    /* Enable the USART2 */
    USART_Cmd(USART2, ENABLE);  // 开启串口
    /* Enable USARTy DMA TX request */
    USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);  // 开启串口DMA发送
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE); // 开启串口DMA接收

}
 
 
void DMA_Uart2_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
   
    /* DMA clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); // 开启DMA1时钟
   
   
//=DMA_Configuration==============================================================================//
 
/*--- 串口2 UART_Tx_DMA_Channel DMA Config 使用DMA1_Channel7、DMA1_Channel6---*/    
    DMA_Cmd(DMA1_Channel7, DISABLE);                           // 关DMA通道
    DMA_DeInit(DMA1_Channel7);                                 // 恢复缺省值
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);// 设置串口发送数据寄存器
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART2SendTCB;         // 设置发送缓冲区首地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                      // 设置外设位目标，内存缓冲区 -> 外设寄存器
    DMA_InitStructure.DMA_BufferSize = 256;                     // 需要发送的字节数，这里其实可以设置为0，因为在实际要发送的时候，会重新设置次值
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // 外设地址不做增加调整，调整不调整是DMA自动实现的
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // 内存缓冲区地址增加调整
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据宽度8位，1个字节
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // 内存数据宽度8位，1个字节
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // 单次传输模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // 优先级设置
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // 关闭内存到内存的DMA模式
    DMA_Init(DMA1_Channel7, &DMA_InitStructure);               // 写入配置
    DMA_ClearFlag(DMA1_FLAG_GL7);                                 // 清除DMA所有标志，DMA1_FLAG_TC4 | DMA1_FLAG_TE4 
    DMA_Cmd(DMA1_Channel7, DISABLE); // 关闭DMA
    DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);            // 开启发送DMA通道中断
   
/*--- DMA1_Channel6 DMA Config ---*/
 
    DMA_Cmd(DMA1_Channel6, DISABLE);                           // 关DMA通道，接收
    DMA_DeInit(DMA1_Channel6);                                 // 恢复缺省值
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);// 设置串口接收数据寄存器
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RecDataBuffer2;         // 设置接收缓冲区首地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                      // 设置外设为数据源，外设寄存器 -> 内存缓冲区
    DMA_InitStructure.DMA_BufferSize = 256;                     // 需要最大可能接收到的字节数
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // 外设地址不做增加调整，调整不调整是DMA自动实现的
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // 内存缓冲区地址增加调整
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据宽度8位，1个字节
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // 内存数据宽度8位，1个字节
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // 单次传输模式;循环模式DMA_Mode_Circular
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // 优先级设置
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // 关闭内存到内存的DMA模式
    DMA_Init(DMA1_Channel6, &DMA_InitStructure);               // 写入配置
    DMA_ClearFlag(DMA1_FLAG_GL6);                                 // 清除DMA所有标志
    DMA_Cmd(DMA1_Channel6, ENABLE);                            // 开启接收DMA通道，等待接收数据
   
}

//串口2--DMA定义结束 

 
/*--- Usart3 DMA发送+中断接收 Config ---------------------------------------*/ 
 
void Uart3_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
 
    /* System Clocks Configuration */
//= System Clocks Configuration ====================================================================//
   
    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD ,  ENABLE ); // 开启串口所在IO端口的时钟
    /* Enable USART Clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); // 开始串口时钟
   
   
//=NVIC_Configuration==============================================================================//
 
    /* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
 
    /* Enable the DMA Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;   // 发送DMA通道的中断配置
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     // 优先级设置
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
 
    /* Enable the USART Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;     // 串口中断配置
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
   
//=GPIO_Configuration==============================================================================//
 
//    GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);  // 我这里没有用默认IO口，所以进行了重新映射，这个可以根据自己的硬件情况配置选择
   
    /* Configure USART3 Rx as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   // 串口接收IO口的设置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//USART3--RX管脚
    GPIO_Init(GPIOD, &GPIO_InitStructure);
 
    /* Configure USART3 Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // 串口发送IO口的设置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // 这里设置成复用形式的推挽输出   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;//USART3--TX管脚
    GPIO_Init(GPIOD, &GPIO_InitStructure);
 
   
 
    /* USART Format configuration ------------------------------------------------------*/
 
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;    // 串口格式配置
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;//USART_Parity_Even  USART_Parity_No
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
 
    /* Configure USART3 */
    USART_InitStructure.USART_BaudRate = 9600;  //  波特率设置
    USART_Init(USART3, &USART_InitStructure);
 
    /* Enable USART3 Receive and Transmit interrupts */
//    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);  // 开启 串口空闲IDEL 中断
		USART_ClearFlag(USART3, USART_IT_RXNE);             //清中断，以免一启用中断后立即产生中断  
	  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);		   
   
    /* Enable the USART3 */
    USART_Cmd(USART3, ENABLE);  // 开启串口
    /* Enable USARTy DMA TX request */
    USART_DMACmd(USART3, USART_DMAReq_Tx, DISABLE);  // 开启串口DMA发送
//    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE); // 开启串口DMA接收

}
 
 
void DMA_Uart3_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
   
    /* DMA clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); // 开启DMA1时钟
   
   
//=DMA_Configuration==============================================================================//
 
/*--- 串口3 UART_Tx_DMA_Channel DMA Config 使用DMA1_Channel2---*/    
    DMA_Cmd(DMA1_Channel2, DISABLE);                           // 关DMA通道
    DMA_DeInit(DMA1_Channel2);                                 // 恢复缺省值
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);// 设置串口发送数据寄存器
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART3SendTCB;         // 设置发送缓冲区首地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                      // 设置外设位目标，内存缓冲区 -> 外设寄存器
    DMA_InitStructure.DMA_BufferSize = 128;                     // 需要发送的字节数，这里其实可以设置为0，因为在实际要发送的时候，会重新设置次值
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // 外设地址不做增加调整，调整不调整是DMA自动实现的
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // 内存缓冲区地址增加调整
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据宽度8位，1个字节
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // 内存数据宽度8位，1个字节
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // 单次传输模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // 优先级设置
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                            // 关闭内存到内存的DMA模式
    DMA_Init(DMA1_Channel2, &DMA_InitStructure);               // 写入配置
    DMA_ClearFlag(DMA1_FLAG_GL2);                                 // 清除DMA所有标志，DMA1_FLAG_TC2 | DMA1_FLAG_TE2 
    DMA_Cmd(DMA1_Channel2, DISABLE); // 关闭DMA
    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);            // 开启发送DMA通道中断
}

//串口3--DMA定义结束
//串口4--DMA定义开始
/*1.时钟RCC配置：串口时钟 + DMA时钟 + IO时钟*/
void Uart4_Init(void)	
{      
	 NVIC_InitTypeDef NVIC_InitStructure;
   GPIO_InitTypeDef GPIO_InitStructure;
   USART_InitTypeDef USART_InitStructure;
	
       RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);  //串口时钟       
       RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2,ENABLE);  //DMA2时钟 
       RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC| RCC_APB2Periph_AFIO, ENABLE);//IO时钟
       
 
/*2.GPIO配置：
UART4的TX为PC10脚，发送端配置为复用推挽输出模式（GPIO_Mode_AF_PP）
UART4的RX为PC11脚，接收端配置为浮空输入模式（GPIO_Mode_IN_FLOATING）*/
                            
       GPIO_InitStructure.GPIO_Pin =GPIO_Pin_10;              
       GPIO_InitStructure.GPIO_Mode =GPIO_Mode_AF_PP;  //TX复用推挽输出模式
       GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz;
       GPIO_Init(GPIOC,&GPIO_InitStructure);               
 
       GPIO_InitStructure.GPIO_Pin =GPIO_Pin_11;             
       GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //RX浮空输入模式
       GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz;
       GPIO_Init(GPIOC,&GPIO_InitStructure);

 
/*3.中断NVIC配置：配置两个DMA通道中断：
UART4的RX的DMA通道为DMA2的通道3；
UART4的TX的DMA通道为DMA2的通道5；*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
   
    NVIC_InitStructure.NVIC_IRQChannel =UART4_IRQn;//串口中断，接收
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority= 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd =ENABLE;
    NVIC_Init(&NVIC_InitStructure);      

    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel4_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelCmd= ENABLE; 
    NVIC_Init(&NVIC_InitStructure);

 
/*4.串口配置：即填充串口配置结构体*/
     
       USART_InitStructure.USART_BaudRate =115200;
       USART_InitStructure.USART_WordLength =USART_WordLength_8b;//数据位8位
       USART_InitStructure.USART_StopBits =USART_StopBits_1;//停止位1位
       USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
       USART_InitStructure.USART_HardwareFlowControl= USART_HardwareFlowControl_None;//不采用硬件流控
       USART_InitStructure.USART_Mode =USART_Mode_Rx | USART_Mode_Tx;//TX、RX都开启 
       USART_Init(UART4,&USART_InitStructure);
			 USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);  // 开启 串口空闲IDEL 中断
       USART_Cmd(UART4, ENABLE); //使能UART4外设
			 USART_DMACmd(UART4, USART_DMAReq_Tx,DISABLE);//配置串口向DMA发出Tx请求，请求传输数据
			 USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);//配置串口向DMA发出Rx请求，请求传输数据
}
 
/*5.DMA配置：
DMA可以把数据从外设转移到内存（如串口接收的时候），也可以从内存转移到外设（如串口发送的时候）；
不同方向的数据转移要各做相应的配置*/
 
//串口发送：
void DMA_Uart4_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;   
   
    /* DMA2 Channel5  Config */
    DMA_InitStructure.DMA_PeripheralBaseAddr =(u32)&(UART4->DR);//外设基地址，串口4数据寄存器
    DMA_InitStructure.DMA_MemoryBaseAddr =(u32)UART4SendTCB;//内存基地址，数组UART4_DMA_HeadBuf
    DMA_InitStructure.DMA_DIR =DMA_DIR_PeripheralDST; //内存到DST外设
    DMA_InitStructure.DMA_BufferSize =128;//DMA数据传输长度
    DMA_InitStructure.DMA_PeripheralInc =DMA_PeripheralInc_Disable;//外设地址不自增
    DMA_InitStructure.DMA_MemoryInc =DMA_MemoryInc_Enable;//内存地址自增
    DMA_InitStructure.DMA_PeripheralDataSize =DMA_PeripheralDataSize_Byte;//外设数据单位为1字节
    DMA_InitStructure.DMA_MemoryDataSize =DMA_MemoryDataSize_Byte;//内存数据单位为1字节
    DMA_InitStructure.DMA_Mode =DMA_Mode_Normal;//DMA传输数据模式，正常模式，传一轮
    DMA_InitStructure.DMA_Priority =DMA_Priority_High;//DMA通道优先级
    DMA_InitStructure.DMA_M2M =DMA_M2M_Disable;//禁止DMA内存到内存传输      

    DMA_Init(DMA2_Channel5,&DMA_InitStructure);
	  DMA_ClearFlag(DMA2_FLAG_GL5);     // 清除DMA所有标志，DMA2_FLAG_TC5 | DMA1_FLAG_TE5
	  DMA_Cmd(DMA2_Channel5, DISABLE);//正式开启DMA
    DMA_ITConfig(DMA2_Channel5, DMA_IT_TC,ENABLE);//配置DMA2发送完成后产生中断
     
//串口接收：     
    DMA_InitStructure.DMA_PeripheralBaseAddr =(u32)(&UART4->DR);//外设基地址，串口4数据寄存器
    DMA_InitStructure.DMA_MemoryBaseAddr =(uint32_t)RecDataBuffer4; 
    DMA_InitStructure.DMA_DIR =DMA_DIR_PeripheralSRC;//外设到内存SRC 
    DMA_InitStructure.DMA_BufferSize =128; 
    DMA_InitStructure.DMA_PeripheralInc =DMA_PeripheralInc_Disable; 
    DMA_InitStructure.DMA_MemoryInc =DMA_MemoryInc_Enable; 
    DMA_InitStructure.DMA_PeripheralDataSize =DMA_PeripheralDataSize_Byte; 
    DMA_InitStructure.DMA_MemoryDataSize =DMA_PeripheralDataSize_Byte; 
    DMA_InitStructure.DMA_Mode =DMA_Mode_Normal; 
    DMA_InitStructure.DMA_Priority =DMA_Priority_High; 
    DMA_InitStructure.DMA_M2M =DMA_M2M_Disable;             

    DMA_Init(DMA2_Channel3,&DMA_InitStructure);          
    DMA_ClearFlag(DMA2_FLAG_GL3);              // 清除DMA所有标志
    DMA_Cmd(DMA2_Channel3, ENABLE);//正式开启DMA 
		DMA_ITConfig(DMA2_Channel3, DMA_IT_TC,ENABLE);
}
//串口4--DMA定义结束

void TIM2_Configuration(void)//周期1s
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
	TIM_DeInit(TIM2);

	TIM_TimeBaseStructure.TIM_Prescaler = 7200-1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period    = 10000-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);

	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;	             //下降沿捕获
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;					 //每个下降沿都捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;			 
	TIM_ICInitStructure.TIM_ICFilter = 0x01;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;//PA0；硬件规定的；光照
	TIM_ICInit(TIM2,&TIM_ICInitStructure);
	
	TIM_ITConfig(TIM2,TIM_IT_CC1,DISABLE);
	TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE );	
}

void TIM5_Configuration(void)//周期1us
{
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5 , ENABLE);
	TIM_DeInit(TIM5);
	
	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period    = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);

	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;	 //上、下降沿捕获TIM_ICPolarity_BothEdge
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;					 //每个上、下降沿都捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;			 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;//PA1；硬件规定的；空气温湿度
	TIM_ICInit(TIM5,&TIM_ICInitStructure);	
	
	TIM_ITConfig(TIM5,TIM_IT_CC2,DISABLE);
	TIM_ITConfig(TIM5,TIM_IT_Update,DISABLE ); 
}
 void EXTI1_Enable(Boolean bEnable)//PA1,空气温湿度
{
	NVIC_InitTypeDef NVIC_InitStructure;  
  EXTI_InitTypeDef EXTI_InitStructure;  
    EXTI_ClearITPendingBit(EXTI_Line1);  
  
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = bEnable ? ENABLE : DISABLE;  
    NVIC_Init(&NVIC_InitStructure);
	 
	  EXTI_ClearITPendingBit(EXTI_Line1);
    /*将EXTI线1连接到PA1，空气温湿度数据线*/
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource1);
    /*配置EXTI线1上出现下降沿，则产生中断*/
    EXTI_InitStructure.EXTI_Line    =  EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode    =  EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger =  EXTI_Trigger_Falling;//EXTI_Trigger_Rising;EXTI_Trigger_Rising_Falling上升沿下降沿触发
    EXTI_InitStructure.EXTI_LineCmd =  bEnable ? ENABLE : DISABLE;  
    EXTI_Init(&EXTI_InitStructure);  
}

void EXIT0_Enable(Boolean bEnable)
{
	NVIC_InitTypeDef NVIC_InitStructure;  
  EXTI_InitTypeDef EXTI_InitStructure; 
    EXTI_ClearITPendingBit(EXTI_Line0);  
  
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = bEnable ? ENABLE : DISABLE;  
    NVIC_Init(&NVIC_InitStructure);	
	
	  EXTI_ClearITPendingBit(EXTI_Line0);
    /*将EXTI线0连接到PC.0,光照度数据线*/
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource0);
    /*配置EXTI线0上出现下降沿，则产生中断*/
    EXTI_InitStructure.EXTI_Line    =  EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode    =  EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger =  EXTI_Trigger_Falling;//EXTI_Trigger_Falling;EXTI_Trigger_Rising_Falling上升沿下降沿触发
    EXTI_InitStructure.EXTI_LineCmd =  bEnable ? ENABLE : DISABLE;  
    EXTI_Init(&EXTI_InitStructure);	  
}
void wgfrist_init_readflash(void)     //网关初始化读flash;flash从0x0800 0000~0x0807 FFFF，共512k
{
  u8 init_flash_flg[2]={0,0};
	   Flash_Read(0x0807B000,  init_flash_flg, 2);//触摸屏下发出厂网关设定参数需要写入flash0x0807 C000，从256k开始写入，0x0804加1，则增加64k,每次写入必需2Kbyte
	   if(init_flash_flg[0]!=0xFF&&init_flash_flg[1]!=0xFF) //如果已经经过出厂配置，读出厂网关设定。否则使用程序初值
		 {
			 Flash_Read(0x0807B000,  factory_gateway_set, 221);			
		 }		 
}
void SYS_Confgiuration(void)
{	
	RCC_Configuration();
	GPIO_Configuration();
	EXTI_Configuration(); 		   	 		
	NVIC_Configuration();
  wgfrist_init_readflash();
	UART5_Configuration();
	SPI1_Configuration(); 	
	IWDG_Configuration();
	ADC1_Configuration();//采集配置 
	YX_LED_ON;	 //点亮有线指示灯
	WX_LED_ON;	 //点亮无线指示灯
	SysTick->LOAD  = 0x00FFFFFF- 1;    
  SysTick->VAL   = 0;
  SysTick->CTRL  = 0x05;
	TimerInit();
  TIM5_Configuration();
	TIM2_Configuration();
	Uart1_Init();
  DMA_Uart1_Init();   // 串口1 DMA 配置
	Uart2_Init();
  DMA_Uart2_Init();   // 串口2 DMA 配置
	Uart3_Init();
  DMA_Uart3_Init();   // 串口3 DMA 配置
	Uart4_Init();
  DMA_Uart4_Init();   // 串口4 DMA 配置
	GPIO_ResetBits(GPIOD, GPIO_Pin_10);	 //USART3收使能
	GPIO_ResetBits(GPIOB, GPIO_Pin_9);//UART4收使能
	Set_Event(SYS_INIT_EVT);
}
