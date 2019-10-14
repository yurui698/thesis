#include "Stm32_Configuration.h"

int main(void)
{	
	SYS_Confgiuration();
	softspi = 0;	 		 //	  切换为硬件SPI
	SPI1_Configuration();	 //	  对SPI重新进行配置	
  	while(1)
    {
		SysClock();		
    }				   	 
}
