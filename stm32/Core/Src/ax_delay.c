#include "ax_delay.h"
#include "OLED.h"


  void AX_DELAY_Init(void)
{
    // 使能 DWT 外设
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk))
    {   
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; 
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;           
		//	OLED_ShowString8X16(0,0,"ROBOT delay init");
    }
}


void AX_Delayus(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (HAL_RCC_GetHCLKFreq() / 1000000);
    while ((DWT->CYCCNT - start) < cycles);
	//OLED_ShowString8X16(0,0,"ROBOT delay");
}


void AX_Delayms(uint32_t ms)
{
    HAL_Delay(ms);
	//OLED_ShowString8X16(0,0,"robotdelay2000");
}
