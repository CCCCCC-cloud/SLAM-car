#ifndef __AX_DELAY_H
#define __AX_DELAY_H

#include "stm32f4xx_hal.h"

void AX_DELAY_Init(void);         // 启用 DWT 计数器（用于 us 延时）
void AX_Delayus(uint32_t us);     // 微秒延时
void AX_Delayms(uint32_t ms);     // 毫秒延时（封装 HAL_Delay）

#endif
