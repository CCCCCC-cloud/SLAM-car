#ifndef __AX_DELAY_H
#define __AX_DELAY_H

#include "stm32f4xx_hal.h"

void AX_DELAY_Init(void);         // ���� DWT ������������ us ��ʱ��
void AX_Delayus(uint32_t us);     // ΢����ʱ
void AX_Delayms(uint32_t ms);     // ������ʱ����װ HAL_Delay��

#endif
