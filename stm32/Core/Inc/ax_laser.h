#ifndef __AX_LASER_H
#define __AX_LASER_H

#include "stm32f4xx_hal.h"

#define LS_F_LEN  12 // ʾ�����ȣ�����ʵ��Э���޸�
#define LS_HEADER1 0xA  // ��4λʾ������ʵ�ʶ����޸�
#define LS_HEADER2 0x5

typedef struct {
    uint16_t angle;
    uint16_t distance;
} LaserPointTypeDef;

void AX_LASER_Init(void);
void AX_LASER_RxCpltHandler(uint8_t rx_byte);
void AX_LASER_Start(void);
void AX_LASER_Stop(void);

extern LaserPointTypeDef ax_ls_point[250];

#endif
