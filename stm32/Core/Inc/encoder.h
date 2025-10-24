#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h" // ����������ͷ�ļ���ͨ��������HAL��Ķ���

// --- �������� ---

/**
  * @brief  ��ʼ��������������ʱ�� (TIM1 �� TIM2)
  * @param  None
  * @retval None
  */
void Encoder_Init(void);

/**
  * @brief  ��ȡ���������ֱ������ļ���ֵ
  * @note   �������ÿ�α����ú󣬶�Ӧ��ʱ���ļ���ֵ����㣬
  * ���ص��Ǵ��ϴε��õ����ε���֮�������������
  * @param  None
  * @retval int16_t �������� (������ʾ��ת��������ʾ��ת)
  */
int16_t Encoder_Read_Left(void);

/**
  * @brief  ��ȡ���������ֱ������ļ���ֵ
  * @note   ͬ�ϣ���ȡ��������������
  * @param  None
  * @retval int16_t ��������
  */
int16_t Encoder_Read_Right(void);

#endif /* __ENCODER_H */

