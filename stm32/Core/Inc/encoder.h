#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h" // 引入您的主头文件，通常包含了HAL库的定义

// --- 函数声明 ---

/**
  * @brief  初始化两个编码器定时器 (TIM1 和 TIM2)
  * @param  None
  * @retval None
  */
void Encoder_Init(void);

/**
  * @brief  读取并清零左轮编码器的计数值
  * @note   这个函数每次被调用后，对应定时器的计数值会归零，
  * 返回的是从上次调用到本次调用之间的脉冲增量。
  * @param  None
  * @retval int16_t 脉冲增量 (正数表示正转，负数表示反转)
  */
int16_t Encoder_Read_Left(void);

/**
  * @brief  读取并清零右轮编码器的计数值
  * @note   同上，读取的是脉冲增量。
  * @param  None
  * @retval int16_t 脉冲增量
  */
int16_t Encoder_Read_Right(void);

#endif /* __ENCODER_H */

