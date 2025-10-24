#ifndef __SPEED_SOLVER_H
#define __SPEED_SOLVER_H

#include "main.h"

// --- 函数声明 (Public Function Prototypes) ---

/**
 * @brief 初始化速度解算模块
 */
void Speed_Solver_Init(void);

/**
 * @brief 更新速度计算 (应在固定周期中断中调用，例如10ms)
 */
void Speed_Solver_Update(void);

/**
 * @brief 获取左轮的当前速度
 * @return float 左轮速度，单位：转/秒 (RPS)
 */
float Get_Left_Speed_RPS(void);

/**
 * @brief 获取右轮的当前速度
 * @return float 右轮速度，单位：转/秒 (RPS)
 */
float Get_Right_Speed_RPS(void);


#endif /* __SPEED_SOLVER_H */
