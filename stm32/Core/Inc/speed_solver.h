#ifndef __SPEED_SOLVER_H
#define __SPEED_SOLVER_H

#include "main.h"

// --- �������� (Public Function Prototypes) ---

/**
 * @brief ��ʼ���ٶȽ���ģ��
 */
void Speed_Solver_Init(void);

/**
 * @brief �����ٶȼ��� (Ӧ�ڹ̶������ж��е��ã�����10ms)
 */
void Speed_Solver_Update(void);

/**
 * @brief ��ȡ���ֵĵ�ǰ�ٶ�
 * @return float �����ٶȣ���λ��ת/�� (RPS)
 */
float Get_Left_Speed_RPS(void);

/**
 * @brief ��ȡ���ֵĵ�ǰ�ٶ�
 * @return float �����ٶȣ���λ��ת/�� (RPS)
 */
float Get_Right_Speed_RPS(void);


#endif /* __SPEED_SOLVER_H */
