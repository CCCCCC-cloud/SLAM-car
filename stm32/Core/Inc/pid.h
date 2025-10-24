#ifndef __PID_H
#define __PID_H

// PID控制器的数据结构
typedef struct {
    // 1. PID增益参数
    float Kp; // 比例增益
    float Ki; // 积分增益
    float Kd; // 微分增益

    // 2. 目标值
    float setpoint;

    // 3. 内部状态变量
    float integral;   // 积分累加值
    float last_error; // 上一次的误差 (用于计算微分)

    // 4. 输出限制
    float output_min; // 输出最小值
    float output_max; // 输出最大值
    
} PID_Controller_t;

// --- 函数声明 ---

/**
 * @brief 初始化一个PID控制器
 * @param pid 指向PID控制器结构体的指针
 * @param kp, ki, kd P, I, D增益
 * @param dt 控制周期，单位：秒 (例如 0.01s)
 * @param out_min, out_max PID输出的限制范围
 */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float out_min, float out_max);

/**
 * @brief 计算PID输出
 * @param pid 指向PID控制器结构体的指针
 * @param process_variable 当前的测量值 (例如，实际转速)
 * @return float 计算出的控制量 (例如，PWM调节值)
 */
float PID_Calculate(PID_Controller_t *pid, float process_variable);

/**
 * @brief 更新PID的目标值
 * @param pid 指向PID控制器结构体的指针
 * @param setpoint 新的目标值
 */
void PID_Set_Setpoint(PID_Controller_t *pid, float setpoint);

#endif /* __PID_H */
