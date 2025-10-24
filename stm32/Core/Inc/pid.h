#ifndef __PID_H
#define __PID_H

// PID�����������ݽṹ
typedef struct {
    // 1. PID�������
    float Kp; // ��������
    float Ki; // ��������
    float Kd; // ΢������

    // 2. Ŀ��ֵ
    float setpoint;

    // 3. �ڲ�״̬����
    float integral;   // �����ۼ�ֵ
    float last_error; // ��һ�ε���� (���ڼ���΢��)

    // 4. �������
    float output_min; // �����Сֵ
    float output_max; // ������ֵ
    
} PID_Controller_t;

// --- �������� ---

/**
 * @brief ��ʼ��һ��PID������
 * @param pid ָ��PID�������ṹ���ָ��
 * @param kp, ki, kd P, I, D����
 * @param dt �������ڣ���λ���� (���� 0.01s)
 * @param out_min, out_max PID��������Ʒ�Χ
 */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float out_min, float out_max);

/**
 * @brief ����PID���
 * @param pid ָ��PID�������ṹ���ָ��
 * @param process_variable ��ǰ�Ĳ���ֵ (���磬ʵ��ת��)
 * @return float ������Ŀ����� (���磬PWM����ֵ)
 */
float PID_Calculate(PID_Controller_t *pid, float process_variable);

/**
 * @brief ����PID��Ŀ��ֵ
 * @param pid ָ��PID�������ṹ���ָ��
 * @param setpoint �µ�Ŀ��ֵ
 */
void PID_Set_Setpoint(PID_Controller_t *pid, float setpoint);

#endif /* __PID_H */
