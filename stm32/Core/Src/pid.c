#include "pid.h"

// Assuming our control cycle period is fixed (for example, determined by TIM6 at 10ms)
#define PID_CONTROL_PERIOD_S    0.01f 

void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->output_min = out_min;
    pid->output_max = out_max;
}

float PID_Calculate(PID_Controller_t *pid, float process_variable)
{
    float error = pid->setpoint - process_variable;

    float p_term = pid->Kp * error;
    
    float i_term = pid->Ki * pid->integral;

    float derivative = (error - pid->last_error) / PID_CONTROL_PERIOD_S;
    float d_term = pid->Kd * derivative;

    pid->last_error = error;

    float output = p_term + i_term + d_term;
    // ---- New, more effective anti-windup strategy ----
    // Only allow integral accumulation if output is within bounds
    if (output < pid->output_max && output > pid->output_min)
    {
        pid->integral += error * PID_CONTROL_PERIOD_S;
    }

   
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    }

    return output;
}

void PID_Set_Setpoint(PID_Controller_t *pid, float setpoint)
{
    pid->setpoint = setpoint;
}
