#include "motor.h"
#include "tim.h"
#include "oled.h"
#include "car_control.h"
volatile int V_A,V_B,g_nMotorPulse,g_nMotor2Pulse;
 float va,vb,Aima_v, Aimb_v;

void GetMotorPulse(void)//读取电机脉冲
{
	g_nMotorPulse = (short)(__HAL_TIM_GET_COUNTER(&htim2));//获取计数器值
	V_A+=g_nMotorPulse ;
	__HAL_TIM_SET_COUNTER(&htim2,0);//TIM3计数器清零
 
	g_nMotor2Pulse = (short)(__HAL_TIM_GET_COUNTER(&htim8));//获取计数器值
	V_B+=g_nMotor2Pulse;
	__HAL_TIM_SET_COUNTER(&htim8,0);//TIM2计数器清零
}

void l_go(void)
{
	 //   HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_2, GPIO_PIN_RESET);
		 // HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_3, GPIO_PIN_SET);
}
void r_go(void)
{
		//  HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_5, GPIO_PIN_RESET);
		//  HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4, GPIO_PIN_SET);
}

void l_back(void)
{
	  //  HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_2, GPIO_PIN_SET);
		 // HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_3, GPIO_PIN_RESET);
}

void r_back(void)
{
		//  HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_5, GPIO_PIN_SET);
		//  HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4, GPIO_PIN_RESET);
}



int PID_A(int Aima_v)
{
  static	float Kp =0.1, Ki = 0, Kd = 0;
  int32_t aim , error_now = 0, error_last = 0, v_now = 0, error_i = 0, out = 0;
  aim = Aima_v;
	v_now = (-V_A);
	error_now = aim - v_now;
	error_i = error_i + error_now;
	out = Kp * error_now + Ki * error_i + Kd * (error_now - error_last);
	error_last=error_now;
	 OLED_Showdecimal8X16(0,2,v_now,7,2);
	if(V_A==0)return Aima_v;
	return (out+Aima_v);
}

int PID_B(int Aimb_v)
{
	 float Kp =0.1, Ki = 0, Kd = 0;
	 int16_t aim , error_now = 0, error_last = 0, v_now = 0, error_i = 0, out = 0;
   aim = Aimb_v;
	 v_now =( -V_B);
	 error_now = aim - v_now;
	 error_i = error_i + error_now;	
	 if(Aima_v==0)error_i=0;
	 out = Kp * error_now + Ki * error_i + Kd * (error_now - error_last);
	 error_last=error_now;
	 OLED_Showdecimal8X16(0,0,v_now,7,2);
	if(V_B==0)return Aimb_v;
   return (out+Aimb_v);
}

//int angle(float Angle,float Gyroy,float Mechanical_Angle)
//{
//	float Kp = 1.5; //       
 // float Kd = 0;//      
	//float Bias; //角度误差值out=(-4500+(out-4500)/10);
	//int balance_up; //直立环控制PWM
	//Bias=Angle-Mechanical_Angle; //角度误差值==测量的俯仰角-理想角度（机械平衡角度）
	//if (Bias>180) Bias=Bias-360;
	//balance_up= Kp*Bias+ Kd*Gyroy; //计算平衡控制的电机PWM  PD控制   Up_balance_KP是P系数,Up_balance_KD是D系数
	//return balance_up;
//}

int angle(float Angle, float Gyroy, float Mechanical_Angle)
{
    float Kp = 100;      // 比例系数
    float Ki = 0.1;      // 积分系数
    float Kd = 0.5;       // 微分系数 (你可以根据实际调试调整)
    float Bias;           // 当前角度误差
    static float last_Bias = 0;       // 上一次角度误差，用于计算微分项
    static float integral_error = 0;  // 积分项累加
    float derivative;     // 微分项
    int balance_up;       // 平衡控制输出PWM

    // 计算角度误差
    Bias = Angle - Mechanical_Angle;

    // 限制误差在 -180 到 180 度之间
    if (Bias > 180) Bias -= 360;
    if (Bias < -180) Bias += 360;

    // 积分项累积
    integral_error += Bias;

    // 可选：防止积分饱和（防止漂移过大）
 //   if (integral_error > 1000) integral_error = 1000;
  //  if (integral_error < -1000) integral_error = -1000;

    // 计算微分项（由当前误差变化率得到）
    derivative = Bias - last_Bias;
    last_Bias = Bias; // 保存本次误差供下次使用

    // PID 控制计算
    balance_up = Kp * Bias + Ki * integral_error + Kd * derivative;

    return balance_up;
}





