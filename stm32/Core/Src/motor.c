#include "motor.h"
#include "tim.h"
#include "oled.h"
#include "car_control.h"
volatile int V_A,V_B,g_nMotorPulse,g_nMotor2Pulse;
 float va,vb,Aima_v, Aimb_v;

void GetMotorPulse(void)//��ȡ�������
{
	g_nMotorPulse = (short)(__HAL_TIM_GET_COUNTER(&htim2));//��ȡ������ֵ
	V_A+=g_nMotorPulse ;
	__HAL_TIM_SET_COUNTER(&htim2,0);//TIM3����������
 
	g_nMotor2Pulse = (short)(__HAL_TIM_GET_COUNTER(&htim8));//��ȡ������ֵ
	V_B+=g_nMotor2Pulse;
	__HAL_TIM_SET_COUNTER(&htim8,0);//TIM2����������
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
	//float Bias; //�Ƕ����ֵout=(-4500+(out-4500)/10);
	//int balance_up; //ֱ��������PWM
	//Bias=Angle-Mechanical_Angle; //�Ƕ����ֵ==�����ĸ�����-����Ƕȣ���еƽ��Ƕȣ�
	//if (Bias>180) Bias=Bias-360;
	//balance_up= Kp*Bias+ Kd*Gyroy; //����ƽ����Ƶĵ��PWM  PD����   Up_balance_KP��Pϵ��,Up_balance_KD��Dϵ��
	//return balance_up;
//}

int angle(float Angle, float Gyroy, float Mechanical_Angle)
{
    float Kp = 100;      // ����ϵ��
    float Ki = 0.1;      // ����ϵ��
    float Kd = 0.5;       // ΢��ϵ�� (����Ը���ʵ�ʵ��Ե���)
    float Bias;           // ��ǰ�Ƕ����
    static float last_Bias = 0;       // ��һ�νǶ������ڼ���΢����
    static float integral_error = 0;  // �������ۼ�
    float derivative;     // ΢����
    int balance_up;       // ƽ��������PWM

    // ����Ƕ����
    Bias = Angle - Mechanical_Angle;

    // ��������� -180 �� 180 ��֮��
    if (Bias > 180) Bias -= 360;
    if (Bias < -180) Bias += 360;

    // �������ۻ�
    integral_error += Bias;

    // ��ѡ����ֹ���ֱ��ͣ���ֹƯ�ƹ���
 //   if (integral_error > 1000) integral_error = 1000;
  //  if (integral_error < -1000) integral_error = -1000;

    // ����΢����ɵ�ǰ���仯�ʵõ���
    derivative = Bias - last_Bias;
    last_Bias = Bias; // ���汾�����´�ʹ��

    // PID ���Ƽ���
    balance_up = Kp * Bias + Ki * integral_error + Kd * derivative;

    return balance_up;
}





