#ifndef __motor_H
#define __motor_H
#include "main.h"
////#include "mpu6050.h"
//#include "DMP_test.h"
 
#include "math.h"

extern float Aima_v, Aimb_v;

extern volatile int i,V_A,V_B,g_nMotorPulse,g_nMotor2Pulse;


extern short letf_wheel,right_wheel;//全局变量， 保存电机脉冲数值
void GetMotorPulse(void);//读取电机脉冲

void l_go(void);
void r_go(void);
void l_back(void);
void r_back(void);

void right(int out);
void left(int out);
int PID_A(int Aima_v);
int PID_B(int Aimb_v);
int angle(float Angle,float Gyroy,float Mechanical_Angle);
#endif
