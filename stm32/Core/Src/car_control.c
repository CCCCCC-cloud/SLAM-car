
#include "car_control.h"
#include "math.h"

#include "OLED.h"

#include "tim.h"
#include "pid.h"           
#include "speed_solver.h"
#include "mpu6050.h"
#include "motor.h"
// 3. Define the maximum and minimum PWM periods and associate them with the timer
#define PWM_MAX_PERIOD          8400
#define PWM_MIN_PERIOD          4000
#define MOTOR_TIM               htim3 // Ensure that htim3 is available in this file (usually included through tim.h)


#define MOTOR_A_IN1_CHANNEL     TIM_CHANNEL_1
#define MOTOR_A_IN2_CHANNEL     TIM_CHANNEL_2


#define MOTOR_B_IN1_CHANNEL     TIM_CHANNEL_3
#define MOTOR_B_IN2_CHANNEL     TIM_CHANNEL_4
const float Kp = 15.0f;
const float Ki = 0.0f;
const float Kd =5.0f;
#define ANGLE_TOLERANCE 1.0f
#define DERIVATIVE_TOLERANCE 0.5f

#define PWM_MIN 4500
#define PWM_MAX 8000
#define MOTOR_DEADBAND 4500
extern volatile uint8_t status;
extern MPU6050_t mpu6050;
extern volatile float yaw_angle;
static PID_Controller_t s_pid_left;  // Left wheel PID controller instance
static PID_Controller_t s_pid_right; // Right wheel PID controller instance
extern volatile int V_A;
// 4. Declare the functions thatvo will be defined as static
// static void means these functions cannot be used in other files (e.g., they are only called in main.c)
// They must be included in the main function for execution.
static void motorA_goForward(uint16_t speed);
static void motorA_goBackward(uint16_t speed);
static void motorA_stop(void);
static void motorB_goForward(uint16_t speed);
static void motorB_goBackward(uint16_t speed);
static void motorB_stop(void);
static void Motor_Set_Pwm_Left(int16_t pwm);
static void Motor_Set_Pwm_Right(int16_t pwm);

void Speed_PID_Init(void)
{
	
    float initial_Kp = 600.0f;
    float initial_Ki = 3.0f; 
    float initial_Kd = 50.0f;
    
      PID_Init(&s_pid_left, initial_Kp, initial_Ki, initial_Kd, -PWM_MAX_PERIOD, PWM_MAX_PERIOD);
      PID_Init(&s_pid_right, initial_Kp, initial_Ki, initial_Kd, -PWM_MAX_PERIOD, PWM_MAX_PERIOD);
	
	
    PID_Set_Setpoint(&s_pid_left, 0.0f);
    PID_Set_Setpoint(&s_pid_right, 0.0f);
	
    s_pid_left.integral = 0.0f;
    s_pid_right.integral = 0.0f;
    s_pid_left.last_error = 0.0f;
    s_pid_right.last_error = 0.0f;
}

void Speed_PID_Update(void)
{
    float actual_speed_left = Get_Left_Speed_RPS();
    //float actual_speed_right = Get_Right_Speed_RPS();

    int16_t pwm_left = (int16_t)PID_Calculate(&s_pid_left, actual_speed_left);
    //int16_t pwm_right = (int16_t)PID_Calculate(&s_pid_right, actual_speed_right);
    
   
    Motor_Set_Pwm_Left(pwm_left);
    //Motor_Set_Pwm_Right(pwm_right);
	  motorB_stop(); 
}

static void Motor_Set_Pwm_Left(int16_t pwm)//// Function to set PWM for the left motor
{
    if (pwm > 0) 
    {
        
        motorA_goForward((uint16_t)pwm + MOTOR_DEADBAND);
    }
    else if (pwm < 0)
    {
        
        motorA_goBackward((uint16_t)(-pwm) + MOTOR_DEADBAND);
    }
    else 
    {
        
        motorA_stop();
    }
}

static void Motor_Set_Pwm_Right(int16_t pwm)
{
    if (pwm > 0) 
    {
        motorB_goForward((uint16_t)pwm + MOTOR_DEADBAND);
    }
    else if (pwm < 0) 
    {
        motorB_goBackward((uint16_t)(-pwm) + MOTOR_DEADBAND);
    }
    else 
    {
        motorB_stop();
    }
}




void GoForward_PID(float target_rps) {
    	s_pid_left.integral = 0.0f;
      s_pid_right.integral = 0.0f;
      s_pid_left.last_error = 0.0f;
      s_pid_right.last_error = 0.0f;
      PID_Set_Setpoint(&s_pid_left, target_rps);
    //PID_Set_Setpoint(&s_pid_right, target_rps);
}

void Stop_PID(void) {
    PID_Set_Setpoint(&s_pid_left,  0.0f);
    PID_Set_Setpoint(&s_pid_right, 0.0f);
	  s_pid_left.integral = 0.0f;
    s_pid_right.integral = 0.0f;
}


void goForward(uint16_t speed) {
    motorA_goForward(speed);
    motorB_goForward(speed);
}

void goBackward(uint16_t speed) {
    motorA_goBackward(speed);
    motorB_goBackward(speed);
}
//ֹͣ
void stopMotors() {
    motorA_stop();
    motorB_stop();
}

void turnLeft(uint16_t speed) {
    motorA_goBackward(speed); 
	motorB_goForward(speed);  
}

void turnRight(uint16_t speed) {
    motorA_goForward(speed);  
    motorB_goBackward(speed); 
}


void PID_TurnToLeft(float angle){
 // Initial angle is set to yaw
	//int i=0;
	 float yaw=mpu6050.KalmanAngleZ;
	float start_angle = yaw;
    float target_angle = start_angle+angle ;
	//OLED_Showdecimal8X16(0,2,yaw,3,3);
// Target angle adjusted to [–180, 180] range
  
   // if (target_angle < 0.f) {
     //   target_angle += 360.0f;
  //  }
   //  if(target_angle>360.f){
	//	 target_angle-=360.f;
		// }
    float error, last_error = 0, integral = 0, derivative;
    float output;
    uint16_t pwm;

    while (1)
    {  
      // === 1. Calculate error ===
			 // MPU6050_DMP_Get_Date(&pitch,&roll,&yaw);
		//	MPU6050_Read_All(&hi2c1,&mpu6050);
			 yaw=mpu6050.KalmanAngleZ;
		//	OLED_Showdecimal8X16(0,2,yaw,3,3);
        error = target_angle - yaw;
			

        
       // if (error > 180.0f) error -= 360.0f;
       // if (error < -180.0f) error += 360.0f;

        // === 2. PID Control ===
        integral += error;
        derivative = error - last_error;
        last_error = error;

        output = Kp * error + Ki * integral + Kd * derivative;
        
       // === 3. Control output limits ===
        if (output > PWM_MAX) output = PWM_MAX;
        if (output < -PWM_MAX) output = -PWM_MAX;

        pwm = (uint16_t)(fabsf(output));
        if (pwm < PWM_MIN) pwm = PWM_MIN;

       // === 4. Turn left or right (A backward, B forward) ===
        if (output > 0)
        {
            motorA_goBackward(pwm);  
            motorB_goForward(pwm);   
        }
        else
        {
           
            motorA_goForward(pwm);
            motorB_goBackward(pwm);
        }

       if (fabsf(error) < ANGLE_TOLERANCE&& fabsf(derivative) < DERIVATIVE_TOLERANCE)
        { 
					break;
        }
				

        HAL_Delay(10); 
    }

   
    motorA_stop();
    motorB_stop();
		 
   // yaw_angle=0;

}

void PID_TurnToRight(void){
	//the same implement logic of that of PID_TurnToRight
	float yaw;
	
float start_angle = yaw;
    float target_angle = start_angle +90.0f;

   
    if (target_angle < -180.0f) {
        target_angle += 360.0f;
    }

    float error, last_error = 0, integral = 0, derivative;
    float output;
    uint16_t pwm;

    while (1)
    {
       
		//	  MPU6050_DMP_Get_Date(&pitch,&roll,&yaw);
			 OLED_Showdecimal8X16(0,2,yaw,3,3);
        error = target_angle - yaw;

       
        if (error > 180.0f) error -= 360.0f;
        if (error < -180.0f) error += 360.0f;

        integral += error;
        derivative = error - last_error;
        last_error = error;

        output = Kp * error + Ki * integral + Kd * derivative;

        
        if (output > PWM_MAX) output = PWM_MAX;
        if (output < -PWM_MAX) output = -PWM_MAX;

        pwm = (uint16_t)(fabsf(output));
        if (pwm < PWM_MIN) pwm = PWM_MIN;

        if (output > 0)
        {
            motorA_goBackward(pwm);  
            motorB_goForward(pwm);   
        }
        else
        {
           
            motorA_goForward(pwm);
            motorB_goBackward(pwm);
        }

      
        if (fabsf(error) < ANGLE_TOLERANCE && fabsf(derivative) < DERIVATIVE_TOLERANCE)
        {
            break; 
        }
         if (fabsf(error) < ANGLE_TOLERANCE && fabsf(derivative) > DERIVATIVE_TOLERANCE)
        {
            break; 
        }
				
        HAL_Delay(10);  
    }

 
    motorA_stop();
    motorB_stop();


}

int  limit(int out)
{ 
	if(0<=out&&out<4400)
		out=4500;
	else if(out>=4400&&out<=4500)
		out=4500;
  else	if(out>7000)      //依照最大输出占空比来限制，我的最大为999
		out=7000;
	else if(out<-7000)
		out=-7000;
	else if(out<0&&out>-4500)
		out=-4500;
	//out=(-4500+(out-4500)/10);
return out;
}

void PID1(void){
	status=2;

float	 yaw=mpu6050.KalmanAngleZ;
	//  if(yaw>180)yaw=yaw-360;
		OLED_Showdecimal8X16(0,2,yaw,7,2);
	
      float output = angle(yaw,0,0);
	    //int  output1 = PID_A(4700 -output);
      //int  output2 = PID_B( 4700+  output);
	//	OLED_Showdecimal8X16(0,0,output,5,3);
		// OLED_Showdecimal8X16(0,4,output1,5,3);
//int 	output3=limit(output);
//OLED_Showdecimal8X16(0,2,V_A,5,3);
	   motorA_goBackward(4800);
     right(4800+output);
     
}



 void motorA_goForward(uint16_t speed)
{
    if (speed > PWM_MAX_PERIOD) speed = PWM_MAX_PERIOD;
    __HAL_TIM_SET_COMPARE(&MOTOR_TIM, MOTOR_A_IN1_CHANNEL, speed);
    __HAL_TIM_SET_COMPARE(&MOTOR_TIM, MOTOR_A_IN2_CHANNEL, 0);
}

 void motorA_goBackward(uint16_t speed)
{
    if (speed > PWM_MAX_PERIOD) speed = PWM_MAX_PERIOD;
    __HAL_TIM_SET_COMPARE(&MOTOR_TIM, MOTOR_A_IN1_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&MOTOR_TIM, MOTOR_A_IN2_CHANNEL, speed);
}

 void motorA_stop(void)
{
    __HAL_TIM_SET_COMPARE(&MOTOR_TIM, MOTOR_A_IN1_CHANNEL, PWM_MAX_PERIOD);
    __HAL_TIM_SET_COMPARE(&MOTOR_TIM, MOTOR_A_IN2_CHANNEL, PWM_MAX_PERIOD);
}


 void motorB_goForward(uint16_t speed)
{
    if (speed > PWM_MAX_PERIOD) speed = PWM_MAX_PERIOD;
    __HAL_TIM_SET_COMPARE(&MOTOR_TIM, MOTOR_B_IN1_CHANNEL, speed);
    __HAL_TIM_SET_COMPARE(&MOTOR_TIM, MOTOR_B_IN2_CHANNEL, 0);
}

 void motorB_goBackward(uint16_t speed)
{
    if (speed > PWM_MAX_PERIOD) speed = PWM_MAX_PERIOD;
    __HAL_TIM_SET_COMPARE(&MOTOR_TIM, MOTOR_B_IN1_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(&MOTOR_TIM, MOTOR_B_IN2_CHANNEL, speed);
}

 void motorB_stop(void)
{
    __HAL_TIM_SET_COMPARE(&MOTOR_TIM, MOTOR_B_IN1_CHANNEL, PWM_MAX_PERIOD);
    __HAL_TIM_SET_COMPARE(&MOTOR_TIM, MOTOR_B_IN2_CHANNEL, PWM_MAX_PERIOD);
}

void right(int out)
{
	out=limit(out);
		  OLED_Showdecimal8X16(0,6,out,5,3);
	if(out>0)
 {   motorB_goBackward(out);}
  else if(out<0)
 { motorB_goForward(-out);}
}

void left(int out)
{
out=limit(out);
	OLED_Showdecimal8X16(0,4,out,5,3);
	if(out>0)
 {   motorA_goForward(out);}
  else if(out<0)
 { motorA_goBackward(-out);}
}

