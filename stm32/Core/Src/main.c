/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include <string.h>
#include "communication.h"
#include "car_control.h"
#include "OLED.h"
#include "encoder.h"
#include <stdio.h> 
#include "speed_solver.h"
#include "pid.h"
#include "laser_task.h"
#include "motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
volatile float current_yaw;
extern float angle_new,distance_new;
extern volatile int t;
extern volatile int V_A,V_B;
extern volatile float yaw_angle;
extern uint8_t uart3_rx_buffer[ ];
//extern volatile float angle_array[ ];   // 存储计算出的角度
//extern volatile float distance_array[ ]; // 存储计算出的距离
volatile int flag=0;
volatile int choice=0;
  char message[520*50];  // 每个数据项大约50个字符, 一共360个数据项
       int message_length = 0;   // 用来记录总消息长度
#define BUFFER_SIZE  2600
//#define HALF_SIZE    (BUFFER_SIZE / 2)
extern uint8_t buffer1[BUFFER_SIZE];
volatile int cmd2;
// 使用 volatile 来防止编译器优化
volatile uint16_t angle_q6[520];   // 存储计算出的角度
volatile uint16_t distance_q2[520]; // 存储计算出的距离
volatile uint8_t Quality[520];
volatile float angle_t[520];
volatile 	int w=0;
volatile int cmd_id=0;
volatile uint8_t status=0;
float angle_array[520];
float distance_array[520];
volatile float distance;
volatile int goal=0;

typedef struct __attribute__((packed)) {
    uint8_t Quality;    
    uint16_t angle_q6;  
    uint16_t distance_q2; 
} RPLIDAR_Scan_Data_Send;

typedef struct __attribute__((packed)) {
    uint16_t data_count;
		RPLIDAR_Scan_Data_Send send_data[520];
} RPLIDAR_Send_Data;

void send_packetPacNew(
		uint8_t Send_Begin_1,
    uint8_t Send_Begin_2,

    uint32_t time_us,
    uint16_t cmd_id,
    uint8_t status,

    int encoder_l,
    int encoder_r,
    float current_yaw
	//	RPLIDAR_Send_Data rplidar520
    
) ;
RPLIDAR_Send_Data createRPLIDARData(
     uint8_t Quality[520],
    uint16_t angle_q6[520],
     uint16_t distance_q2[520]
) ;
RPLIDAR_Send_Data createRPLIDARData(
     uint8_t Quality[520],
    uint16_t angle_q6[520],
    uint16_t distance_q2[520]
); 
		
		 struct __attribute__((packed)) {
        uint8_t Send_Begin_1; // ???????1
        uint8_t Send_Begin_2; // ???????2
        uint32_t time_us;     // ???
        uint16_t cmd_id;      // ??ID
        uint8_t status;       // ??
        int encoder_l;        // ????
        int encoder_r;        // ????
        float current_yaw;    // ????
     //  RPLIDAR_Send_Data rplidar_data; // LIDAR??
    } packet;
		 uint8_t tx_buffer[2600];  // DMA 发送用缓冲区
//encoder_l == V_A  encoder_r == V_B
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile int16_t g_speed_pulse_left = 0;
volatile int16_t g_speed_pulse_right = 0;
 RPLIDAR_Send_Data rplidar_data; 
PID_Controller_t pid_left;  
PID_Controller_t pid_right; 
volatile int tip=0;
volatile int tip2=0;
MPU6050_t mpu6050;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void processCommand(uint8_t command);
void Safe_UART_Transmit(UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size, uint32_t Timeout);
void send_packetPac(float anglePac,float distancePac,float pitchPac,float rollPac,float yawPac,float speedLeftPac,float speedRightPac);
int clc(float d){

return (int)(d*6625);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define PWM_MAX_PERIOD          8400

#define MOTOR_TIM               htim3
//static uint32_t tickCnt = 0;

/*void HAL_SYSTICK_Callback(void)
{
  tickCnt++;
  // 姣? 100 ms 瑙﹀彂涓?娆″彂閫?
  if (tickCnt >= 100) {
    tickCnt = 0;
		float left_speed = Get_Left_Speed_RPS();
		float right_speed = Get_Right_Speed_RPS();
		
    send_packetPac(angle_new,distance_new,pitch,roll,yaw,left_speed,right_speed);
  }
}*/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_UART4_Init();
  MX_TIM7_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
 HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	stopMotors();
	Encoder_Init(); 
	Speed_Solver_Init();
	Communication_Init();
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start(&htim7);
	Speed_PID_Init(); 
	OLED_Init();
	OLED_ShowString8X16(0,0,"56789");
		int ret=1;
	do{
	ret=MPU6050_Init(&hi2c1);
			
	}while(ret);
	OLED_ShowString8X16(0,0,"12345");
		__HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_UPDATE);
		Laser_Task_Init();

//	HAL_TIM_Base_Start_IT(&htim5);
	//HAL_TIM_Base_Start_IT(&htim5);
	
	
	
	//PID1();
	
	
	
	//goForward(4500);
//	OLED_ShowString8X16(0,0,"MPU6050 Init");

	
	
	 
//	stopMotors( );


//	int ret=0;
	//do{
//	ret=MPU6050_DMP_init();
//			
//	}while(ret);
  
	  //HAL_Delay(1000);
		
	  //float set_speed = 3.0f;
		//GoForward_PID(set_speed); 
		
		
    //HAL_Delay(1000);
    //Stop_PID();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//OLED_Showdecimal8X16(0,2,V_A,5,3);
		//OLED_ShowNum8X16(0,4,tip,8);
		while(tip==1){
			//OLED_ShowString8X16(0,6,"it itff");
			 CommandPacket_t temp_cmd;
        memcpy(&temp_cmd, &uart3_rx_buffer[2], sizeof(CommandPacket_t));
	if(temp_cmd.data_len==10){
				cmd2=temp_cmd.cmd_id;
				OLED_ShowNum8X16(0,2,cmd2,8);
	//OLED_ShowString8X16(0,2,"it itff");
		OLED_Showdecimal8X16(0,4,temp_cmd.turn_rad,6,3);
			if(temp_cmd.turn_rad<185&&temp_cmd.turn_rad>-185){
			status=1;
			PID_TurnToLeft(temp_cmd.turn_rad);
			yaw_angle=0;
			status=2;
				HAL_Delay(7);
			distance=	temp_cmd.distance;
			goal=V_A+clc(distance);
				
			while(V_A<goal){
			 GetMotorPulse();
				PID1();
			
			}
			stopMotors();
			
		 
			//	
			//	OLED_ShowNum8X16(0,2,status,8);
			
		
			
		}}
	  status=0;
		tip=0;
		cmd_id=cmd2;
		HAL_UART_Receive_DMA(&huart3, uart3_rx_buffer, 16);
		
		}
		status=0;
		//OLED_Showdecimal8X16(0,4,yaw_angle,6,3);
	//	PID_TurnToLeft(180);
	//		OLED_Showdecimal8X16(0,4,current_yaw,6,3);
	//	yaw_angle=0;
	//	HAL_Delay(3000);
		
		//PID_TurnToLeft(180);
	//	OLED_Showdecimal8X16(0,2,mpu6050.KalmanAngleZ,3,3);
		//HAL_Delay(100);
		
		 //HAL_Delay(300);
	//while(tip==1){
	//	PID_TurnToLeft(CommandPacket_t.turn_rad);
		

//  }
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void processCommand(uint8_t command)
{
  
  uint16_t move_speed = 5500;
  uint16_t turn_speed = 5000;
//	OLED_ShowString8X16(0,4,"456");
	
  switch (command)
  {
    case 0x01:
      goForward(move_speed);
		  //HAL_UART_Transmit(&huart3, &command, 1, HAL_MAX_DELAY); 
      break;
    case 0x02:
      goBackward(move_speed);
		  //HAL_UART_Transmit(&huart3, &command, 1, HAL_MAX_DELAY);
      break;
    case 0x03:
      stopMotors();
		  //HAL_UART_Transmit(&huart3, &command, 1, HAL_MAX_DELAY);
      break;
    case 0x04:
			//PID_TurnToLeft();
      //PID_TurnToLeft();
		  //HAL_UART_Transmit(&huart3, &command, 1, HAL_MAX_DELAY);
      break;
    case 0x05:
			//PID_TurnToRight();
      turnRight(turn_speed);
		  //HAL_UART_Transmit(&huart3, &command, 1, HAL_MAX_DELAY);
      break;
    default:
    
		  stopMotors();
      break;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //中断10ms1次 
{
	
	if(htim == (&htim5))//100ms中断
	{  if(flag==1){  
	
		//volatile uint8_t dummy = status;
	
	/*	 volatile uint8_t *data = buffer1;  // 使用 volatile 以确保每次都读取最新值
			  int j=0;
		//	send_packetPacNew(0x55,0xAA,HAL_GetTick(),(int)cmd_id,(uint8_t)status,(int)V_A,(int)V_B,(float)current_yaw)	;
	//	rplidar_data.data_count=520;
	//	HAL_UART_Transmit(&huart3, (uint8_t*)&rplidar_data.data_count, sizeof(rplidar_data.data_count), HAL_MAX_DELAY);
       for (uint16_t i = 0; i < BUFFER_SIZE; i += 5)   // 这里根据你的数据帧长度修改步进
        { // rplidar_data.send_data[j].Quality=(uint8_t)data[i];
           //rplidar_data.send_data[j].angle_q6 = (((uint16_t)(data[i + 2] << 7) + (data[i + 1] ))) ;
					angle_t[j++]=(((uint16_t)(data[i + 2] << 7) + (data[i + 1]>>1 ))) /64.0f;}
          // rplidar_data.send_data[j++].distance_q2 = (((uint16_t)(data[i + 4] << 8) + (data[i + 3]))) ;}
					//send_packetPacNew(0x55,0xAA,HAL_GetTick(),(int)cmd_id,(uint8_t)status,(int)V_A,(int)V_B,(float)current_yaw)	;
					// rplidar_data.data_count=520;
		     //  HAL_UART_Transmit(&huart3, (uint8_t*)&rplidar_data.data_count, sizeof(rplidar_data.data_count), HAL_MAX_DELAY);
				//	 for(int t=0;t<j;t++){
        //    HAL_UART_Transmit(&huart3, &rplidar_data.send_data[t].Quality, sizeof(rplidar_data.send_data[t].Quality), HAL_MAX_DELAY);
         //    HAL_UART_Transmit(&huart3,(uint8_t*) &rplidar_data.send_data[t].angle_q6, sizeof(rplidar_data.send_data[t].angle_q6), HAL_MAX_DELAY);
         //     HAL_UART_Transmit(&huart3,(uint8_t*) &rplidar_data.send_data[t].distance_q2, sizeof(rplidar_data.send_data[t].distance_q2), HAL_MAX_DELAY);}
				//	HAL_UART_Transmit(&huart3, &rplidar_data.send_data[j].angle_q6, sizeof(rplidar_data.send_data[j].angle_q6), HAL_MAX_DELAY);
       
				
				//createRPLIDARData((uint8_t*)Quality,(uint16_t*)angle_q6,(uint16_t*)distance_q2);
		//send_packetPacNew(0x55,0xAA,HAL_GetTick(),(int)cmd_id,(uint8_t)status,(int)V_A,(int)V_B,(float)current_yaw,rplidar_data)	;
			  for (uint16_t i = 0; i< 520; i++) {
       //格式化每一对 angle 和 distance 数据，存入 message
        int len = sprintf(message + message_length, "angle: %f;\r\n", angle_t[i]);
        message_length += len; // 更新总消息长度
    }
			  int len = sprintf(message + message_length, "over\n");
		   message_length += len; 

        // 通过 UART 发送所有的数据
      HAL_UART_Transmit(&huart3, (uint8_t*)message, message_length,HAL_MAX_DELAY);	
		message_length=0;
	//	HAL_UART_Transmit_DMA(&huart3,buffer1,2600);
			 cmd_id++;
	 */
	//	OLED_ShowNum8X16(0,4,w++,8);
		 volatile uint8_t *data = buffer1;  // 使用 volatile 以确保每次都读取最新值
			  int j=0;
       for (uint16_t i = 0; i < BUFFER_SIZE; i += 5)   // 这里根据你的数据帧长度修改步进
        {  int quality=data[i];
           int angle_new = (((uint16_t)(data[i + 2] << 7) + (data[i + 1] >> 1))) ;
           int distance_new = (((uint16_t)(data[i + 4] << 8) + (data[i + 3]))) ;
           Quality[j]=quality;
					angle_q6[j]=angle_new;
					distance_q2[j++]=distance_new;
					//angle_array[j++]=angle_new;   // 存储计算出的角度
            //distance_array[j++]=distance_new;
       }
				
			 send_packetPacNew(0x55,0xAA,HAL_GetTick(),(uint16_t)cmd_id,(uint8_t)status,(int)V_A,(int)(-V_B),(float)current_yaw)	;
		rplidar_data.data_count=520;
		HAL_UART_Transmit(&huart3, (uint8_t*)&rplidar_data.data_count, sizeof(rplidar_data.data_count), HAL_MAX_DELAY);
	/*	 for(int t=0;t<520;t++){
            HAL_UART_Transmit(&huart3, (uint8_t*)&Quality[t], sizeof(Quality[t]), HAL_MAX_DELAY);
             HAL_UART_Transmit(&huart3,(uint8_t*) &angle_q6[t], sizeof(angle_q6[t]), HAL_MAX_DELAY);
             HAL_UART_Transmit(&huart3,(uint8_t*) &distance_q2[t], sizeof(distance_q2[t]), HAL_MAX_DELAY);}*/
			 uint16_t offset = 0;
for (int t = 0; t < 520; t++) {
    tx_buffer[offset++] = Quality[t];
    tx_buffer[offset++] = (uint8_t)(angle_q6[t] & 0xFF);       // angle 低字节
    tx_buffer[offset++] = (uint8_t)((angle_q6[t] >> 8) & 0xFF); // angle 高字节
    tx_buffer[offset++] = (uint8_t)(distance_q2[t] & 0xFF);     // distance 低字节
    tx_buffer[offset++] = (uint8_t)((distance_q2[t] >> 8) & 0xFF); // distance 高字节
}

HAL_UART_Transmit_DMA(&huart3, tx_buffer, 2600);
			 
			 
				//	HAL_UART_Transmit(&huart3, &rplidar_data.send_data[j].angle_q6, sizeof(rplidar_data.send_data[j].angle_q6), HAL_MAX_DELAY);
       	 
			
				//int start=0;
			//	if(choice==0) {start=0;choice=1;}
		//		else {choice=0; start=180;}
			 
	
	
	
	}
    
		
	}
	
if(htim == (&htim6)){//10ms中断

  GetMotorPulse();
	MPU6050_Read_All(&hi2c1,&mpu6050);
	current_yaw=mpu6050.KalmanAngleZ;
	//OLED_Showdecimal8X16(0,2,current_yaw,3,3);
	//if(tip2==1&&V_A<goal){	status=2;PID1();		OLED_ShowNum8X16(0,2,status,8);	}
	//else if(tip2==1&&V_A>=goal){
//	yaw_angle=0;
	//stopMotors();
	//		tip2=0;
		//status=0;
	//	cmd_id=cmd2;
//	}
//PID1();
}

}

extern UART_HandleTypeDef huart3;

/**
 * @brief Redefines the write function.
 * @note printf() function is implemented by calling this function.
 * @param file: X - write descriptor.
 * @param ptr: Pointer to the data to be sent.
 * @param len: Length of the data to be sent.
 * @retval: Returns the length of data sent.
 */
int _write(int file, char *ptr, int len)
{
  
  HAL_StatusTypeDef status = HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  
  
  if (status == HAL_OK)
  {
    return len;
  }
  else 
  {
    
    return -1;
  }
}

void Safe_UART_Transmit(UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size, uint32_t Timeout)
{
    
    HAL_NVIC_DisableIRQ(USART3_IRQn);

    
    HAL_UART_Transmit(huart, pData, Size, Timeout);

  
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

void send_packetPac(
    float anglePac,
    float distancePac,
    float pitchPac,
    float rollPac,
    float yawPac,
    float speedLeftPac,
		float speedRightPac
) {
    uint8_t packetPac[31];
    uint8_t idxPac = 0;

    packetPac[idxPac++] = 0xA5;
 
	  memcpy(&packetPac[idxPac], &anglePac, 4); idxPac += 4;
    memcpy(&packetPac[idxPac], &distancePac, 4);  idxPac += 4;

    memcpy(&packetPac[idxPac], &pitchPac, 4); idxPac += 4;
    memcpy(&packetPac[idxPac], &rollPac, 4);  idxPac += 4;
    memcpy(&packetPac[idxPac], &yawPac, 4);   idxPac += 4;
    memcpy(&packetPac[idxPac], &speedLeftPac, 4); idxPac += 4;
	  memcpy(&packetPac[idxPac], &speedRightPac, 4); idxPac += 4;

    uint16_t sumPac = 0;
    for (int iPac = 1; iPac <= 28; ++iPac) {
        sumPac += packetPac[iPac];
    }
    packetPac[idxPac++] = (uint8_t)(sumPac & 0xFF);

    packetPac[idxPac++] = 0x5A;
    HAL_UART_Transmit(&huart3, packetPac, sizeof(packetPac), HAL_MAX_DELAY);
}

void send_packetPacNew(
		uint8_t Send_Begin_1,
    uint8_t Send_Begin_2,

    uint32_t time_us,
    uint16_t cmd_id,
    uint8_t status,

    int encoder_l,
    int encoder_r,
    float current_yaw
	//	RPLIDAR_Send_Data rplidar520
    
) {
     

    // ???????
    packet.Send_Begin_1 = Send_Begin_1;
    packet.Send_Begin_2 = Send_Begin_2;
    packet.time_us = time_us;
    packet.cmd_id = cmd_id;
    packet.status = status;
    packet.encoder_l = encoder_l;
    packet.encoder_r = encoder_r;
    packet.current_yaw = current_yaw;
  // packet.rplidar_data = rplidar520; // ?LIDAR?????????

    // ???????
   HAL_UART_Transmit(&huart3, (uint8_t *)&packet, sizeof(packet), HAL_MAX_DELAY);
}

RPLIDAR_Send_Data createRPLIDARData(
     uint8_t Quality[520],
    uint16_t angle_q6[520],
     uint16_t distance_q2[520]
) {
   // RPLIDAR_Send_Data rplidar_data; // 定义返回的结构体

    // 设置数据计数
    rplidar_data.data_count = 520;
    
    // 循环填充send_data数组
	//OLED_ShowNum8X16(0,2,w++,8);
    for (int i = 0; i < 520; i++) {
        rplidar_data.send_data[i].Quality = Quality[i];       // 填充质量
        rplidar_data.send_data[i].angle_q6 = angle_q6[i];    // 填充角度
        rplidar_data.send_data[i].distance_q2 = distance_q2[i];  // 填充距离
    }
		//OLED_ShowNum8X16(0,4,w++,8);
			OLED_ShowNum8X16(0,2,w++,8);
	   return rplidar_data; // 返回填充好的结构体
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
