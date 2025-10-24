#include "ax_laser.h"
#include "ax_delay.h"
#include "usart.h"  
#include "string.h"
#include <stdio.h>
#include "OLED.h"
#include "speed_solver.h"
#include "main.h"
#include "mpu6050.h"

extern UART_HandleTypeDef huart1;  
extern UART_HandleTypeDef huart4;
extern float roll,yaw,pitch;
static uint8_t uart4_rx_con=0;       
static uint8_t uart4_rx_chksum;      
static uint8_t uart4_rx_buf[100];     
static uint8_t uart4_tx_buf[10];     
float angle_new,distance_new;
//MPU6050_t mpu6050;
 LaserPointTypeDef ax_ls_point[250];


static void LS_DataHandle(void);

//extern void send_packetPac(float anglePac,float distancePac,float pitchPac,float rollPac,float yawPac,float speedLeftPac,float speedRightPac);


void AX_LASER_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_UART4_CLK_ENABLE();

    // TX PC10
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // RX PC11
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		/*
    // UART4 
    huart4.Instance = UART4;
    huart4.Init.BaudRate = 460800;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;

    HAL_UART_Init(&huart4);

    HAL_NVIC_SetPriority(UART4_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
		*/
}


void AX_LASER_RxCpltHandler(uint8_t rx_byte)
{     if( uart4_rx_con == 7){

uint8_t  value = rx_byte; //my date

uint8_t  bit7 = (value >> 7) & 0x01;  // accqure the first bite
uint8_t  bit6 = (value >> 6) & 0x01;  // the second bit

if ((bit7 ^ bit6) == 1) {
    
           uart4_rx_buf[uart4_rx_con++] = rx_byte;
} else {
    uart4_rx_con = 7 ; //  discard the data
}

}
else if( uart4_rx_con == 8){
    uint8_t  bit7 = (rx_byte >> 7) & 0x01;
	if(bit7==1) {
    
           uart4_rx_buf[uart4_rx_con++] = rx_byte;
} else {
    uart4_rx_con = 7 ; // dsicard the data
}


}
 else   if (uart4_rx_con < LS_F_LEN-1) { 
           
           uart4_rx_buf[uart4_rx_con++] = rx_byte;
	//	 OLED_ShowNum8X16(0,6,uart4_rx_buf[uart4_rx_con-1],6);
        
    } else {
			uart4_rx_buf[uart4_rx_con++] = rx_byte;
    //    OLED_ShowNum8X16(0,6,uart4_rx_buf[uart4_rx_con-1],6);
            LS_DataHandle();
            uart4_rx_con = 7;
        }
    }

static void LS_DataHandle(void)
{
   

   // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);  
    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);  
     angle_new = (((uint16_t)(uart4_rx_buf[9]  << 7) + (uart4_rx_buf[8]>>1))) / 64.0 ;
	   distance_new=(((uint16_t)(uart4_rx_buf[11]  << 8) + (uart4_rx_buf[10]))) / 4.0;
	   float left_speed = Get_Left_Speed_RPS();
		float right_speed = Get_Right_Speed_RPS();
	if(angle_new>360)angle_new=angle_new-360;
    //	 OLED_ShowNum8X16(0,6,angle_new,6);
  // MPU6050_Read_All(&hi2c1,&mpu6050);
	//	send_packetPac(angle_new,distance_new,mpu6050.KalmanAngleX,mpu6050.KalmanAngleY,mpu6050.KalmanAngleZ,left_speed,right_speed);
		
      // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   
     // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); 


       // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);  
       // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

 void AX_LASER_Start(void)
{
    uint8_t cmd[] = {0xA5, 0x20};
    HAL_UART_Transmit(&huart4, cmd, sizeof(cmd), HAL_MAX_DELAY);
}


void AX_LASER_Stop(void)
{
    uint8_t i;

    uart4_tx_buf[0] = 0xA5;       
    uart4_tx_buf[1] = 0x25;       
    uart4_tx_buf[2] = 0xA5 + 0x25;  

    for (i = 0; i < 3; i++) {
        HAL_UART_Transmit(&huart4, &uart4_tx_buf[i], 1, HAL_MAX_DELAY);
    }
}
