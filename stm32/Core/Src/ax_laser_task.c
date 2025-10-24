// ax_laser_task.c
#include "ax_laser.h"
#include "ax_delay.h"
#include "usart.h"
#include <stdio.h>
#include "OLED.h"
extern LaserPointTypeDef ax_ls_point[250];
extern UART_HandleTypeDef huart4;

uint8_t uart4_rx_byte = 0;


void Laser_Task_Init(void)
{
	  HAL_UART_Receive_IT(&huart4, &uart4_rx_byte, 1);  
    AX_DELAY_Init();      
    AX_LASER_Init();       

    AX_Delayms(2000);      
    AX_LASER_Start();      
   // OLED_ShowString8X16(0,6,"ROBOT Init");
    
}


void Laser_Task_Loop(void)
{
    //for (int i = 0; i < 250; i++)
    //{
        //printf("Angle: %5d\tDistance: %5d mm\r\n", ax_ls_point[i].angle, ax_ls_point[i].distance);
    //}a
    //AX_Delayms(200); 
}
