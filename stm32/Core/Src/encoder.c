#include "encoder.h" 


extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim8;

/**
 * @brief Initializes two encoder timers (TIM1 and TIM2)
 * @param None
 * @retval None
 */
void Encoder_Init(void)
{
    
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim8, 0);
}

/**
 * @brief Reads and clears the count value of the left encoder
 * This function retrieves the count value of the encoder after it has been used, 
 * and resets the count value of the corresponding timer to zero.
 * The returned value is the difference in the count value between the last invocation 
 * and the current invocation.
 * @param None
 * @retval int16_t The count increment (positive for forward, negative for reverse)
 */
int16_t Encoder_Read_Left(void)
{
    int16_t count;
  
    count = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
  
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    return count;
}

/**
 * @brief Reads and clears the count value of the right encoder
 * @note The returned value is the count increment.
 * @param None
 * @retval int16_t The count increment
 */
int16_t Encoder_Read_Right(void)
{
    int16_t count;
    count = (int16_t)__HAL_TIM_GET_COUNTER(&htim8);
    __HAL_TIM_SET_COUNTER(&htim8, 0);
    return count;
}

