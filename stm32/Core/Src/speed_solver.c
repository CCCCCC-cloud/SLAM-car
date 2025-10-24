#include "speed_solver.h"
#include "encoder.h" // Required to use Encoder_Read_...() functions

// --- Motor parameters definition ---
// Your motor specifications are as follows:
#define ENCODER_PPR                 13.0f     // Encoder Pulses per Revolution
#define REDUCTION_RATIO             30.0f    // Reduction Ratio
#define ENCODER_MULTIPLIER          4.0f     // STM32 Encoder mode frequency (AB phase and the two channels)
// --- Timer parameters definition ---
// Timer interruption period, 10ms = 0.01s
#define TIMER_INTERRUPT_PERIOD_S    0.01f   


#define TOTAL_PULSES_PER_WHEEL_REV  (ENCODER_PPR * REDUCTION_RATIO * ENCODER_MULTIPLIER)

// --- Filter parameters definition ---
// Filter coefficient, can be (0.1 ~ 0.4)
#define FILTER_ALPHA                0.2f 


static volatile float raw_left_speed = 0.0f;
static volatile float raw_right_speed = 0.0f;


static volatile float s_filtered_left_speed_rps = 0.0f;
static volatile float s_filtered_right_speed_rps = 0.0f;

extern TIM_HandleTypeDef htim7;

/**
 * @brief Initializes the speed calculation module
 */
void Speed_Solver_Init(void)
{
    
    s_filtered_left_speed_rps = 0.0f;
    s_filtered_right_speed_rps = 0.0f;
}

/**
 * @brief pdates the speed (should be called periodically, e.g., every 10ms)
 */
void Speed_Solver_Update(void)
{

	// define a static variable
    static uint32_t last_timestamp_us = 0;

    //  accure a prise time priod
    uint32_t current_timestamp_us = __HAL_TIM_GET_COUNTER(&htim7);
	
	  //  calucate the last distance
	  uint32_t elapsed_ticks = current_timestamp_us - last_timestamp_us;
	
	  //  update the time
	  last_timestamp_us = current_timestamp_us;

	  //  trsanlate to ms
    float real_delta_t = (float)elapsed_ticks / 1000000.0f;

    if (real_delta_t <= 0) {
        return;
    }
		
    // 1. Read the delta pulses from the encoders
    int16_t left_delta_pulses = Encoder_Read_Left();
    int16_t right_delta_pulses = Encoder_Read_Right();

    // 2. Calculate the left wheel speed (unit: revolutions per minute (RPM))
    // Formula: (delta pulses in 10ms / total pulses per wheel revolution) / time (s)
    raw_left_speed = (float)left_delta_pulses / (TOTAL_PULSES_PER_WHEEL_REV * real_delta_t);//transalte TIMER_INTERRUPT_PERIOD_S to real_delta_t

    // 3. Calculate the right wheel speed (unit: revolutions per minute (RPM))
    // Formula: (delta pulses in 10ms / total pulses per wheel revolution) / time (s
    raw_right_speed = (float)right_delta_pulses / (TOTAL_PULSES_PER_WHEEL_REV * real_delta_t);
	
	  // 3. use the first order filter
    // formula: new_filtered = alpha * new_raw + (1-alpha) * old_filtered
    s_filtered_left_speed_rps = FILTER_ALPHA * raw_left_speed + (1.0f - FILTER_ALPHA) * s_filtered_left_speed_rps;
    s_filtered_right_speed_rps = FILTER_ALPHA * raw_right_speed + (1.0f - FILTER_ALPHA) * s_filtered_right_speed_rps;
		
}

/**
 * @brief Get the current speed of the left wheel
 * @return float Left wheel speed, unit: revolutions per second (RPS)
 */
float Get_Left_Speed_RPS(void)
{
    return s_filtered_left_speed_rps;
}

/**
 * @brief Get the current speed of the right wheel
 * @return float Right wheel speed, unit: revolutions per second (RPS)
 */
float Get_Right_Speed_RPS(void)
{
    return s_filtered_right_speed_rps;
}