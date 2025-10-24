#ifndef INC_COMMUNICATION_H_
#define INC_COMMUNICATION_H_

#include "main.h"

// --- Public Global Variables Declaration ---
/** 
 * Use the extern keyword to declare these variables. These variables 
 * are defined elsewhere (in communication.c), please feel free to use.
 */
 
 typedef __packed struct {
    uint16_t data_len;
    uint16_t cmd_id;
    float    turn_rad;
    float    distance;
} CommandPacket_t;
 
extern volatile CommandPacket_t g_latest_command;
extern volatile uint8_t g_new_command_flag;

extern volatile int16_t g_speed_pulse_left;
extern volatile int16_t g_speed_pulse_right;

// --- Public Function Prototypes ---
/**
 * @brief Initializes the communication module and starts the reception 
 * of data in the interface.
 */
void Communication_Init(void);


#endif /* INC_COMMUNICATION_H_ */
