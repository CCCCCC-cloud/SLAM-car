#ifndef INC_CAR_CONTROL_H_
#define INC_CAR_CONTROL_H_


// 1. Include necessary header files
// The function parameters are of type HAL and uint16_t, which are usually included in main.h
#include "main.h"

// 2. Define macros (if applicable, give other files usage)
// In this example, speed-related definitions are in a separate module, so we won't place them here

// 3. Public function prototypes
// These are the functions required by main.c, so they must be declared here
void goForward(uint16_t speed);
void goBackward(uint16_t speed);
void stopMotors(void);
void turnLeft(uint16_t speed);
void turnRight(uint16_t speed);
void PID_TurnToLeft(float angle);
void PID_TurnToRight(void);

void PID1(void);
void Speed_PID_Init(void);
void Speed_PID_Update(void);
void GoForward_PID(float target_rps);
void Stop_PID(void);
void motorA_goForward(uint16_t speed);
void motorA_goBackward(uint16_t speed);
void motorA_stop(void);
void motorB_goForward(uint16_t speed);
void motorB_goBackward(uint16_t speed);
void motorB_stop(void);
#endif /* INC_CAR_CONTROL_H_ */
