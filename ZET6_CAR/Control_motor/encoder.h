#ifndef ENCODER_H_
#define ENCODER_H_

#include "tim.h"
#include "Motor.h"

void MotorEncoder_Init(void);
int Read_Encoder(TIM_HandleTypeDef *htim);
void speed_control(pid_t* pid,float get,float set);
void Clear_Encoder(void);

#endif
