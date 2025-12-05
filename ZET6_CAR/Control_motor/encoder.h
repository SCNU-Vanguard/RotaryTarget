#ifndef ENCODER_H_
#define ENCODER_H_

#include "tim.h"
#include "Motor.h"

#define PI 3.1415926

void MotorEncoder_Init(void);
int Read_Encoder(TIM_HandleTypeDef *htim);
void speed_control(pid_t* pid,float get,float set,int i);
void Clear_Encoder(int i);
void Set_Encoder(int i);
void Chassis_Solution(pid_t* pid1,pid_t* pid2,pid_t* pid3,pid_t* pid4,float Vx,float Vy,float angle);
float count_data(int data);	
	
#endif
