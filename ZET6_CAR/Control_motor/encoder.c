#include "encoder.h"
#include "tim.h"
#include "Motor.h"
#include "math.h"

extern uint16_t get_data[4];

void MotorEncoder_Init(void)
{
    HAL_TIM_Base_Init(&htim1);
    // 清除TIM的更新标志位
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    // 更新中断使能
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
    // 初始化清空编码器数值
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    // 使能定时器1
    HAL_TIM_Base_Start_IT(&htim1);
	
	  HAL_TIM_Base_Init(&htim2);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_Base_Start_IT(&htim2);
	
	  HAL_TIM_Base_Init(&htim3);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_TIM_Base_Start_IT(&htim3);
		
		HAL_TIM_Base_Init(&htim4);
    __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    HAL_TIM_Base_Start_IT(&htim4);
		
		HAL_TIM_Base_Init(&htim5);
		HAL_TIM_Base_Start_IT(&htim5);
		
		HAL_TIM_Base_Init(&htim8);
		HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
}

int Read_Encoder(TIM_HandleTypeDef *htim)
{
    int Encoder_TIM = __HAL_TIM_GET_COUNTER(htim);
    return Encoder_TIM;
}

void speed_control(pid_t* pid,float get,float set,int i)
{
	pid_calc(pid,get,set);
	SetPWM(pid->pos_out,i);
}

void Clear_Encoder(int i)
{
	switch(i)
	{
		case 0:
	__HAL_TIM_SET_COUNTER(&htim1,0);
		break;
		case 1:
	__HAL_TIM_SET_COUNTER(&htim2,0);
		break;
		case 2:
	__HAL_TIM_SET_COUNTER(&htim3,0);
		break;
		case 3:
	__HAL_TIM_SET_COUNTER(&htim4,0);
	}
}

void Set_Encoder(int i)
{
		switch(i)
	{
		case 0:
	__HAL_TIM_SET_COUNTER(&htim1,65535);
		break;
		case 1:
	__HAL_TIM_SET_COUNTER(&htim2,65535);
		break;
		case 2:
	__HAL_TIM_SET_COUNTER(&htim3,65535);
		break;
		case 3:
	__HAL_TIM_SET_COUNTER(&htim4,65535);
	}
}

void Chassis_Solution(pid_t* pid1,pid_t* pid2,pid_t* pid3,pid_t* pid4,float Vx,float Vy,float angle)
{
	//float R=; //
	float x=PI/4;
	float k=0.2;
	pid1->control_speed=(-1)*Vx*cosf(x)+(-1)*Vy*cosf(x)+angle*k;
	pid2->control_speed=(-1)*Vx*cosf(x)+Vy*cosf(x)+angle*k;
	pid3->control_speed=     Vx*cosf(x)+(-1)*Vy*cosf(x)+angle*k;
	pid4->control_speed=     Vx*cosf(x)+     Vy*cosf(x)+angle*k;
	if(pid1->control_speed>2.6)
		pid1->control_speed=2.6;
	if(pid1->control_speed<-2.6)
		pid1->control_speed=-2.6;
	
	if(pid2->control_speed>2.6)
		pid2->control_speed=2.6;
	if(pid2->control_speed<-2.6)
		pid2->control_speed=-2.6;
	
	if(pid3->control_speed>2.6)
		pid3->control_speed=2.6;
	if(pid3->control_speed<-2.6)
		pid3->control_speed=-2.6;
	
	if(pid4->control_speed>2.6)
		pid4->control_speed=2.6;
	if(pid4->control_speed<-2.6)
		pid4->control_speed=-2.6;
}

float count_speed=2000;
float count_data(int data)
{
	if((data>1900)&&(data<2200))
		count_speed=0;
	else if(data>2400)
	{
		if(data>4000)
			count_speed=4;
		else
		  count_speed=(data-2400)/400;
	}
	else if(data<1600)
	{
		if(data<100)
			count_speed=-4;
		else
			count_speed=(data-1600)/400;
	}
  return count_speed*0.75;
}
