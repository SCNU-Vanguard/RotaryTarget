#include "encoder.h"
#include "tim.h"
#include "Motor.h"

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

void speed_control(pid_t* pid,float get,float set)
{
	pid_calc(pid,get,set);
	SetPWM(pid->pos_out);
}

void Clear_Encoder(void)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);
	__HAL_TIM_SET_COUNTER(&htim2,0);
	__HAL_TIM_SET_COUNTER(&htim3,0);
	__HAL_TIM_SET_COUNTER(&htim4,0);
}
