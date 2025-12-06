/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "NRF24L01.h"
#include "encoder.h"
#include "Motor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t RX_data[32]={0};
//uint8_t RX_data[8]={0};
//int not_exist_flag = 0;
int get_sta=1,i=0;
int turn=0,test=500;
float speed[4]={0},Vx,Vy;//set_speed[4]={-3.0,3.0,-3.0,3.0}
uint8_t tail[4]={0x00,0x00,0x80,0x7f};
float data[4];
float circle_0_1=836.0;//11*4*19
float circle_2_3=1560.0;//13*4*30
uint16_t get_data[4];
int start_error=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM5)
	{
		if(start_error==1)
		{
          get_data[0] = (RX_data[0]<<8)+RX_data[1];//
		  get_data[1] = (RX_data[2]<<8)+RX_data[3];//
		  get_data[2] = (RX_data[4]<<8)+RX_data[5];//
		  get_data[3] = (RX_data[6]<<8)+RX_data[7];//
		}
      Vx=count_data(get_data[2]);
		  Vy=count_data(get_data[3]);
  		Chassis_Solution(&pid_spd[0],&pid_spd[1],&pid_spd[2],&pid_spd[3],Vx,Vy,0);
		
			pid_spd[0].encoder.last_encoder=pid_spd[0].encoder.get_encoder;
			pid_spd[0].encoder.get_encoder=Read_Encoder(&htim1);
			pid_spd[0].encoder.delta_encoder=pid_spd[0].encoder.get_encoder-pid_spd[0].encoder.last_encoder;
		
			pid_spd[1].encoder.last_encoder=pid_spd[1].encoder.get_encoder;
			pid_spd[1].encoder.get_encoder=Read_Encoder(&htim2);
			pid_spd[1].encoder.delta_encoder=pid_spd[1].encoder.get_encoder-pid_spd[1].encoder.last_encoder;
		
  		pid_spd[2].encoder.last_encoder=pid_spd[2].encoder.get_encoder;
			pid_spd[2].encoder.get_encoder=Read_Encoder(&htim3);
			pid_spd[2].encoder.delta_encoder=pid_spd[2].encoder.get_encoder-pid_spd[2].encoder.last_encoder;
		
			pid_spd[3].encoder.last_encoder=pid_spd[3].encoder.get_encoder;
			pid_spd[3].encoder.get_encoder=Read_Encoder(&htim4);
			pid_spd[3].encoder.delta_encoder=pid_spd[3].encoder.get_encoder-pid_spd[3].encoder.last_encoder;

			for(i=0;i<4;i++)
		  {
				if((pid_spd[i].control_speed>0)&&(pid_spd[i].encoder.get_encoder>60000))
				{
					Clear_Encoder(i);
					pid_spd[i].encoder.last_encoder=pid_spd[i].encoder.get_encoder=0;
				}
				else if((pid_spd[i].control_speed<0)&&(pid_spd[i].encoder.get_encoder<5000))
				{
					Set_Encoder(i);
				  pid_spd[i].encoder.last_encoder=pid_spd[i].encoder.get_encoder=65535;
				}
		  }

			for(i=0;i<4;i++)
			{
//				   speed[i]=pid_spd[i].encoder.delta_encoder/(0.01*circle);
//   				 speed_control(&pid_spd[i],speed[i],pid_spd[i].control_speed,i);
				  if((i==0)||(i==1))
  				 speed[i]=pid_spd[i].encoder.delta_encoder/(0.01*circle_0_1);
					else if((i==2)||(i==3))
					 speed[i]=pid_spd[i].encoder.delta_encoder/(0.01*circle_2_3);
   				 speed_control(&pid_spd[i],speed[i],pid_spd[i].control_speed,i);
			}

//      data[0]=speed[0];
//      data[1]=speed[1];
//			data[0]=speed[2];
//      data[1]=speed[3];
//			HAL_UART_Transmit(&huart1,(uint8_t*)data,4*sizeof(float),100);
//		  HAL_UART_Transmit(&huart1,tail,4*sizeof(uint8_t),100);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
   pid_init();
	 MotorEncoder_Init();
	NRF24L01_RX_Mode();		
	Data_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		get_sta=NRF24L01_RxPacket(RX_data);
	    if(get_sta==0)
			start_error=1;
//		HAL_UART_Receive(&huart1,RX_data,8*sizeof(uint8_t),100);
//		SetPWM(5000);
//		i=Read_Encoder(&htim1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
