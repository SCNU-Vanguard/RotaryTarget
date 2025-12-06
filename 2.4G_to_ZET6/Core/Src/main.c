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
extern uint8_t Flag;
extern uint8_t message[128];
extern int16_t SpeedTarget;
uint16_t SpeedSet;
uint32_t count_2 = 0;
uint32_t count_1 = 0;
uint32_t PWM_duty[7] = {100, 100, 100, 50, 50, 50, 50};
uint16_t test = 1000;
uint8_t RX_data[32];
uint8_t RX_status = 1;
uint16_t get_data[4];
uint8_t check;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM3) 
	{
		if(SpeedTarget == 800)
		{
			(&htim4)->Instance->CCR4 = SpeedTarget;
		}
		else if(SpeedTarget != 0 && SpeedTarget != 800)
		{
			count_1++;
			count_2++;
			SpeedSet = SpeedTarget;
			if(count_1 == 20)
			{
				(&htim4)->Instance->CCR4 = 0;
			}
			else if(count_1 == 500)
			{
				(&htim4)->Instance->CCR4 = SpeedSet;
				count_1 = 0;
			}
			
			if(count_2 == 18000)
			{
				SpeedTarget = 0;
				(&htim4)->Instance->CCR4 = SpeedTarget;
				count_2 = 0;
			}
		}
		else if(SpeedTarget == 0)
		{
			count_2 = 0;
		}
	}
}

void data_get()
{
	get_data[0] = (RX_data[0]<<8)+RX_data[1];//左摇杆左右值
	get_data[1] = (RX_data[2]<<8)+RX_data[3];//左摇杆上下值
	get_data[2] = (RX_data[4]<<8)+RX_data[5];//右摇杆左右值
	get_data[3] = (RX_data[6]<<8)+RX_data[7];//右摇杆上下值
}

void Set_G80_Speed()
{
	if(get_data[0] < 100)
	{
		SpeedTarget = 800;
	}
	else if(get_data[0] > 4000 && get_data[1] < 100)
	{
		SpeedTarget = 0;
	}
	else if(get_data[0] > 4000 && get_data[1] > 4000)
	{
		SpeedTarget = 1200;
	}
	else if(get_data[0] > 4000)
	{
		SpeedTarget = get_data[1] / 3900.0 * 400 + 801;
	}
	
//	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, SpeedTarget);
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  
  //PWM
//  HAL_TIM_Base_Start(&htim4);
//  HAL_TIM_Base_MspInit(&htim4);
//  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  
//  HAL_TIM_Base_MspInit(&htim3);
//  HAL_TIM_Base_Start_IT(&htim3);
  
//  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, test);
  check = NRF24L01_Check();
  NRF24L01_RX_Mode();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  if (Flag)	  
//	  {
//		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, SpeedTarget);
//		Flag = 0x00;
//	  }
	  RX_status = NRF24L01_RxPacket(RX_data);
	  if(!RX_status)
	  {
		data_get();
//		Set_G80_Speed();
		HAL_UART_Transmit_IT(&huart1, RX_data, 8);
	  }
	  HAL_Delay(50);
	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
