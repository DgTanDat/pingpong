/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "usbd_cdc_if.h"
#include "stm32f429i_discovery_sdram.h"
#include "stm32f429i_discovery_gyroscope.h"
#include "stm32f429i_discovery_lcd.h"
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
/* Definitions for Task01 */
osThreadId_t Task01Handle;
const osThreadAttr_t Task01_attributes = {
  .name = "Task01",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task02 */
osThreadId_t Task02Handle;
const osThreadAttr_t Task02_attributes = {
  .name = "Task02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task03 */
osThreadId_t Task03Handle;
const osThreadAttr_t Task03_attributes = {
  .name = "Task03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* USER CODE BEGIN PV */
float dataRec[4];
uint8_t buffer1 [20];
uint8_t buffer2 [10];
int xPos, yPos, radius;
int hitFlag;
int state, realScore;
int upFlag;
int lastScore, upDown, leftRight;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartTask01(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void adjustBallDirection()
{
	if(upDown == -1) yPos++;
	if(upDown == 1) yPos--;
	if(leftRight == -1) xPos++;
	if(leftRight == 1) xPos--;
	if((xPos + radius)>240)
	{
		leftRight = !leftRight;
		xPos-=5;
	}
	if((xPos - radius)<0)
	{
		leftRight = !leftRight;
		xPos+=5;
	}
	if(((yPos + radius)>320))
	{
		upDown = !upDown;
		yPos-=5;
	}
	if((yPos - radius)<0)
	{
		upDown = !upDown;
		yPos+=5;
	}
};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	xPos = 120;
	yPos = 160;
	radius = 50;
	hitFlag = 0;
	realScore = 0;
	state = 0;
	upFlag = 0;
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
  /* USER CODE BEGIN 2 */
  BSP_SDRAM_Init();
  BSP_GYRO_Init();
  BSP_LCD_Init();
  //set the layer buffer address into SDRAM
  BSP_LCD_LayerDefaultInit(1, SDRAM_DEVICE_ADDR);
  BSP_LCD_SelectLayer(1);
  BSP_LCD_DisplayOn();
  BSP_LCD_Clear(LCD_COLOR_BLUE);
  BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (90, sizeof(float), &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task01 */
  Task01Handle = osThreadNew(StartTask01, NULL, &Task01_attributes);

  /* creation of Task02 */
  Task02Handle = osThreadNew(StartTask02, NULL, &Task02_attributes);

  /* creation of Task03 */
  Task03Handle = osThreadNew(StartTask03, NULL, &Task03_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask01 */
/**
  * @brief  Function implementing the Task01 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask01 */
void StartTask01(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  if(state == 0)
	  {
		  BSP_GYRO_GetXYZ(dataRec);
		  for(int i=0; i<3; i++)
		  {
	  	  	  dataRec[i] = dataRec[i]*0.001;
	  	  }
		  	  //
		  if(abs(dataRec[0]) > 20)
		  {
			  hitFlag = 1;
		  	  dataRec[3] = abs(dataRec[0]*radius);
		  	  for(int i=0; i<4; i++)
		  	  {
		  	   	osMessageQueuePut(myQueue01Handle, &dataRec[i], 0, 0);
		  	  }
		  }
	  	  else
	  	  {
			  hitFlag = 0;
	  	  }
		  sprintf(buffer1, "x:%0.1f;y:%0.1f;z:%0.1f\n", dataRec[0], dataRec[1], dataRec[2]);
		  CDC_Transmit_HS(buffer1, sizeof(buffer1));

	  	  osDelay(250);
	  }

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Task02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  BSP_LCD_FillCircle(xPos, yPos, radius);
  float getBuf[4];
  float temp;

  for(;;)
  {
	lastScore = realScore;
    if(state == 0 && hitFlag == 1)
    {
    	for(int i=0; i<4; i++)
    	{
    		osMessageQueueGet(myQueue01Handle, &getBuf[i], 0, 0);
    	}
   		if(getBuf[3] > 500)
   		{
   			temp = 500;
   		}
    	else
   		{
   			temp = getBuf[3];
   		}
   		if(getBuf[0] < -1) upDown = -1;
   		else if(getBuf[0] > 1) upDown = 1;
   		else upDown = 0;
   		if(getBuf[1] < -1) leftRight = -1;
   		else if(getBuf[1] > 1) leftRight = 1;
   		else leftRight = 0;
   		for (int i = 0; i <= temp; i = i + 10)
   		{
//    		sprintf(buffer2, "\n%d", i);
//    		CDC_Transmit_HS(buffer2, sizeof(buffer2));

    		BSP_LCD_Clear(LCD_COLOR_BLUE);
//   			if(upDown == -1) yPos++;
//   			if(upDown == 1) yPos--;
//   			if(leftRight == -1) xPos++;
//   			if(leftRight == 1) xPos--;
//   			if((xPos + radius)>240)
//   			{
//   				leftRight = !leftRight;
//   				xPos-=5;
//   			}
//   			if((xPos - radius)<0)
//   			{
//   				leftRight = !leftRight;
//   				xPos+=5;
//   			}
//   			if(((yPos + radius)>320))
//   			{
//   				upDown = !upDown;
//   				yPos-=5;
//   			}
//   			if((yPos - radius)<0)
//   			{
//   				upDown = !upDown;
//   				yPos+=5;
//   			}
    		adjustBallDirection();
   			BSP_LCD_FillCircle(xPos, yPos, radius);
   			radius = radius + 5;
   			if (radius > 90)
   			{
   				break;
    		}
 			osDelay(200);
    	}
    	for (int i = temp; i >= 0; i = i - 10)
    	{
// 			sprintf(buffer2, "\n%d", i);
//   		CDC_Transmit_HS(buffer2, sizeof(buffer2));
    		BSP_LCD_Clear(LCD_COLOR_BLUE);
//    		if(upDown == -1) yPos++;
//    		if(upDown == 1) yPos--;
//   			if(leftRight == -1) xPos++;
//   			if(leftRight == 1) xPos--;
//    		if((xPos + radius)>240)
//    		{
//    			leftRight = !leftRight;
//    			xPos-=5;
//    		}
//    		if((xPos - radius)<0)
//    		{
//    			leftRight = !leftRight;
//    			xPos+=5;
//    		}
//    		if((yPos + radius)>320)
//    		{
//    			upDown = !upDown;
//    			yPos-=5;
//    		}
//    		if((yPos - radius)<0)
//    		{
//    			upDown = !upDown;
//    			yPos+=5;
//       		}
    		adjustBallDirection();
 			BSP_LCD_FillCircle(xPos, yPos, radius);
 			radius = radius - 5;
 			if (radius >= 15 && radius <= 40)
 			{
 				if (hitFlag == 1)
 				{
 					realScore = realScore + 1;
 					break;
    			}
   			}
 			else if (radius < 15)
   			{
   				break;
   			}
   			osDelay(200);
    	}
    	if (realScore == lastScore)
   		{
   			state = 1;
   		}
    }
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the Task03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
	 if(state == 0)
     {
		 sprintf(buffer2, "Radius:  %d", radius);
		 BSP_LCD_DisplayStringAtLine(0, buffer2);
		 sprintf(buffer2, "Score:  %d", realScore);
		 BSP_LCD_DisplayStringAtLine(1, buffer2);
     }
	 else
	 {
	    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	    BSP_LCD_Clear(LCD_COLOR_RED);
	    BSP_LCD_SetBackColor(LCD_COLOR_RED);
	    BSP_LCD_DisplayStringAtLine(1, "Game Over!");
	    BSP_LCD_DisplayStringAtLine(2, "Press Restart");
	    BSP_LCD_DisplayStringAtLine(3, "For New Game");
	    osDelay(1000);
	 }
  }
  /* USER CODE END StartTask03 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
