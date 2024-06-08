/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"

#include <stdlib.h>
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COLOR_GREEN 2
#define COLOR_YELLOW 1
#define COLOR_RED 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint8_t isStrated = 0;
uint8_t trainDistance = 255;
uint32_t trainDistanceRaw = 2550000 << 8;
uint8_t lightStatus = 0;
uint8_t lightStatusNew = 1;
uint8_t timeLast = 0;
uint8_t lightFlashStatus = 0;

// Light channels in 4 combination to show
uint8_t lightPerset[4][10] = {
    {COLOR_GREEN, COLOR_RED, COLOR_GREEN, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_GREEN, COLOR_GREEN, COLOR_RED, COLOR_RED},
    {COLOR_GREEN, COLOR_GREEN, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_GREEN, COLOR_GREEN, COLOR_RED, COLOR_GREEN, COLOR_RED},
    {COLOR_RED, COLOR_GREEN, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_GREEN, COLOR_GREEN, COLOR_RED, COLOR_GREEN, COLOR_GREEN},
    {COLOR_RED, COLOR_RED, COLOR_RED, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_RED, COLOR_GREEN, COLOR_GREEN, COLOR_RED}};

uint8_t lightTimePerset[4] = {10, 10, 10, 10};

uint8_t lightScreenCoord[4][2] = {
    {3, 56},
    {3, 51},
    {123, 37},
    {56, 3}};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void OLED_ShowLightStatus(uint8_t lightIndex, uint8_t lightStatus)
{
  if (lightStatus == COLOR_GREEN || lightStatus == COLOR_YELLOW)
  {
    lightStatus = 1;
  }
  else
  {
    lightStatus = 0;
  }
  switch (lightIndex)
  {
  case 0:
    OLED_DrawPoint(lightScreenCoord[lightIndex][0], lightScreenCoord[lightIndex][1], lightStatus);
    OLED_DrawPoint(lightScreenCoord[lightIndex][0] + 1, lightScreenCoord[lightIndex][1], lightStatus);
    OLED_DrawPoint(lightScreenCoord[lightIndex][0], lightScreenCoord[lightIndex][1] + 1, lightStatus);
    OLED_DrawPoint(lightScreenCoord[lightIndex][0] + 1, lightScreenCoord[lightIndex][1] + 1, lightStatus);
    OLED_DrawLine(7, 56, 120, 56, lightStatus);
    break;
  case 1:
    OLED_DrawPoint(lightScreenCoord[lightIndex][0], lightScreenCoord[lightIndex][1], lightStatus);
    OLED_DrawPoint(lightScreenCoord[lightIndex][0] + 1, lightScreenCoord[lightIndex][1], lightStatus);
    OLED_DrawPoint(lightScreenCoord[lightIndex][0], lightScreenCoord[lightIndex][1] + 1, lightStatus);
    OLED_DrawPoint(lightScreenCoord[lightIndex][0] + 1, lightScreenCoord[lightIndex][1] + 1, lightStatus);
    OLED_DrawLine(7, 51, 70, 51, lightStatus);
    OLED_DrawLine(70, 7, 70, 51, lightStatus);
    break;
  case 2:
    OLED_DrawPoint(lightScreenCoord[lightIndex][0], lightScreenCoord[lightIndex][1], lightStatus);
    OLED_DrawPoint(lightScreenCoord[lightIndex][0] + 1, lightScreenCoord[lightIndex][1], lightStatus);
    OLED_DrawPoint(lightScreenCoord[lightIndex][0], lightScreenCoord[lightIndex][1] + 1, lightStatus);
    OLED_DrawPoint(lightScreenCoord[lightIndex][0] + 1, lightScreenCoord[lightIndex][1] + 1, lightStatus);
    OLED_DrawLine(7, 37, 120, 37, lightStatus);
    break;
  case 3:
    OLED_DrawPoint(lightScreenCoord[lightIndex][0], lightScreenCoord[lightIndex][1], lightStatus);
    OLED_DrawPoint(lightScreenCoord[lightIndex][0] + 1, lightScreenCoord[lightIndex][1], lightStatus);
    OLED_DrawPoint(lightScreenCoord[lightIndex][0], lightScreenCoord[lightIndex][1] + 1, lightStatus);
    OLED_DrawPoint(lightScreenCoord[lightIndex][0] + 1, lightScreenCoord[lightIndex][1] + 1, lightStatus);
    OLED_DrawLine(57, 7, 57, 51, lightStatus);
    OLED_DrawLine(57, 51, 120, 51, lightStatus);
    break;

  case 4:
    OLED_DrawLine(53, 41, 53, 58, lightStatus);
    break;
  case 5:
    OLED_DrawLine(53, 36, 53, 39, lightStatus);
    break;
  case 6:
    OLED_DrawLine(55, 34, 58, 34, lightStatus);
    break;
  case 7:
    OLED_DrawLine(60, 34, 72, 34, lightStatus);
    break;
  case 8:
    OLED_DrawLine(74, 36, 74, 39, lightStatus);
    break;
  case 9:
    OLED_DrawLine(74, 41, 74, 58, lightStatus);
    break;

  default:
    break;
  }
}
void OLED_ShowTrainStatus(uint8_t isTrain)
{
  if (isTrain)
  {
    OLED_DrawLine(63, 2, 63, 46, 1);
    OLED_DrawLine(64, 2, 64, 46, 1);
    OLED_DrawLine(2, 44, 64, 44, 1);
    OLED_DrawLine(2, 45, 64, 45, 1);
  }
  else
  {
    OLED_DrawLine(63, 2, 63, 46, 0);
    OLED_DrawLine(64, 2, 64, 46, 0);
    OLED_DrawLine(2, 44, 64, 44, 0);
    OLED_DrawLine(2, 45, 64, 45, 0);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  // 2Hz
  if (htim == &htim2)
  {
    if (timeLast == 0)
    {
      if ((!((lightStatus == 1 || lightStatus == 2) && trainDistance < 20)) || lightStatusNew == 3)
      {
        OLED_ShowTrainStatus(0);
      }
      for (uint8_t i = 0; i < 10; i++)
      {
        if (lightPerset[lightStatus][i] > lightPerset[lightStatusNew][i])
        {
          OLED_ShowLightStatus(i, lightPerset[lightStatusNew][i]);
        }
      }
      for (uint8_t i = 0; i < 10; i++)
      {
        if (lightPerset[lightStatus][i] < lightPerset[lightStatusNew][i])
        {
          OLED_ShowLightStatus(i, lightPerset[lightStatusNew][i]);
        }
      }

      // Set new status
      lightStatus = lightStatusNew;
      lightStatusNew = (lightStatusNew + 1) % 4;
      timeLast = lightTimePerset[lightStatus];
    }
    else if (timeLast < 3)
    {
      for (uint8_t i = 0; i < 10; i++)
      {
        if (lightPerset[lightStatus][i] > lightPerset[lightStatusNew][i])
        {
          OLED_ShowLightStatus(i, (lightPerset[lightStatus][i] && lightFlashStatus));
        }
      }
      lightFlashStatus = (lightFlashStatus + 1) % 2;
    }
    if ((lightStatus == 1 || lightStatus == 2) && trainDistance < 20)
    {
      OLED_ShowTrainStatus(1);
    }
    else if (lightStatus == 1 || lightStatus == 2)
    {
      OLED_ShowTrainStatus(0);
    }
    OLED_ShowNum(0, 0, trainDistance, 3, 8, 1);
    OLED_Refresh();

    return;
  }
  // 1Hz
  else if (htim == &htim5)
  {
    // Decrease time
    if (timeLast)
    {
      timeLast--;
      // Train arrived
      if (timeLast > 3 && trainDistance < 20 && (lightStatus == 0 || lightStatus == 3))
      {
        timeLast = 3;
        lightStatusNew = 1;
      }
      else if (timeLast < 6 && trainDistance < 20 && lightStatus == 2)
      {
        timeLast = 6;
      }
    }

    // SR04 get distance
    uint8_t data = 0xA0;
    HAL_UART_Transmit(&huart1, &data, 1, 50);
    uint8_t distanceBytes[3] = {0};
    HAL_UART_Receive(&huart1, distanceBytes, 3, 250);
    trainDistanceRaw = ((uint32_t)distanceBytes[0] << 16) | ((uint32_t)distanceBytes[1] << 8) | distanceBytes[2];
    trainDistance = (trainDistanceRaw / 10000) < 100 ? trainDistanceRaw / 10000 : 100;

    return;
  }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  // Initial screen
  OLED_Init();
  OLED_DisPlay_On();
  OLED_Clear();
  OLED_ShowString(0, 0, (unsigned char *)"Traffic Light", 16, 1);
  OLED_ShowString(0, 18, (unsigned char *)"at The West Gate", 16, 1);
  OLED_ShowString(0, 36, (unsigned char *)"of UESTC", 16, 1);
  OLED_ShowString(0, 54, (unsigned char *)"Made By: ZhongWwwHhh", 8, 1);
  OLED_Refresh();
  HAL_Delay(1500);

  // Basic content
  OLED_Clear();
  OLED_DrawLine(2, 33, 53, 33, 1);
  OLED_DrawLine(2, 34, 53, 34, 1);
  OLED_DrawLine(52, 2, 52, 34, 1);
  OLED_DrawLine(53, 2, 53, 35, 1);
  OLED_DrawLine(74, 2, 74, 34, 1);
  OLED_DrawLine(75, 2, 75, 34, 1);
  OLED_DrawLine(74, 33, 125, 33, 1);
  OLED_DrawLine(74, 34, 125, 34, 1);
  OLED_DrawLine(60, 2, 60, 32, 1);
  OLED_DrawLine(61, 2, 61, 32, 1);
  OLED_DrawLine(66, 2, 66, 32, 1);
  OLED_DrawLine(67, 2, 67, 32, 1);
  OLED_DrawLine(2, 41, 51, 41, 1);
  OLED_DrawLine(2, 42, 51, 42, 1);
  OLED_DrawLine(2, 47, 51, 47, 1);
  OLED_DrawLine(2, 48, 51, 48, 1);
  for (uint8_t i = 41; i <= 48; i++)
  {
    OLED_DrawLine(76, i, 125, i, 1);
  }
  OLED_DrawLine(2, 60, 125, 60, 1);
  OLED_DrawLine(2, 61, 125, 61, 1);
  for (uint8_t i = 0; i < 4; i++)
  {
    OLED_DrawLine(lightScreenCoord[i][0] - 1, lightScreenCoord[i][1] - 1, lightScreenCoord[i][0] + 2, lightScreenCoord[i][1] - 1, 1);
    OLED_DrawLine(lightScreenCoord[i][0] - 1, lightScreenCoord[i][1] + 2, lightScreenCoord[i][0] + 2, lightScreenCoord[i][1] + 2, 1);
    OLED_DrawLine(lightScreenCoord[i][0] - 1, lightScreenCoord[i][1] - 1, lightScreenCoord[i][0] - 1, lightScreenCoord[i][1] + 2, 1);
    OLED_DrawLine(lightScreenCoord[i][0] + 2, lightScreenCoord[i][1] - 1, lightScreenCoord[i][0] + 2, lightScreenCoord[i][1] + 3, 1);
  }

  OLED_Refresh();

  // Mark to started
  isStrated = 1;

  // Start timer
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim5);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Useless
    HAL_Delay(1000);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 40000000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 80000000 - 1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, COM_OUT_CLK_Pin | COM_OUT_DATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI_SCK_Pin | SPI_SDA_Pin | SPI_Screen_RES_Pin | SPI_Screen_DC_Pin | SPI_Screen_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : COM_OUT_CLK_Pin COM_OUT_DATA_Pin */
  GPIO_InitStruct.Pin = COM_OUT_CLK_Pin | COM_OUT_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : COM_IN_CLK_Pin */
  GPIO_InitStruct.Pin = COM_IN_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(COM_IN_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : COM_IN_DATA_Pin */
  GPIO_InitStruct.Pin = COM_IN_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(COM_IN_DATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_SCK_Pin SPI_SDA_Pin SPI_Screen_RES_Pin SPI_Screen_DC_Pin
                           SPI_Screen_CS_Pin */
  GPIO_InitStruct.Pin = SPI_SCK_Pin | SPI_SDA_Pin | SPI_Screen_RES_Pin | SPI_Screen_DC_Pin | SPI_Screen_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
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
