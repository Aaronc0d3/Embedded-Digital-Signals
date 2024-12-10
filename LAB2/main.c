/* USER CODE BEGIN Header */
/**

*****************************************************************************
*
 * @file : main.c
 * @brief : Main program body

*****************************************************************************
*
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE
file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *

*****************************************************************************
*
 */
/* USER CODE END Header */
/* Includes -----------------------------------------------------------------
-*/
#include "main.h"
#include "arm_math.h"
#define N 20
int n;

int delta[N], shifted_delta[N], rect[N];
int step[N], shifted_step[N];
float expon[N], shifted_expon[N];
float sinus[N], shifted_sinus[N];
float sinus2[N], composite[N];
float a = 0.8, w0 = PI / 4, w1 = PI / 8;


/* Private includes ---------------------------------------------------------
-*
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
/* Private typedef ----------------------------------------------------------
-*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* Private define -----------------------------------------------------------
-*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */
/* Private macro ------------------------------------------------------------
-*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables --------------------------------------------------------
-*/
UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */
/* USER CODE END PV */
/* Private function prototypes ----------------------------------------------
-*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* Private user code --------------------------------------------------------
-*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
/**
 * @brief The application entry point.
 * @retval int
 */
int main(void)
{
 /* USER CODE BEGIN 1 */
 /* USER CODE END 1 */
 /* MCU Configuration-------------------------------------------------------
-*/
 /* Reset of all peripherals, Initializes the Flash interface and the
Systick. */
 HAL_Init();
 /* USER CODE BEGIN Init */
 /* USER CODE END Init */
 /* Configure the system clock */
 SystemClock_Config();
 /* USER CODE BEGIN SysInit */
 /* USER CODE END SysInit */
 /* Initialize all configured peripherals */

 MX_GPIO_Init();
 MX_USART2_UART_Init();
 /* USER CODE BEGIN 2 */
 // Unit impulse signal (Delta)
  for(n = 0; n < N; n++)
  if (n == 0) delta[n] = 1;
  else delta[n] = 0;
  // Unit step signal
  for(n = 0; n < N; n++)
  step[n] = 1;
  // Rectangular signal (non-zero between n=0 and n=5)
  for(n = 0; n < N; n++)
  if ((n >= 0) & (n < 6))
  rect[n] = 1;
  else
  rect[n] = 0;
  // Exponential signal
  for(n = 0; n < N; n++)
  expon[n] = pow(a, (float)n);
  // Original sinusoidal signal with w0 = pi/4
  for(n = 0; n < N; n++)
  sinus[n] = sin(w0 * (float)n);
  // New sinusoidal signal with w1 = pi/8
  for(n = 0; n < N; n++)
  sinus2[n] = sin(w1 * (float)n);
  // Composite signal (sum of the two sinusoids)
  for(n = 0; n < N; n++)
  composite[n] = sinus[n] + sinus2[n];
  // Time shift the delta signal by 3 units to the right
  for(n = 0; n < N; n++) {
  if (n < 3)
  shifted_delta[n] = 0;
  else
  shifted_delta[n] = delta[n - 3];
  }
  // Time shift the step signal by 3 units to the right
  for(n = 0; n < N; n++) {
  if (n < 3)
  shifted_step[n] = 0;
  else
  shifted_step[n] = step[n - 3];
  }
  // Time shift the exponential signal by 5 units to the right
  for(n = 0; n < N; n++) {
  if (n < 5)
  shifted_expon[n] = 0.0f;
  else
  shifted_expon[n] = expon[n - 5];
  }
  // Time shift the sinusoidal signal by 2 units to the right
  for(n = 0; n < N; n++) {
  if (n < 2)
  shifted_sinus[n] = 0.0f;
  else
  shifted_sinus[n] = sinus[n - 2];
  }
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
 if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) !=
HAL_OK)
 {
 Error_Handler();
 }
 /** Initializes the RCC Oscillators according to the specified parameters
 * in the RCC_OscInitTypeDef structure.
 */
 RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
 RCC_OscInitStruct.HSIState = RCC_HSI_ON;
 RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
 RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
 RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
 RCC_OscInitStruct.PLL.PLLM = 1;
 RCC_OscInitStruct.PLL.PLLN = 10;
 RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
 RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
 RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
 RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
 RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
 if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
 {
 Error_Handler();
 }
}
/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{
 /* USER CODE BEGIN USART2_Init 0 */
 /* USER CODE END USART2_Init 0 */
 /* USER CODE BEGIN USART2_Init 1 */
 /* USER CODE END USART2_Init 1 */
 huart2.Instance = USART2;
 huart2.Init.BaudRate = 115200;
 huart2.Init.WordLength = UART_WORDLENGTH_8B;
 huart2.Init.StopBits = UART_STOPBITS_1;
 huart2.Init.Parity = UART_PARITY_NONE;
 huart2.Init.Mode = UART_MODE_TX_RX;
 huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
 huart2.Init.OverSampling = UART_OVERSAMPLING_16;
 huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
 huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
 if (HAL_UART_Init(&huart2) != HAL_OK)
 {
 Error_Handler();
 }
 /* USER CODE BEGIN USART2_Init 2 */
 /* USER CODE END USART2_Init 2 */
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
 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
 /*Configure GPIO pin : B1_Pin */
 GPIO_InitStruct.Pin = B1_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
 /*Configure GPIO pin : LD2_Pin */
 GPIO_InitStruct.Pin = LD2_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}
/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
/**
 * @brief This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
 /* USER CODE BEGIN Error_Handler_Debug */
 /* User can add his own implementation to report the HAL error return state
*/
 __disable_irq();
 while (1)
 {
 }
 /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
 /* USER CODE BEGIN 6 */
 /* User can add his own implementation to report the file name and line
number,
 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)
*/
 /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
