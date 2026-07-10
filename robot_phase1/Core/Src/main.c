/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2; // USART2 -> PA2/PA3 -> USB/COM8 (laptop debug link)
UART_HandleTypeDef huart1; // USART1 -> PA9/PA10 -> Raspberry Pi link

/* USER CODE BEGIN PV */
// ---- Command receiving: laptop (USART2) ----
volatile uint8_t rx_byte2;
volatile char rx_buffer2[64];
volatile uint8_t rx_index2 = 0;
volatile uint8_t command_ready2 = 0;
char command_copy2[64];

// ---- Command receiving: Raspberry Pi (USART1) ----
volatile uint8_t rx_byte1;
volatile char rx_buffer1[64];
volatile uint8_t rx_index1 = 0;
volatile uint8_t command_ready1 = 0;
char command_copy1[64];

// ---- Speed calculation ----
int32_t last_left_count = 0;
int16_t last_right_count = 0;
uint32_t last_speed_tick = 0;
int32_t left_speed = 0;   // counts per 100ms
int32_t right_speed = 0;  // counts per 100ms
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void PWM_Init(void);
void Direction_Init(void);
void Encoder_Init(void);
void MX_USART1_UART_Init(void);
void Process_Command(char *cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void PWM_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |= (2 << 12) | (2 << 14);
    GPIOA->AFR[0] |= (2 << 24) | (2 << 28);

    TIM3->PSC = 0;
    TIM3->ARR = 8999;

    TIM3->CCMR1 |= (6 << 4) | (6 << 12);

    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM3->CR1 |= TIM_CR1_CEN;

    TIM3->CCR1 = 4500;  // left motor
    TIM3->CCR2 = 4500;  // right motor
}

void Direction_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER |= (1 << 0) | (1 << 2) | (1 << 4) | (1 << 6);

    // default: forward
    GPIOB->ODR |= (1 << 0) | (1 << 2);
    GPIOB->ODR &= ~((1 << 1) | (1 << 3));
}

void Encoder_Init(void)
{
    // ---- Left encoder: PA0/PA1 -> TIM2_CH1/CH2 ----
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |= (2 << 0) | (2 << 2);
    GPIOA->AFR[0] |= (1 << 0) | (1 << 4);   // AF1 = TIM2

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->SMCR |= (3 << 0);
    TIM2->CCMR1 |= (1 << 0) | (1 << 8);
    TIM2->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
    TIM2->ARR = 0xFFFFFFFF;
    TIM2->CR1 |= TIM_CR1_CEN;

    // ---- Right encoder: PB6/PB7 -> TIM4_CH1/CH2 ----
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER |= (2 << 12) | (2 << 14);
    GPIOB->AFR[0] |= (2 << 24) | (2 << 28); // AF2 = TIM4 -- verified working from your test

    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->SMCR |= (3 << 0);
    TIM4->CCMR1 |= (1 << 0) | (1 << 8);
    TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
    TIM4->ARR = 0xFFFF;
    TIM4->CR1 |= TIM_CR1_CEN;
}

/**
  * @brief USART1 Initialization -- dedicated link to the Raspberry Pi.
  * PA9  = TX (STM32 sends -> Pi RXD)
  * PA10 = RX (STM32 receives <- Pi TXD)
  */
void MX_USART1_UART_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // PA9, PA10 -> alternate function mode
    GPIOA->MODER &= ~((3 << 18) | (3 << 20));
    GPIOA->MODER |= (2 << 18) | (2 << 20);
    // AF7 = USART1 on PA9/PA10
    GPIOA->AFR[1] &= ~((0xF << 4) | (0xF << 8));
    GPIOA->AFR[1] |= (7 << 4) | (7 << 8);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
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
}

// ---- Command parser ----
// Supported commands (case-sensitive, no quotes, sent with a newline):
//   FORWARD          - both wheels forward direction
//   BACKWARD         - both wheels backward direction
//   LEFT             - pivot: left wheel backward, right wheel forward
//   RIGHT            - pivot: right wheel backward, left wheel forward
//   STOP             - PWM to 0 on both wheels
//   SPEED <0-8999>   - set PWM duty cycle on both wheels (e.g. "SPEED 4500")
// Shared by both the laptop (USART2) and the Pi (USART1) -- same protocol either way.
void Process_Command(char *cmd)
{
    if (strncmp(cmd, "FORWARD", 7) == 0)
    {
        GPIOB->ODR |= (1 << 0) | (1 << 2);
        GPIOB->ODR &= ~((1 << 1) | (1 << 3));
    }
    else if (strncmp(cmd, "BACKWARD", 8) == 0)
    {
        GPIOB->ODR |= (1 << 1) | (1 << 3);
        GPIOB->ODR &= ~((1 << 0) | (1 << 2));
    }
    else if (strncmp(cmd, "LEFT", 4) == 0)
    {
        GPIOB->ODR |= (1 << 1) | (1 << 2);
        GPIOB->ODR &= ~((1 << 0) | (1 << 3));
    }
    else if (strncmp(cmd, "RIGHT", 5) == 0)
    {
        GPIOB->ODR |= (1 << 0) | (1 << 3);
        GPIOB->ODR &= ~((1 << 1) | (1 << 2));
    }
    else if (strncmp(cmd, "STOP", 4) == 0)
    {
        TIM3->CCR1 = 0;
        TIM3->CCR2 = 0;
    }
    else if (strncmp(cmd, "SPEED ", 6) == 0)
    {
        int val = atoi(cmd + 6);
        if (val < 0) val = 0;
        if (val > 8999) val = 8999;
        TIM3->CCR1 = val;
        TIM3->CCR2 = val;
    }
    // unrecognized commands are silently ignored
}

/* USER CODE END 0 */

/**
  * @brief  Fires when a UART byte finishes receiving on either USART1 or USART2.
  * Builds up a line until '\n' or '\r', then flags it ready for that port.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        if (rx_byte2 == '\n' || rx_byte2 == '\r')
        {
            if (rx_index2 > 0)
            {
                rx_buffer2[rx_index2] = '\0';
                for (uint8_t i = 0; i <= rx_index2; i++) command_copy2[i] = rx_buffer2[i];
                command_ready2 = 1;
                rx_index2 = 0;
            }
        }
        else if (rx_index2 < sizeof(rx_buffer2) - 1)
        {
            rx_buffer2[rx_index2++] = rx_byte2;
        }
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_byte2, 1);
    }
    else if (huart->Instance == USART1)
    {
        if (rx_byte1 == '\n' || rx_byte1 == '\r')
        {
            if (rx_index1 > 0)
            {
                rx_buffer1[rx_index1] = '\0';
                for (uint8_t i = 0; i <= rx_index1; i++) command_copy1[i] = rx_buffer1[i];
                command_ready1 = 1;
                rx_index1 = 0;
            }
        }
        else if (rx_index1 < sizeof(rx_buffer1) - 1)
        {
            rx_buffer1[rx_index1++] = rx_byte1;
        }
        HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_byte1, 1);
    }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  PWM_Init();
  Direction_Init();
  Encoder_Init();
  MX_USART1_UART_Init();

  // Enable both UART interrupts in the NVIC -- without this, HAL_UART_Receive_IT()
  // arms the peripheral but the interrupt never actually fires.
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  HAL_UART_Receive_IT(&huart2, (uint8_t*)&rx_byte2, 1); // listen for laptop commands
  HAL_UART_Receive_IT(&huart1, (uint8_t*)&rx_byte1, 1); // listen for Pi commands
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  char buf[80];
  while (1)
  {
    // ---- handle completed commands from either source ----
    if (command_ready2)
    {
        Process_Command(command_copy2);
        command_ready2 = 0;
    }
    if (command_ready1)
    {
        Process_Command(command_copy1);
        command_ready1 = 0;
    }

    // ---- read current encoder counts ----
    int32_t left_count  = (int32_t)TIM2->CNT;
    int16_t right_count = (int16_t)TIM4->CNT;

    // ---- speed calculation every 100ms ----
    if (HAL_GetTick() - last_speed_tick >= 100)
    {
        left_speed  = left_count - last_left_count;
        right_speed = right_count - last_right_count;
        last_left_count = left_count;
        last_right_count = right_count;
        last_speed_tick = HAL_GetTick();
    }

    // ---- report status: PWM, encoder counts, speed ----
    // Sent to BOTH the laptop (USART2, for your debugging) AND the Pi
    // (USART1, so it has live encoder/speed feedback for navigation).
    sprintf(buf, "%d,%d,%ld,%d,%ld,%ld\n",
            (int)TIM3->CCR1, (int)TIM3->CCR2,
            left_count, right_count,
            left_speed, right_speed);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
    HAL_Delay(20); // 50Hz
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

/**
  * @brief USART2 Initialization Function (laptop / ST-Link debug link)
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
