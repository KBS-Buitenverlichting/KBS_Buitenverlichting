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
#include "app_subghz_phy.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "radio.h"
#include "radio_conf.h"
#include "radio_def.h"
#include "radio_driver.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "stm32wlxx_hal.h"
#include "stm32wlxx_hal_gpio.h"
#include "utilities_def.h"
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
const uint8_t TCXO_TIME_MARGIN = 5; // Extra time to ensure proper startup
const uint16_t TRANSMIT_PERIOD_MS = 5000;
const uint32_t FREQUENCY = 868100000; // 868,1 MHz
#define DELAY_MS 266
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SUBGHZ_HandleTypeDef hsubghz;

/* USER CODE BEGIN PV */
static RadioEvents_t RadioEvents;
static unsigned char buffer[] = "STM32WLE5JCIx";
/* USER CODE END PV */

/* Private function prototypes
   -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
inline void delay(const uint32_t delay);
void Gpio_Init(void);
static void OnTxDone(void);
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi,
                     int8_t LoraSnr_FskCfo);
static void OnTxTimeout(void);
static void OnRxTimeout(void);
static void OnRxError(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  Gpio_Init();
  /* USER CODE END SysInit */

  /* USER CODE BEGIN 2 */
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init(&RadioEvents);

  Radio.SetTxConfig(MODEM_LORA, 22, 0, LORA_BW_500, LORA_SF7, LORA_CR_4_6,
                    RADIO_PREAMBLE_DETECTOR_32_BITS, LORA_PACKET_FIXED_LENGTH,
                    LORA_CRC_ON, 0, 0, LORA_IQ_INVERTED, 1000);
  // Radio.SetRxConfig(MODEM_LORA, LORA_BW_500, LORA_SF7, LORA_CR_4_6, 0,
  //                   RADIO_PREAMBLE_DETECTOR_32_BITS, 0,
  //                   LORA_PACKET_FIXED_LENGTH, 0, LORA_CRC_ON, 0, 0,
  //                   LORA_IQ_INVERTED, false);

  Radio.SetMaxPayloadLength(MODEM_LORA, strlen(buffer));
  Radio.SetChannel(FREQUENCY);

  SUBGRF_SetDioIrqParams(IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_RX_DONE,
                         IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT, IRQ_RX_DONE,
                         IRQ_RADIO_NONE);

  // Transmitter code
  delay(DELAY_MS * (Radio.GetWakeupTime() + TCXO_TIME_MARGIN));
  Radio.Send(buffer, strlen(buffer));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
    delay(DELAY_MS * 1000);
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3 | RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief SUBGHZ Initialization Function
 * @param None
 * @retval None
 */
void MX_SUBGHZ_Init(void) {

  /* USER CODE BEGIN SUBGHZ_Init 0 */

  /* USER CODE END SUBGHZ_Init 0 */

  /* USER CODE BEGIN SUBGHZ_Init 1 */

  /* USER CODE END SUBGHZ_Init 1 */
  hsubghz.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_8;
  if (HAL_SUBGHZ_Init(&hsubghz) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SUBGHZ_Init 2 */

  /* USER CODE END SUBGHZ_Init 2 */
}

/* USER CODE BEGIN 4 */
void delay(const uint32_t delay) {
  volatile int i;
  for (i = 0; i < delay; i++) {
  };
}

void Gpio_Init() {
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void OnTxDone(void) {
  Radio.Sleep();

  while (1) {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
    delay(DELAY_MS * 200);
  }
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi,
                     int8_t LoraSnr_FskCfo) {}

static void OnTxTimeout(void) {
  while (1) {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
    delay(DELAY_MS * 200);
  }
}

static void OnRxTimeout(void) {}

static void OnRxError(void) {}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
    delay(DELAY_MS * 100);
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
