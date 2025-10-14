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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_bus_stm32.h"
#include "eeprom_emul.h"
#include "heartbeat_slave.h"
#include "led_ctrl.h"
#include "lss_slave.h"
#include "pdo_slave.h"
#include "pins.h"
#include "sdo_slave.h"

#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    bool stable_state;
    bool last_raw;
    uint32_t last_transition;
} input_channel_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
static input_channel_state_t input_state[INPUT_CHANNEL_COUNT];
static uint32_t inputs_bitmap = 0U;
static uint8_t input_change_counter = 0U;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
static void process_can_frames(void);
static void monitor_can_health(uint32_t now_ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if CAN_TEST_BROADCAST
static void Drive_Led_On(bool on)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void CAN_TestToggle_Init(void)
{
    CAN_FilterTypeDef filter = {0};
    filter.FilterActivation = ENABLE;
    filter.FilterBank = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = (uint16_t)(0x100U << 5);
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = (uint16_t)(0x7FFU << 5);
    filter.FilterMaskIdLow = 0U;
    filter.SlaveStartFilterBank = 14U;

    if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_GetState(&hcan) == HAL_CAN_STATE_READY)
    {
        if (HAL_CAN_Start(&hcan) != HAL_OK)
        {
            Error_Handler();
        }
    }

    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }

    Drive_Led_On(false);
}

static void CAN_TestToggle_ProcessFrame(const can_frame_t *frame)
{
    if ((frame == NULL) || (frame->id != 0x100U) || (frame->dlc != 1U))
    {
        return;
    }

    if (frame->data[0] == 0x01U)
    {
        Drive_Led_On(true);
    }
    else if (frame->data[0] == 0x00U)
    {
        Drive_Led_On(false);
    }
}
#endif

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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  CAN_Bus_Init();
#if CAN_TEST_BROADCAST
  CAN_TestToggle_Init();
#endif
  LED_Ctrl_Init();
  SDO_Slave_Init();
  PDO_Slave_Init();
  Heartbeat_Slave_Init();
  LSS_Slave_Init();
  CAN_Bus_Start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

      uint32_t now = HAL_GetTick();
      process_can_frames();

      for (uint8_t i = 0; i < INPUT_CHANNEL_COUNT; ++i)
      {
          GPIO_PinState pin = HAL_GPIO_ReadPin(INPUT_PORTS[i], INPUT_PINS[i]);
          bool active = (pin == GPIO_PIN_RESET);
          const input_config_t *cfg = SDO_Slave_GetInputConfig(i);
          if (cfg->inverted)
          {
              active = !active;
          }
          if (input_state[i].last_raw != active)
          {
              input_state[i].last_raw = active;
              input_state[i].last_transition = now;
          }
          uint32_t debounce = cfg->debounce_ms;
          if (debounce == 0U)
          {
              debounce = 1U;
          }
          if ((now - input_state[i].last_transition) >= debounce)
          {
              if (input_state[i].stable_state != active)
              {
                  input_state[i].stable_state = active;
                  if (active)
                  {
                      inputs_bitmap |= (1UL << i);
                  }
                  else
                  {
                      inputs_bitmap &= ~(1UL << i);
                  }
                  input_change_counter++;
                  PDO_Slave_OnInputChange(inputs_bitmap, input_change_counter);
              }
          }
      }

      LED_Ctrl_Task(now);
      PDO_Slave_Task(now);
      Heartbeat_Slave_Task(now);
      LSS_Slave_Task(now);
      monitor_can_health(now);

      HAL_Delay(1);
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

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OUT0_Pin|OUT1_Pin|OUT2_Pin|OUT3_Pin
                          |OUT4_Pin|OUT5_Pin|OUT6_Pin|OUT7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT0_Pin OUT1_Pin OUT2_Pin OUT3_Pin
                           OUT4_Pin OUT5_Pin OUT6_Pin OUT7_Pin */
  GPIO_InitStruct.Pin = OUT0_Pin|OUT1_Pin|OUT2_Pin|OUT3_Pin
                          |OUT4_Pin|OUT5_Pin|OUT6_Pin|OUT7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IN0_Pin IN1_Pin IN2_Pin IN10_Pin
                           IN11_Pin IN12_Pin IN13_Pin IN14_Pin
                           IN15_Pin IN3_Pin IN4_Pin IN5_Pin
                           IN6_Pin IN7_Pin IN8_Pin IN9_Pin */
  GPIO_InitStruct.Pin = IN0_Pin|IN1_Pin|IN2_Pin|IN10_Pin
                          |IN11_Pin|IN12_Pin|IN13_Pin|IN14_Pin
                          |IN15_Pin|IN3_Pin|IN4_Pin|IN5_Pin
                          |IN6_Pin|IN7_Pin|IN8_Pin|IN9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void monitor_can_health(uint32_t now_ms)
{
    static uint32_t last_check = 0U;
    static bool bus_fault_latched = false;
    static bool master_missing_latched = false;

    if (LED_Ctrl_IsIdentifyActive())
    {
        last_check = now_ms;
        return;
    }

    if ((now_ms - last_check) < 200U)
    {
        return;
    }
    last_check = now_ms;

    can_bus_diagnostics_t diag = {0};
    CAN_Bus_GetDiagnostics(&diag);

    bool bus_fault = diag.bus_off || diag.error_passive || (diag.error_code != HAL_CAN_ERROR_NONE);
    bool master_missing = false;

    if (!bus_fault)
    {
        if ((diag.rx_received == 0U) && (diag.tx_successful > 0U))
        {
            master_missing = (now_ms > 2000U);
        }
        else if (diag.last_rx_tick != 0U)
        {
            master_missing = (now_ms - diag.last_rx_tick) > 2000U;
        }
    }

    if ((bus_fault != bus_fault_latched) || (master_missing != master_missing_latched))
    {
        if (bus_fault)
        {
            LED_Ctrl_Command(0x01, 0U, 2U);
        }
        else if (master_missing)
        {
            LED_Ctrl_Command(0x01, 0U, 1U);
        }
        else
        {
            LED_Ctrl_Command(0x00, 0U, 0U);
        }

        bus_fault_latched = bus_fault;
        master_missing_latched = master_missing;
    }
}

static void process_can_frames(void)
{
    can_frame_t frame;
    for (;;)
    {
#if CAN_TEST_BROADCAST
        if (!CAN_Bus_Read(&frame))
        {
            break;
        }
#else
        if (HAL_CAN_GetRxFifoFillLevel(CAN_Bus_GetHandle(), CAN_RX_FIFO0) == 0U)
        {
            break;
        }
        if (!CAN_Bus_Read(&frame))
        {
            break;
        }
#endif
#if CAN_TEST_BROADCAST
        CAN_TestToggle_ProcessFrame(&frame);
#endif
        LSS_Slave_OnFrame(&frame);
        PDO_Slave_OnFrame(&frame);
        SDO_Slave_OnFrame(&frame);
        LED_Ctrl_AnnounceTraffic(40U);
    }
}
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
      HAL_GPIO_TogglePin(LED_IDENTIFY_PORT, LED_IDENTIFY_PIN);
      HAL_Delay(100);
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
