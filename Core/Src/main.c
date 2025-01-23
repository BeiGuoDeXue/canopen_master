/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bsp_can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <applicfg.h>
#include <stdlib.h>
#include "canfestival.h"
// #include "config.h" // dardware下
#include "TestAll.h"
#include "data.h"
#include <stdio.h>
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
static UNS32 SDO_TIMEOUT = 1000; // 1秒超时
static volatile UNS8 SDO_COMPLETE = 0;
static volatile UNS32 SDO_RESULT = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SDOCallback(CO_Data* d, UNS8 nodeId)
{
    UNS32 abortCode;
    UNS8 res = getWriteResultNetworkDict(d, nodeId, &abortCode);
    
    if (res == SDO_FINISHED) {
        printf("SDO写入成功: nodeId=0x%02X\n", nodeId);
        SDO_RESULT = 0;
    } else {
        printf("SDO写入失败: nodeId=0x%02X, res=0x%02X, abortCode=0x%08X\n", 
               nodeId, res, abortCode);
        SDO_RESULT = abortCode;
    }
    SDO_COMPLETE = 1;
}

// SDO写入函数
UNS8 SDO_Write(CO_Data* d, UNS8 nodeId, UNS16 index, UNS8 subIndex, void* data, UNS32 size)
{
    UNS8 err;
    UNS32 startTime;
    
    // 重置状态
    SDO_COMPLETE = 0;
    SDO_RESULT = 0;
    
    // 发送SDO写请求
    // err = writeNetworkDictCallBack(d, nodeId, index, subIndex, size, data, SDOCallback, 0);
    err = writeNetworkDictCallBack(d, 
        nodeId,         // 目标节点
        index,          // 目标索引
        subIndex,       // 子索引
        size,          // 数据长度
        0,             // 数据类型（0表示自动）
        data,          // 数据指针
        SDOCallback,   // 回调函数
        0              // 字节序
    );

    if (err != 0) {
        printf("SDO写入请求发送失败: err=0x%02X\n", err);
        return err;
    }
    
    // 等待响应
    startTime = HAL_GetTick();
    while (!SDO_COMPLETE) {
        if (HAL_GetTick() - startTime > SDO_TIMEOUT) {
            printf("SDO写入超时\n");
            return 0xFF;
        }
    }
    
    return (SDO_RESULT == 0) ? 0 : 0xFF;
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
  MX_CAN_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	printf("hello world\n");
  Configure_Filter();
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_TIM_Base_Start_IT(&htim3);

	unsigned char nodeID = 0x00;                   //节点ID
	setNodeId(&TestAll_Data, nodeID);
	setState(&TestAll_Data, Initialisation);				//节点初始化
	setState(&TestAll_Data, Operational);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    printf("master test\n");

    // 打印当前状态
    printf("主站状态: %d\n", getState(&TestAll_Data));
    printf("SDO状态: %d\n", TestAll_Data.CurrentCommunicationState.csSDO);

    // SDO写入测试
    INTEGER8 data1 = 0x12;
    UNS8 res = SDO_Write(&TestAll_Data, 0x02, 0x2001, 0x00, &data1, sizeof(INTEGER8));

    HAL_Delay(1000);
	
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
