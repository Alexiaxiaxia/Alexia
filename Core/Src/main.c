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
#include "lsm6dsl.h"
#include "b_l475e_iot01a1_bus.h"
#include "ai_platform.h"
#include "network.h"
#include "network_data.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lps22hb.h"
#include "lps22hb_reg.h"

#include "lis3mdl.h"
#include "lis3mdl_reg.h"

#include "hts221.h"
#include "hts221_reg.h"
#include "stm32l4xx_hal_tim.h"
#include "stm32l4xx_hal.h"

#define MAX_PRESSURE_POINTS 10  // 数组最大长度，用于存储压力数据
#define MAX_STRING_LENGTH 1024  // 字符串最大长度，用于打印消息
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

static LPS22HB_Object_t PressureSensor;
static float pressure;
static uint32_t dataRdyIntReceived = 0;


float pressureArray[MAX_PRESSURE_POINTS];  // 存储压力数据的数组
int currentPressureIndex = 0;  // 当前索引，用于追踪数组中的存储位置
LSM6DSL_Axes_t angular_velocity;    // LSM6DSL gyroscope
static HTS221_Object_t HTS221Sensor;
HTS221_Object_t hts221_obj; // 创建HTS221对象
HTS221_IO_t hts221_io;      // 创建与硬件通信接口相关的结构体


LIS3MDL_Object_t LIS3MDLSensor; // 全局变量声明
LIS3MDL_Axes_t magnetic_axes;

CRC_HandleTypeDef hcrc;

DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

QSPI_HandleTypeDef hqspi;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
LSM6DSL_Object_t MotionSensor;
//volatile uint32_t dataRdyIntReceived;

TIM_HandleTypeDef htim2;  // 定时器句柄

ai_handle network;
float aiInData[AI_NETWORK_IN_1_SIZE];
float aiOutData[AI_NETWORK_OUT_1_SIZE];
ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
const char* activities[AI_NETWORK_OUT_1_SIZE] = {
  "stationary", "walking", "running"
};
ai_buffer * ai_input;
ai_buffer * ai_output;

float pressureBuffer[40];
LSM6DSL_Axes_t angularVelocityBuffer[40]; // 假设角速度数据类型为 LSM6DSL_Axes_t
LIS3MDL_Axes_t magneticBuffer[80];
uint32_t dataCount = 0;
uint32_t angularCount = 0;
uint32_t pressureCount = 0;
uint32_t magneticCount = 0;

float humidity = 0.0f;
float temperature = 0.0f;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void Timer_Init(void);
void Timer_Start(void);
uint32_t Timer_Stop(void);

static void PRESSURE_Init(void);
void printAllPressureValues();//存压力数据
static void HTS221_Init_Custom(void);
static void LIS3MDL_Init_Custom(void);


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_CRC_Init(void);
void printArrays(void);
/* USER CODE BEGIN PFP */
static void MEMS_Init(void);
static void AI_Init(void);
static void AI_Run(float *pIn, float *pOut);
static uint32_t argmax(const float * values, uint32_t len);
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
  float humidity = 0.0f;
  float temperature = 0.0f;


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DFSDM1_Init();
  MX_QUADSPI_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_CRC_Init();
  Timer_Init();

  /* USER CODE BEGIN 2 */
  dataRdyIntReceived = 0;
  MEMS_Init();
  AI_Init();
  PRESSURE_Init(); // 初始化压力传感器
  HTS221_Init_Custom();
  LIS3MDL_Init_Custom();

  // 获取系统核心时钟频率
  uint32_t sysclk_freq = HAL_RCC_GetHCLKFreq();

  // 定时器预分频器
  uint32_t prescaler = htim2.Init.Prescaler + 1; // 实际的预分频值需要加1

  // 计算定时器频率
  uint32_t tim_freq = sysclk_freq / prescaler;

  printf("TIM2 Frequency: %lu Hz\n", tim_freq);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t write_index = 0;
  float hts221Frequence = 0.0;
  HTS221_HUM_GetOutputDataRate(&HTS221Sensor, &hts221Frequence);
  printf("f of HTS221_HUM: %.2f", hts221Frequence);

  float hts221TEMPFrequence = 0.0;
  HTS221_TEMP_GetOutputDataRate(&HTS221Sensor, &hts221TEMPFrequence);
  printf("f of HTS221_TEMP: %.2f", hts221TEMPFrequence);

  float is3mdFrequence = 0.0;
  LIS3MDL_MAG_GetOutputDataRate(&LIS3MDLSensor, &is3mdFrequence);
  printf("f of LIS3MDL: %.2f", is3mdFrequence);
  float lp22hbFrequence = 0.0;
  LPS22HB_PRESS_GetOutputDataRate(&PressureSensor, &lp22hbFrequence);
  printf("f of LPS22HB: %.2f", lp22hbFrequence);
  float lsm6dsl_acc_frequence = 0.0;
  LSM6DSL_ACC_GetOutputDataRate(&MotionSensor, &lsm6dsl_acc_frequence);
  printf("f of  LSM6DS_ACC: %.2f", lsm6dsl_acc_frequence);

  float lsm6dsl_gyro_frequence = 0.0;
  LSM6DSL_GYRO_GetOutputDataRate(&MotionSensor, &lsm6dsl_gyro_frequence);
  printf("f of  LSM6DS_gyro: %.2f", lsm6dsl_gyro_frequence);
  int slot = 0;
  while (1)
  {

      LIS3MDL_Axes_t magnetic_axes;
      Timer_Start();
//      HTS221_HUM_GetHumidity(&HTS221Sensor, &humidity);
//      uint32_t hts_elapsedTime = Timer_Stop();
//      float humidity = 0.0f;
//      float temperature = 0.0f;
//      printf("poll execution clock: %d \n", hts_elapsedTime);
      if(slot == 3){
    	  HTS221_HUM_GetHumidity(&HTS221Sensor, &humidity);
    	  HTS221_TEMP_GetTemperature(&HTS221Sensor, &temperature);
      }
      int slotD10 = slot % 10;
      if(slotD10 == 0){
    	  LSM6DSL_Axes_t acc_axes;
    	  LSM6DSL_ACC_GetAxes(&MotionSensor, &acc_axes);
    //	  printf("% 5d, % 5d, % 5d\r\n",  (int) acc_axes.x, (int) acc_axes.y, (int) acc_axes.z);

          /* Normalize data to [-1; 1] and accumulate into input buffer */
          /* Note: window overlapping can be managed here */
          aiInData[write_index + 0] = (float) acc_axes.x / 4000.0f;
          aiInData[write_index + 1] = (float) acc_axes.y / 4000.0f;
          aiInData[write_index + 2] = (float) acc_axes.z / 4000.0f;
          write_index += 3;
          // 3 * 25
          write_index %= 75;
      }
      if(slotD10 == 1){
    	  LSM6DSL_GYRO_GetAxes(&MotionSensor, &angular_velocity);
    	  angularVelocityBuffer[angularCount ++] = angular_velocity;
    	  angularCount %= 40;
      }
      if(slotD10 == 2){
    	  LPS22HB_PRESS_GetPressure(&PressureSensor, &pressure);
    	  pressureBuffer[pressureCount ++] = pressure;
    	  pressureCount %= 40;
      }
      int slotD5 = slot % 5;
      if(slotD5 == 4 && slot!=394){
    	  LIS3MDL_MAG_GetAxes(&LIS3MDLSensor, &magnetic_axes);
    	  magneticBuffer[magneticCount++] = magnetic_axes;
    	  magneticCount %= 80;
      }
      if(slot == 393){
    	  // 执行推理
          printf("Running inference\r\n");
          AI_Run(aiInData, aiOutData);

          /* Output results */
          for (uint32_t i = 0; i < AI_NETWORK_OUT_1_SIZE; i++) {
            printf("%8.6f ", aiOutData[i]);
          }
          uint32_t class = argmax(aiOutData, AI_NETWORK_OUT_1_SIZE);
          printf(": %d - %s\r\n", (int) class, activities[class]);
      }
      if(slot == 383){
    	  // output data
    	  printArrays();
      }
      slot ++;
      slot %= 400;
      uint32_t clockNum = Timer_Stop();
//      printf("clockNum %.d  \r\n",clockNum);

      double costTime = ((double)clockNum) / tim_freq ;
      int roundedUp = (int)ceil(costTime);

//      printf("executing time %.2e ms \r\n", costTime * 1000);
      HAL_Delay(costTime >= 1.5? 0 : (int)(2.5 -costTime));
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

//void printArrays(float* anglarSpeed, int size1, float* pressureArr, int size2, float* magneticArr, int size3, float* speedArr, int size4) {
void printArrays() {
	LSM6DSL_Axes_t* anglarSpeed = angularVelocityBuffer;
    int bufferSize = 1024;
	int size1 = 40;
    char* result = malloc(bufferSize); // 初始分配
    if (!result) {
        printf("Memory allocation failed\n");
        return;
    }
    result[0] = '\0'; // 初始化为空字符串

    int usedLength = 0;
    int tem_written =  snprintf(result + usedLength, bufferSize - usedLength, "\r\n angular speed data:");
    if (usedLength >= bufferSize - 1) { // 检查是否需要扩展缓冲区
        bufferSize *= 2;
        result = realloc(result, bufferSize);
        if (!result) {
            printf("Memory reallocation failed\n");
            return;
        }
    }
    usedLength += tem_written; // 更新已使用的长度
    for (int i = 0; i < size1; ++i) {
        int written = snprintf(result + usedLength, bufferSize - usedLength, "x:%.2f y:%.2f z:%.2f,", anglarSpeed[i].x,  anglarSpeed[i].y,  anglarSpeed[i].z);
        usedLength += written; // 更新已使用的长度
        if (usedLength >= bufferSize - 1) { // 检查是否需要扩展缓冲区
            bufferSize *= 2;
            result = realloc(result, bufferSize);
            if (!result) {
                printf("Memory reallocation failed\n");
                return;
            }
        }
    }
    float* pressureArr = pressureBuffer;
    int size2 = 40;
    tem_written =  snprintf(result + usedLength, bufferSize - usedLength, "\r\n pressure data:");
    usedLength += tem_written; // 更新已使用的长度
    // 遍历第二个数组
    if (usedLength >= bufferSize - 1) { // 检查是否需要扩展缓冲区
        bufferSize *= 2;
        result = realloc(result, bufferSize);
        if (!result) {
            printf("Memory reallocation failed\n");
            return;
        }
    }
    for (int i = 0; i < size2; ++i) {
        int written = snprintf(result + usedLength, bufferSize - usedLength, "%.2f, ", pressureArr[i]);
        usedLength += written; // 更新已使用的长度
        if (usedLength >= bufferSize - 1) { // 检查是否需要扩展缓冲区
            bufferSize *= 2;
            result = realloc(result, bufferSize);
            if (!result) {
                printf("Memory reallocation failed\n");
                return;
            }
        }
    }
    tem_written =  snprintf(result + usedLength, bufferSize - usedLength, "\r\n magnetic data:");
    usedLength += tem_written; // 更新已使用的长度
    if (usedLength >= bufferSize - 1) { // 检查是否需要扩展缓冲区
        bufferSize *= 2;
        result = realloc(result, bufferSize);
        if (!result) {
            printf("Memory reallocation failed\n");
            return;
        }
    }
    LIS3MDL_Axes_t* magneticArr = magneticBuffer;
    int size3 = 80;
    // 遍历第三个数组
    for (int i = 0; i < size3; ++i) {
        int written = snprintf(result + usedLength, bufferSize - usedLength, "x:%.2f y:%.2f z:%.2f, ", magneticArr[i].x, magneticArr[i].y, magneticArr[i].z);
        usedLength += written; // 更新已使用的长度
        if (usedLength >= bufferSize - 1) { // 检查是否需要扩展缓冲区
            bufferSize *= 2;
            result = realloc(result, bufferSize);
            if (!result) {
                printf("Memory reallocation failed\n");
                return;
            }
        }
    }
    tem_written =  snprintf(result + usedLength, bufferSize - usedLength, "\r\n speedArr data:");
    usedLength += tem_written; // 更新已使用的长度
    if (usedLength >= bufferSize - 1) { // 检查是否需要扩展缓冲区
        bufferSize *= 2;
        result = realloc(result, bufferSize);
        if (!result) {
            printf("Memory reallocation failed\n");
            return;
        }
    }
    // 遍历第四个数组
    float* speedArr = aiInData;
    int size4 = AI_NETWORK_IN_1_SIZE;
    for (int i = 0; i < size4; i+=3) {
        int written = snprintf(result + usedLength, bufferSize - usedLength, "x:%.2f y:%.2f z:%.2f ", speedArr[i], speedArr[i + 1], speedArr[i + 2]);
        usedLength += written; // 更新已使用的长度
        if (usedLength >= bufferSize - 1) { // 检查是否需要扩展缓冲区
            bufferSize *= 2;
            result = realloc(result, bufferSize);
            if (!result) {
                printf("Memory reallocation failed\n");
                return;
            }
        }
    }
    tem_written =  snprintf(result + usedLength, bufferSize - usedLength, "\r\n  humidity:%.2f", humidity);
    usedLength += tem_written; // 更新已使用的长度
    if (usedLength >= bufferSize - 1) { // 检查是否需要扩展缓冲区
        bufferSize *= 2;
        result = realloc(result, bufferSize);
        if (!result) {
            printf("Memory reallocation failed\n");
            return;
        }
    }
    tem_written =  snprintf(result + usedLength, bufferSize - usedLength, "\r\n  temperature:%.2f", temperature);
    usedLength += tem_written; // 更新已使用的长度
    if (usedLength >= bufferSize - 1) { // 检查是否需要扩展缓冲区
        bufferSize *= 2;
        result = realloc(result, bufferSize);
        if (!result) {
            printf("Memory reallocation failed\n");
            return;
        }
    }

    printf("%s\n", result); // 打印结果
    free(result); // 释放内存
}

void Timer_Init(void) {
  __HAL_RCC_TIM2_CLK_ENABLE();  // 使能TIM2时钟

  htim2.Instance = TIM2;  // 指定定时器实例
  htim2.Init.Prescaler = (uint32_t)((SystemCoreClock / 2) / 1000000) - 1;  // 预分频器，假设时钟频率为80MHz，我们想要定时器时钟为1MHz
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;  // 向上计数模式
  htim2.Init.Period = 0xFFFFFFFF;  // 设置自动重装载寄存器周期的最大值
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;  // 不分频
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;  // 禁用自动重载预装载
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {  // 初始化基本定时器
    Error_Handler();
  }
}
/* 启动定时器 */
void Timer_Start(void) {
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);  // 清除更新标志
  HAL_TIM_Base_Start(&htim2);  // 启动定时器
}

/* 停止定时器并获取当前计时 */
uint32_t Timer_Stop(void) {
    HAL_TIM_Base_Stop(&htim2);  // 停止定时器
    uint32_t elapsedTime = __HAL_TIM_GET_COUNTER(&htim2);  // 获取当前计数值
    __HAL_TIM_SET_COUNTER(&htim2, 0);  // 重置计数器为0
    return elapsedTime;  // 返回计时
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
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

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 2;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
                           ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin
                          |ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D4_Pin */
  GPIO_InitStruct.Pin = ARD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin
                          |PMOD_IRQ_EXTI12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

static void AI_Init(void)
{
  ai_error err;

  /* Create a local array with the addresses of the activations buffers */
  const ai_handle act_addr[] = { activations };
  /* Create an instance of the model */
  err = ai_network_create_and_init(&network, act_addr, NULL);
  if (err.type != AI_ERROR_NONE) {
    printf("ai_network_create error - type=%d code=%d\r\n", err.type, err.code);
    Error_Handler();
  }
  ai_input = ai_network_inputs_get(network, NULL);
  ai_output = ai_network_outputs_get(network, NULL);
}

static void AI_Run(float *pIn, float *pOut)
{
  ai_i32 batch;
  ai_error err;

  /* Update IO handlers with the data payload */
  ai_input[0].data = AI_HANDLE_PTR(pIn);
  ai_output[0].data = AI_HANDLE_PTR(pOut);

  batch = ai_network_run(network, ai_input, ai_output);
  if (batch != 1) {
    err = ai_network_get_error(network);
    printf("AI ai_network_run error - type=%d code=%d\r\n", err.type, err.code);
    Error_Handler();
  }
}

static uint32_t argmax(const float * values, uint32_t len)
{
  float max_value = values[0];
  uint32_t max_index = 0;
  for (uint32_t i = 1; i < len; i++) {
    if (values[i] > max_value) {
      max_value = values[i];
      max_index = i;
    }
  }
  return max_index;
}


static void MEMS_Init(void)
{
  LSM6DSL_IO_t io_ctx;
  uint8_t id;
  LSM6DSL_AxesRaw_t axes;

  /* Link I2C functions to the LSM6DSL driver */
  io_ctx.BusType     = LSM6DSL_I2C_BUS;
  io_ctx.Address     = LSM6DSL_I2C_ADD_L;
  io_ctx.Init        = BSP_I2C2_Init;
  io_ctx.DeInit      = BSP_I2C2_DeInit;
  io_ctx.ReadReg     = BSP_I2C2_ReadReg;
  io_ctx.WriteReg    = BSP_I2C2_WriteReg;
  io_ctx.GetTick     = BSP_GetTick;
  LSM6DSL_RegisterBusIO(&MotionSensor, &io_ctx);

  /* Read the LSM6DSL WHO_AM_I register */
  LSM6DSL_ReadID(&MotionSensor, &id);
  if (id != LSM6DSL_ID) {
    Error_Handler();
  }

  /* Initialize the LSM6DSL sensor */
  LSM6DSL_Init(&MotionSensor);

  /* Configure the LSM6DSL accelerometer (ODR, scale and interrupt) */
  LSM6DSL_ACC_SetOutputDataRate(&MotionSensor, 25.0f); /* 26 Hz */
  LSM6DSL_ACC_SetFullScale(&MotionSensor, 4);          /* [-4000mg; +4000mg] */
  LSM6DSL_ACC_Set_INT1_DRDY(&MotionSensor, ENABLE);    /* Enable DRDY */
  LSM6DSL_ACC_GetAxesRaw(&MotionSensor, &axes);        /* Clear DRDY */

  /* Start the LSM6DSL accelerometer */
  LSM6DSL_ACC_Enable(&MotionSensor);
}

//气压计初始化
static void PRESSURE_Init(void)
{
  /* Initialize the pressure sensor */
  PressureSensor.IO.BusType = LPS22HB_I2C_BUS;
  PressureSensor.IO.Address = LPS22HB_I2C_ADD_H;
  PressureSensor.IO.Init = BSP_I2C2_Init;
  PressureSensor.IO.DeInit = BSP_I2C2_DeInit;
  PressureSensor.IO.ReadReg = BSP_I2C2_ReadReg;
  PressureSensor.IO.WriteReg = BSP_I2C2_WriteReg;
  PressureSensor.IO.Delay = HAL_Delay;

  if (LPS22HB_RegisterBusIO(&PressureSensor, &PressureSensor.IO) != LPS22HB_OK)
  {
    Error_Handler();
  }

  if (LPS22HB_Init(&PressureSensor) != LPS22HB_OK)
  {
    Error_Handler();
  }

  /* Enable pressure sensor */
  if (LPS22HB_PRESS_Enable(&PressureSensor) != LPS22HB_OK)
  {
    Error_Handler();
  }

  /* Set pressure output data rate */
  if (LPS22HB_PRESS_SetOutputDataRate(&PressureSensor, 25.0f) != LPS22HB_OK)
  {
    Error_Handler();
  }
}


//温度计初始化
static void HTS221_Init_Custom(void)
{
    HTS221_IO_t io_ctx = {0};
    HTS221_Capabilities_t cap;

    /* Configure the HTS221 driver with the I2C bus handlers */
    io_ctx.BusType = HTS221_I2C_BUS; /* Assuming I2C bus */
    io_ctx.Address = HTS221_I2C_ADDRESS;
    io_ctx.Init = BSP_I2C2_Init;
    io_ctx.DeInit = BSP_I2C2_DeInit;
    io_ctx.ReadReg = BSP_I2C2_ReadReg;
    io_ctx.WriteReg = BSP_I2C2_WriteReg;
    io_ctx.GetTick = BSP_GetTick;

    /* Register the bus IOs */
    if (HTS221_RegisterBusIO(&HTS221Sensor, &io_ctx) != HTS221_OK)
    {
        Error_Handler();
    }

    /* Initialize the HTS221 sensor */
    if (HTS221_Init(&HTS221Sensor) != HTS221_OK)
    {
        Error_Handler();
    }

    /* Check device ID */
    uint8_t id;
    if (HTS221_ReadID(&HTS221Sensor, &id) != HTS221_OK || id != HTS221_ID)
    {
        Error_Handler();
    }

    /* Get capabilities */
    if (HTS221_GetCapabilities(&HTS221Sensor, &cap) != HTS221_OK)
    {
        Error_Handler();
    }
}


//磁力计初始化
static void LIS3MDL_Init_Custom(void)
{
    LIS3MDL_IO_t io_ctx;
    uint8_t id;
    float sensitivity;

    // 初始化接口函数
    io_ctx.BusType = LIS3MDL_I2C_BUS;
    io_ctx.Address = LIS3MDL_I2C_ADD_H; // 确保这是您的设备I2C地址
    io_ctx.Init = BSP_I2C2_Init;
    io_ctx.DeInit = BSP_I2C2_DeInit;
    io_ctx.ReadReg = BSP_I2C2_ReadReg;
    io_ctx.WriteReg = BSP_I2C2_WriteReg;
    io_ctx.GetTick = BSP_GetTick;


    // 注册接口函数到LIS3MDL对象
    if (LIS3MDL_RegisterBusIO(&LIS3MDLSensor, &io_ctx) != LIS3MDL_OK) {
        Error_Handler(); // 错误处理
    }

    // 初始化LIS3MDL
    if (LIS3MDL_Init(&LIS3MDLSensor) != LIS3MDL_OK) {
        Error_Handler(); // 错误处理
    }

    // 检查设备ID
    if (LIS3MDL_ReadID(&LIS3MDLSensor, &id) != LIS3MDL_OK || id != LIS3MDL_ID) {
        Error_Handler(); // 错误处理
    }

}




void printAllPressureValues() {
    char buffer[MAX_STRING_LENGTH] = "";  // 创建一个足够大的字符串缓冲区
    for (int i = 0; i < MAX_PRESSURE_POINTS; i++) {
        // 将当前压力值追加到buffer中
        // 注意：确保不要溢出buffer
        char temp[64];
        snprintf(temp, sizeof(temp), "%sPressure [%d] [hPa]: %.2f", i == 0 ? "" : ", ", i + 1, pressureArray[i]);
        strcat(buffer, temp);
    }

    // 打印所有压力值
    printf("%s\n", buffer);

    // 重置索引，从头开始存储
    currentPressureIndex = 0;
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin) {
  case LPS22HB_INT_DRDY_EXTI0_Pin:
    dataRdyIntReceived++;
    break;
  case 2048:
	    dataRdyIntReceived++;
  default:
    break;
  }
}



//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//  if (GPIO_Pin == GPIO_PIN_11) {
//    dataRdyIntReceived++;
//  }
//}

int _write(int fd, char * ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t *) ptr, len, HAL_MAX_DELAY);
  return len;
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
	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	HAL_Delay(50); /* wait 50 ms */

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
