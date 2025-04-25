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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "mpu6050.h"
#include "micros.h"
#include "quaternion.h"
#include "FusionAHRS.h"
#include "LowPassFilter.h"
#include "NotchFilter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
MPU6050_t imu_t;

Quaternion_t quaternion_t;

NotchFilter_t NF_gyro_x, NF_gyro_y, NF_gyro_z;
LPFTwoPole_t LPF_accel_x, LPF_accel_y, LPF_accel_z, LPF_gyro_x, LPF_gyro_y, LPF_gyro_z;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define 	SAMPLE_FREQ_HZ 		1000.0f	// Data sample frequency in Hz
#define 	LPF_ACCEL_CTOFF_HZ 	260.0f	// LPF Cut-Off or accelerometer
#define 	LPF_GYRO_CTOFF_HZ 	256.0f	// LPF Cut-Off or gyro
#define 	NF_GYRO_CFREQ_HZ	74.0f	// NF center frequency for Gyro
#define 	NF_GYRO_NWDTH_HZ	5.0f	// NF notch-width frequency for Gyro
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint32_t Timer_GetElapsed(TIM_HandleTypeDef *htim, uint32_t timer_start);
uint8_t indeks(uint8_t *dizi, char harf);
void UartTxWriter();
void UartRxDataUse();
void MotorValueUpload(uint8_t indis, uint8_t PWM);
int16_t MotorValueDownload(uint8_t indis);
int8_t faz_indeks(uint8_t *dizi, uint8_t sira);
void SendOrientationData(float yaw, float pitch, float roll);
void Komut_A();
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float sample_time_sec_f32 = 1.0f / SAMPLE_FREQ_HZ;
float sample_time_us_f32 = (1.0f / SAMPLE_FREQ_HZ) * 1000000.0f;
float accelLowPassFiltered_f32[3], gyroLowPassFiltered_f32[3], gyroNotchFiltered_f32[3];
float eulerAngles_f32[3];

uint32_t sayac = 0;

#define RXDATASIZE 21
uint8_t RxData[RXDATASIZE];

#define TXDATASIZE 220
char TxData[TXDATASIZE];

#define CopyDATASIZE 17
uint8_t CopyData[CopyDATASIZE];

char buffer[64];

double yaw_base = 0;

uint64_t timer_u64 = 0;
uint64_t lastTime_u64 = 0;
uint32_t baslangic = 0;
uint32_t bitis = 0;
uint32_t bitis2 = 0;

float yaw, pitch, roll, depth, voltage;
float dt;
float pid_p, pid_i, pid_d;

int16_t MotorPWM[8] = {1500};
int16_t MotorValue[8] = {0};

uint8_t onsol8,onsag8,arkasol8,arkasag8,onsolbatirma8,onsagbatirma8,arkasolbatirma8,arkasagbatirma8; // 8 motor
uint8_t onsol6,onsag6,arkasol6,arkasag6,ortabatirmasol6,ortabatirmasag6; // 6 motor
uint8_t hizcarpan;
uint8_t huart_flag = 1;
uint8_t Komut = 0;
uint8_t konum[8] = {0};
uint8_t a[8] = {0};
uint8_t b[8] = {0};
uint8_t c[8] = {0};
uint8_t d[8] = {0};
uint8_t e[8] = {0};
uint8_t f[8] = {0};
uint8_t g[8] = {0};
uint8_t h[8] = {0};
uint8_t i[8] = {0};
uint8_t j[8] = {0};
bool otonomi = false;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1)
    {
    	huart_flag = 0;
    	UartRxDataUse();
    	HAL_UART_Receive_DMA(&huart1, RxData, RXDATASIZE);
    	huart_flag = 1;
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_USART1_UART_Init();
  MX_TIM11_Init();
  MX_TIM4_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
  //Init DWT Clock for proper us time tick
  DWT_Init();

  //Init filter with predefined settings
  LPFTwoPole_Init(&LPF_accel_x, LPF_TYPE_BESSEL, LPF_ACCEL_CTOFF_HZ, sample_time_sec_f32);
  LPFTwoPole_Init(&LPF_accel_y, LPF_TYPE_BESSEL, LPF_ACCEL_CTOFF_HZ, sample_time_sec_f32);
  LPFTwoPole_Init(&LPF_accel_z, LPF_TYPE_BESSEL, LPF_ACCEL_CTOFF_HZ, sample_time_sec_f32);

  LPFTwoPole_Init(&LPF_gyro_x, LPF_TYPE_BESSEL, LPF_GYRO_CTOFF_HZ, sample_time_sec_f32);
  LPFTwoPole_Init(&LPF_gyro_y, LPF_TYPE_BESSEL, LPF_GYRO_CTOFF_HZ, sample_time_sec_f32);
  LPFTwoPole_Init(&LPF_gyro_z, LPF_TYPE_BESSEL, LPF_GYRO_CTOFF_HZ, sample_time_sec_f32);

  NotchFilterInit(&NF_gyro_x, NF_GYRO_CFREQ_HZ, NF_GYRO_NWDTH_HZ, sample_time_sec_f32);
  NotchFilterInit(&NF_gyro_y, NF_GYRO_CFREQ_HZ, NF_GYRO_NWDTH_HZ, sample_time_sec_f32);
  NotchFilterInit(&NF_gyro_z, NF_GYRO_CFREQ_HZ, NF_GYRO_NWDTH_HZ, sample_time_sec_f32);

  //Init state estimators
  quaternionInit(&quaternion_t, sample_time_us_f32);

  //Init sensors
	while (MPU6050_Init(&hi2c2, &imu_t));

	if (imu_t.CALIBRATIN_OK_u8 == TRUE)
	{
	   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
	}
	uint8_t newData_u8;

	HAL_TIM_Base_Start(&htim11);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  __HAL_TIM_SET_COUNTER(&htim11, 0);
	  baslangic = __HAL_TIM_GET_COUNTER(&htim11);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		//Get system time in us
		timer_u64 = micros();

		if ( ((timer_u64 - lastTime_u64) >= sample_time_us_f32) && (imu_t.CALIBRATIN_OK_u8 == TRUE) )
		{
			lastTime_u64 = micros();

			//Read MPU6050 sensor data
			readMPU6050(&hi2c2, &imu_t);

			//Get accelerometer data in "g" and run LPF
			accelLowPassFiltered_f32[0] = (LPFTwoPole_Update(&LPF_accel_x, imu_t.MPU6050_Accel_f32[0]));
			accelLowPassFiltered_f32[1] = (LPFTwoPole_Update(&LPF_accel_y, imu_t.MPU6050_Accel_f32[1]));
			accelLowPassFiltered_f32[2] = (LPFTwoPole_Update(&LPF_accel_z, imu_t.MPU6050_Accel_f32[2]));

			//Get gyro data in "deg/s" and run LPF
			gyroLowPassFiltered_f32[0] = NotchFilter_Update(&NF_gyro_x, imu_t.MPU6050_Gyro_f32[0]);
			gyroLowPassFiltered_f32[1] = NotchFilter_Update(&NF_gyro_y, imu_t.MPU6050_Gyro_f32[1]);
			gyroLowPassFiltered_f32[2] = NotchFilter_Update(&NF_gyro_z, imu_t.MPU6050_Gyro_f32[2]);

			//Put gyro data into Notch Filter to flat-out any data in specific frequency band
			gyroNotchFiltered_f32[0] = (LPFTwoPole_Update(&LPF_gyro_x, gyroLowPassFiltered_f32[0]));
			gyroNotchFiltered_f32[1] = (LPFTwoPole_Update(&LPF_gyro_y, gyroLowPassFiltered_f32[1]));
			gyroNotchFiltered_f32[2] = (LPFTwoPole_Update(&LPF_gyro_z, gyroLowPassFiltered_f32[2]));

			//Get state estimations, using quaternion and fusion-quaternion based estimators
			quaternionUpdate(&quaternion_t, accelLowPassFiltered_f32[0], accelLowPassFiltered_f32[1], accelLowPassFiltered_f32[2],
					gyroNotchFiltered_f32[0]*(M_PI/180.0f), gyroNotchFiltered_f32[1]*(M_PI/180.0f),gyroNotchFiltered_f32[2]*(M_PI/180.0f));


			newData_u8 = TRUE; //Set newData to high for activate UART printer

		}//end of timer if
/*
		if(newData_u8 && sayac > 1000)
		{
			printf("%f, %f, %f\r\n",
					quaternion_t.yaw, quaternion_t.pitch, quaternion_t.roll);
			newData_u8 = FALSE;
		}
*/
		if(newData_u8 && sayac > 1000)
		{
			newData_u8 = FALSE;
			SendOrientationData((float)quaternion_t.yaw,(float)quaternion_t.pitch,(float)quaternion_t.roll);
			sayac = 0;
		}sayac++;

		bitis = Timer_GetElapsed(&htim11, baslangic);

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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 96-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 96-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 960-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 96-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void SendOrientationData(float yaw, float pitch, float roll)
{

    int len = snprintf(buffer, sizeof(buffer), "%.2f %.2f %.2f\n", yaw, pitch, roll);

    if (len > 0 && len < sizeof(buffer)) {
        CDC_Transmit_FS((uint8_t*)buffer, len);
    }
}

uint32_t Timer_GetElapsed(TIM_HandleTypeDef *htim, uint32_t timer_start) {
    uint32_t timer_end = __HAL_TIM_GET_COUNTER(htim);  // Timer pointer'ı ile sayaç değeri alınır

    if (timer_end >= timer_start) {
        return timer_end - timer_start;
    } else {
        return (0xFFFFFFFF - timer_start) + timer_end + 1;
    }
}

void UartRxDataUse()
{
	if (RxData[0] == '@' && RxData[RXDATASIZE-1] == '#' )
	{
		if (RxData[1] == 'K' && RxData[RXDATASIZE-2] == 'S' )
		{
			memcpy(CopyData, &RxData[2], 17);
			otonomi = true;
			Komut = CopyData[0];
			hizcarpan = CopyData[1] - 48;
			// BURADA OTONOM YAZILIM BASLATILACAKTIR. EKLENMELI
		}
		else if (RxData[1] == '!' && RxData[RXDATASIZE-2] == '!' )
		{
			for (uint8_t sayac = 0; sayac < 8; sayac++)
			{
				konum[sayac] = RxData[3 + sayac*2];
			}
			if (konum[6] == '0' || konum[7] == '0')
			{
				onsol6 = indeks(konum, 'a');
				onsag6 = indeks(konum, 'b');
				ortabatirmasol6 = indeks(konum, 'c');
				ortabatirmasag6 = indeks(konum, 'd');
				arkasol6 = indeks(konum, 'e');
				arkasag6 = indeks(konum, 'f');
			}
			else
			{
				onsolbatirma8 = indeks(konum, 'a');
				onsagbatirma8 = indeks(konum, 'b');
				onsol8 = indeks(konum, 'c');
				onsag8 = indeks(konum, 'd');
				arkasol8 = indeks(konum, 'e');
				arkasag8 = indeks(konum, 'f');
				arkasolbatirma8 = indeks(konum, 'g');
				arkasagbatirma8 = indeks(konum, 'h');
			}
		}
		else if (RxData[1] == '*' && RxData[RXDATASIZE-2] == '*' )
		{
			memcpy(CopyData, &RxData[2], 17);
			sscanf((char*)CopyData, "%f %f %f", &pid_p, &pid_i, &pid_d);
		}
		else if (RxData[1] == 'A' && RxData[RXDATASIZE-2] == 'A' )
		{
			for (uint8_t sayac = 0; sayac < 8; sayac++)
			{
				a[sayac] = RxData[3 + sayac*2];
			}
		}
		else if (RxData[1] == 'B' && RxData[RXDATASIZE-2] == 'B' )
		{
			for (uint8_t sayac = 0; sayac < 8; sayac++)
			{
				b[sayac] = RxData[3 + sayac*2];
			}
		}
		else if (RxData[1] == 'C' && RxData[RXDATASIZE-2] == 'C' )
		{
			for (uint8_t sayac = 0; sayac < 8; sayac++)
			{
				c[sayac] = RxData[3 + sayac*2];
			}
		}
		else if (RxData[1] == 'D' && RxData[RXDATASIZE-2] == 'D' )
		{
			for (uint8_t sayac = 0; sayac < 8; sayac++)
			{
				d[sayac] = RxData[3 + sayac*2];
			}
		}
		else if (RxData[1] == 'E' && RxData[RXDATASIZE-2] == 'E' )
		{
			for (uint8_t sayac = 0; sayac < 8; sayac++)
			{
				e[sayac] = RxData[3 + sayac*2];
			}
		}
		else if (RxData[1] == 'F' && RxData[RXDATASIZE-2] == 'F' )
		{
			for (uint8_t sayac = 0; sayac < 8; sayac++)
			{
				f[sayac] = RxData[3 + sayac*2];
			}
		}
		else if (RxData[1] == 'G' && RxData[RXDATASIZE-2] == 'G' )
		{
			for (uint8_t sayac = 0; sayac < 8; sayac++)
			{
				g[sayac] = RxData[3 + sayac*2];
			}
		}
		else if (RxData[1] == 'H' && RxData[RXDATASIZE-2] == 'H' )
		{
			for (uint8_t sayac = 0; sayac < 8; sayac++)
			{
				h[sayac] = RxData[3 + sayac*2];
			}
		}
		else if (RxData[1] == 'I' && RxData[RXDATASIZE-2] == 'I' )
		{
			for (uint8_t sayac = 0; sayac < 8; sayac++)
			{
				i[sayac] = RxData[3 + sayac*2];
			}
		}
		else if (RxData[1] == 'J' && RxData[RXDATASIZE-2] == 'J' )
		{
			for (uint8_t sayac = 0; sayac < 8; sayac++)
			{
				j[sayac] = RxData[3 + sayac*2];
			}
		}
	}
}

void UartTxWriter()
{
	snprintf(TxData, TXDATASIZE, "@_%d_%d_%d_%d_%d#",(int)(yaw * 100), (int)(pitch * 100), (int)(roll * 100), (int)(depth * 100), (int)(voltage * 100));
}

uint8_t indeks(uint8_t *dizi, char harf)
{
	for (uint8_t sayac = 0; sayac < sizeof(dizi); sayac++)
	{
		if (dizi[sayac] == harf)
		{
			return sayac;
		}
	}
	return 10;
}

void MotorValueUpload(uint8_t indis, uint8_t PWM)
{
	if (indis == 0){__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM);}
	if (indis == 1){__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM);}
	if (indis == 2){__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, PWM);}
	if (indis == 3){__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM);}
	if (indis == 4){__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWM);}
	if (indis == 5){__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, PWM);}
	if (indis == 6){__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, PWM);}
	if (indis == 7){__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, PWM);}
}

int16_t MotorValueDownload(uint8_t indis)
{
	if (indis == 0){ return (int16_t)__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);}
	if (indis == 1){ return (int16_t)__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);}
	if (indis == 2){ return (int16_t)__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_3);}
	if (indis == 3){ return (int16_t)__HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_4);}
	if (indis == 4){ return (int16_t)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);}
	if (indis == 5){ return (int16_t)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2);}
	if (indis == 6){ return (int16_t)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_3);}
	if (indis == 7){ return (int16_t)__HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_4);}
}

int8_t faz_indeks(uint8_t *dizi, uint8_t sira)
{
	if (dizi[sira] == '0')
	{
		return 0;
	}
	else if (dizi[sira] == '1')
	{
		return -1;
	}
	else if (dizi[sira] == '2')
	{
		return 1;
	}
	return 0;
}

void Komut_A()
{

}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM9 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM9)
  {
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
