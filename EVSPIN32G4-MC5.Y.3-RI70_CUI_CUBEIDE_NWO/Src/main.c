/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum FSM_State {
	FSM_START = 0, /**< @brief Starting uC.*/
	FSM_INIT = 1,
	FSM_READY_TO_OPERATE = 2,
	FSM_OPERATION_ENABLE = 3, /**< @brief Enable power.*/

	FSM_CALIBRATION_PHASE_0 = 100,
	FSM_CALIBRATION_PHASE_1 = 101,
	FSM_CALIBRATION_PHASE_2 = 102,
	FSM_CALIBRATION_PHASE_3 = 103,
	FSM_CALIBRATION_PHASE_4 = 104,
	FSM_CALIBRATION_PHASE_5 = 105,
	FSM_CALIBRATION_PHASE_6 = 106,

	FSM_TRANSITION_START_TO_INIT = 10,
	FSM_TRANSITION_INIT_TO_READY_TO_OPERATE = 11,
	FSM_TRANSITION_READY_TO_OPERATE_TO_OPERATION_ENABLE = 12,
	FSM_TRANSITION_OPERATION_ENABLE_TO_READY_TO_OPERATE = 13,
	FSM_TRANSITION_FAULT_REACTION_ACTIVE_TO_FAULT = 14,
	FSM_TRANSITION_FAULT_TO_INIT = 15,

	FSM_TRANSITION_INIT_TO_CALIBRATION_PHASE_0 = 110,
	FSM_TRANSITION_CALIBRATION_PHASE_0_TO_CALIBRATION_PHASE_1 = 111,
	FSM_TRANSITION_CALIBRATION_PHASE_1_TO_CALIBRATION_PHASE_2 = 112,
	FSM_TRANSITION_CALIBRATION_PHASE_2_TO_CALIBRATION_PHASE_3 = 113,
	FSM_TRANSITION_CALIBRATION_PHASE_3_TO_CALIBRATION_PHASE_4 = 114,
	FSM_TRANSITION_CALIBRATION_PHASE_4_TO_CALIBRATION_PHASE_5 = 115,
	FSM_TRANSITION_CALIBRATION_PHASE_5_TO_CALIBRATION_PHASE_6 = 116,
	FSM_TRANSITION_CALIBRATION_PHASE_6_TO_INIT = 117,

	FSM_FAULT_REACTION_ACTIVE = 254,
	FSM_FAULT = 255
} FSM_State_t;

typedef enum Working_Mode {
	WORKING_MODE_NOT_SELECTED = 0, /**< @brief Starting uC.*/
	TORQUE_MODE,
	SPEED_MODE,
	POSITION_MODE

} Working_Mode_t;

typedef enum Motor {
	MOTOR_TYPE_NOT_SELECTED = 0, /**< @brief Starting uC.*/
	RI70,
	RI80

} Motor_t;

typedef struct {
	uint32_t main;
	uint32_t timer6;
	uint32_t timer7;
	uint32_t can_rx_counter;
	uint32_t can_tx_counter;
} __attribute__ ((packed)) Counters_Handle_t;

typedef struct {
	uint8_t can_node_id;
	uint8_t gear_ratio;
	Motor_t motor_type;
	uint16_t encoder_resolution;

	bool ma730_working;
	bool canbus_working;
	bool canbus_watchdog_working;
} __attribute__ ((packed)) Joint_Configuration_Handle_t;

typedef struct {
	Working_Mode_t working_mode;
	float joint_torque;				// Nm
	float motor_torque;				// Nm
	float joint_speed;				// rads
	float motor_speed;				// rads
	float joint_position;			// rad
	float motor_position;			// rad
	int16_t _motor_torque;			//
	int16_t _motor_speed;			//
	int32_t _motor_position;		//
} __attribute__ ((packed)) App_Command_Handle_t;

typedef struct {
	float f_current_motor_position; 	// rad
	float f_current_joint_position; 	// rad
	float f_current_motor_speed; 		// rad/s
	float f_current_joint_speed; 		// rad/s
	float f_current_motor_torque; 		// Nm
	float f_current_joint_torque; 		// Nm
	float f_current_temperature;		// C
	float f_current_voltage;				// V

	int16_t mc_current_motor_speed;
	int32_t mc_current_motor_rotation;
	int32_t mc_current_motor_position_multiturn;
	int32_t mc_current_motor_position;
	int32_t mc_previous_motor_position;
	int32_t mc_current_electric_rotation;
	int16_t mc_current_electric_position;
	int16_t mc_previous_electric_position;

	uint8_t errors;
	uint8_t warnings;

//	PosCtrlStatus_t mc_position_control_status;
//	AlignStatus_t mc_encoder_align_status;
	State_t stm_state_motor;
	uint16_t mc_current_faults_motor;
	uint16_t mc_occured_faults_motor;

	int16_t _current_torque_data[CURRENT_TORQUE_DATA_SIZE];
	uint8_t _current_torque_index;
} __attribute__ ((packed)) Joint_Status_Handle_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

COMP_HandleTypeDef hcomp1;
COMP_HandleTypeDef hcomp2;
COMP_HandleTypeDef hcomp4;

CORDIC_HandleTypeDef hcordic;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c3;

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
FDCAN_RxHeaderTypeDef can_rx_header; // CAN Bus Transmit Header
FDCAN_TxHeaderTypeDef can_tx_header; // CAN Bus Transmit Header
uint8_t can_rx_data[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  //CAN Bus Receive Buffer
uint8_t can_tx_data[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  //CAN Bus Send Buffer

Counters_Handle_t volatile g_counters =
{
	.main = 0,
	.timer6 = 0,
	.timer7 = 0,
};

App_Command_Handle_t g_app_command =
{
//	.working_mode = TORQUE_MODE,
	.motor_torque = 0.0,
};

Joint_Configuration_Handle_t g_joint_configuration =
{
	.can_node_id = 0x01,
	.gear_ratio = 121
};

Joint_Status_Handle_t g_joint_status =
{
	.mc_current_motor_position = 0
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_COMP1_Init(void);
static void MX_COMP2_Init(void);
static void MX_COMP4_Init(void);
static void MX_CORDIC_Init(void);
static void MX_I2C3_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_COMP1_Init();
  MX_COMP2_Init();
  MX_COMP4_Init();
  MX_CORDIC_Init();
  MX_I2C3_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_MotorControl_Init();
  MX_FDCAN1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

	// FDCAN

	HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 10, 0);
	HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);

//
	// CAN FILTERS
	FDCAN_FilterTypeDef   can_filter_config_0; // BROADCAST
	can_filter_config_0.IdType = FDCAN_STANDARD_ID;
	can_filter_config_0.FilterIndex = 0;
	can_filter_config_0.FilterType = FDCAN_FILTER_MASK;
	can_filter_config_0.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	can_filter_config_0.FilterID1 = 0x0F0;
	can_filter_config_0.FilterID2 = 0x70F ;
	HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter_config_0); //Initialize CAN Filter

	FDCAN_FilterTypeDef   can_filter_config_1; // UNICAST
	can_filter_config_1.IdType = FDCAN_STANDARD_ID;
	can_filter_config_1.FilterIndex = 1;
	can_filter_config_1.FilterType = FDCAN_FILTER_MASK;
	can_filter_config_1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	can_filter_config_1.FilterID1 = 0x100 + g_joint_configuration.can_node_id;
	can_filter_config_1.FilterID2 = 0x70F;
	HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter_config_1); //Initialize CAN Filter

	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, 3, 3, FDCAN_FILTER_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_Start(&hfdcan1); //Initialize CAN Bus
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);// Initialize CAN Bus Rx Interrupt
	HAL_FDCAN_EnableISOMode(&hfdcan1);

	// TIMERS
	HAL_TIM_Base_Start_IT(&htim6); 		// Enable 10 kHz timer for fast calculation
	HAL_TIM_Base_Start_IT(&htim7);  	// Enable  1 kHz timer for FSM

//	PID_PosParamsM1.hKpGain =  5000;
//	PID_PosParamsM1.hKiGain = 8000;
//	PID_PosParamsM1.hKdGain = 600;
//
//	MC_StartMotor1();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  g_counters.main++;

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
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
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C3
                              |RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM1_BRK_TIM15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 4, 1);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
  /* TIM1_UP_TIM16_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  /* ADC1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 3, 1);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp1.Init.InputMinus = COMP_INPUT_MINUS_1_4VREFINT;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}

/**
  * @brief COMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP2_Init(void)
{

  /* USER CODE BEGIN COMP2_Init 0 */

  /* USER CODE END COMP2_Init 0 */

  /* USER CODE BEGIN COMP2_Init 1 */

  /* USER CODE END COMP2_Init 1 */
  hcomp2.Instance = COMP2;
  hcomp2.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp2.Init.InputMinus = COMP_INPUT_MINUS_1_4VREFINT;
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp2.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP2_Init 2 */

  /* USER CODE END COMP2_Init 2 */

}

/**
  * @brief COMP4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP4_Init(void)
{

  /* USER CODE BEGIN COMP4_Init 0 */

  /* USER CODE END COMP4_Init 0 */

  /* USER CODE BEGIN COMP4_Init 1 */

  /* USER CODE END COMP4_Init 1 */
  hcomp4.Instance = COMP4;
  hcomp4.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp4.Init.InputMinus = COMP_INPUT_MINUS_1_4VREFINT;
  hcomp4.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp4.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp4.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp4.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP4_Init 2 */

  /* USER CODE END COMP4_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */

  /* USER CODE END CORDIC_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV2;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 5;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 12;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 2;
  hfdcan1.Init.DataTimeSeg1 = 12;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

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
  hi2c3.Init.Timing = 0x00802172;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /** I2C Fast mode Plus enable
  */
  __HAL_SYSCFG_FASTMODEPLUS_ENABLE(I2C_FASTMODEPLUS_I2C3);
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief OPAMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP1_Init(void)
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp1.Init.Mode = OPAMP_STANDALONE_MODE;
  hopamp1.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp1.Init.InternalOutput = DISABLE;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP1_Init 2 */

  /* USER CODE END OPAMP1_Init 2 */

}

/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void)
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp2.Init.Mode = OPAMP_STANDALONE_MODE;
  hopamp2.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO1;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp2.Init.InternalOutput = DISABLE;
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP2_Init 2 */

  /* USER CODE END OPAMP2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = ((TIM_CLOCK_DIVIDER) - 1);
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = ((PWM_PERIOD_CYCLES) / 2);
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim1.Init.RepetitionCounter = (REP_COUNTER);
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP1;
  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_HIGH;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK2, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP2;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK2, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP4;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK2, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
  sBreakDeadTimeConfig.DeadTime = ((DEAD_TIME_COUNTS) / 2);
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_ENABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 5;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = M1_PULSE_NBR;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = M1_ENC_IC_FILTER;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = M1_ENC_IC_FILTER;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 16999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 9;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 16999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart1.Init.BaudRate = 1843200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GD_WAKE_GPIO_Port, GD_WAKE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : GD_WAKE_Pin */
  GPIO_InitStruct.Pin = GD_WAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GD_WAKE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GD_READY_Pin GD_NFAULT_Pin */
  GPIO_InitStruct.Pin = GD_READY_Pin|GD_NFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Read_MC_Encoder() // odczyt danych enkdera i sprawdzanie obrotow
{
	// CALCULATE CHANGE ELECTRIC ROTATION
	g_joint_status.mc_previous_electric_position = g_joint_status.mc_current_electric_position; // store old position
	g_joint_status.mc_current_electric_position = ENCODER_M1._Super.hElAngle;

	if (g_joint_status.mc_previous_electric_position > 28000 && g_joint_status.mc_current_electric_position < - 28000)
	{
		g_joint_status.mc_current_electric_rotation++;
	}
	if (g_joint_status.mc_current_electric_position > 28000  && g_joint_status.mc_previous_electric_position < - 28000)
	{
		g_joint_status.mc_current_electric_rotation--;
	}

	// READ POSITION
	g_joint_status.mc_previous_motor_position = g_joint_status.mc_current_motor_position; // store old position
	g_joint_status.mc_current_motor_position = ENCODER_M1.PreviousCapture; // 0 ... 14336 - 1 mechanical motor rotation

	if (g_joint_status.mc_previous_motor_position > ENCODER_M1.PulseNumber - ENCODER_M1.PulseNumber / 4 && g_joint_status.mc_current_motor_position  < ENCODER_M1.PulseNumber / 4)
	{
		g_joint_status.mc_current_motor_rotation++;
	}
	if (g_joint_status.mc_current_motor_position  > ENCODER_M1.PulseNumber - ENCODER_M1.PulseNumber / 4 && g_joint_status.mc_previous_motor_position < ENCODER_M1.PulseNumber / 4)
	{
		g_joint_status.mc_current_motor_rotation--;
	}

	g_joint_status.mc_current_motor_position_multiturn = g_joint_status.mc_current_motor_position + g_joint_status.mc_current_motor_rotation * ENCODER_M1.PulseNumber;

	// READ SPEED
	g_joint_status.mc_current_motor_speed = ENCODER_M1._Super.hAvrMecSpeedUnit;
}

void Read_MC_Torque() // odczyt surowych danych torque
{
	// READ TORQUE
	g_joint_status._current_torque_data[g_joint_status._current_torque_index++] = MC_GetPhaseCurrentAmplitudeMotor1();
	g_joint_status._current_torque_index %= CURRENT_TORQUE_DATA_SIZE;
}

void Read_MC_State()
{
	g_joint_status.stm_state_motor 				= MC_GetSTMStateMotor1();
	g_joint_status.mc_current_faults_motor 		= MC_GetCurrentFaultsMotor1();
	g_joint_status.mc_occured_faults_motor 		= MC_GetOccurredFaultsMotor1();
//	g_joint_status.mc_encoder_align_status  	= MC_GetAlignmentStatusMotor1();
//	g_joint_status.mc_position_control_status 	= MC_GetControlPositionStatusMotor1();
}

void FSM_Action()
{
}

// CAN
// Odbior paczki
//

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6) 	// 10kHz (5,0) - fast recalculation
	{
		g_counters.timer6++;
		Read_MC_Encoder();
		Read_MC_Torque();
		Read_MC_State();
	}

	if(htim->Instance == TIM7) 	// 1kHz - FSM_Tasks
	{
		g_counters.timer7++;
//		FSM_Action();
		switch (g_app_command.working_mode)
		{
			case TORQUE_MODE:
			{
				MC_ProgramTorqueRampMotor1(g_app_command._motor_torque, 0);
				MC_StartMotor1();
				break;
			}

			case SPEED_MODE:
			{
				MC_ProgramSpeedRampMotor1(g_app_command._motor_speed, 0);
				MC_StartMotor1();
				break;
			}

//			case POSITION_MODE:
//			{
//				MC_ProgramPositionCommandMotor1(g_app_command.motor_position, 0.1);
////				MC_ProgramSpeedRampMotor1(g_app_command._motor_position, 0);
////				MC_StartMotor1();
////				MC_ProgramPositionCommandMotor1(g_app_command.joint_position, 0.1);
////				MC_StartMotor1();
////				HAL_Delay(2000);
//				break;
//			}

			default:
			{
				MC_StopMotor1();
			}
		}
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan1, uint32_t RxFifo0ITs)
{

	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
	{
		// RETRIEVE CAN MESSAGE -------------------------------------------------------------------------
		HAL_FDCAN_GetRxMessage(hfdcan1, FDCAN_RX_FIFO0, &can_rx_header, can_rx_data);
		g_counters.can_rx_counter++;

		// RESET SAFE TIMER -----------------------------------------------------------------------------
//		HAL_TIM_Base_Stop(&htim6);
//		TIM6->CNT = 0;
//		HAL_TIM_Base_Start_IT(&htim6);

		bool l_send_response = true;
		uint8_t l_cmd 	= (can_rx_header.Identifier & 0x0F0) >> 4; // Ustalenie polecenie
		bool l_unicast 	= (can_rx_header.Identifier & 0x100) >> 8; // Ustalenie typu transmisji: BROADCAST|UNICAST

		// CAN TRANSMITION CONFIGURATION
		can_tx_header.Identifier = can_rx_header.Identifier | g_joint_configuration.can_node_id | 0x01 << 9;
		can_tx_header.IdType = FDCAN_STANDARD_ID;
		can_tx_header.TxFrameType = FDCAN_DATA_FRAME;
		can_tx_header.DataLength = FDCAN_DLC_BYTES_12;
		can_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
		can_tx_header.BitRateSwitch = FDCAN_BRS_ON;
		can_tx_header.FDFormat = FDCAN_FD_CAN;
		can_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
		can_tx_header.MessageMarker = 0;

		// UPDATE JOINT INFO ----------------------------------------------------------------------------
		uint8_t numer_w_szeregu = 0;
		uint8_t dlugosc_danych_polecenia = 0;

		if (!l_unicast)
		{
			uint16_t numer_w_szeregu = g_joint_configuration.can_node_id; // ???
		}

		switch (l_cmd) {

			case 0x0: // Wykonaj akcję
			{
				dlugosc_danych_polecenia = 2;
				// int16_t w zaleznosci od trybu pracy - torque, speed
				uint8_t offset = dlugosc_danych_polecenia * numer_w_szeregu;

				int16_t goal;
				goal  = can_rx_data[offset] << 8;
				goal += can_rx_data[offset + 1];

				// GOAL TORQUE
				// -------------------------------------------------------------------------------------------------
				// recalculate torque goal data from CAN to floats
				g_app_command.joint_torque = ((float) goal * 256.0) / (float) INT16_MAX; // convert from int16 to float
				g_app_command.motor_torque = g_app_command.joint_torque / g_joint_configuration.gear_ratio;

				// recalculate torque goal data from floats to MC
				g_app_command._motor_torque	= (((float) g_app_command.motor_torque) * INT16_MAX) / (33.0 * KT);  // convert from float to s16A


				// -------------------------------------------------------------------------------------------------
				// Recalculate data from MC to Floats
				// POSITION
				g_joint_status.f_current_motor_position = (double) g_joint_status.mc_current_motor_position_multiturn / ENCODER_M1.PulseNumber;
				g_joint_status.f_current_joint_position = (double) g_joint_status.f_current_motor_position / g_joint_configuration.gear_ratio;

				// SPEED
				g_joint_status.f_current_motor_speed = g_joint_status.mc_current_motor_speed * 6 * M_TWOPI / 60;
				g_joint_status.f_current_joint_speed = g_joint_status.f_current_motor_speed / g_joint_configuration.gear_ratio;

				// TORQUE
				float temp_torque = 0;
				for (int i = 0; i < CURRENT_TORQUE_DATA_SIZE; i++)
				{
					temp_torque += (float) g_joint_status._current_torque_data[i];
				}

				g_joint_status.f_current_motor_torque			= (((float) (temp_torque / CURRENT_TORQUE_DATA_SIZE)) / INT16_MAX) * 33.0 * KT;
				g_joint_status.f_current_joint_torque			= g_joint_status.f_current_motor_torque * g_joint_configuration.gear_ratio;

				// -------------------------------------------------------------------------------------------------
				// Recalculate data from floats to CAN
				int16_t l_joint_position_in_s16degree = (int16_t) (g_joint_status.f_current_joint_position * (UINT16_MAX / M_TWOPI));

				int16_t l_speed_in_dpp = g_joint_status.f_current_joint_speed * (float) INT16_MAX / M_TWOPI;

				int16_t l_current_torque_in_s16a = (g_joint_status.f_current_joint_torque / UINT8_MAX) * (float) INT16_MAX;
				if (g_app_command.motor_torque < 0)
				{
					l_current_torque_in_s16a = -l_current_torque_in_s16a;
				}

				// -------------------------------------------------------------------------------------------------
				can_tx_data[0] 	= l_joint_position_in_s16degree >> 8;
				can_tx_data[1] 	= l_joint_position_in_s16degree;
				can_tx_data[2] 	= l_speed_in_dpp >> 8;
				can_tx_data[3] 	= l_speed_in_dpp;
				can_tx_data[4] 	= l_current_torque_in_s16a >> 8;
				can_tx_data[5] 	= l_current_torque_in_s16a;
				can_tx_header.DataLength = FDCAN_DLC_BYTES_6;

				break;
			}

			case 0x1: // Zmien stan FSM
			{
				dlugosc_danych_polecenia = 1;
				// uint8_t - FSM
				uint8_t offset = dlugosc_danych_polecenia * numer_w_szeregu;
				can_tx_header.DataLength = FDCAN_DLC_BYTES_0;
				break;
			}

			case 0xF: // Konfiguracja
			{
				dlugosc_danych_polecenia = 5;
				// uint8_t typ_silnika - 1=RI70, 2=RI80
				// uint8_t przelozenie przekladni
				// uint8_t tryb pracy
				// uint16_t enkoder resolution
				uint8_t offset = dlugosc_danych_polecenia * numer_w_szeregu;
				g_joint_configuration.motor_type	= can_rx_data[offset];
				g_joint_configuration.gear_ratio 	= can_rx_data[offset + 1];
				g_app_command.working_mode 			= can_rx_data[offset + 2];
				uint16_t l_encoder_resolution		= can_rx_data[offset + 3] << 8;
				l_encoder_resolution 			   += can_rx_data[offset + 4];
//				htim4.Init.Period = l_encoder_resolution / 4;
//				ENCODER_M1.PulseNumber = l_encoder_resolution;
				can_tx_header.DataLength = FDCAN_DLC_BYTES_0;
				break;
			}

			default:
			{
				l_send_response = false;

			}

		}

		//	SEND FRAME VIA FD CAN -------------------------------------------------------------------------------
		if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan1, &can_tx_header, can_tx_data) == HAL_OK && l_send_response)
		{
			g_counters.can_tx_counter++;
		}

		// clear global rx and tx buffer
		for (int i = 0; i < 24; i++)
		{
			can_rx_data[i]	= 0;
			can_tx_data[i]	= 0;
		}
	}
}

int32_t MCM_Sqrt( int32_t wInput )
{
	int32_t wtemprootnew;

	if ( wInput > 0 )
	{
		  uint8_t biter = 0u;
		  int32_t wtemproot;

		    if ( wInput <= ( int32_t )2097152 )
		    {
		      wtemproot = ( int32_t )128;
		    }
		    else
		    {
		      wtemproot = ( int32_t )8192;
		    }

		    do
		    {
		      wtemprootnew = ( wtemproot + wInput / wtemproot ) / ( int32_t )2;
		      if ( wtemprootnew == wtemproot )
		      {
		        biter = 6u;
		      }
		      else
		      {
		        biter ++;
		        wtemproot = wtemprootnew;
		      }
		    }
		    while ( biter < 6u );

	}
	else
	{
		wtemprootnew = ( int32_t )0;
	}

	return ( wtemprootnew );
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
