/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// ERRORRS
#define  JOINT_NO_ERROR  						(uint8_t)(0x00u)      /**< @brief No error.*/
#define  JOINT_POSITION_ENCODER_FAILED  		(uint8_t)(0x01u)
#define  JOINT_MC_FAILED  						(uint8_t)(0x02u)
#define  JOINT_JOINT_SPEED_TO_HIGH  			(uint8_t)(0x04u)
#define  JOINT_NOT_CALIBRATED_YET				(uint8_t)(0x08u)

// WARNINGS
#define  JOINT_NO_WARNING  						(uint8_t)(0x00u)      /**< @brief No error.*/
#define  JOINT_POSITION_NOT_ACCURATE			(uint8_t)(0x01u)
#define  JOINT_OUTSIDE_WORKING_AREA				(uint8_t)(0x02u)
#define  JOINT_MA730_NOT_PROPER_MAGNETOC_FIELD	(uint8_t)(0x02u)

#define CURRENT_TORQUE_DATA_SIZE (uint8_t) 50

#define BUFF_SIZE 32

//#if defined CALIBRATION
typedef enum Calibration_State {
	CALIBRATION_NOT_FINISHED = 0, /**< @brief Starting uC.*/
	CALIBRATION_TABLE_NOT_FILLED = 1, // No difference between hi and low value of the sector
	SECTORS_NARROW = 2, // No difference between hi and low value of the sector
	SECTORS_INTERCECTION = 3, // intersection of the sectors > 0
	SECTORS_NOT_SEPARETED = 3, // intersection of the sectors > 0
	CALIBRATION_TABLE_CONTAINS_ZEROES = 4,
	MISSED_CENTER_POSITION = 100,
	FLASH_DOESNT_WRITE_PROPERLY = 100,
	CALIBRATION_OK = 255
} Calibration_State_t;

typedef enum FSM_State {
	FSM_START = 0, /**< @brief Starting uC.*/
	FSM_INIT = 1,
	FSM_READY_TO_OPERATE = 2,
	FSM_OPERATION_ENABLE = 3, /**< @brief Enable power.*/

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

	FSM_TRANSITION_INIT_TO_CALIBRATION_PHASE_1 = 110,
	FSM_TRANSITION_CALIBRATION_PHASE_1_TO_CALIBRATION_PHASE_2 = 111,
	FSM_TRANSITION_CALIBRATION_PHASE_2_TO_CALIBRATION_PHASE_3 = 112,
	FSM_TRANSITION_CALIBRATION_PHASE_3_TO_CALIBRATION_PHASE_4 = 113,
	FSM_TRANSITION_CALIBRATION_PHASE_4_TO_CALIBRATION_PHASE_5 = 114,
	FSM_TRANSITION_CALIBRATION_PHASE_5_TO_CALIBRATION_PHASE_6 = 115,
	FSM_TRANSITION_CALIBRATION_PHASE_6_TO_INIT = 111,

	FSM_FAULT_REACTION_ACTIVE = 254,
	FSM_FAULT = 255
} FSM_State_t;

//#endif

typedef enum Joint_Position_State {
	POSITION_UNDER_WORKING_AREA = -1,
	POSITION_IN_WORKING_AREA = 0, /**< @brief Starting uC.*/
	POSITION_OVER_WORKING_AREA = 1
} Joint_Position_State_t;

typedef enum Encoder_Position_State {
	POSITION_UNKNOWN = 0, /**< @brief Starting uC.*/
	POSITION_APROXIMATED = 1,
	POSITION_ACCURATE = 2,
	POSITION_ESTIMATION_FAILED = 255
} Encoder_Position_State_t;

typedef enum Motor_Mode {
	TORQUE = 0, /**< @brief Starting uC.*/
	SPEED = 1
} Motor_Mode_t;

typedef enum Motor_State {
	MOTOR_STOPPED = 0, /**< @brief Starting uC.*/
	MOTOR_STARTED_IN_TORQUE_MODE = 1,
	MOTOR_STARTED_IN_SPEED_MODE = 2
} Motor_State_t;

typedef enum Heartbeat {
	HEARTBEAT_DISABLED = 0, /**< @brief Starting uC.*/
	HEARTBEAT_ENABLED = 1
} Heartbeat_t;

typedef enum Security {
	SECURITY_DISABLED = 0, /**< @brief Starting uC.*/
	SECURITY_ENABLED = 1
} Security_t;

typedef enum Node_Type {
	UNKNOWN = 0, /**< @brief Starting uC.*/
	JOINT_SMALL = 1, JOINT_MEDIUM = 2, JOINT_BIG = 3, GRIPPER_ONE = 4, GRIPPER_TWO = 5
} Node_Type_t;

typedef enum Joint_Calibratation_State {
	JOINT_NOT_CALIBRATED = 0, /**< @brief Starting uC.*/
	JOINT_CALIBRATED = 1
} Joint_Calibratation_State_t;

typedef struct {
	FSM_State_t state;
	bool state_is_running;
	bool transition_is_running;
} __attribute__ ((packed)) FSMStatus_t;

typedef struct {
	Node_Type_t node_type;
	uint8_t can_node_id;
	Heartbeat_t hartbeat;
	Security_t security;
} __attribute__ ((packed)) NodeStatus_t;

typedef struct {
	bool dip1;
	bool dip2;
	bool dip3;
	bool dip4;
	bool termistor_exists;
	bool ma730_exists;
	bool working_area_constrain; /**< @brief settings to enable/disable constrain */
	Joint_Calibratation_State_t calibration_state;
	uint16_t gear_ratio;
	uint16_t pole_pairs;
	uint16_t maximum_electrical_rotations;
	uint16_t reachable_electrical_rotations;
	uint16_t number_of_sectors;
	int16_t  zero_electric_position;
	uint16_t zero_electric_rotation;
	double joint_working_area_in_rad; // +- hall encoder ticks of two direction torque
	uint16_t calibration_sector_size; // in electrical rotations
	uint16_t calibration_table_size;
	uint16_t calibration_table_1[1680];
	uint16_t calibration_table_2[1680];
} __attribute__ ((packed)) JointConfiguration_t;

typedef struct {
	int16_t goal;
//	int16_t last_goal;
	double  goal_motor_torque_in_nm;
	double  goal_joint_torque_in_nm;
	Motor_Mode_t mode;
	Motor_State_t state;
} __attribute__ ((packed)) MotorCommand_t;

typedef struct {
//	uint16_t read_buffer;
	uint16_t angle;
	uint8_t z0;
	uint8_t z1;
	uint8_t bct;
	uint8_t ety;
	uint8_t etx;
	uint8_t ppt0;
	uint8_t ppt1;
	uint8_t ilip;
	uint8_t mglt;
	uint8_t mght;
	uint8_t rd;
	uint8_t mgh;
	uint8_t mgl;
} __attribute__ ((packed)) MA730_t;

typedef struct {
	Joint_Position_State_t current_joint_position;
	Encoder_Position_State_t encoder_position_state;

	double current_encoder_position_offset_in_rad;

	int32_t current_encoder_position;
	double current_encoder_position_in_rad;

	int16_t current_encoder_speed;
	double current_encoder_speed_in_rads;

	int32_t current_mechanical_rotation;
	int32_t current_mechanical_position;


	int32_t previous_mechanical_position;

	int32_t current_electric_rotation;

	int16_t current_electric_position;
	int16_t previous_electric_position;
	double current_joint_position_in_rad;

	double  current_joint_speed_in_rads;

	int16_t current_motor_torque;
	double  current_motor_torque_in_nm;
	double  current_joint_torque_in_nm;

	uint8_t current_temperature;
	double  current_voltage;

	uint16_t current_ma730_value;

	bool     ma730_is_running;

	uint8_t errors;
	uint8_t warnings;

	State_t stm_state_motor;
	uint16_t mc_current_faults_motor;
	uint16_t mc_occured_faults_motor;

	int16_t _current_torque_data[CURRENT_TORQUE_DATA_SIZE];
	uint8_t _current_torque_index;
} __attribute__ ((packed)) MotorStatus_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Calibrations parameters -----------------------------
#define SECTOR_SIZE 					(uint16_t) 5
#define POLE_PAIRS 						(uint16_t) 14
#define GEAR_RATIO 						(uint16_t) 121
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define NOP asm("NOP")
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CORDIC_HandleTypeDef hcordic;

FDCAN_HandleTypeDef hfdcan1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// COUNTERS
volatile uint16_t g_can_rx_counter 	= 0;
volatile uint16_t g_can_tx_counter 	= 0;
volatile uint32_t g_counter_10000hz = 0;
volatile uint32_t g_counter_1000hz 	= 0;
volatile uint32_t g_counter_100hz 	= 0;
volatile uint32_t g_counter_10hz 	= 0;
volatile uint32_t g_counter_1hz 	= 0;
volatile uint32_t g_counter_IT 		= 0;
volatile uint32_t g_counter_main	= 0;
//volatile uint16_t g_timer_counter 	= 0;

uint8_t buff[2];

int16_t g_current_sector_number = -1;
int16_t g_current_estimated_electric_rotation = -1;

volatile NodeStatus_t 	g_node_status 		=
{
	.can_node_id = 0,
};

volatile MotorCommand_t 	g_motor_command 	=
{
	.goal = 0,
//	.last_goal = 0,
};

volatile JointConfiguration_t 	g_joint_configuration 	=
{
	.calibration_state = JOINT_NOT_CALIBRATED,
	.working_area_constrain = true,
	.termistor_exists = true,
	.ma730_exists = true,
};

volatile MotorStatus_t 	g_motor_status =
{
	.encoder_position_state = POSITION_UNKNOWN,
	.current_encoder_position_offset_in_rad = 0.0,
	.current_electric_position = 0,
	.previous_electric_position = 0,
	.current_electric_rotation = 0,
	.warnings = 0,
	.errors = 0,
	.current_ma730_value = 0,
//	.previous_ma730_value = 0,
	.ma730_is_running = false,
};

volatile MA730_t 	g_ma730 =
{
};

volatile FSMStatus_t 	g_fsm_status =
{
	.state = FSM_START,
	.state_is_running = false,
	.transition_is_running = false,
};

// FDCAN
FDCAN_FilterTypeDef   g_can_filter_config; //CAN Bus Filter
FDCAN_RxHeaderTypeDef g_can_rx_header; // CAN Bus Transmit Header
FDCAN_TxHeaderTypeDef g_can_tx_header; // CAN Bus Receive Header
uint8_t g_can_rx_data[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  //CAN Bus Receive Buffer
uint8_t g_can_tx_data[12] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xA0, 0xB0, 0xC0};
HAL_StatusTypeDef g_can_tx_status;

// FLASH
uint32_t g_flash_address_configuration 		= 0x08018000; // FIXME zmienic na 0x08018000
uint32_t g_flash_address_calibration_table 	= 0x08018100;
uint16_t g_calibration_config[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t g_data[2] = {0, 0};

//#if defined CALIBRATION
// Calibration variables
volatile Calibration_State_t g_calibration_state = CALIBRATION_NOT_FINISHED;
volatile int32_t g_min_encoder_position = 0;
volatile int32_t g_max_encoder_position = 0;
volatile int32_t g_center_encoder_position = 0;
volatile int16_t g_max_electric_rotation_cw  = 0;
volatile int16_t g_max_electric_rotation_ccw = 0;
volatile uint16_t g_calibration_data_1_errors = 0;
volatile uint16_t g_calibration_data_2_errors = 0;
volatile uint16_t g_calibration_speed = 10; // speed of configuration rotations
volatile uint16_t g_calibration_torque = 800;
//#else
//volatile uint16_t g_current_sector_number;
volatile int16_t g_previous_sector_number = -1;
//#endif

//// ONLY FOR TESTING
//int16_t g_goal_torque;
//int16_t g_current_torque;
//uint16_t g_mc_current_faults_motor;
//uint16_t g_mc_occured_faults_motor;
//State_t g_stm_state_motor;

double g_max_voltage = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_CORDIC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM6_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void Flash_Read_Data (uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords);
int16_t get_sector_number_from_electric_rotation_number(int16_t electric_rotation);
int16_t get_sector_number_from_ma730(int16_t ma730_value);
//void motor_start_in_speed_mode(int16_t speed);
//void motor_start_in_torque_mode(int16_t torque);
void motor_start(Motor_Mode_t mode, int16_t goal);
void motor_stop();
bool motor_reach_torque_limit();
bool motor_in_position(volatile int32_t position);
//#if defined CALIBRATION
bool check_calibration_data_cw(int16_t size);
bool check_calibration_data_ccw(int16_t size);
//#else
//#endif
FSM_State_t FSM_Get_State(void);
bool FSM_Set_State(FSM_State_t new_state);
bool FSM_Activate_State(FSM_State_t new_state);
bool FSM_Activate_Transition(FSM_State_t new_transition);
void MA730_ReadRegister(uint8_t reg_number);
void MA730_ReadAngle();
void MA730_WriteRegister(uint8_t reg_number, uint8_t reg_value);
//uint8_t parse_cmd(void);

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CORDIC_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_MotorControl_Init();
  MX_FDCAN1_Init();
  MX_SPI2_Init();
  MX_TIM6_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
//#if defined CALIBRATION
  g_joint_configuration.calibration_sector_size = SECTOR_SIZE;
  g_joint_configuration.pole_pairs = POLE_PAIRS;
  g_joint_configuration.gear_ratio = GEAR_RATIO;
  g_joint_configuration.maximum_electrical_rotations = g_joint_configuration.gear_ratio * g_joint_configuration.pole_pairs;
  g_joint_configuration.calibration_table_size = (uint16_t) (g_joint_configuration.maximum_electrical_rotations / g_joint_configuration.calibration_sector_size);
//#else
//#endif

	// Reset MA730 register to defaults
	MA730_WriteRegister(0, 0b00000000);
	MA730_WriteRegister(1, 0b00000000);
	MA730_WriteRegister(2, 0b00000000);
	MA730_WriteRegister(3, 0b00000000);
	MA730_WriteRegister(4, 0b11000000);
	MA730_WriteRegister(5, 0b11111111);
	MA730_WriteRegister(6, 0b00011100);
	MA730_WriteRegister(9, 0b00000000);

	MA730_ReadRegister(0);
	MA730_ReadRegister(1);
	MA730_ReadRegister(2);
	MA730_ReadRegister(3);
	MA730_ReadRegister(4);
	MA730_ReadRegister(5);
	MA730_ReadRegister(6);
	MA730_ReadRegister(9);

	HAL_TIM_Base_Start_IT(&htim6); // Enable 10 kHz timer // Stop to check MA730

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	MA730_ReadRegister(0x1b);
	g_counter_main++;
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
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 80;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 3, 1);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
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

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
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
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 2;
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
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_14;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
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
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 2;
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
  sConfigInjected.InjectedChannel = ADC_CHANNEL_14;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 2;
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
  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
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
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hfdcan1.Init.NominalSyncJumpWidth = 3;
  hfdcan1.Init.NominalTimeSeg1 = 12;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 3;
  hfdcan1.Init.DataTimeSeg1 = 12;
  hfdcan1.Init.DataTimeSeg2 = 3;
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
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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
  htim6.Init.Period = 15999;
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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MA730_CS_Pin|SEC_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIP2_Pin DIP1_Pin SEC_IN_Pin */
  GPIO_InitStruct.Pin = DIP2_Pin|DIP1_Pin|SEC_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MA730_CS_Pin SEC_OUT_Pin */
  GPIO_InitStruct.Pin = MA730_CS_Pin|SEC_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DIP4_Pin DIP3_Pin */
  GPIO_InitStruct.Pin = DIP4_Pin|DIP3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	return;
	static uint16_t g_timer_counter = 0;

	if(htim->Instance == TIM6) // 10 000 Hz
	{

		// 10 000 Hz
		{
			g_counter_10000hz++;

			// CALCULATE CHANGE ELECTRIC ROTATION
			g_motor_status.previous_electric_position = g_motor_status.current_electric_position; // store old position
			g_motor_status.current_electric_position = ENCODER_M1._Super.hElAngle;

			if (g_motor_status.previous_electric_position > 28000 && g_motor_status.current_electric_position < - 28000)
			{
				g_motor_status.current_electric_rotation++;
			}
			if (g_motor_status.current_electric_position > 28000  && g_motor_status.previous_electric_position < - 28000)
			{
				g_motor_status.current_electric_rotation--;
			}

			// READ POSITION
			g_motor_status.previous_mechanical_position = g_motor_status.current_mechanical_position; // store old position
			g_motor_status.current_mechanical_position = ENCODER_M1.PreviousCapture; // 0 ... 14336 - 1 mechanical motor rotation

			if (g_motor_status.previous_mechanical_position > ENCODER_M1.PulseNumber - 4000 && g_motor_status.current_mechanical_position  < 4000)
			{
				g_motor_status.current_mechanical_rotation++;
			}
			if (g_motor_status.current_mechanical_position  > ENCODER_M1.PulseNumber - 4000 && g_motor_status.previous_mechanical_position < 4000)
			{
				g_motor_status.current_mechanical_rotation--;
			}

			// READ SPEED
			g_motor_status.current_encoder_speed = ENCODER_M1._Super.hAvrMecSpeedUnit;
			g_motor_status.current_encoder_speed_in_rads = g_motor_status.current_encoder_speed * 6 * M_TWOPI / 60;

//			g_motor_status.current_joint_speed_in_rads = g_motor_status.current_encoder_speed / g_joint_configuration.gear_ratio;
			g_motor_status.current_joint_speed_in_rads = g_motor_status.current_encoder_speed_in_rads / 120.0;

			// READ TORQUE
			g_motor_status._current_torque_data[g_motor_status._current_torque_index++] = MC_GetPhaseCurrentAmplitudeMotor1();
			g_motor_status._current_torque_index %= CURRENT_TORQUE_DATA_SIZE;

			// ERRORS:
			// 1. SPEED  (pi/2 rads)
//			if (g_motor_status.current_joint_speed_in_rads > M_PI_2 || g_motor_status.current_joint_speed_in_rads < -1 * M_PI_2)
//			{
//				// STOP MOTOR - SPEED IS TO HIGH
//				g_motor_status.errors = g_motor_status.errors | JOINT_JOINT_SPEED_TO_HIGH;
//				FSM_Activate_State(FSM_FAULT_REACTION_ACTIVE);
//			}
			// 2. TORQUE (256 Nm)
			// 3. TEMP

			// WARNINGS:
			// WORKING AREA
			if (g_motor_status.current_joint_position_in_rad < -1 * g_joint_configuration.joint_working_area_in_rad && g_joint_configuration.working_area_constrain)
			{
				g_motor_status.current_joint_position = POSITION_UNDER_WORKING_AREA;
				g_motor_status.warnings = g_motor_status.warnings | JOINT_OUTSIDE_WORKING_AREA;
			}
			else if(g_motor_status.current_joint_position_in_rad > g_joint_configuration.joint_working_area_in_rad && g_joint_configuration.working_area_constrain)
			{
				g_motor_status.current_joint_position = POSITION_OVER_WORKING_AREA;
				g_motor_status.warnings = g_motor_status.warnings | JOINT_OUTSIDE_WORKING_AREA;
			}
			else
			{
				g_motor_status.current_joint_position = POSITION_IN_WORKING_AREA;
				g_motor_status.warnings = g_motor_status.warnings & (0xFF ^ JOINT_OUTSIDE_WORKING_AREA);
			}

			// ENCODER_NOT_ACCURATE
			if (g_motor_status.encoder_position_state != POSITION_ACCURATE)
			{
				g_motor_status.warnings = g_motor_status.warnings | JOINT_POSITION_NOT_ACCURATE;

			}
			else
			{
				g_motor_status.warnings = g_motor_status.warnings & (0xFF ^ JOINT_POSITION_NOT_ACCURATE);
			}

		}

		// 1000 Hz
		if (g_timer_counter % 10 == 0)
		{
			g_counter_1000hz++;

			g_motor_status.current_encoder_position = g_motor_status.current_mechanical_rotation * ENCODER_M1.PulseNumber + g_motor_status.current_mechanical_position;

			g_motor_status.current_encoder_position_in_rad = M_TWOPI * (double) g_motor_status.current_encoder_position / ENCODER_M1.PulseNumber;
			g_motor_status.current_joint_position_in_rad = g_motor_status.current_encoder_position_in_rad / g_joint_configuration.gear_ratio + g_motor_status.current_encoder_position_offset_in_rad;

			// Get states
//			g_motor_status.current_voltage 			= (double) RealBusVoltageSensorParamsM1.aBuffer[RealBusVoltageSensorParamsM1.index] / 1225; // FIXME sprawdzic jak to jest interpretowane
			g_motor_status.current_voltage 			= (double) RealBusVoltageSensorParamsM1.aBuffer[RealBusVoltageSensorParamsM1.index] / 1225; // FIXME sprawdzic jak to jest interpretowane
			if (g_max_voltage < g_motor_status.current_voltage) {
				g_max_voltage = g_motor_status.current_voltage;
			}
			g_motor_status.stm_state_motor 			= MC_GetSTMStateMotor1();
			g_motor_status.mc_current_faults_motor 	= MC_GetCurrentFaultsMotor1();
			g_motor_status.mc_occured_faults_motor 	= MC_GetOccurredFaultsMotor1();
			g_motor_status.current_temperature 		= (uint8_t) NTC_GetAvTemp_C(&TempSensorParamsM1);

			g_motor_command.goal_motor_torque_in_nm = g_motor_command.goal_joint_torque_in_nm / g_joint_configuration.gear_ratio;
			g_motor_command.goal					= (((double) g_motor_command.goal_motor_torque_in_nm) * 32768.0) / (33.0 * 0.1118);

			// CURRENT TORQUE
			double temp_torque = 0;
			for (int i = 0; i < CURRENT_TORQUE_DATA_SIZE; i++)
			{
				temp_torque += (double) g_motor_status._current_torque_data[i];
			}
			g_motor_status.current_motor_torque 		= (int16_t) (temp_torque / CURRENT_TORQUE_DATA_SIZE);
			g_motor_status.current_motor_torque_in_nm	= (((double) g_motor_status.current_motor_torque) / 32768.0) * 33.0 * 0.1118;
			g_motor_status.current_joint_torque_in_nm	= g_motor_status.current_motor_torque_in_nm * g_joint_configuration.gear_ratio;

			// READ MA730 Position
			if (g_joint_configuration.ma730_exists)
			{
				if (g_counter_1hz > 1) // wait 1 sec to analyse data
				{
					g_motor_status.ma730_is_running = true;
					MA730_ReadAngle();
					g_motor_status.current_ma730_value = g_ma730.angle;
				}
			}

			if (g_motor_status.ma730_is_running == true && g_joint_configuration.calibration_state == JOINT_CALIBRATED)
			{
				// JOINT POSITION ESTIMATION
				switch (g_motor_status.encoder_position_state)
				{
					case POSITION_ESTIMATION_FAILED:
					{
						// RAISE ERROR
						break;
					}

					case POSITION_ACCURATE:
					{
						break;
					}

					case POSITION_APROXIMATED:
					{
						g_current_sector_number = get_sector_number_from_ma730(g_motor_status.current_ma730_value);
						// If motor running and sector is change update
						if (g_current_sector_number != g_previous_sector_number && g_previous_sector_number != -1) // pass 1 time, to load previous and current
						{
							double electric_rotation_width = M_TWOPI / (g_joint_configuration.pole_pairs * g_joint_configuration.gear_ratio);
							uint16_t l_current_electric_rotation;

							if (g_current_sector_number - g_previous_sector_number > 0)
							{ // rotation in CCW direction => "+"
								l_current_electric_rotation = (g_current_sector_number) * g_joint_configuration.calibration_sector_size;
							}
							else
							{ // rotation in CW direction ==> "-"
								l_current_electric_rotation = (g_current_sector_number + 1) * g_joint_configuration.calibration_sector_size - 1;
							}
							int32_t l_diff_electric_position = g_joint_configuration.zero_electric_position - g_motor_status.current_electric_position;

							g_motor_status.current_encoder_position_offset_in_rad = -1 * ((g_joint_configuration.zero_electric_rotation - l_current_electric_rotation) + ( (double) l_diff_electric_position / 65536) - 1.0) * electric_rotation_width;

							g_motor_status.encoder_position_state = POSITION_ACCURATE;
						}

						g_previous_sector_number = g_current_sector_number;
						break;
					}
//
					case POSITION_UNKNOWN:
					{
						g_current_sector_number = get_sector_number_from_ma730(g_motor_status.current_ma730_value);
						if (g_current_sector_number != -1)
						{
							double electric_rotation_width = M_TWOPI / (g_joint_configuration.pole_pairs * g_joint_configuration.gear_ratio);

			//				g_zero_sector_number = get_sector_number_from_electric_rotation_number(g_joint_configuration.zero_electric_rotation);
							g_current_estimated_electric_rotation = g_current_sector_number *  g_joint_configuration.calibration_sector_size; // center current sector

							g_motor_status.current_encoder_position_offset_in_rad = -1 * (g_joint_configuration.zero_electric_rotation - g_current_estimated_electric_rotation) * electric_rotation_width;

							// calculate encoder positioon offset
							g_motor_status.encoder_position_state = POSITION_APROXIMATED;
						}
						break;
					}
				}

			}

			// FSM
			switch (FSM_Get_State()) {

//				case FSM_START:
//				{
//					g_center_encoder_position = -1;
//					g_fsm_status.state = FSM_CALIBRATION_PHASE_1;
//					break;
//				}

				case FSM_CALIBRATION_PHASE_1:
				{

					if (motor_reach_torque_limit()) // REACH MINIMUM EDGE
					{
						motor_stop();
						g_motor_status.current_electric_rotation = 0; // zeroing electric rotation counter
						g_min_encoder_position = g_motor_status.current_encoder_position; // encoder value

						g_fsm_status.state = FSM_CALIBRATION_PHASE_2;

					}
					else
					{
						motor_start(SPEED, -1 * g_calibration_speed);
					}

					break;
				}

				case FSM_CALIBRATION_PHASE_2:
				{
					if (g_motor_status.current_electric_rotation % g_joint_configuration.calibration_sector_size == (0) &&
							g_joint_configuration.calibration_table_1[g_motor_status.current_electric_rotation / g_joint_configuration.calibration_sector_size] == 0)
					{
						g_joint_configuration.calibration_table_1[g_motor_status.current_electric_rotation / g_joint_configuration.calibration_sector_size] = g_motor_status.current_ma730_value;
					}

					if (motor_reach_torque_limit()) // REACH MAXIMUM EDGE
					{
						motor_stop();
						g_joint_configuration.calibration_table_2[g_motor_status.current_electric_rotation / g_joint_configuration.calibration_sector_size] = g_motor_status.current_ma730_value;
						g_max_electric_rotation_cw 	= g_motor_status.current_electric_rotation; // max electric rotation counter

						g_max_encoder_position = g_motor_status.current_encoder_position; // max encoder value
						g_fsm_status.state = FSM_CALIBRATION_PHASE_3;

					}
					else
					{
						motor_start(SPEED, g_calibration_speed);
					}

					break;
				}

				case FSM_CALIBRATION_PHASE_3:
				{
					if (g_motor_status.current_electric_rotation % g_joint_configuration.calibration_sector_size == (g_joint_configuration.calibration_sector_size - 1) &&
							g_joint_configuration.calibration_table_2[g_motor_status.current_electric_rotation / g_joint_configuration.calibration_sector_size] == 0)
					{
						g_joint_configuration.calibration_table_2[g_motor_status.current_electric_rotation / g_joint_configuration.calibration_sector_size] = g_motor_status.current_ma730_value;
					}

					if (motor_reach_torque_limit()) // REACH MINIMUM EDGE
					{
						motor_stop();
						g_joint_configuration.calibration_table_1[g_motor_status.current_electric_rotation / g_joint_configuration.calibration_sector_size] = g_motor_status.current_ma730_value;
		  //				g_fsm_current_state = CALIBRATION_FINISHED_WITH_SUCCESS;
						g_fsm_status.state = FSM_CALIBRATION_PHASE_4;

					}
					else
					{
						motor_start(SPEED, -1 * g_calibration_speed);
					}

					break;
				}

				case FSM_CALIBRATION_PHASE_4:
				{
					// Sprawdzenie tablicy kalibracyjnej
					if (!check_calibration_data_cw(g_max_electric_rotation_cw / g_joint_configuration.calibration_sector_size) ||
						!check_calibration_data_ccw(g_max_electric_rotation_cw / g_joint_configuration.calibration_sector_size))
					{
						g_fsm_status.state = FSM_CALIBRATION_PHASE_2;
		  //				g_fsm_current_state = CALIBRATION_PHASE_5;
						g_calibration_state = CALIBRATION_TABLE_CONTAINS_ZEROES;
		  //				g_error_1++;
					}
					else
					{
						g_max_electric_rotation_ccw = g_motor_status.current_electric_rotation; // Should be 0 right now
		  //								g_center_hall_encoder_position = (g_max_hall_encoder_position + 1) / 2;
						g_center_encoder_position = (g_max_encoder_position - g_min_encoder_position + 1) / 2 + g_min_encoder_position;

						g_joint_configuration.number_of_sectors = (uint16_t) (g_max_electric_rotation_cw / g_joint_configuration.calibration_sector_size);
						g_joint_configuration.reachable_electrical_rotations = g_max_electric_rotation_cw;
						g_calibration_state = CALIBRATION_OK;
						g_fsm_status.state = FSM_CALIBRATION_PHASE_5;
					}
					break;
				}

				case FSM_CALIBRATION_PHASE_5:
				{
					if (motor_in_position(g_center_encoder_position))
					{
						g_fsm_status.state = FSM_CALIBRATION_PHASE_6;
						motor_start(SPEED, 0);
						motor_stop();

						g_joint_configuration.zero_electric_rotation = g_motor_status.current_electric_rotation;
						g_joint_configuration.zero_electric_position = (uint16_t) (g_motor_status.current_electric_position);
						g_joint_configuration.zero_electric_position = (int16_t) (g_motor_status.current_electric_position);
					}
					else
					{
						motor_start(SPEED, g_calibration_speed);
					}

					if (motor_reach_torque_limit()) // REACH EDGE - FAILURE
					{
						motor_start(SPEED, 0);
						motor_stop();
						g_calibration_state = MISSED_CENTER_POSITION;
//						g_fsm_status.state = FSM_STOPPED_WITH_ERRORS;
						FSM_Activate_State(FSM_FAULT_REACTION_ACTIVE);

					}

					break;
				}
				case FSM_CALIBRATION_PHASE_6:
				{
					volatile int d;
					volatile uint32_t error;

					HAL_TIM_Base_Stop_IT(&htim6); // Disable 10 kHz timer

					uint64_t data;

					HAL_FLASH_Unlock();

					FLASH_EraseInitTypeDef EraseInitStruct;
					EraseInitStruct.TypeErase 	= FLASH_TYPEERASE_PAGES;
					EraseInitStruct.Banks 		= FLASH_BANK_1;
					EraseInitStruct.Page 		= (g_flash_address_configuration & 0x07FFFFFF) / FLASH_PAGE_SIZE;
					EraseInitStruct.NbPages 	= 2; // 1 - 2kB
					uint32_t PageError;

					if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
					{
						error = HAL_FLASH_GetError ();
					}

					// Configuration
//					flash_data.d16[0] = g_joint_configuration.pole_pairs;
//					flash_data.d16[1] = g_joint_configuration.gear_ratio;
					data = (g_joint_configuration.gear_ratio << 16) | g_joint_configuration.pole_pairs;
					if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, g_flash_address_configuration, data) != HAL_OK)
//					if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, g_flash_address_configuration, flash_data.d64) != HAL_OK)
					{
						error = HAL_FLASH_GetError ();
					}

					data = (g_joint_configuration.reachable_electrical_rotations << 16) | g_joint_configuration.calibration_sector_size;
//					flash_data.d16[0] = g_joint_configuration.calibration_sector_size;
//					flash_data.d16[1] = g_joint_configuration.reachable_electrical_rotations;
					if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, g_flash_address_configuration + 8, data) != HAL_OK)
					{
						error = HAL_FLASH_GetError ();
					}

					data = (g_joint_configuration.calibration_sector_size << 16) | g_joint_configuration.number_of_sectors;
					if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, g_flash_address_configuration + 16, data) != HAL_OK)
					{
						error = HAL_FLASH_GetError ();
					}

					uint16_t * temp = (uint16_t *) &g_joint_configuration.zero_electric_position;
					data = (g_joint_configuration.zero_electric_rotation << 16) | (* temp) ;
					if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, g_flash_address_configuration + 24, data) != HAL_OK)
					{
						error = HAL_FLASH_GetError ();
					}

					for(int i = 0; i < g_joint_configuration.number_of_sectors; i++ )
					{
						data = g_joint_configuration.calibration_table_1[i] << 16 | g_joint_configuration.calibration_table_2[i];
						if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, g_flash_address_calibration_table + i * 8, data) != HAL_OK)
						{
							error = HAL_FLASH_GetError ();
						}
					}

					HAL_FLASH_Lock();

					FSM_Activate_State(FSM_INIT); // CALIBRATION FINISHED - GO TO INIT STATE

					HAL_TIM_Base_Start_IT(&htim6); // Enable 10 kHz timer
					break;
				}

				case FSM_START:
				{
					HAL_TIM_Base_Stop_IT(&htim6); // Enable 10 kHz timer

					// Configure CAN ID
					const uint8_t _can_start_address = 10;

//					uint8_t _dip1 = (HAL_GPIO_ReadPin(GPIOB, DIP1_Pin) == GPIO_PIN_RESET) ? (0) : (1);
//					uint8_t _dip2 = (HAL_GPIO_ReadPin(GPIOB, DIP2_Pin) == GPIO_PIN_RESET) ? (0) : (4);
//					uint8_t _dip3 = (HAL_GPIO_ReadPin(GPIOB, DIP3_Pin) == GPIO_PIN_RESET) ? (0) : (2);
//					uint8_t _dip4 = (HAL_GPIO_ReadPin(GPIOB, DIP4_Pin) == GPIO_PIN_RESET) ? (0) : (1);
					g_joint_configuration.dip1 = (HAL_GPIO_ReadPin(GPIOC, DIP1_Pin) == GPIO_PIN_RESET) ? (0) : (1);
					g_joint_configuration.dip2 = (HAL_GPIO_ReadPin(GPIOC, DIP2_Pin) == GPIO_PIN_RESET) ? (0) : (1);
					g_joint_configuration.dip3 = (HAL_GPIO_ReadPin(GPIOB, DIP3_Pin) == GPIO_PIN_RESET) ? (0) : (1);
					g_joint_configuration.dip4 = (HAL_GPIO_ReadPin(GPIOB, DIP4_Pin) == GPIO_PIN_RESET) ? (0) : (1);

//					g_node_status.can_node_id = _dip2 + _dip3 + _dip4;
					g_node_status.can_node_id = g_joint_configuration.dip2 << 2 + g_joint_configuration.dip3 << 1 + g_joint_configuration.dip4;

					// disable dip4 allow multiturn
					g_joint_configuration.working_area_constrain = g_joint_configuration.dip1;
	//	  			g_joint_configuration.ma730_exists = _dip1;

					// CAN FILTER
					g_can_filter_config.IdType = FDCAN_STANDARD_ID;
					g_can_filter_config.FilterIndex = 0;
					g_can_filter_config.FilterType = FDCAN_FILTER_DUAL;
					g_can_filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
					g_can_filter_config.FilterID1 = g_node_status.can_node_id + _can_start_address;
					g_can_filter_config.FilterID2 = 0x0AA ;

					// CAN TRANSMITION CONFIGURATION
					g_can_tx_header.Identifier = (g_node_status.can_node_id + _can_start_address) * 16;
					g_can_tx_header.IdType = FDCAN_STANDARD_ID;
					g_can_tx_header.TxFrameType = FDCAN_DATA_FRAME;
					g_can_tx_header.DataLength = FDCAN_DLC_BYTES_12;
					g_can_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
					g_can_tx_header.BitRateSwitch = FDCAN_BRS_ON;
					g_can_tx_header.FDFormat = FDCAN_FD_CAN;
					g_can_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
					g_can_tx_header.MessageMarker = 0;

					HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 10, 0);
					HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);

					HAL_FDCAN_ConfigFilter(&hfdcan1, &g_can_filter_config); //Initialize CAN Filter
					HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, 3, 3, FDCAN_FILTER_REMOTE, FDCAN_REJECT_REMOTE);
					HAL_FDCAN_Start(&hfdcan1); //Initialize CAN Bus
					HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);// Initialize CAN Bus Rx Interrupt
					HAL_FDCAN_EnableISOMode(&hfdcan1);

					// Read configuration from FLASH
					Flash_Read_Data (g_flash_address_configuration, (uint32_t *) g_calibration_config, 8);

					g_joint_configuration.calibration_table_size 			= g_calibration_config[8]; // FIXME !!!!

					if (g_joint_configuration.calibration_table_size > 0 && g_joint_configuration.calibration_table_size < 65535) {
						// Read joint configuration from FLASH
						g_joint_configuration.pole_pairs 						= g_calibration_config[0];
						g_joint_configuration.gear_ratio 						= g_calibration_config[1];
						g_joint_configuration.calibration_sector_size 			= g_calibration_config[4];
						g_joint_configuration.reachable_electrical_rotations	= g_calibration_config[5];
						g_joint_configuration.number_of_sectors 				= g_calibration_config[8];
						g_joint_configuration.zero_electric_position 			= (uint8_t) g_calibration_config[12];
						g_joint_configuration.zero_electric_rotation 			= g_calibration_config[13];

						// Read calibration table from FLASH
						//	for (int i = 0; i <= g_joint_configuration.calibration_table_size / 2 + 1; i++) {
						for (int i = 0; i <= g_joint_configuration.calibration_table_size; i++) {
							Flash_Read_Data (g_flash_address_calibration_table + i * 8, (uint32_t *) g_data, 1);
							g_joint_configuration.calibration_table_1[i]	= g_data[0] >> 16;
							g_joint_configuration.calibration_table_2[i] 	= g_data[0];
						}

						g_joint_configuration.maximum_electrical_rotations = g_joint_configuration.gear_ratio * g_joint_configuration.pole_pairs;


						g_joint_configuration.calibration_state = JOINT_CALIBRATED;

					}
					//	g_joint_configuration.hall_working_area = (6 * g_joint_configuration.pole_pairs * g_joint_configuration.gear_ratio) * 165.0 / 360.0;
					g_joint_configuration.joint_working_area_in_rad = M_PI * 165.0 / 180.0;
					// --- DLA TOMKA (BRAK KALIBRACJI WE FLASH)
					//	g_joint_configuration.pole_pairs = 14;
					//	g_joint_configuration.gear_ratio = 120;
					//	g_joint_configuration.calibration_state = JOINT_CALIBRATED;
					// --- END DLA TOMKA ---------------------

					g_node_status.security = SECURITY_DISABLED;
					g_node_status.hartbeat = HEARTBEAT_DISABLED;
					g_node_status.node_type = JOINT_MEDIUM;

					FSM_Activate_Transition(FSM_TRANSITION_START_TO_INIT);

					HAL_TIM_Base_Start_IT(&htim6); // Enable 10 kHz timer

					break;
				}

				case FSM_TRANSITION_START_TO_INIT:
				{
					FSM_Activate_State(FSM_INIT);
					break;
				}

				case FSM_INIT:
				{
//					FSM_Activate_Transition(FSM_TRANSITION_INIT_TO_READY_TO_OPERATE);
					break;
				}

				case FSM_TRANSITION_INIT_TO_READY_TO_OPERATE:
				{
					// Check the calibration
					// If joint is not calibrated go to failure
					FSM_Activate_State(FSM_READY_TO_OPERATE);
					break;
				}

				case FSM_READY_TO_OPERATE:
				{
					motor_stop();
					break;
				}

				case FSM_TRANSITION_READY_TO_OPERATE_TO_OPERATION_ENABLE:
				{
					FSM_Activate_State(FSM_OPERATION_ENABLE);
					break;
				}

				case FSM_TRANSITION_OPERATION_ENABLE_TO_READY_TO_OPERATE:
				{
					FSM_Activate_State(FSM_READY_TO_OPERATE);
					// Stop the motor
					motor_stop();
					break;
				}

				case FSM_OPERATION_ENABLE:
				{
					g_motor_command.mode = TORQUE;
					if (g_joint_configuration.working_area_constrain)
					{
						switch (g_motor_status.current_joint_position)
						{
							case POSITION_UNDER_WORKING_AREA: // Accept only positive torque
								if (g_motor_command.goal < 0)
								{
									motor_stop();
								}
								else
								{
									motor_start(g_motor_command.mode, g_motor_command.goal);
								}
								break;

							case POSITION_IN_WORKING_AREA:
								motor_start(g_motor_command.mode, g_motor_command.goal);
								break;

							case POSITION_OVER_WORKING_AREA: // Accept only negative torque
								if (g_motor_command.goal > 0)
								{
									motor_stop();
								}
								else
								{
									motor_start(g_motor_command.mode, g_motor_command.goal);
								}
								break;
						}

					}
					else
					{
						motor_start(g_motor_command.mode, g_motor_command.goal);
					}

					break;

				}

				case FSM_TRANSITION_FAULT_TO_INIT:
				{
					MC_AcknowledgeFaultMotor1();
					g_motor_status.mc_occured_faults_motor = 0;
					FSM_Activate_State(FSM_INIT);
					break;
				}

				case FSM_TRANSITION_FAULT_REACTION_ACTIVE_TO_FAULT:
				{
					motor_stop();
					FSM_Activate_State(FSM_FAULT);
					break;
				}

				case FSM_FAULT_REACTION_ACTIVE:
				{
					FSM_Activate_Transition(FSM_TRANSITION_FAULT_REACTION_ACTIVE_TO_FAULT);
					break;
				}

				case FSM_FAULT:
				{
					break;
				}

				default:
				{
					break;
				}
			}


			// Reaction on error
			if (g_motor_status.mc_current_faults_motor > 0 || g_motor_status.mc_occured_faults_motor > 0) {
				g_motor_status.errors =  g_motor_status.errors | JOINT_MC_FAILED;
				if (FSM_Get_State() != FSM_FAULT)
				{
					FSM_Set_State(FSM_FAULT_REACTION_ACTIVE);
				}
			}
			else
			{
				g_motor_status.errors = g_motor_status.errors & (0xFF ^ JOINT_MC_FAILED);
			}

		} // end 1000 Hz

		// 100 Hz
		if (g_timer_counter % 100 == 0)
		{
			g_counter_100hz++;
		} // end 100 Hz

		// 10 Hz
		if (g_timer_counter % 1000 == 0)
		{
			g_counter_10hz++;

		} // end 10 Hz

		// 1 Hz
		if (g_timer_counter % 10000 == 0)
		{
//			// SEND HEARTBEAT BY CANBUS
//			if (g_can_heartbeat == CAN_HEARTBEAT_ENABLED)
//			{
//				const uint8_t _can_start_address = 10;
//
//				FDCAN_TxHeaderTypeDef can_tx_header; // CAN Bus Receive Header
//				uint8_t can_tx_data[8]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//
//				can_tx_header.Identifier = (g_node_status.can_node_id + _can_start_address);
//				can_tx_header.IdType = FDCAN_STANDARD_ID;
//				can_tx_header.TxFrameType = FDCAN_DATA_FRAME;
//				can_tx_header.DataLength = FDCAN_DLC_BYTES_6;
//				can_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//				can_tx_header.BitRateSwitch = FDCAN_BRS_ON;
//				can_tx_header.FDFormat = FDCAN_FD_CAN;
//				can_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//				can_tx_header.MessageMarker = 0;
//
//				if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header, can_tx_data) == HAL_OK)
//				{
//					g_can_tx_counter++;
//				}
//
//			}
			g_counter_1hz++;
		} // end 1 Hz

		g_timer_counter++;

		// reset counter
		if (g_timer_counter >= 10000) {
			g_timer_counter = 0;
		}
	}
}
// Current FDBK
void R3_2_GetPhaseCurrents( PWMC_Handle_t * pHdl, ab_t * Iab )
{
#if defined (__ICCARM__)
  #pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_2_Handle_t * pHandle = ( PWMC_R3_2_Handle_t * )pHdl;
#if defined (__ICCARM__)
  #pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  uint8_t Sector;
  int32_t Aux;
  uint32_t ADCDataReg1;
  uint32_t ADCDataReg2;

  Sector = ( uint8_t )pHandle->_Super.Sector;
  ADCDataReg1 = *pHandle->pParams_str->ADCDataReg1[Sector];
  ADCDataReg2 = *pHandle->pParams_str->ADCDataReg2[Sector];

  /* disable ADC trigger source */
  //LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH4);
  LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);

  switch ( Sector )
  {
    case SECTOR_4:
    case SECTOR_5:
      /* Current on Phase C is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
//      Aux = ( int32_t )( pHandle->PhaseAOffset ) - ( int32_t )( ADCDataReg1 );
		Aux = (int32_t) (ADCDataReg1) - (int32_t) (pHandle->PhaseAOffset);

      /* Saturation of Ia */
      if ( Aux < -INT16_MAX )
      {
        Iab->a = -INT16_MAX;
      }
      else  if ( Aux > INT16_MAX )
      {
        Iab->a = INT16_MAX;
      }
      else
      {
        Iab->a = ( int16_t )Aux;
      }

      /* Ib = PhaseBOffset - ADC converted value) */
//      Aux = ( int32_t )( pHandle->PhaseBOffset ) - ( int32_t )( ADCDataReg2 );
      Aux = ( int32_t )( ADCDataReg2 ) - ( int32_t )( pHandle->PhaseBOffset );

      /* Saturation of Ib */
      if ( Aux < -INT16_MAX )
      {
        Iab->b = -INT16_MAX;
      }
      else  if ( Aux > INT16_MAX )
      {
        Iab->b = INT16_MAX;
      }
      else
      {
        Iab->b = ( int16_t )Aux;
      }
      break;

    case SECTOR_6:
    case SECTOR_1:
      /* Current on Phase A is not accessible     */
      /* Ib = PhaseBOffset - ADC converted value) */
//      Aux = ( int32_t )( pHandle->PhaseBOffset ) - ( int32_t )( ADCDataReg1 );
    	Aux = ( int32_t )( ADCDataReg1 ) - ( int32_t )( pHandle->PhaseBOffset );

      /* Saturation of Ib */
      if ( Aux < -INT16_MAX )
      {
        Iab->b = -INT16_MAX;
      }
      else  if ( Aux > INT16_MAX )
      {
        Iab->b = INT16_MAX;
      }
      else
      {
        Iab->b = ( int16_t )Aux;
      }

      /* Ia = -Ic -Ib */
//      Aux = ( int32_t )( ADCDataReg2 ) - ( int32_t )( pHandle->PhaseCOffset ); /* -Ic */
      Aux = ( int32_t )( pHandle->PhaseCOffset ) - ( int32_t )( ADCDataReg2 ); /* Ic reversed */
      Aux -= ( int32_t )Iab->b;             /* Ia  */

      /* Saturation of Ia */
      if ( Aux > INT16_MAX )
      {
        Iab->a = INT16_MAX;
      }
      else  if ( Aux < -INT16_MAX )
      {
        Iab->a = -INT16_MAX;
      }
      else
      {
        Iab->a = ( int16_t )Aux;
      }
      break;

    case SECTOR_2:
    case SECTOR_3:
      /* Current on Phase B is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
//      Aux = ( int32_t )( pHandle->PhaseAOffset ) - ( int32_t )( ADCDataReg1 );
    	Aux = ( int32_t )( ADCDataReg1 ) - ( int32_t )( pHandle->PhaseAOffset );

      /* Saturation of Ia */
      if ( Aux < -INT16_MAX )
      {
        Iab->a = -INT16_MAX;
      }
      else  if ( Aux > INT16_MAX )
      {
        Iab->a = INT16_MAX;
      }
      else
      {
        Iab->a = ( int16_t )Aux;
      }

      /* Ib = -Ic -Ia */
//      Aux = ( int32_t )( ADCDataReg2 ) - ( int32_t )( pHandle->PhaseCOffset ); /* -Ic */
      Aux = ( int32_t )( pHandle->PhaseCOffset ) - ( int32_t )( ADCDataReg2 ); /* -Ic */
      Aux -= ( int32_t )Iab->a;             /* Ib */

      /* Saturation of Ib */
      if ( Aux > INT16_MAX )
      {
        Iab->b = INT16_MAX;
      }
      else  if ( Aux < -INT16_MAX )
      {
        Iab->b = -INT16_MAX;
      }
      else
      {
        Iab->b = ( int16_t )Aux;
      }
      break;

    default:
      break;
  }

  pHandle->_Super.Ia = Iab->a;
  pHandle->_Super.Ib = Iab->b;
  pHandle->_Super.Ic = -Iab->a - Iab->b;
}

///**
//  * @brief  It set instantaneous rotor mechanical angle.
//  *         As a consequence, timer counted is computed and updated.
//  * @param  pHandle: handler of the current instance of the encoder component
//  * @param  hMecAngle new value of rotor mechanical angle (s16degrees)
//  * @retval none
//  */
//void ENC_SetMecAngle( ENCODER_Handle_t * pHandle, int16_t hMecAngle )
//{
//  TIM_TypeDef * TIMx = pHandle->TIMx;
//
//  uint16_t hAngleCounts;
//  uint16_t hMecAngleuint;
//
//  pHandle->_Super.hMecAngle = hMecAngle;
//  pHandle->_Super.hElAngle = hMecAngle * pHandle->_Super.bElToMecRatio;
//
//
//  if ( hMecAngle < 0 )
//  {
//    hMecAngle *= -1;
//    hMecAngleuint = 65535u - ( uint16_t )hMecAngle;
//  }
//  else
//  {
//    hMecAngleuint = ( uint16_t )hMecAngle;
//  }
//
//  hAngleCounts = ( uint16_t )( ( ( uint32_t )hMecAngleuint *
//                                 ( uint32_t )pHandle->PulseNumber ) / 65535u );
//
//  g_motor_status.previous_mechanical_position = g_motor_status.current_mechanical_position; // store old position
//  g_motor_status.current_mechanical_position = ENCODER_M1.PreviousCapture; // 0 ... 14336 - 1 mechanical motor rotation
//
//  if (g_motor_status.previous_mechanical_position > ENCODER_M1.PulseNumber - 4000 && g_motor_status.current_mechanical_position  < 4000)
//  {
//	g_motor_status.current_mechanical_rotation++;
//  }
//  if (g_motor_status.current_mechanical_position  > ENCODER_M1.PulseNumber - 4000 && g_motor_status.previous_mechanical_position < 4000)
//  {
//	g_motor_status.current_mechanical_rotation--;
//  }
//  g_motor_status.current_encoder_position = g_motor_status.current_mechanical_rotation * ENCODER_M1.PulseNumber + g_motor_status.current_mechanical_position;
//
//  TIMx->CNT = ( uint16_t )( hAngleCounts );
//
//}
///**
//  * @brief  IRQ implementation of the TIMER ENCODER
//  * @param  pHandle: handler of the current instance of the encoder component
//  * @param  flag used to distinguish between various IRQ sources
//  * @retval none
//  */
//void * ENC_IRQHandler( void * pHandleVoid )
//{
//  ENCODER_Handle_t * pHandle = ( ENCODER_Handle_t * ) pHandleVoid;
//
//  /*Updates the number of overflows occurred*/
//  /* the handling of overflow error is done in ENC_CalcAvrgMecSpeedUnit */
//  pHandle->TimerOverflowNb += 1u;
//
////g_motor_status.previous_mechanical_position = g_motor_status.current_mechanical_position; // store old position
////g_motor_status.current_mechanical_position = ENCODER_M1.PreviousCapture; // 0 ... 14336 - 1 mechanical motor rotation
//
//if (ENCODER_M1.PreviousCapture  < 6000)
//{
//g_motor_status.current_mechanical_rotation--;
//}
//if (ENCODER_M1.PreviousCapture  > ENCODER_M1.PulseNumber - 6000)
//{
//g_motor_status.current_mechanical_rotation++;
//}
////g_motor_status.current_encoder_position = g_motor_status.current_mechanical_rotation * ENCODER_M1.PulseNumber + g_motor_status.current_mechanical_position;
//
//  return MC_NULL;
//}

/**
* @brief  It calculates the rotor electrical and mechanical angle, on the basis
*         of the instantaneous value of the timer counter.
* @param  pHandle: handler of the current instance of the encoder component
* @retval int16_t Measured electrical angle in s16degree format.
*/
//__weak int16_t ENC_CalcAngle( ENCODER_Handle_t * pHandle )
//{
//  int32_t wtemp1;
//  int16_t elAngle;  /* s16degree format */
//  int16_t mecAngle; /* s16degree format */
//  /* PR 52926 We need to keep only the 16 LSB, bit 31 could be at 1
//   if the overflow occurs just after the entry in the High frequency task */
//  wtemp1 = ( int32_t )( LL_TIM_GetCounter( pHandle->TIMx ) & 0xffff ) *
//           ( int32_t )( pHandle->U32MAXdivPulseNumber );
//
//  /*Computes and stores the rotor mechanical angle*/
//  mecAngle = ( int16_t )( wtemp1 / 65536 );
//
//  int16_t hMecAnglePrev = pHandle->_Super.hMecAngle;
//
//  pHandle->_Super.hMecAngle = mecAngle;
//
//  /*Computes and stores the rotor electrical angle*/
//  elAngle = mecAngle * pHandle->_Super.bElToMecRatio;
//
//  pHandle->_Super.hElAngle = elAngle;
//
//  int16_t hMecSpeedDpp = mecAngle - hMecAnglePrev;
//  pHandle->_Super.wMecAngle += (int32_t)(hMecSpeedDpp);
//
//  /*Returns rotor electrical angle*/
//  return ( elAngle );
//}

uint16_t NTC_SetFaultState( NTC_Handle_t * pHandle )
{
  uint16_t hFault;

//  if ( pHandle->hAvTemp_d > pHandle->hOverTempThreshold )
//  {
//    hFault = MC_OVER_TEMP;
//  }
//  else if ( pHandle->hAvTemp_d < pHandle->hOverTempDeactThreshold )
//  {
//    hFault = MC_NO_ERROR;
//  }
//  else
//  {
    hFault = pHandle->hFaultState;
//  }
  return hFault;
}

void Flash_Read_Data(uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords)
{
	while (1)
	{
		*RxBuf = *(__IO uint32_t *)StartPageAddress;
		StartPageAddress += 4;
		RxBuf++;
		if (!(numberofwords--)) break;
	}
}

int16_t get_sector_number_from_electric_rotation_number(int16_t electric_rotation)
{
	int16_t l_sector_number 		= -1;

	l_sector_number = (electric_rotation - g_joint_configuration.calibration_sector_size + 1) / g_joint_configuration.calibration_sector_size;

	return l_sector_number;
}

int16_t get_sector_number_from_ma730(int16_t ma730_value)
{
	int16_t l_sector_number 		= -1;

	// in sector
	for (int i = 0; i < g_joint_configuration.calibration_table_size; i++)
	{
		volatile uint16_t l_lower_value 		= 0;
		volatile uint16_t l_higher_value 		= 0;
		volatile bool  	  l_sector_with_zero	= false;
		volatile bool  	  l_wynik;

		l_higher_value 	= g_joint_configuration.calibration_table_1[i];
		l_lower_value 	= g_joint_configuration.calibration_table_2[i];

		if (l_higher_value < l_lower_value)
		{
			volatile uint16_t l_tmp_value = l_higher_value;
			l_higher_value 		 = l_lower_value;
			l_lower_value 		 = l_tmp_value;
		}

		l_wynik = (l_higher_value - l_lower_value > (uint16_t) 8000);

		if (l_wynik)
		{
			l_sector_with_zero = true;
		}

		if ((l_lower_value <= ma730_value && ma730_value <= l_higher_value) && !l_sector_with_zero)
		{
			return i;
		}

		if (( (0 <= ma730_value && ma730_value <= l_lower_value) || (l_higher_value <= ma730_value && ma730_value <= 16536) ) && l_sector_with_zero)
		{
			return i;
		}

//		if ((0 <= ma730_value && ma730_value <= l_higher_value) || (l_lower_value <= ma730_value && ma730_value <= 16536))
//		{
//			return i;
//		}

		//		if (l_lower_value > l_higher_value) // cross
//		{
//			if ((0 <= ma730_value && ma730_value <= l_higher_value) || (l_lower_value <= ma730_value && ma730_value <= 16536))
//			{
//				return i;
//			}
//		}

//		if (l_lower_value <= ma730_value && ma730_value <= l_higher_value)
//		{
//			return i;
//		}
	}

	// in space between readings
	if (g_current_sector_number != -1) // FIXME WORKAROUND: place between sectors - return last
	{
		return g_current_sector_number;
	}

//	if (g_current_sector_number == -1) // FIXME WORKAROUND: place between sectors - no last sector, select farest from 0 position
//	{
//		uint16_t l_zero_sector = get_sector_number_from_electric_rotation_number(g_joint_configuration.zero_electric_rotation);
//		for (int i = 1; i < g_joint_configuration.calibration_table_size; i++)
//		{
//			uint16_t l_lower_value 	= 0;
//			uint16_t l_higher_value = 0;
//
//			l_higher_value 	= g_joint_configuration.calibration_table_1[i];
//			l_lower_value 	= g_joint_configuration.calibration_table_2[i + 1];
//
//			if (l_lower_value <= ma730_value && ma730_value <= l_higher_value)
//			{
////				return i;
//				// select farest from 0
//				if (i > l_zero_sector)
//				{
//					return g_joint_configuration.calibration_table_size;
//				}
//				else
//				{
//					return 0;
//				}
//			}
//		}
//	}

//	FSM_Transition14(); // RAISE ERROR

	return l_sector_number;
}

void motor_start(Motor_Mode_t mode, int16_t goal)
{
	g_motor_command.goal = goal;
//	g_motor_command.last_goal = goal;
	g_motor_command.mode = mode;

	switch (mode)
	{
		case TORQUE:
		{
			g_motor_command.state = MOTOR_STARTED_IN_TORQUE_MODE;
			MC_ProgramTorqueRampMotor1(g_motor_command.goal, 0);
			break;
		}

		case SPEED:
		{
			g_motor_command.state = MOTOR_STARTED_IN_SPEED_MODE;
			MC_ProgramSpeedRampMotor1(g_motor_command.goal, 0);
			break;
		}

		default:
		{
			g_motor_command.goal = 0;
//			g_motor_command.last_goal = 0;
			return;
		}
	}

	MC_StartMotor1();
}

void motor_stop()
{
//	if (g_motor_command.state != MOTOR_STOPPED)
//	{
			g_motor_command.state = MOTOR_STOPPED;
			g_motor_command.goal = 0;
//			g_motor_command.last_goal = 0;

			// clear torque readings
			for (int i = 0; i < CURRENT_TORQUE_DATA_SIZE; i++)
			{
				g_motor_status._current_torque_data[i] = 0;
			}

			g_motor_status.current_motor_torque = 0;

		    MC_StopMotor1();

//	}
}

bool motor_reach_torque_limit()
{
	if (g_motor_status.current_motor_torque > 2000 && g_motor_status.stm_state_motor == RUN) return true;

	return false;
}

bool motor_in_position(volatile int32_t position)
{
	if (abs(g_motor_status.current_encoder_position - position) < 50) return true;

	return false;
}

bool FSM_Activate_State(FSM_State_t new_state)
{
	g_fsm_status.state = new_state;
	return true;
}

bool FSM_Activate_Transition(FSM_State_t new_transition)
{
	g_fsm_status.state = new_transition;
	return true;
}

bool FSM_Set_State(FSM_State_t new_state) // FIXME running transition should block changing state to new one - add flagg transition is running
{

	switch (new_state)
	{
//#if defined CALIBRATION
//#else
		case FSM_START:
		{
			break;
		}

		case FSM_INIT:
		{
			if (FSM_Get_State() == FSM_START)
			{
				return FSM_Activate_Transition(FSM_TRANSITION_START_TO_INIT);
			}

			if (FSM_Get_State() == FSM_FAULT)
			{
				return FSM_Activate_Transition(FSM_TRANSITION_FAULT_TO_INIT);
			}
			break;
		}

		case FSM_READY_TO_OPERATE:
		{

			if (FSM_Get_State() == FSM_INIT)
			{
				return FSM_Activate_Transition(FSM_TRANSITION_INIT_TO_READY_TO_OPERATE);
			}

			if (FSM_Get_State() == FSM_OPERATION_ENABLE)
			{
				return FSM_Activate_Transition(FSM_TRANSITION_OPERATION_ENABLE_TO_READY_TO_OPERATE);
			}

			break;
		}

		case FSM_OPERATION_ENABLE:
		{
			if (FSM_Get_State() == FSM_READY_TO_OPERATE)
			{
				return FSM_Activate_Transition(FSM_TRANSITION_READY_TO_OPERATE_TO_OPERATION_ENABLE);
			}
			break;
		}

		case FSM_CALIBRATION_PHASE_1:
		{

			if (FSM_Get_State() == FSM_INIT)
			{
				return FSM_Activate_Transition(FSM_TRANSITION_INIT_TO_CALIBRATION_PHASE_1);
			}

			break;
		}

		case FSM_FAULT_REACTION_ACTIVE:
		{
			if (FSM_Get_State() != FSM_START)
			{
				return FSM_Activate_Transition(FSM_TRANSITION_FAULT_REACTION_ACTIVE_TO_FAULT);
			}
			break;
		}

		case FSM_FAULT:
		{
			if (FSM_Get_State() == FSM_READY_TO_OPERATE)
			{
				return FSM_Activate_Transition(FSM_TRANSITION_READY_TO_OPERATE_TO_OPERATION_ENABLE);
			}
			break;
		}

		default:
		{
			break;
		}
//#endif
	}

	return false;
}


FSM_State_t FSM_Get_State(void)
{
	return g_fsm_status.state;
}

//#if defined CALIBRATION
bool check_calibration_data_cw(int16_t size) {
	g_calibration_data_1_errors = 0;

	for (int i = 0; i < size; i++) {
//		if (calibration_data_1[i] == 0) g_calibration_data_1_errors++;
		if (g_joint_configuration.calibration_table_1[i] == 0) g_calibration_data_1_errors++;


	}

	if (g_calibration_data_1_errors > 0) return false;

	return true;
}

bool check_calibration_data_ccw(int16_t size) {
	g_calibration_data_2_errors = 0;

	for (int i = 0; i < size; i++) {
//		if (calibration_data_2[i] == 0) g_calibration_data_2_errors++;
		if (g_joint_configuration.calibration_table_2[i] == 0) g_calibration_data_2_errors++;
	}

	if (g_calibration_data_2_errors > 0) return false;

	return true;
}
//#else

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan1, uint32_t RxFifo0ITs)
{

	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
	{

		// RETRIEVE CAN MESSAGE -------------------------------------------------------------------------
		HAL_FDCAN_GetRxMessage(hfdcan1, FDCAN_RX_FIFO0, &g_can_rx_header, g_can_rx_data);
		g_can_rx_counter++;

		// RESET SAFE TIMER -----------------------------------------------------------------------------
//		HAL_TIM_Base_Stop(&htim6);
//		TIM6->CNT = 0;
//		HAL_TIM_Base_Start_IT(&htim6);

		// UPDATE JOINT INFO ----------------------------------------------------------------------------
//		RecalculateJointState();

//		uint8_t l_can_cmd_motor_run = 0;

		if (g_can_rx_header.Identifier == 0x0AA)
		{
			int16_t goal;
			goal  = g_can_rx_data[g_node_status.can_node_id * 3] << 8;
			goal += g_can_rx_data[g_node_status.can_node_id * 3 + 1];

			g_motor_command.goal_joint_torque_in_nm = ((double) goal * 256.0) / (double) INT16_MAX; // convert from int16 to double

			uint8_t temp_new_fsm_state   = g_can_rx_data[g_node_status.can_node_id * 3 + 2];

			if (FSM_Get_State() != temp_new_fsm_state) {
				FSM_Set_State(temp_new_fsm_state);
			}

			int16_t l_current_torque = (g_motor_status.current_joint_torque_in_nm / 256.0) * (double) INT16_MAX;
			if (g_motor_command.goal < 0)
			{
				l_current_torque = -l_current_torque;
			}

			// ENCODER_NOT_ACCURATE
			if (g_motor_status.encoder_position_state != POSITION_ACCURATE)
			{
				g_motor_status.warnings = JOINT_POSITION_NOT_ACCURATE;
//				g_motor_status.warnings = g_motor_status.warnings | JOINT_POSITION_NOT_ACCURATE;

			}
			else
			{
				g_motor_status.warnings = 0;
//				g_motor_status.warnings = g_motor_status.warnings & (0xFF ^ JOINT_POSITION_NOT_ACCURATE);
			}

			int16_t l_joint_position_in_s16degree = (int16_t) (g_motor_status.current_joint_position_in_rad * (65535.0 / M_TWOPI));
			g_can_tx_data[0] 	= l_joint_position_in_s16degree >> 8;
			g_can_tx_data[1] 	= l_joint_position_in_s16degree;
			int16_t speed = g_motor_status.current_joint_speed_in_rads * (double) INT16_MAX / M_TWOPI;

			g_can_tx_data[2] 	= speed >> 8;
			g_can_tx_data[3] 	= speed;
			g_can_tx_data[4] 	= l_current_torque >> 8;
			g_can_tx_data[5] 	= l_current_torque;
			g_can_tx_data[6] 	= g_motor_status.current_temperature;
			g_can_tx_data[7] 	= FSM_Get_State();
			g_can_tx_data[8] 	= g_motor_status.mc_current_faults_motor;
			g_can_tx_data[9] 	= g_motor_status.mc_occured_faults_motor;
			g_can_tx_data[10]	= g_motor_status.errors;
			g_can_tx_data[11]	= g_motor_status.warnings;

//			 SEND FRAME VIA FD CAN -------------------------------------------------------------------------------
			if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan1, &g_can_tx_header, g_can_tx_data) == HAL_OK)
			{
				g_can_tx_counter++;
			}
		}
	}
}
//#endif

void MA730_ReadRegister(uint8_t reg_number) {
	uint16_t send_data      = 0b010 << 13 | (reg_number & (0b00011111)) << 8 ;

	uint16_t angle_value    = 0;
	uint16_t register_value = 0;

	for (uint16_t i = 0; i < 24; i++) NOP;  // wait about 150ns

	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_RESET);

	for (uint16_t i = 0; i < 130; i++) NOP;  // wait about 80ns

	// SEND READ REGISTER COMMAND - RECEIVE READ ANGLE RESULT
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t * ) &send_data, (uint8_t * ) &angle_value, 1, 1);

	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_SET);

	for (uint16_t i = 0; i < 120; i++) NOP;  // wait about 750ns

	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_RESET);

	for (uint16_t i = 0; i < 13; i++) NOP;  // wait about 80ns

	send_data      = 0x0000;

	// SEND READ ANGLE COMMAND - RECEIVE READ REGISTER RESULT
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t * ) &send_data, (uint8_t * ) &register_value, 1, 1);

//	g_MA730_read_buffer = register_value >> 8;

	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_SET);

	for (uint16_t i = 0; i < 120; i++) NOP;  // wait about 750ns

	g_ma730.angle = (angle_value >> 2) & 0b0011111111111111;

	register_value = register_value >> 8;
	switch (reg_number)
	{
		case 0x00:
		{
			g_ma730.z0 = (uint8_t) register_value;
			break;
		}
		case 0x01:
		{
			g_ma730.z1 = (uint8_t) register_value;
			break;
		}
		case 0x02:
		{
			g_ma730.bct = (uint8_t) register_value;
			break;
		}
		case 0x03:
		{
			g_ma730.ety = ((uint8_t) register_value & 0b00000010) >> 1;
			g_ma730.etx = ((uint8_t) register_value & 0b00000001);
			break;
		}
		case 0x04:
		{
			g_ma730.ppt0 = ((uint8_t) register_value & 0b11000000) >> 6;
			g_ma730.ilip = ((uint8_t) register_value & 0b00111100) >> 2;
			break;
		}
		case 0x05:
		{
			g_ma730.ppt1 = (uint8_t) register_value;
			break;
		}
		case 0x06:
		{
			g_ma730.mglt = ((uint8_t) register_value & 0b11100000) >> 5;
			g_ma730.mght = ((uint8_t) register_value & 0b00011100) >> 2;
			break;
		}
		case 0x09:
		{
			g_ma730.rd = (uint8_t) register_value >> 7;
			break;
		}
		case 0x1b:
		{
			g_ma730.mgl = ((uint8_t) register_value & 0b01000000) >> 6;
			g_ma730.mgh = ((uint8_t) register_value & 0b10000000) >> 7;
			break;
		}
		default:
		{
			break;
		}
	}

//	if (g_counter_1hz > 1) // wait 1 sec to analyse data
//	{
//		g_motor_status.ma730_is_running = true;
//		g_motor_status.previous_ma730_value = g_motor_status.current_ma730_value;
//		g_motor_status.current_ma730_value = (g_MA730_read_buffer >> 2) & 0b0011111111111111;
//	}

	if (g_ma730.mgl || g_ma730.mgh) // not proper magnetic field strentgh
	{
		g_motor_status.warnings |= JOINT_MA730_NOT_PROPER_MAGNETOC_FIELD;
	}
	else
	{
		g_motor_status.warnings = g_motor_status.warnings & (0xFF ^ JOINT_MA730_NOT_PROPER_MAGNETOC_FIELD);

	}

}

void MA730_ReadAngle() {
	uint16_t send_data      = 0x0000 ;

	uint16_t angle_value    = 0;

//	for (uint16_t i = 0; i < 24; i++) NOP;  // wait about 150ns

	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_RESET);

//	for (uint16_t i = 0; i < 13; i++) NOP;  // wait about 80ns

	// SEND READ REGISTER COMMAND - RECEIVE READ ANGLE RESULT
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t * ) &send_data, (uint8_t * ) &angle_value, 1, 1);

	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_SET);

//	for (uint16_t i = 0; i < 12; i++) NOP;  // wait about 80ns

	g_ma730.angle = (angle_value >> 2) & 0b0011111111111111;

}

void MA730_WriteRegister(uint8_t reg_number, uint8_t reg_value) {
	uint16_t send_data      = 0b100 << 13 | (reg_number & (0b00011111)) << 8 | reg_value;

	uint16_t angle_value    = 0;
	uint16_t register_value = 0;

	for (uint16_t i = 0; i < 24; i++) NOP;  // wait about 150ns

	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_RESET);

	for (uint16_t i = 0; i < 130; i++) NOP;  // wait about 80ns

	// SEND READ REGISTER COMMAND - RECEIVE READ ANGLE RESULT
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t * ) &send_data, (uint8_t * ) &angle_value, 1, 1);

	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_SET);

	HAL_Delay(20); // Wait 20 ms after write command

	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_RESET);

	for (uint16_t i = 0; i < 13; i++) NOP;  // wait about 80ns

	send_data      = 0x0000;

	// SEND READ ANGLE COMMAND - RECEIVE READ REGISTER RESULT
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t * ) &send_data, (uint8_t * ) &register_value, 1, 1);

	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_SET);

	for (uint16_t i = 0; i < 120; i++) NOP;  // wait about 750ns

}

// Commands
// FSM XX  - set FSM STATE
// FSM     - get FSM STATE
// DIRECTION 1/0 -
// TORQUE XXXXX - set torque - 13
// SPEED XXXXX - set speed
// MULTITURN 1/0 - disabling rotation constrains
// MA730 1/0
// GETDATA

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	//HAL_StatusTypeDef status;
//	uint8_t rez;
//	if (huart->Instance == USART2) {
//		rez = buff[1];
//		HAL_UART_Receive_IT(huart, &buff[1], 1);
//		huart2.gState = HAL_UART_STATE_READY;
//		//status =
//		HAL_UART_Transmit_IT(&huart2, &rez, 1);
//		//do {
//		//} while(status != HAL_OK);
//
//		cmd_buff[cmd_idx] = rez;
//		cmd_idx++;
//		if (cmd_idx >= sizeof(cmd_buff))
//			cmd_idx = 0;
//
//		if (rez == '\r') {
//			cmd_exec = parse_cmd();
//		}
//	}
//}

//typedef struct {
//	const uint8_t mask[9];
//	const uint8_t text[9];
//} one_cmd;

//uint8_t parse_cmd(void)
//{
//	one_cmd cmds[] = { { "xxxxxxxccccddr", "xxxxxxxFSM 00r" },
//                       { "xxxxxxxxxxcccr", "xxxxxxxxxxFSMr" },
//                       { "xxccccccccccdr", "xxDIRECTION 0r" },
//                       { "xxxxcccccccccr", "xxxxDIRECTIONr" },
//                       { "xcccccccdddddr", "xTORQUE 00000r" },
//                       { "xxccccccdddddr", "xxSPEED 00000r" },
//                       { "xxccccccccccdr", "xxMULTITURN 0r" },
//                       { "xxxxxxxxxxcccr", "xxxxxxxxxxFSMr" },
//					   { "xxxxxxccccccdr", "xxxxxxMA730 0r" },
//					   { "xxxxxxxcccccdr", "xxxxxxxtest 0r" }
//	};
//
//	for (uint8_t cmd = 0; cmd < sizeof(cmds) / sizeof(one_cmd); cmd++) {
//
////		uint8_t start_pos = (cmd_idx + BUFF_SIZE - 9) & BUFF_MASK;
////		uint16_t value = 0;
////		uint8_t rez = 0;
//
//		for (uint8_t i = 0; i < 13; i++) {
//			if (cmds[cmd].mask[i] == 'x') {
////				//dont carry
//			} else if (cmds[cmd].mask[i] == 'c') {
////				//character
////				if (cmd_buff[start_pos] != cmds[cmd].text[i])
////					break;
//			} else if (cmds[cmd].mask[i] == 'd') {
////				//digit
////				uint8_t digit = cmd_buff[start_pos];
////				if ((digit >= '0') || (digit <= '9')) {
////					value *= 10;
////					value += digit - '0';
////				} else
////					break;
//			} else if (cmds[cmd].mask[i] == 'r') {
////				//return
////				if (cmd_buff[start_pos] != '\r')
////					break;
////				rez = 1;
//			}
////			//next in cmd buffer
////			start_pos = (start_pos + 1) & BUFF_MASK;
//		}
////		if (rez) {
////			cmd_value = value;
////			return cmd + 1;
////		}
//	}
//	return 0;
//}

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
