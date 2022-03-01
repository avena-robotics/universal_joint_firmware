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
#include "adc.h"
#include "cordic.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
//	uint16_t read_buffer;
	bool started;
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

typedef enum Joint_Calibratation_State {
	JOINT_NOT_CALIBRATED = 0, /**< @brief Starting uC.*/
	JOINT_CALIBRATED = 1
} Joint_Calibratation_State_t;

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
	uint32_t spi_rx_counter;
	uint32_t spi_tx_counter;
	uint32_t spi_txrx_counter;
} __attribute__ ((packed)) Counters_Handle_t;

typedef struct {
	uint8_t can_node_id;
	uint8_t gear_ratio;
	uint8_t pole_pairs;
	Motor_t motor_type;
//	uint16_t encoder_resolution;

	bool dip1;
	bool dip2;
	bool dip3;
	bool dip4;
	bool ma730_enabled;
//	bool canbus_enabled;
//	bool canbus_watchdog_enabled;
	bool working_area_constrain_enabled; /**< @brief settings to enable/disable constrain */

	Joint_Calibratation_State_t calibration_state;

	uint16_t maximum_electrical_rotations;
	uint16_t reachable_electrical_rotations;

	uint16_t number_of_sectors;
	int16_t  zero_electric_position;
	uint16_t zero_electric_rotation;
	float joint_working_area;
	uint16_t calibration_sector_size; // in electrical rotations
	uint16_t calibration_table_size;
	uint16_t calibration_table_1[512];
	uint16_t calibration_table_2[512];

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

typedef struct {
	Joint_Position_State_t current_joint_position;
	Encoder_Position_State_t encoder_position_state;

	float f_current_encoder_position_offset;	// rad

	float f_current_motor_position; 			// rad
	float f_current_joint_position; 			// rad
	float f_current_motor_speed; 				// rad/s
	float f_current_joint_speed; 				// rad/s
	float f_current_motor_torque; 				// Nm
	float f_current_joint_torque; 				// Nm
	float f_current_motor_temperature;			// C
	float f_current_voltage;					// V

	int16_t mc_current_motor_torque;
	int16_t mc_current_motor_speed;
	int32_t mc_current_motor_rotation;
	int32_t mc_current_motor_position_multiturn;
	uint16_t mc_current_motor_position;
	uint16_t mc_previous_motor_position;
	int32_t mc_current_electric_rotation;
	int16_t mc_current_electric_position;
	int16_t mc_previous_electric_position;

	uint8_t errors;
	uint8_t warnings;

//	PosCtrlStatus_t mc_position_control_status;
//	AlignStatus_t mc_encoder_align_status;
	State_t stm_state_motor;
	uint8_t mc_current_faults_motor;
	uint8_t mc_occured_faults_motor;

	int16_t _current_torque_data[CURRENT_TORQUE_DATA_SIZE];
	uint8_t _current_torque_index;

	int16_t _current_speed_data[CURRENT_SPEED_DATA_SIZE];
	uint8_t _current_speed_index;
} __attribute__ ((packed)) Joint_Status_Handle_t;

typedef struct {
	FSM_State_t state;
	bool state_is_running;
	bool transition_is_running;
} __attribute__ ((packed)) FSMStatus_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GEAR_RATIO							(uint16_t) 121
#define SECTOR_SIZE							(uint16_t) 5
#define POLE_PAIRS							(uint16_t) 7
#define JOINT_SPEED_LIMIT					(float) 1.0
//#define KT									(float) 0.1118
//#define JOINT_TYPE							JOINT_MEDIUM
#define CALIBRATION_TORQUE_LIMIT			2000
#define CALIBRATION_ZERO_POSITION_OFFSET	10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define NOP asm("NOP")
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FDCAN_RxHeaderTypeDef can_rx_header; // CAN Bus Transmit Header
FDCAN_TxHeaderTypeDef can_tx_header; // CAN Bus Transmit Header
uint8_t can_rx_data[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  //CAN Bus Receive Buffer
uint8_t can_tx_data[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  //CAN Bus Send Buffer

uint16_t g_spi_rx_data    = 0x0000 ;
uint16_t g_spi_tx_data    = 0x0000 ;

// FLASH
uint32_t g_flash_address_configuration 		= 0x08018000; // FIXME zmienic na 0x08018000
uint32_t g_flash_address_calibration_table 	= 0x08018100;
uint16_t g_calibration_config[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint32_t g_data[2] = {0, 0};

Counters_Handle_t volatile g_counters =
{
	.main = 0,
	.timer6 = 0,
	.timer7 = 0,
	.spi_rx_counter = 0,
	.spi_tx_counter = 0,
	.spi_txrx_counter = 0,

};

App_Command_Handle_t g_app_command =
{
	.working_mode = TORQUE_MODE,
	.motor_torque = 0.0,
};

Joint_Configuration_Handle_t g_joint_configuration =
{
	.ma730_enabled = false,
	.working_area_constrain_enabled = false,
	.motor_type = RI80,
	.can_node_id = 0x00,
	.gear_ratio = 121
};

Joint_Status_Handle_t g_joint_status =
{
	.mc_current_motor_position = 0
};

volatile MA730_t 	g_ma730 =
{
	.started = false,
};

volatile FSMStatus_t 	g_fsm_status =
{
	.state = FSM_START,
	.state_is_running = false,
	.transition_is_running = false,
};

// Calibration variables
volatile Calibration_State_t g_calibration_state = CALIBRATION_NOT_FINISHED;

volatile uint16_t g_calibration_speed = 10; // speed of configuration rotations
volatile uint16_t g_calibration_torque_limit = CALIBRATION_TORQUE_LIMIT;

volatile int32_t g_min_encoder_position = 0;
volatile int32_t g_max_encoder_position = 0;
volatile int32_t g_center_encoder_position = 0;
volatile int16_t g_max_electric_rotation_cw  = 0;
volatile int16_t g_max_electric_rotation_ccw = 0;
volatile uint16_t g_calibration_data_1_errors = 0;
volatile uint16_t g_calibration_data_2_errors = 0;


int16_t g_current_sector_number = -1;
int16_t g_previous_sector_number = -1;
int16_t g_current_estimated_electric_rotation = -1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
FSM_State_t FSM_Get_State(void);
bool FSM_Set_State(FSM_State_t new_state);
bool FSM_Activate_State(FSM_State_t new_state);
bool FSM_Activate_Transition(FSM_State_t new_transition);
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
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_MotorControl_Init();
  MX_FDCAN1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  // TIMERS
  HAL_TIM_Base_Start_IT(&htim6); 	// Enable 10 kHz timer for fast calculation
  HAL_TIM_Base_Start_IT(&htim7);  	// Enable  1 kHz timer for FSM

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
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
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

/* USER CODE BEGIN 4 */
void motor_start(Working_Mode_t mode, int16_t goal)
{

	g_app_command.working_mode = mode;

	switch (g_app_command.working_mode)
	{
		case TORQUE_MODE:
		{
			g_app_command._motor_torque = goal;
			MC_ProgramTorqueRampMotor1(g_app_command._motor_torque, 0);
			MC_StartMotor1();
			break;
		}

		case SPEED_MODE:
		{
			g_app_command._motor_speed = goal;
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


//	g_motor_command.goal = goal;
////	g_motor_command.last_goal = goal;
//	g_motor_command.mode = mode;
//
//	switch (mode)
//	{
//		case TORQUE:
//		{
//			g_motor_command.state = MOTOR_STARTED_IN_TORQUE_MODE;
//			MC_ProgramTorqueRampMotor1(g_motor_command.goal, 0);
//			break;
//		}
//
//		case SPEED:
//		{
//			g_motor_command.state = MOTOR_STARTED_IN_SPEED_MODE;
//			MC_ProgramSpeedRampMotor1(g_motor_command.goal, 0);
//			break;
//		}
//
//		default:
//		{
//			g_motor_command.goal = 0;
////			g_motor_command.last_goal = 0;
//			return;
//		}
//	}
//
//	MC_StartMotor1();
}

void motor_stop()
{

	// clear torque readings
	for (int i = 0; i < CURRENT_TORQUE_DATA_SIZE; i++)
	{
		g_joint_status._current_torque_data[i] = 0;
	}

	g_app_command._motor_torque = 0;
	g_app_command.motor_torque = 0;
	g_app_command.joint_torque = 0;

	g_app_command._motor_speed = 0;
	g_app_command.motor_speed = 0;
	g_app_command.joint_speed = 0;

	MC_StopMotor1();
}

bool motor_reach_torque_limit()
{
	if (g_joint_status.mc_current_motor_torque > g_calibration_torque_limit && g_joint_status.stm_state_motor == RUN) return true;

	return false;
}

bool motor_in_position(volatile int32_t position)
{
	if (abs(g_joint_status.mc_current_motor_position_multiturn - position) < CALIBRATION_ZERO_POSITION_OFFSET) return true;

	return false;
}

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

// l - poczatek, r - koniec, x - szukane, arr - lista
// l - lewy sektor, r - prawy sektor, x - ma730, o - offset
#pragma GCC push_options
#pragma GCC optimize ("O0")
int16_t get_sector_number_from_calibration(uint16_t left_index, uint16_t right_index, uint16_t ma730_value, uint16_t offset)
{
    if (right_index >= left_index) {
    	int16_t mid = left_index + (right_index - left_index) / 2; // srodek

    	uint32_t mid_left_value  = g_joint_configuration.calibration_table_1[mid];    // wartosc lewego brzegu sektora
    	uint32_t mid_right_value = g_joint_configuration.calibration_table_2[mid];    // wartosc prawego brzegu sektora
    	uint32_t searched_value  = ma730_value; // wartosc szukana

    	// Przesuniecie wartosci o offset, by funkcja byla w całej długości ciągła
    	if (mid_left_value <= offset)
    	{
    		mid_left_value += 16384;
    	}

    	if (mid_right_value <= offset)
    	{
    		mid_right_value += 16384;
    	}

    	if (searched_value <= offset)
    	{
    		searched_value += 16384;
    	}

        // If the element is present at the middle
        // itself
		if (searched_value >= mid_left_value && searched_value <= mid_right_value ) // czy jest w sektorze srodkowym - jezeli tak, to koniec
		{
			return mid; // dobry sektor
		}

    	if (left_index == right_index) {
    		return -1;
    	}

        // If element is smaller than mid, then
        // it can only be present in left subarray
        if (mid_left_value > searched_value) // element jest mniejszsy niz srodkowy
        {
			uint16_t index = get_sector_number_from_calibration(left_index, mid - 1, ma730_value, offset); // szukaj z lewej strony
			return index;
        }

        // Else the element can only be present
        // in right subarray
        return get_sector_number_from_calibration(mid + 1, right_index, ma730_value, offset);  // szukaj z prawej strony
    }

    // We reach here when element is not
    // present in array
    return -1; // element poza sektorami
}
#pragma GCC pop_options
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

void Read_MC_Encoder_10kHz() // odczyt danych enkdera i sprawdzanie obrotow
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
}

void Read_MC_Encoder_1kHz() // odczyt danych enkdera i sprawdzanie obrotow
{
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
	if (g_joint_status.mc_previous_motor_position - g_joint_status.mc_current_motor_position > ENCODER_M1.PulseNumber / 2)
	{
		g_joint_status._current_speed_data[g_joint_status._current_speed_index++] = g_joint_status.mc_previous_motor_position - g_joint_status.mc_current_motor_position - ENCODER_M1.PulseNumber;
	}
	else if (g_joint_status.mc_previous_motor_position - g_joint_status.mc_current_motor_position < -1 * ENCODER_M1.PulseNumber / 2)
	{
		g_joint_status._current_speed_data[g_joint_status._current_speed_index++] = g_joint_status.mc_previous_motor_position - g_joint_status.mc_current_motor_position + ENCODER_M1.PulseNumber;
	}
	else
	{
		g_joint_status._current_speed_data[g_joint_status._current_speed_index++] = g_joint_status.mc_previous_motor_position - g_joint_status.mc_current_motor_position;
	}
	g_joint_status._current_speed_index %= CURRENT_SPEED_DATA_SIZE;

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
	g_joint_status.mc_current_faults_motor 		= (uint8_t) MC_GetCurrentFaultsMotor1();
	g_joint_status.mc_occured_faults_motor 		= (uint8_t) MC_GetOccurredFaultsMotor1();
//	g_joint_status.mc_encoder_align_status  	= MC_GetAlignmentStatusMotor1();
//	g_joint_status.mc_position_control_status 	= MC_GetControlPositionStatusMotor1();
	g_joint_status.f_current_motor_temperature 	= (uint8_t) NTC_GetAvTemp_C(&TempSensorParamsM1);
}

void Update_Data_From_MC()
{
	// POSITION
	g_joint_status.f_current_motor_position = ((double) g_joint_status.mc_current_motor_position_multiturn / ENCODER_M1.PulseNumber) * M_TWOPI;
	g_joint_status.f_current_joint_position = (double) g_joint_status.f_current_motor_position / g_joint_configuration.gear_ratio + g_joint_status.f_current_encoder_position_offset;

	// SPEED
	float temp_speed = 0;
	for (int i = 0; i < CURRENT_SPEED_DATA_SIZE; i++)
	{
		temp_speed += (float) g_joint_status._current_speed_data[i];
	}

	g_joint_status.mc_current_motor_speed	= temp_speed / CURRENT_SPEED_DATA_SIZE; // impulses per 1ms
	g_joint_status.f_current_motor_speed 	= ((float) temp_speed / CURRENT_SPEED_DATA_SIZE * 1000.0 / ENCODER_M1.PulseNumber) * M_TWOPI;
	g_joint_status.f_current_joint_speed 	= g_joint_status.f_current_motor_speed / g_joint_configuration.gear_ratio;

	// TORQUE
	// recalculate_torques
	float temp_torque = 0;
	for (int i = 0; i < CURRENT_TORQUE_DATA_SIZE; i++)
	{
		temp_torque += (float) g_joint_status._current_torque_data[i];
	}

	g_joint_status.mc_current_motor_torque			= temp_torque / CURRENT_TORQUE_DATA_SIZE;
	g_joint_status.f_current_motor_torque			= (((float) (temp_torque / CURRENT_TORQUE_DATA_SIZE)) / INT16_MAX) * MAX_READABLE_CURRENT * KT;
	g_joint_status.f_current_joint_torque			= g_joint_status.f_current_motor_torque * g_joint_configuration.gear_ratio;

}

void CheckErrorsAndWarnings()
{
	bool error = false;

	// MOTOR CONTROL ERROR REACTION
	if ((g_joint_status.mc_current_faults_motor > 0 || g_joint_status.mc_occured_faults_motor > 0) && FSM_Get_State() != FSM_TRANSITION_FAULT_TO_INIT) {
		g_joint_status.errors =  g_joint_status.errors | JOINT_MC_FAILED;
		error = true;
	}
	else
	{
		g_joint_status.errors = g_joint_status.errors & (0xFF ^ JOINT_MC_FAILED);
	}

	// OVERSPEED ERROR REACTION
	if ((fabs(g_joint_status.f_current_joint_speed) > JOINT_SPEED_LIMIT) && FSM_Get_State() != FSM_TRANSITION_FAULT_TO_INIT) {
		g_joint_status.errors =  g_joint_status.errors | JOINT_JOINT_SPEED_TO_HIGH;
		error = true;
//	}
//	else
//	{
//		g_joint_status.errors = g_joint_status.errors & (0xFF ^ JOINT_JOINT_SPEED_TO_HIGH);
	}

	// WARNINGS:
	// WORKING AREA
	if (g_joint_status.f_current_joint_position < -1 * g_joint_configuration.joint_working_area && g_joint_configuration.working_area_constrain_enabled)
	{
		g_joint_status.current_joint_position = POSITION_UNDER_WORKING_AREA;
		g_joint_status.warnings = g_joint_status.warnings | JOINT_OUTSIDE_WORKING_AREA;
	}
	else if(g_joint_status.f_current_joint_position > g_joint_configuration.joint_working_area && g_joint_configuration.working_area_constrain_enabled)
	{
		g_joint_status.current_joint_position = POSITION_OVER_WORKING_AREA;
		g_joint_status.warnings = g_joint_status.warnings | JOINT_OUTSIDE_WORKING_AREA;
	}
	else
	{
		g_joint_status.current_joint_position = POSITION_IN_WORKING_AREA;
		g_joint_status.warnings = g_joint_status.warnings & (0xFF ^ JOINT_OUTSIDE_WORKING_AREA);
	}

	// ENCODER_NOT_ACCURATE
	if (g_joint_status.encoder_position_state != POSITION_ACCURATE)
	{
		g_joint_status.warnings = g_joint_status.warnings | JOINT_POSITION_NOT_ACCURATE;

	}
	else
	{
		g_joint_status.warnings = g_joint_status.warnings & (0xFF ^ JOINT_POSITION_NOT_ACCURATE);
	}

	if (error == true && FSM_Get_State() != FSM_FAULT && FSM_Get_State() != FSM_FAULT_REACTION_ACTIVE && FSM_Get_State() != FSM_TRANSITION_FAULT_REACTION_ACTIVE_TO_FAULT)
	{
		FSM_Activate_State(FSM_FAULT_REACTION_ACTIVE);
	}

}

void FSM_Action()
{
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi)
{
	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_SET);

    // TX-RX Done .. Do Something ...
	g_counters.spi_txrx_counter++;

	g_ma730.angle = (g_spi_rx_data >> 2) & 0b0011111111111111;

	if (g_ma730.started)
	{
		HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_RESET);

		if (HAL_SPI_TransmitReceive_IT(&hspi2, (uint8_t * ) &g_spi_tx_data, (uint8_t * ) &g_spi_rx_data, 1) != HAL_OK)
		{
			Error_Handler();
		}
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6) 	// 10kHz (5,0) - fast recalculation
	{
		g_counters.timer6++;
		Read_MC_Encoder_10kHz();
		Read_MC_Torque();
		Read_MC_State();
	}

	if(htim->Instance == TIM7) 	// 1kHz - FSM_Tasks
	{
		Read_MC_Encoder_1kHz();
		CheckErrorsAndWarnings();
		Update_Data_From_MC();

		// Estimate joint position from absolute encoder
		if (g_joint_configuration.ma730_enabled == true && g_joint_configuration.calibration_state == JOINT_CALIBRATED && g_ma730.started == true)
		{
			// JOINT POSITION ESTIMATION
			switch (g_joint_status.encoder_position_state)
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
					g_current_sector_number = get_sector_number_from_calibration(0, g_joint_configuration.number_of_sectors - 1, g_ma730.angle, g_joint_configuration.calibration_table_1[0]);

					// If motor running and sector is change update
					if (g_current_sector_number != g_previous_sector_number && g_previous_sector_number != -1 && g_current_sector_number != -1) // pass 1 time, to load previous and current
					{
						float electric_rotation_width = M_TWOPI / (g_joint_configuration.pole_pairs * g_joint_configuration.gear_ratio);
						uint16_t l_current_electric_rotation;

						if (g_current_sector_number - g_previous_sector_number > 0)
						{ // rotation in CCW direction => "+"
							l_current_electric_rotation = (g_current_sector_number) * g_joint_configuration.calibration_sector_size;
						}
						else
						{ // rotation in CW direction ==> "-"
							l_current_electric_rotation = (g_current_sector_number + 1) * g_joint_configuration.calibration_sector_size - 1;
						}
						int32_t l_diff_electric_position = g_joint_configuration.zero_electric_position - g_joint_status.mc_current_motor_position_multiturn;
						g_joint_status.mc_current_electric_rotation = l_current_electric_rotation;

						float l_electric_offset_to_zero_in_rad   = -1 * ((g_joint_configuration.zero_electric_rotation - l_current_electric_rotation) + ((float) l_diff_electric_position / 65536) - 1.0) * electric_rotation_width;
						float l_encoder_offset_to_current_in_rad = -1 * ((float) g_joint_status.mc_current_motor_position_multiturn / (ENCODER_M1.PulseNumber / g_joint_configuration.pole_pairs)) * electric_rotation_width;

//							g_motor_status.current_encoder_position_offset_in_rad = -1 * ((g_joint_configuration.zero_electric_rotation - l_current_electric_rotation) + ((float) l_diff_electric_position / 65536) - 1.0) * electric_rotation_width;
						g_joint_status.f_current_encoder_position_offset = l_electric_offset_to_zero_in_rad + l_encoder_offset_to_current_in_rad;

						g_joint_status.encoder_position_state = POSITION_ACCURATE;
					}

					if (g_current_sector_number != -1)
					{
						g_previous_sector_number = g_current_sector_number;
					}

					break;
				}

				case POSITION_UNKNOWN:
				{
					g_current_sector_number = get_sector_number_from_calibration(0, g_joint_configuration.number_of_sectors - 1, g_ma730.angle, g_joint_configuration.calibration_table_1[0]);
					if (g_current_sector_number != -1)
					{
						float electric_rotation_width = M_TWOPI / (g_joint_configuration.pole_pairs * g_joint_configuration.gear_ratio);

						g_current_estimated_electric_rotation = g_current_sector_number *  g_joint_configuration.calibration_sector_size; // center current sector

						g_joint_status.f_current_encoder_position_offset = -1 * (g_joint_configuration.zero_electric_rotation - g_current_estimated_electric_rotation) * electric_rotation_width;

						// calculate encoder positioon offset
						g_joint_status.encoder_position_state = POSITION_APROXIMATED;
					}
					break;
				}
			}

		}
		// FSM
		switch (FSM_Get_State()) {


			case FSM_CALIBRATION_PHASE_0:
			{
				g_joint_configuration.calibration_state = JOINT_NOT_CALIBRATED;

				g_joint_configuration.calibration_sector_size = SECTOR_SIZE;
				g_joint_configuration.pole_pairs = POLE_PAIRS;
				g_joint_configuration.gear_ratio = GEAR_RATIO;
				g_joint_configuration.maximum_electrical_rotations = g_joint_configuration.gear_ratio * g_joint_configuration.pole_pairs;
				g_joint_configuration.calibration_table_size = (uint16_t) (g_joint_configuration.maximum_electrical_rotations / g_joint_configuration.calibration_sector_size);

				g_joint_configuration.reachable_electrical_rotations = 0;
				g_joint_configuration.number_of_sectors = 0;
				g_joint_configuration.zero_electric_position = 0;
				g_joint_configuration.zero_electric_rotation = 0;
				for (int i = 0; i < g_joint_configuration.calibration_table_size; i++)
				{
					g_joint_configuration.calibration_table_1[i] = 0;
					g_joint_configuration.calibration_table_2[i] = 0;
				}

				g_fsm_status.state = FSM_CALIBRATION_PHASE_1;
				break;
			}

			case FSM_CALIBRATION_PHASE_1:
			{

				if (motor_reach_torque_limit()) // REACH MINIMUM EDGE
				{
					motor_stop();
					g_joint_status.mc_current_electric_rotation = 0; // zeroing electric rotation counter
					g_min_encoder_position = g_joint_status.mc_current_motor_position_multiturn; // encoder value

					g_fsm_status.state = FSM_CALIBRATION_PHASE_2;

				}
				else
				{
					motor_start(SPEED_MODE, -1 * g_calibration_speed);
				}

				break;
			}

			case FSM_CALIBRATION_PHASE_2:
			{
				if (g_joint_status.mc_current_electric_rotation % g_joint_configuration.calibration_sector_size == (0) &&
						g_joint_configuration.calibration_table_1[g_joint_status.mc_current_electric_rotation / g_joint_configuration.calibration_sector_size] == 0)
				{
					g_joint_configuration.calibration_table_1[g_joint_status.mc_current_electric_rotation / g_joint_configuration.calibration_sector_size] = g_ma730.angle;
				}

				if (motor_reach_torque_limit()) // REACH MAXIMUM EDGE
				{
					motor_stop();
					g_joint_configuration.calibration_table_2[g_joint_status.mc_current_electric_rotation / g_joint_configuration.calibration_sector_size] = g_ma730.angle;
					g_max_electric_rotation_cw 	= g_joint_status.mc_current_electric_rotation; // max electric rotation counter

					g_max_encoder_position = g_joint_status.mc_current_motor_position_multiturn; // max encoder value
					g_fsm_status.state = FSM_CALIBRATION_PHASE_3;

				}
				else
				{
					motor_start(SPEED_MODE, g_calibration_speed);
				}

				break;
			}

			case FSM_CALIBRATION_PHASE_3:
			{
				if (g_joint_status.mc_current_electric_rotation % g_joint_configuration.calibration_sector_size == (g_joint_configuration.calibration_sector_size - 1) &&
						g_joint_configuration.calibration_table_2[g_joint_status.mc_current_electric_rotation / g_joint_configuration.calibration_sector_size] == 0)
				{
					g_joint_configuration.calibration_table_2[g_joint_status.mc_current_electric_rotation / g_joint_configuration.calibration_sector_size] = g_ma730.angle;
				}

				if (motor_reach_torque_limit()) // REACH MINIMUM EDGE
				{
					motor_stop();
					g_joint_configuration.calibration_table_1[g_joint_status.mc_current_electric_rotation / g_joint_configuration.calibration_sector_size] = g_ma730.angle;
					g_fsm_status.state = FSM_CALIBRATION_PHASE_4;

				}
				else
				{
					motor_start(SPEED_MODE, -1 * g_calibration_speed);
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
					g_calibration_state = CALIBRATION_TABLE_CONTAINS_ZEROES;
				}
				else
				{
					g_max_electric_rotation_ccw = g_joint_status.mc_current_electric_rotation; // Should be 0 right now
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
					motor_start(SPEED_MODE, 0);
					motor_stop();

					g_joint_configuration.zero_electric_rotation = g_joint_status.mc_current_electric_rotation;
//					g_joint_configuration.zero_electric_position = (uint16_t) (g_motor_status.current_electric_position);
					g_joint_configuration.zero_electric_position = (int16_t) (g_joint_status.mc_current_electric_position);
				}
				else
				{
					motor_start(SPEED_MODE, g_calibration_speed);
				}

				if (motor_reach_torque_limit()) // REACH EDGE - FAILURE
				{
					motor_start(SPEED_MODE, 0);
					motor_stop();
					g_calibration_state = MISSED_CENTER_POSITION;
	//						g_fsm_status.state = FSM_STOPPED_WITH_ERRORS;
					FSM_Activate_State(FSM_FAULT_REACTION_ACTIVE);

				}

				break;
			}
			case FSM_CALIBRATION_PHASE_6:
			{
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
				data = (g_joint_configuration.gear_ratio << 16) | g_joint_configuration.pole_pairs;
				if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, g_flash_address_configuration, data) != HAL_OK)
				{
					error = HAL_FLASH_GetError ();
				}

				data = (g_joint_configuration.reachable_electrical_rotations << 16) | g_joint_configuration.calibration_sector_size;
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

				g_joint_configuration.calibration_state = JOINT_CALIBRATED;

				FSM_Activate_State(FSM_INIT); // CALIBRATION FINISHED - GO TO INIT STATE

				HAL_TIM_Base_Start_IT(&htim6); // Enable 10 kHz timer
				break;
			}


			case FSM_START:
			{
				HAL_TIM_Base_Stop_IT(&htim6); // Enable 10 kHz timer

				g_joint_configuration.pole_pairs = POLE_PAIRS;
				g_joint_configuration.gear_ratio = GEAR_RATIO;

				// Read DIP switches
				g_joint_configuration.dip1 = (HAL_GPIO_ReadPin(GPIOC, DIP1_Pin) == GPIO_PIN_RESET) ? (0) : (1);
				g_joint_configuration.dip2 = (HAL_GPIO_ReadPin(GPIOC, DIP2_Pin) == GPIO_PIN_RESET) ? (0) : (1);
				g_joint_configuration.dip3 = (HAL_GPIO_ReadPin(GPIOB, DIP3_Pin) == GPIO_PIN_RESET) ? (0) : (1);
				g_joint_configuration.dip4 = (HAL_GPIO_ReadPin(GPIOB, DIP4_Pin) == GPIO_PIN_RESET) ? (0) : (1);

				// Configure CAN ID
				g_joint_configuration.can_node_id = (uint8_t) (g_joint_configuration.dip2 << 2 | g_joint_configuration.dip3 << 1 | g_joint_configuration.dip4);

				// disable dip4 allow multiturn
	//					g_joint_configuration.working_area_constrain = g_joint_configuration.dip1;
//				g_joint_configuration.working_area_constrain = true;
	//	  			g_joint_configuration.ma730_exists = _dip1;

				// FDCAN
				HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 10, 0);
				HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);

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

				// Read configuration from FLASH
				Flash_Read_Data(g_flash_address_configuration, (uint32_t *) g_calibration_config, 8);

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
				g_joint_configuration.joint_working_area = M_PI * 165.0 / 180.0;

//				g_node_status.security = SECURITY_DISABLED;
//				g_node_status.hartbeat = HEARTBEAT_DISABLED;
//				g_node_status.node_type = JOINT_TYPE;

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
				if (g_joint_configuration.ma730_enabled == true && g_ma730.started == false)
				{
					g_ma730.started = true;

					HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_RESET);

					HAL_SPI_TransmitReceive_IT(&hspi2, (uint8_t * ) &g_spi_tx_data, (uint8_t * ) &g_spi_rx_data, 1);
				}

				if (g_joint_configuration.ma730_enabled == false && g_ma730.started == true)
				{
					g_ma730.started = false;

				}

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
				motor_stop();
				break;
			}

			case FSM_TRANSITION_INIT_TO_CALIBRATION_PHASE_0:
			{
				FSM_Activate_State(FSM_CALIBRATION_PHASE_0);
				break;
			}

			case FSM_OPERATION_ENABLE:
			{

				int16_t goal = 0;

				switch (g_app_command.working_mode)
				{
					case TORQUE_MODE:
					{
						goal = g_app_command._motor_torque;
						break;
					}

					case SPEED_MODE:
					{
						goal = g_app_command._motor_speed;
						break;
					}
				}

				if (g_joint_configuration.working_area_constrain_enabled)
				{
					switch (g_joint_status.current_joint_position)
					{
						case POSITION_UNDER_WORKING_AREA: // Accept only positive torque
							if (goal < 0)
							{
								motor_stop();
							}
							else
							{
								motor_start(g_app_command.working_mode, goal);
							}
							break;

						case POSITION_IN_WORKING_AREA:
							motor_start(g_app_command.working_mode, goal);
							break;

						case POSITION_OVER_WORKING_AREA: // Accept only negative torque
							if (goal > 0)
							{
								motor_stop();
							}
							else
							{
								motor_start(g_app_command.working_mode, goal);
							}
							break;
					}

				}
				else
				{
					motor_start(g_app_command.working_mode, goal);
				}

				break;

			}

			case FSM_TRANSITION_FAULT_TO_INIT:
			{
				MC_AcknowledgeFaultMotor1();
				g_joint_status.errors = 0;
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
				motor_stop();
				FSM_Activate_Transition(FSM_TRANSITION_FAULT_REACTION_ACTIVE_TO_FAULT);
				break;
			}

			case FSM_FAULT:
			{
				motor_stop();
				break;
			}

			default:
			{
				FSM_Activate_State(FSM_FAULT_REACTION_ACTIVE);
				break;
			}
		}
		g_counters.timer7++;
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
			numer_w_szeregu = g_joint_configuration.can_node_id; // ???
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
				g_app_command.joint_torque = ((float) goal * MAX_TORQUE_THROUGH_CAN) / (float) INT16_MAX; // convert from int16 to float
				g_app_command.motor_torque = g_app_command.joint_torque / g_joint_configuration.gear_ratio;

				// recalculate torque goal data from floats to MC
				g_app_command._motor_torque	= (((float) g_app_command.motor_torque) * INT16_MAX) / (MAX_READABLE_CURRENT * KT);  // convert from float to s16A


				// -------------------------------------------------------------------------------------------------
				// Recalculate data from MC to Floats
				Update_Data_From_MC();
				// recalculate date from
//				// POSITION
//				g_joint_status.f_current_motor_position = (double) g_joint_status.mc_current_motor_position_multiturn / ENCODER_M1.PulseNumber;
//				g_joint_status.f_current_joint_position = (double) g_joint_status.f_current_motor_position / g_joint_configuration.gear_ratio;
//
//				// SPEED
//				g_joint_status.f_current_motor_speed = g_joint_status.mc_current_motor_speed * 6 * M_TWOPI / 60;
//				g_joint_status.f_current_joint_speed = g_joint_status.f_current_motor_speed / g_joint_configuration.gear_ratio;
//
//				// TORQUE
//				// recalculate_torques
//				float temp_torque = 0;
//				for (int i = 0; i < CURRENT_TORQUE_DATA_SIZE; i++)
//				{
//					temp_torque += (float) g_joint_status._current_torque_data[i];
//				}
//
//				g_joint_status.f_current_motor_torque			= (((float) (temp_torque / CURRENT_TORQUE_DATA_SIZE)) / INT16_MAX) * 33.0 * KT;
//				g_joint_status.f_current_joint_torque			= g_joint_status.f_current_motor_torque * g_joint_configuration.gear_ratio;

				// -------------------------------------------------------------------------------------------------
				// Recalculate data from floats to CAN
				int32_t l_joint_position_in_s32degree = (int32_t) (g_joint_status.f_current_joint_position * (UINT32_MAX / M_TWOPI));

				int16_t l_speed_in_dpp = g_joint_status.f_current_joint_speed * (float) INT16_MAX / M_TWOPI;

				int16_t l_current_torque_in_s16a = (g_joint_status.f_current_joint_torque / MAX_TORQUE_THROUGH_CAN) * (float) INT16_MAX;
				if (g_app_command.motor_torque < 0)
				{
					l_current_torque_in_s16a = -l_current_torque_in_s16a;
				}

				// -------------------------------------------------------------------------------------------------
				can_tx_data[0] 	= l_joint_position_in_s32degree >> 24;
				can_tx_data[1] 	= l_joint_position_in_s32degree >> 16;
				can_tx_data[2] 	= l_joint_position_in_s32degree >> 8;
				can_tx_data[3] 	= l_joint_position_in_s32degree;
				can_tx_data[4] 	= l_speed_in_dpp >> 8;
				can_tx_data[5] 	= l_speed_in_dpp;
				can_tx_data[6] 	= l_current_torque_in_s16a >> 8;
				can_tx_data[7] 	= l_current_torque_in_s16a;
				can_tx_data[8] 	= (uint8_t) g_joint_status.f_current_motor_temperature; // temp FIXME
				can_tx_data[9] 	= FSM_Get_State(); // FSM
				can_tx_data[10]	= g_joint_status.mc_current_faults_motor;
				can_tx_data[11] = g_joint_status.mc_occured_faults_motor;
				can_tx_data[12] = g_joint_status.errors;
				can_tx_data[13] = g_joint_status.warnings;

				can_tx_header.DataLength = FDCAN_DLC_BYTES_16;

				break;
			}

			case 0x1: // Zmien stan FSM
			{
				dlugosc_danych_polecenia = 1;
				// uint8_t - FSM
				uint8_t offset = dlugosc_danych_polecenia * numer_w_szeregu;

				FSM_Set_State(can_rx_data[offset]);

				can_tx_data[0] = FSM_Get_State();

				can_tx_header.DataLength = FDCAN_DLC_BYTES_1;
				break;
			}

			case 0xF: // Konfiguracja
			{
				dlugosc_danych_polecenia = 2;
				// uint8_t tryb pracy
				uint8_t offset = dlugosc_danych_polecenia * numer_w_szeregu;
				if (FSM_Get_State() == FSM_INIT) {
					g_app_command.working_mode = can_rx_data[offset];
					g_joint_configuration.working_area_constrain_enabled = (can_rx_data[offset + 1] & 0x01);
					g_joint_configuration.ma730_enabled = (can_rx_data[offset + 1] & 0x02) >> 1;
					can_tx_data[0] = 1;
				}
				else
				{
					can_tx_data[0] = 0;
				}

				can_tx_header.DataLength = FDCAN_DLC_BYTES_1;
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

bool FSM_Set_State(FSM_State_t new_state) // FIXME running transition should block changing state to new one - add flag transition is running
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

		case FSM_CALIBRATION_PHASE_0:
		{

			if (FSM_Get_State() == FSM_INIT)
			{
				return FSM_Activate_Transition(FSM_TRANSITION_INIT_TO_CALIBRATION_PHASE_0);
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

//void MA730_ReadRegister(uint8_t reg_number) {
//	uint16_t send_data      = 0b010 << 13 | (reg_number & (0b00011111)) << 8 ;
//
//	uint16_t angle_value    = 0;
//	uint16_t register_value = 0;
//
//	for (uint16_t i = 0; i < 24; i++) NOP;  // wait about 150ns
//
//	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_RESET);
//
//	for (uint16_t i = 0; i < 130; i++) NOP;  // wait about 80ns
//
//	// SEND READ REGISTER COMMAND - RECEIVE READ ANGLE RESULT
//	HAL_SPI_TransmitReceive(&hspi2, (uint8_t * ) &send_data, (uint8_t * ) &angle_value, 1, 1);
//
//	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_SET);
//
//	for (uint16_t i = 0; i < 120; i++) NOP;  // wait about 750ns
//
//	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_RESET);
//
//	for (uint16_t i = 0; i < 13; i++) NOP;  // wait about 80ns
//
//	send_data      = 0x0000;
//
//	// SEND READ ANGLE COMMAND - RECEIVE READ REGISTER RESULT
//	HAL_SPI_TransmitReceive(&hspi2, (uint8_t * ) &send_data, (uint8_t * ) &register_value, 1, 1);
//
////	g_MA730_read_buffer = register_value >> 8;
//
//	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_SET);
//
//	for (uint16_t i = 0; i < 120; i++) NOP;  // wait about 750ns
//
//	g_ma730.angle = (angle_value >> 2) & 0b0011111111111111;
//
//	register_value = register_value >> 8;
//	switch (reg_number)
//	{
//		case 0x00:
//		{
//			g_ma730.z0 = (uint8_t) register_value;
//			break;
//		}
//		case 0x01:
//		{
//			g_ma730.z1 = (uint8_t) register_value;
//			break;
//		}
//		case 0x02:
//		{
//			g_ma730.bct = (uint8_t) register_value;
//			break;
//		}
//		case 0x03:
//		{
//			g_ma730.ety = ((uint8_t) register_value & 0b00000010) >> 1;
//			g_ma730.etx = ((uint8_t) register_value & 0b00000001);
//			break;
//		}
//		case 0x04:
//		{
//			g_ma730.ppt0 = ((uint8_t) register_value & 0b11000000) >> 6;
//			g_ma730.ilip = ((uint8_t) register_value & 0b00111100) >> 2;
//			break;
//		}
//		case 0x05:
//		{
//			g_ma730.ppt1 = (uint8_t) register_value;
//			break;
//		}
//		case 0x06:
//		{
//			g_ma730.mglt = ((uint8_t) register_value & 0b11100000) >> 5;
//			g_ma730.mght = ((uint8_t) register_value & 0b00011100) >> 2;
//			break;
//		}
//		case 0x09:
//		{
//			g_ma730.rd = (uint8_t) register_value >> 7;
//			break;
//		}
//		case 0x1b:
//		{
//			g_ma730.mgl = ((uint8_t) register_value & 0b01000000) >> 6;
//			g_ma730.mgh = ((uint8_t) register_value & 0b10000000) >> 7;
//			break;
//		}
//		default:
//		{
//			break;
//		}
//	}
//
////	if (g_counter_1hz > 1) // wait 1 sec to analyse data
////	{
////		g_motor_status.ma730_is_running = true;
////		g_motor_status.previous_ma730_value = g_motor_status.current_ma730_value;
////		g_motor_status.current_ma730_value = (g_MA730_read_buffer >> 2) & 0b0011111111111111;
////	}
//
////	if (g_ma730.mgl || g_ma730.mgh) // not proper magnetic field strentgh
////	{
////		g_motor_status.warnings |= JOINT_MA730_NOT_PROPER_MAGNETOC_FIELD;
////	}
////	else
////	{
////		g_motor_status.warnings = g_motor_status.warnings & (0xFF ^ JOINT_MA730_NOT_PROPER_MAGNETOC_FIELD);
////
////	}
//
//}

//void MA730_ReadAngle() {
//	uint16_t send_data      = 0x0000 ;
//
//	uint16_t angle_value    = 0;
//
////	for (uint16_t i = 0; i < 24; i++) NOP;  // wait about 150ns
//
//	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_RESET);
//
////	for (uint16_t i = 0; i < 13; i++) NOP;  // wait about 80ns
//
//	// SEND READ REGISTER COMMAND - RECEIVE READ ANGLE RESULT
//	HAL_SPI_TransmitReceive(&hspi2, (uint8_t * ) &send_data, (uint8_t * ) &angle_value, 1, 1);
//
//	HAL_GPIO_WritePin(MA730_CS_GPIO_Port, MA730_CS_Pin, GPIO_PIN_SET);
//
////	for (uint16_t i = 0; i < 12; i++) NOP;  // wait about 80ns
//
//	g_ma730.angle = (angle_value >> 2) & 0b0011111111111111;
//
//}

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
