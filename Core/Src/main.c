/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "event_groups.h"
#include "Display.h"
#include "singleWire.h"
#include "timers.h"
#include "can.h"
//#include "semphr. h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define fun(reg, bitn) reg&(1<<bitn)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for myGpioTask */
osThreadId_t myGpioTaskHandle;
const osThreadAttr_t myGpioTask_attributes = {
  .name = "myGpioTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};
/* Definitions for mutEepromWrite */
osMutexId_t mutEepromWriteHandle;
const osMutexAttr_t mutEepromWrite_attributes = {
  .name = "mutEepromWrite"
};
/* Definitions for mutEepromRead */
osMutexId_t mutEepromReadHandle;
const osMutexAttr_t mutEepromRead_attributes = {
  .name = "mutEepromRead"
};
/* Definitions for xSemCanDetect */
osSemaphoreId_t xSemCanDetectHandle;
const osSemaphoreAttr_t xSemCanDetect_attributes = {
  .name = "xSemCanDetect"
};
/* USER CODE BEGIN PV */
osThreadId_t singleWireTaskHandle;
const osThreadAttr_t singleWireTask_attributes = {
  .name = "singleWireTask",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 128 * 4
};

osThreadId_t displayTaskHandle;
const osThreadAttr_t displayTask_attributes = {
  .name = "displayTask",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 128 * 4
};

osThreadId_t adcTaskHandle;
const osThreadAttr_t adcTask_attributes = {
  .name = "adcTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};

osThreadId_t canTaskHandle;
const osThreadAttr_t canTask_attributes = {
  .name = "canTask",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 128 * 4
};

// Define the message queue handle
QueueHandle_t xQueueSingleWire, xQueueAdc;

//  declare a event grounp handler variable
EventGroupHandle_t  xEventGroupDisplay;

xTimerHandle timerOdoTrip;

typedef struct {
    uint32_t adcVal[ADC_MAX_CHANNEL];
    // Add more fields if needed
} adcData_t;

uint8_t tBaud = 0, dataVar[16], uartBuf[10], dBaud = 3, gv1, gav1, gav2, gav3, gav4, gv2, gv3, gv4;
volatile uint32_t odoVar = 0, tripVar = 0, volt = 0, soc = 0;
volatile int32_t amp;
volatile int16_t amp2;
volatile uint32_t tcnt = 0, tLimit = 0, divDis = 0, ahVar;
volatile uint16_t flt1, flt2, flt3, batteryFault16, soh, rAh, cAh, speedVar2=0, speedVar=0;
volatile uint16_t ntc1, ntc2, ntc3, ntc4, ntc5, ntc6, tempLimit=550, motorRpm, ctempAlert, mtempAlert;
unsigned long  odoVc, odoVarR, tripVarR, odoVarC, tripVarC, prvOdo;
extern uint16_t speedVarF;
_Bool runTime = 0, tog = 1, mTemp;
volatile uint8_t start, dataCountX, dataCountY, varCh, stopTimeOut, thrmlRun, ntcCount;
volatile unsigned int lineCount, CANindex, CANtype;
volatile uint8_t direction, spd, spdLH, emp1, emp2, mcx1, brkMod, gear;
volatile uint8_t canBatFlag, canControllerFlag, controllerFault, batteryFault, motorFault, motorTemp, throtelFault, throttlePos;
volatile uint8_t chargeState, errorState, reGen, reverseMod, parkingMod, thrmlRun, sStand, brake, cruiseMod, Warning;
_Bool bitOdo, tmpH1,tmpH2, tmpH3, tmpH4, tmpH5, tmpH6, freqMode=0;
unsigned char CANbuf[7][7];
extern uint8_t tBuf[200];
extern uint16_t eeCounter;
unsigned int divFact, rMul, rMul1, rMul2, rMul3, divFactFrq, lval=900, vs=9;
unsigned char vers = VERS;
uint8_t keyNo = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void StartGpioTask(void *argument);

/* USER CODE BEGIN PFP */
void StartAdcTask(void *argument);

FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const uint8_t appByte __attribute__(( section(".appByte") )) = 0x55;

uint16_t adcBuf[ADC_DMA_LENGTH+1];

/*
 * Timer event every 100mS
 */
static void vTimerCallback1SecExpired(xTimerHandle pxTimer)
{
  singleWireData_t dataToSend;
  static uint16_t keyCnt;
  //HAL_GPIO_TogglePin(GPIOB, LED6_Pin);
  if(runTime)
  {
	  if(tcnt<tLimit)
	  {
		  tcnt++;
	  }
	  else
	  {
		  tcnt=0;
		  runTime=0;
	  }
  }
  if(stopTimeOut)
  {
	  if((--stopTimeOut == 0)&&(start == 2))
	  {
			start = 0;
			varCh = 0;
			for(int p = 0; p < dataCountX; p++)
			{
				varCh^=dataVar[p];
			}
			if((varCh==0)&&(dataVar[0]!=0)&&(dataVar[4]!=0xff))
			{
				dataToSend.dataArrPtr = dataVar;
				dataToSend.cnt	=	12;
				// Send the data to the message queue
				if (xQueueSendFromISR(xQueueSingleWire, &dataToSend, NULL) == pdTRUE) {
					// Data sent successfully
				} else {
					// Queue is full, data not sent
				}
			}
	  }
  }
	if(!HAL_GPIO_ReadPin(KEY_ODO_GPIO_Port, KEY_ODO_Pin))
	{
		keyCnt++;
		if(keyCnt==20)																			//time out
		{
			keyNo = LONG;
			vTaskNotifyGiveFromISR(myGpioTaskHandle, 0);
		}
	}
	else
	{
		if((keyCnt>2)&&(keyCnt<=19))											//Short Press
		{
			keyNo=SHORT;
			vTaskNotifyGiveFromISR(myGpioTaskHandle, 0);
		}
		else if(keyCnt>19)																//long press
		{
			keyNo=LONG;
			vTaskNotifyGiveFromISR(myGpioTaskHandle, 0);
		}
		keyCnt=0;
	}
}

void modeSet(uint8_t md)
{
	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};
	  TIM_IC_InitTypeDef sConfigIC = {0};
	if(md == FREQMODEL)
	{
		  htim2.Instance = TIM2;
		  htim2.Init.Prescaler = 1000;//0;
		  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
		  htim2.Init.Period = 5000000;
		  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;//TIM_CLOCKDIVISION_DIV1;
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
		  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
		  {
		    Error_Handler();
		  }
		  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
		  {
		    Error_Handler();
		  }
		  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
		  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
		  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
		  sConfigIC.ICFilter = 3;
		  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
		  {
		    Error_Handler();
		  }
		  HAL_TIM_IC_Start_IT (&htim2, TIM_CHANNEL_4);
		  freqMode=1;
	}
	else //if(md == FREQMODEL)
	{
		  htim2.Instance = TIM2;
		  htim2.Init.Prescaler = 400;
		  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
		  htim2.Init.Period = 5000000;
		  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
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
		  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
		  {
		    Error_Handler();
		  }
		  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
		  {
		    Error_Handler();
		  }
		  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
		  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
		  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
		  sConfigIC.ICFilter = 3;
		  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
		  {
		    Error_Handler();
		  }
		  HAL_TIM_IC_Start_IT (&htim2, TIM_CHANNEL_4);
	  freqMode=0;
	}
}

// Function to swap the byte order of a 32-bit integer
uint32_t swapBytes(uint32_t value) {
    return ((value & 0x000000FF) << 24) |
           ((value & 0x0000FF00) << 8) |
           ((value & 0x00FF0000) >> 8) |
           ((value & 0xFF000000) >> 24);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int row, col;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  divDis = 176;
  runTime = 1;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_FDCAN_DeInit(&hfdcan1);
  HAL_DMA_Init(&hdma_adc1);												//
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuf, ADC_DMA_LENGTH);
  //UART_Start_Receive_IT(&huart1, uartBuf, 1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of mutEepromWrite */
  mutEepromWriteHandle = osMutexNew(&mutEepromWrite_attributes);

  /* creation of mutEepromRead */
  mutEepromReadHandle = osMutexNew(&mutEepromRead_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of xSemCanDetect */
  xSemCanDetectHandle = osSemaphoreNew(1, 1, &xSemCanDetect_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  timerOdoTrip = xTimerCreate(
        "timer1", /* name */
        pdMS_TO_TICKS(100), /* period/time */
        pdTRUE, /* auto reload */
        (void*)0, /* timer ID */
        vTimerCallback1SecExpired); /* callback */
  xTimerStart(timerOdoTrip, 0);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  xQueueSingleWire = xQueueCreate(20, sizeof(singleWireData_t));
  xQueueAdc = xQueueCreate(ADC_MAX_CHANNEL, sizeof(adcData_t));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myGpioTask */
  myGpioTaskHandle = osThreadNew(StartGpioTask, NULL, &myGpioTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  singleWireTaskHandle = osThreadNew(StartSingleWireTask, NULL, &singleWireTask_attributes);
  adcTaskHandle = osThreadNew(StartAdcTask, NULL, &adcTask_attributes);
  displayTaskHandle = osThreadNew(StartDisplayTask, NULL, &displayTask_attributes);
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  xEventGroupDisplay  =  xEventGroupCreate();
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV32;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 16;
  hfdcan1.Init.NominalTimeSeg1 = 63;
  hfdcan1.Init.NominalTimeSeg2 = 32;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 4;
  hfdcan1.Init.DataTimeSeg1 = 5;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 400;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 3;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_IC_Start_IT (&htim2, TIM_CHANNEL_4);

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BKLT_GPIO_Port, BKLT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED8_Pin|LED7_Pin|LED3_Pin|BUZZ_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED6_Pin|SEG_DATA_Pin|SEG_WR_Pin|SEG_RD_Pin
                          |SEG_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : CAN_MINUS_Pin CAN_PLUS_Pin */
  GPIO_InitStruct.Pin = CAN_MINUS_Pin|CAN_PLUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BRK_UC_Pin G1_Pin G2_Pin */
  GPIO_InitStruct.Pin = BRK_UC_Pin|G1_Pin|G2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : REV_Pin */
  GPIO_InitStruct.Pin = REV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(REV_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_ODO_Pin */
  GPIO_InitStruct.Pin = KEY_ODO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_ODO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : G3_Pin */
  GPIO_InitStruct.Pin = G3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(G3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BKLT_Pin */
  GPIO_InitStruct.Pin = BKLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BKLT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED8_Pin LED7_Pin LED3_Pin BUZZ_Pin */
  GPIO_InitStruct.Pin = LED8_Pin|LED7_Pin|LED3_Pin|BUZZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LED6_Pin */
  GPIO_InitStruct.Pin = LED6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_DATA_Pin SEG_WR_Pin SEG_RD_Pin SEG_CS_Pin */
  GPIO_InitStruct.Pin = SEG_DATA_Pin|SEG_WR_Pin|SEG_RD_Pin|SEG_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */
void eepromWrite(uint16_t addr, uint8_t *dat, uint16_t siz)
{
	uint16_t devAddr = (uint16_t)0xA0;
	osMutexAcquire(mutEepromWriteHandle, osWaitForever);
	if(addr&0xff00)
	{
		devAddr |= 0x02;
	}
	HAL_I2C_Mem_Write(&hi2c1, devAddr, addr, I2C_MEMADD_SIZE_8BIT, dat, siz, 1000);
	osMutexRelease(mutEepromWriteHandle);
}

void eepromRead(uint16_t addr, uint8_t *dat, uint16_t siz)
{
	uint16_t devAddr = (uint16_t)0xA0, TimeOut;
	osMutexAcquire( mutEepromReadHandle, osWaitForever );
	if(addr&0xff00)
	{
		devAddr |= 0x02;
	}
	TimeOut = 0;
	while(HAL_I2C_Mem_Read(&hi2c1, devAddr,(uint16_t)addr, I2C_MEMADD_SIZE_8BIT, dat, siz, 1000)!= HAL_OK && TimeOut < 10)
	{
		TimeOut++;
	}
	osMutexRelease( mutEepromReadHandle );
}

void MEM_Write(unsigned int aa, unsigned long abc)
{
	tBuf[0]=(unsigned char)((abc&0xff000000)>>24);
	tBuf[1]=(unsigned char)((abc&0x00ff0000)>>16);
	tBuf[2]=(unsigned char)((abc&0x0000ff00)>>8);
	tBuf[3]=(unsigned char)(abc&0x000000ff);
	eepromWrite((uint16_t)aa, tBuf, (uint16_t)4);
	//HAL_I2C_Mem_Write(&hi2c1,(uint16_t)0xA0,(uint16_t)aa, I2C_MEMADD_SIZE_8BIT, tBuf,(uint16_t)4,1000);
	//at24_HAL_WriteBytes(&hi2c1, 0xA0, aa, (uint8_t *)&tBuf, 4);
	//write_eeprom(aa, tBuf, 4);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if( huart->Instance == USART1)
	{
		HAL_UART_Receive_IT(&huart1, &uartBuf, 1);
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint32_t IC_Value1 = 0, IC_Value2 = 0, diffTime, varFreq, bitTm[251];
	static uint32_t tmCnt, lTime, hTime, timeOut;
	uint8_t freqMode, freqSynCnt, p;
	static _Bool capEdge = 1;
	_Bool logicBit, freqDone;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	//TIM_GetITStatus(&htim);
	if ((htim->Instance==TIM2)&&(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4))
	  {
		if(freqMode == 1)
		{
			varFreq = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
			__HAL_TIM_SET_COUNTER(htim, 0);
			freqSynCnt=10;
			freqDone = 1;
		}
		else
		{
			if(capEdge==1)
			{
				capEdge=0;
				IC_Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
				if(IC_Value1 < JHANT)
					return;
				hTime = IC_Value1;
				bitTm[tmCnt]=hTime;
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
				__HAL_TIM_SET_COUNTER(htim, 0);
				if(start == 1)
					start = 2;
				if(lTime>hTime)															// logic 0
				{
					diffTime = lTime - hTime;
					if(diffTime>1500)//<<4))//130000)////130000)//
					{
						if(timeOut>0)
							timeOut--;
						dataCountX=0;
						dataCountY=0;
						start=1;
					}
					else
					{
						if((diffTime>(hTime>>1))&&(diffTime<(lTime<<1)))										//serial model
						{
							logicBit = 0;
						}
						else																			//frequency model
						{
							start = 0;
						}
					}
				}
				else																				// logic 1
				{
					diffTime = hTime - lTime;
					if((diffTime>(lTime>>1))&&(diffTime<(hTime<<1)))										//serial model
					{
						logicBit = 1;
					}
					else																			//frequency model
					{
						start = 0;
					}
				}
				if(start==2)
				{
					if(dataCountX<15)
					{
						if(dataCountY<8)
						{
							varCh=dataVar[dataCountX];
							//if(prg ==1)
							{
								if(logicBit)
								{
									varCh|=0x01;
								}
								else
								{
									varCh&=0xfe;
								}
							}
						/*if((dataCountX == 11)&&(dataCountY == 7))
						{
							start=2;
						}*/
							if(dataCountY!=7)
								varCh<<=1;
							dataVar[dataCountX]=varCh;
							dataCountY++;
						}
						if(dataCountY==8)
						{
							dataCountX++;
							dataCountY=0;
						}
					}
					if((dataCountX == 15))//&&(dataCountY==0))
					{
						stopTimeOut = 1;
					}
				}
			}
			else
			{
				capEdge=1;
				IC_Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
				if(IC_Value2 < JHANT)
					return;
				lTime = IC_Value2;
				stopTimeOut = 2;//lTime>>6;
//				stopTimeOut >>= 1;
				bitTm[tmCnt] = lTime;
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
				varFreq = IC_Value2-IC_Value1;
				__HAL_TIM_SET_COUNTER(htim, 0);
			}
			if(tmCnt<250)
			{
				tmCnt++;
			}
			else
			{
				tmCnt=0;
			}
		}
		}
}

/*
 * 	ADC Interrupt handler
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adcData_t adcValues;
	uint32_t val1 = 0, val2 = 0;
    // Conversion Complete & DMA Transfer Complete As Well
    // So The AD_RES Is Now Updated & Let's Move IT To The PWM CCR1
    // Update The PWM Duty Cycle With Latest ADC Conversion Result
	int inx;
	for(int vc=0; vc<25; vc++)
	{
		inx = vc*2;
		val1 += adcBuf[inx];
		val2  += adcBuf[inx+1];
	}
	val1/=25;
	val2/=25;
	if (xQueueSendFromISR(xQueueAdc, &adcValues, NULL) == pdTRUE) {
		// Data sent successfully
	} else {
		// Queue is full, data not sent
	}
}


/* USER CODE BEGIN Header_StartAdcTask */
/**
  * @brief  Function implementing the ADC thread.
  * @param  argument: array of ADC channels
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartAdcTask(void *argument)
{
	adcData_t adcData;
	uint32_t vbatCount, adcSpeed;
  /* Infinite loop */
  for(;;)
  {
	  if (xQueueReceive(xQueueAdc, &adcData, portMAX_DELAY) == pdTRUE) {
		  vbatCount = adcData.adcVal[0];
		  adcSpeed = adcData.adcVal[1];
	  }
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t f1;
	uint16_t adr;
	_Bool autoBaud = 0;
    if(!HAL_GPIO_ReadPin(CAN_MINUS_GPIO_Port, CAN_MINUS_Pin))
	{
	  tBaud = 2;
	  if(!HAL_GPIO_ReadPin(CAN_PLUS_GPIO_Port, CAN_PLUS_Pin))
	  {
		tBaud = 3;
	  }
	}
    else if(!HAL_GPIO_ReadPin(CAN_PLUS_GPIO_Port, CAN_PLUS_Pin))
    {
    	tBaud = 3;
    }
    else
    {
    	eepromRead(ADDR_BAUD, &tBaud, 1);
    	if(tBaud == 0)
	    {
		  autoBaud = 1;
	    }
    	else if(tBaud>4)
    	{
		  tBaud = dBaud;
    	}
    	osDelay(10);
    }
	eepromRead(ADDR_RMUL, (uint8_t *)&rMul, 1);
	  //at24_HAL_ReadBytes(&hi2c1, 0xA0, ADDR_RMUL, (uint8_t *)&rMul, 1);
	  if(rMul==0xff)
	  {
	 	  rMul = 100;
	  }
	  osDelay(10);
	eepromRead(ADDR_RMUL1, (uint8_t *)&rMul1, 1);
	  //at24_HAL_ReadBytes(&hi2c1, 0xA0, ADDR_RMUL1, (uint8_t *)&rMul1, 1);
	  if(rMul1==0xff)
	  {
	 	  rMul1 = 100;
	  }
	  osDelay(10);
	eepromRead(ADDR_RMUL2, (uint8_t *)&rMul2, 1);
	 // at24_HAL_ReadBytes(&hi2c1, 0xA0, ADDR_RMUL2, (uint8_t *)&rMul2, 1);
	  if(rMul2==0xff)
	  {
	 	  rMul2 = 100;
	  }
	  osDelay(10);
	eepromRead(ADDR_RMUL3, (uint8_t *)&rMul3, 1);
	  //at24_HAL_ReadBytes(&hi2c1, 0xA0, ADDR_RMUL3, (uint8_t *)&rMul3, 1);
	  if(rMul3==0xff)
	  {
	 	  rMul3 = 100;
	  }
	  osDelay(10);
	eepromRead(260, tBuf, 15);
	//  at24_HAL_ReadBytes(&hi2c1, 0xA0, 260, tBuf, 15);
	  divFact = tBuf[ADRINX_DIV];
	  //at24_HAL_ReadBytes(&hi2c1, 0xA0, ADDR_DIV, &divFact, 1);
	  if(divFact==0xff)
	  {
		  divFact = 89;
	  }
	  else if(divFact<50)
		  divFact = 50;

	  else if (divFact>200)
	  {
		  divFact = 200;
	  }
	  osDelay(10);
	  tog = tBuf[ADRINX_TOG];
	  //at24_HAL_ReadBytes(&hi2c1, 0xA0, ADDR_TOG, &tog, 1);
	  //tog=c[0];
	  bitOdo=0;
		//at24_HAL_ReadBytes(&hi2c1, 0xA0, ADDR_TOG, &tog, 1);
	  if(tog > 1UL)
	  {
		  tog=0;
		  eepromWrite(ADDR_TOG, (uint8_t *)&tog, 1);
		 // HAL_I2C_Mem_Write(&hi2c1,(uint16_t)0xA0,(uint16_t)ADDR_TOG, I2C_MEMADD_SIZE_8BIT, &tog,(uint16_t)1,1000);
		  //c[0]=0;
	  }
	eeCounter = tBuf[ADRINX_PTR];
	if(eeCounter>31)   //15
	{
		eeCounter=0;
		eepromWrite(ADDR_TOG, (uint8_t *)&eeCounter, 1);
		//HAL_I2C_Mem_Write(&hi2c1,(uint16_t)0xA0,(uint16_t)ADDR_PTR, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&eeCounter,(uint16_t)1,1000);
		//at24_HAL_WriteBytes(&hi2c1, 0xA0, ADDR_PTR, &eeCounter, 1);
		//i2c_write_byte(ADDR_PTR, eeCounter);
	}
	osDelay(10);
	vs = tBuf[ADRINX_VS];//i2c_read_byte(ADDR_VS);
	if(vs==0xff)
		vs=15;
	lval = tBuf[ADRINX_LVAL];//i2c_read_byte(ADDR_LVAL);
	lval<<=8;
	lval+=tBuf[ADRINX_LVAL+1];//i2c_read_byte(ADDR_LVAL+1);
	if(lval==0xffff)
		lval = 450;
	gv1 = tBuf[ADRINX_G1];//i2c_read_byte(ADDR_G1);
	if(gv1==0xff)
		gv1=50;
	else if(gv1>100)
		gv1=100;
	gv2 = tBuf[ADRINX_G2];//i2c_read_byte(ADDR_G2);
	if(gv2==0xff)
		gv2=50;
	else if(gv2>100)
		gv2=100;
	gv3 = tBuf[ADRINX_G3];//i2c_read_byte(ADDR_G3);
	if(gv3==0xff)
		gv3=50;
	else if(gv3>100)
		gv3=100;
	gv4 = tBuf[ADRINX_G4];//i2c_read_byte(ADDR_G4);
	if(gv4==0xff)
		gv4=45;
	else if(gv4>100)
		gv4=100;
	gav1 = tBuf[ADRINX_GA1];//i2c_read_byte(ADDR_GA1);
	if(gav1==0xff)
		gav1=45;
	else if(gav1>150)
		gav1=150;
	gav2 = tBuf[ADRINX_GA2];//i2c_read_byte(ADDR_GA2);
	if(gav2==0xff)
		gav2=100;
	gav3 = tBuf[ADRINX_GA3];//i2c_read_byte(ADDR_GA3);
	if(gav3==0xff)
		gav3=100;
	gav4 = tBuf[ADRINX_GA4];//i2c_read_byte(ADDR_GA4);
	if(gav4==0xff)
		gav4=100;

	eepromRead(ADDR_DIVDIS, (uint8_t *)&divDis, 1);
	//at24_HAL_ReadBytes(&hi2c1, 0xA0, ADDR_DIVDIS, (uint8_t *)&divDis, 1);
	if(divDis==0xff)
		divDis=176;
	else if(divDis<120)
		divDis=120;
	eepromRead(ADDR_VERS, (uint8_t *)&vers, 1);
	//at24_HAL_ReadBytes(&hi2c1, 0xA0, ADDR_VERS, (uint8_t *)&vers, 1);
	if(vers==0xff)
		vers=VERS;
	osDelay(1);

	eepromRead(ADDR_FREQ, tBuf, 1);
	//at24_HAL_ReadBytes(&hi2c1, 0xA0, ADDR_FREQ, tBuf, 1);
	if(tBuf[0]==1)
		modeSet(1);
	else
		modeSet(0);
	osDelay(1);
	eepromRead(ADDR_DivFrq, (uint8_t *)&divFactFrq, 1);
	//at24_HAL_ReadBytes(&hi2c1, 0xA0, ADDR_DivFrq, (uint8_t *)&divFactFrq, 1);
	if(divFactFrq==0xff)
	{
		divFactFrq = 45;
	}
	osDelay(1);
	adr=eeCounter*4;
	eepromRead(adr, tBuf, 4);
	//at24_HAL_ReadBytes(&hi2c1, 0xA0, adr, (uint8_t *)&tBuf, 4);
	//read_eeprom(adr, tBuf, 4);
	odoVar=tBuf[0];
	odoVar<<=8;
	odoVar+=tBuf[1];
	odoVar<<=8;
	odoVar+=tBuf[2];
	odoVar<<=8;
	odoVar+=tBuf[3];
	if(odoVar>9999999)
	{
		odoVar=0;
		MEM_Write(adr, odoVar);
	}
	osDelay(10);
	prvOdo = odoVar;
	//odoVar=(unsigned long)((tBuf[0]<<24)|(tBuf[1]<<16)|(tBuf[2]<<8)|(tBuf[3]));
	adr+=ADDR_TRIP_OFF;
	eepromRead(adr, tBuf, 4);
	//at24_HAL_ReadBytes(&hi2c1, 0xA0, adr, (uint8_t *)&tBuf, 4);
	//read_eeprom(adr, tBuf, 4);
	tripVar=tBuf[0];
	tripVar<<=8;
	tripVar+=tBuf[1];
	tripVar<<=8;
	tripVar+=tBuf[2];
	tripVar<<=8;
	tripVar+=tBuf[3];
	if(tripVar>999999)
	{
		tripVar=0;
		MEM_Write(adr, tripVar);
	}
	osDelay(10);
	if(tog)
		xEventGroupSetBits(xEventGroupDisplay, TRIP_BIT);
	else
		xEventGroupSetBits(xEventGroupDisplay, ODO_BIT);
		//dispOdo(odoVar/10, tog);
  if(autoBaud)
  {
	  uint8_t itr;
	  for( itr = 1; itr < 5; itr++ )
	  {
		  FDCAN_SetBaud(itr);
		  FDCAN1_Config();
		  osDelay(100);
		  if(osSemaphoreAcquire(xSemCanDetectHandle, 3000) == osOK)
		  {
			  break;
		  }
	  }
	  if(itr >= 5)
	  {
		  tBaud = DBAUD;
	  }
	  else
	  {
		  tBaud = itr;
	  }
  }
  FDCAN_SetBaud(tBaud);
  FDCAN1_Config();
  canTaskHandle = osThreadNew(StartCanTask, NULL, &canTask_attributes);
  /* Infinite loop */
  for(;;)
  {
	 // HAL_UART_Transmit(&huart1, (uint8_t *)"Hello", 5, 300);
	if(runTime == 0)
	{
		if((speedVarF>2))//||(spd>2)||(speedVarF>2)||(speedVar2>2))
		{
			if(tripVar<999999)
				tripVar++;
			else
				tripVar=0;
			if(odoVar<9999999)
			{
				odoVar++;
			}
			else
			{
				odoVar=0;
			}
			f1=odoVar/31250;	//35000;
			if(f1>=32)
			{
				unsigned int vptr=f1/32;
				f1=(f1-(vptr*32));
			}
			if(f1!=eeCounter)
			{
				eeCounter=f1;
				//i2c_write_byte(ADDR_PTR, eeC	ounter);
				eepromWrite((uint16_t)ADDR_PTR, (uint8_t *)&eeCounter, (uint16_t)1);
				//HAL_I2C_Mem_Write(&hi2c1,(uint16_t)0xA0,(uint16_t)ADDR_PTR, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&eeCounter,(uint16_t)1,1000);
				//at24_HAL_WriteBytes(&hi2c1, 0xA0, ADDR_PTR, &eeCounter, 1);
			}
			eeCounter = 0;
				//dispVolt(eeCounter);
			adr=eeCounter*4;
			osDelay(5);
			MEM_Write(adr, odoVar);
			osDelay(5);
			adr+=ADDR_TRIP_OFF;
			MEM_Write(adr, tripVar);
			osDelay(5);
			//rangeInit++;
			prvOdo = odoVar;
			if(tog)
			{
				xEventGroupSetBits(xEventGroupDisplay, TRIP_BIT);
			}
			else
			{
				xEventGroupSetBits(xEventGroupDisplay, ODO_BIT);
			}
		}
		runTime=1;
	}
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartGpioTask */
/**
* @brief Function implementing the myGpioTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGpioTask */
void StartGpioTask(void *argument)
{
  /* USER CODE BEGIN StartGpioTask */
  /* Infinite loop */
  for(;;)
  {
    if(ulTaskNotifyTake(pdTRUE, 500))
    {											//short press
    	if(keyNo == SHORT)											//short press
    	{
    		keyNo=0;
    		if(tog)
    		{
    			tog = 0;
    			xEventGroupSetBits(xEventGroupDisplay, ODO_BIT);
    		}
    		else
    		{
    			tog = 1;
    			xEventGroupSetBits(xEventGroupDisplay, TRIP_BIT);
    		}
    		/*do{
    			//TM1629_ReadKey(5,key_buf);
    		osDelay(10);
    		}
    		while(!HAL_GPIO_ReadPin(KEY_ODO_GPIO_Port, KEY_ODO_Pin));*/
    		eepromWrite(ADDR_TOG, (uint8_t *)&tog, 1);
    		//HAL_I2C_Mem_Write(&hi2c1,(uint16_t)0xA0,(uint16_t)ADDR_TOG, I2C_MEMADD_SIZE_8BIT, &tog,(uint16_t)1,1000);
    		osDelay(10);
    		//odoSwVal=1;
    		//canSync=30;
    		//itr=5;
    	}
    	//else if(chr==2)									//Long Press
    	else if(keyNo == LONG)									//Long Press
    	{
    		keyNo=0;
    		tripVar=0;
    		tog=1;
    		xEventGroupSetBits(xEventGroupDisplay, TRIP_BIT);
    		//while(!HAL_GPIO_ReadPin(KEY_ODO_GPIO_Port, KEY_ODO_Pin));
    		uint16_t adr = eeCounter*4;
    		adr+=ADDR_TRIP_OFF;
    		MEM_Write(adr,tripVar);
    		//odoSwVal=2;
    		//canSync=30;
    		//itr=5;
    		/*do
    		{
    			//TM1629_ReadKey(5,key_buf);
    			osDelay(1000);
    		}while(kbhit());*/
    	}
    }
    osDelay(1);
  }
  /* USER CODE END StartGpioTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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

