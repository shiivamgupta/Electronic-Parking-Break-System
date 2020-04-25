/*Electronic Parking Brake System*/

/*Headers*/
#include "main.h"
#include <math.h>
#include "cmsis_os.h"
#include "LIS3DSH.h"
#include "task.h"
#include "semphr.h"
#include "string.h"

/*Macros*/
#define PI 3.147


/* Private variables*/
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;

LIS3DSH_InitTypeDef myAccConfigDef;
LIS3DSH_DataScaled myData;

//for non blocking delays
TickType_t tickstowait = 100;
TimeOut_t timeout;

SemaphoreHandle_t xSemaphore;

//CAN related variables
CAN_HandleTypeDef hcan1;

CAN_FilterTypeDef filter1;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

char TxData[8];
uint8_t RxData[8];
uint32_t TxMbNum;


/* Definitions for Task01 */
osThreadId_t Task01Handle;
const osThreadAttr_t Task01_attributes = {
  .name = "Task01",
  .priority = (osPriority_t) osPriorityAboveNormal1,
  .stack_size = 240
};
/* Definitions for Task02 */
osThreadId_t Task02Handle;
const osThreadAttr_t Task02_attributes = {
  .name = "Task02",
  .priority = (osPriority_t) osPriorityAboveNormal2,
  .stack_size = 240
};

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
void StartTask01(void *argument);
void StartTask02(void *argument);

static void CAN1_Filter_Config(void);
static void Can_Tx(void);

static void Acc_config(void);
void Sampling_ACC(void);
static void Intensity_Cal(void);


int main(void)
{
  /*System Initialization*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  /*Accelerometer specific configurations*/
  Acc_config();
  LIS3DSH_Init(&hspi1, &myAccConfigDef);
  /*CAN specific configurations*/
  CAN1_Filter_Config();
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//receive it

  /* Init scheduler */
  osKernelInitialize();

  /*Semaphore for synchronization*/
  xSemaphore = xSemaphoreCreateCounting(2,0);

  /* Create the thread(s) */
  /* creation of Task01 */
  Task01Handle = osThreadNew(StartTask01, NULL, &Task01_attributes);

  /* creation of Task02 */
  Task02Handle = osThreadNew(StartTask02, NULL, &Task02_attributes);

  /* Start scheduler */
  osKernelStart();
 
  while (1);
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_3TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_7TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/*Accelerometer configurations and Initialisation*/
static void Acc_config(void){
	myAccConfigDef.dataRate = LIS3DSH_DATARATE_12_5;
	myAccConfigDef.antiAliasingBW = LIS3DSH_FILTER_BW_50;
	myAccConfigDef.fullScale = LIS3DSH_FULLSCALE_4;
	myAccConfigDef.enableAxes = LIS3DSH_XYZ_ENABLE;
	myAccConfigDef.interruptEnable = false;
	LIS3DSH_X_calibrate(-1020, 1020);
	LIS3DSH_Y_calibrate(-1020, 1020);
	LIS3DSH_Z_calibrate(-1020, 1020);
	LIS3DSH_Init(&hspi1, &myAccConfigDef);
}

/*Can receivers filter configuration*/
static void CAN1_Filter_Config(void){
	filter1.FilterActivation	= ENABLE;
	filter1.FilterBank			= 14;
	filter1.FilterFIFOAssignment= CAN_FILTER_FIFO0;
	filter1.FilterIdHigh		= 0x0000;
	filter1.FilterIdLow			= 0x0000;
	filter1.FilterMaskIdHigh	= 0x0000;
	filter1.FilterMaskIdLow		= 0x0000;
	filter1.FilterMode			= CAN_FILTERMODE_IDMASK;
	filter1.FilterScale			= CAN_FILTERSCALE_32BIT;
	filter1.SlaveStartFilterBank= 0;

	HAL_CAN_ConfigFilter(&hcan1, &filter1);
}

/*Can transmit method*/
static void Can_Tx(void){

	TxHeader.DLC		= 8;
	TxHeader.StdId		= 0X600;
	TxHeader.IDE		= CAN_ID_STD;
	TxHeader.RTR		= CAN_RTR_DATA;

	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (uint8_t*)TxData, &TxMbNum);

	while(HAL_CAN_IsTxMessagePending(&hcan1, TxMbNum));

	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
}

/*button debouncing code*/
static int buttonDebounce(void)
{
	static uint16_t buttonState = 0;
	uint8_t pinState = 0;
	pinState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	buttonState = ( ( buttonState << 1 ) | !pinState | 0xE000 );
	if(buttonState == 0xF000){
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
		return true;
	}
	return false;
}

/*Intensity Calculator function*/
double ang_x,ang_y,ang_z;
char intensity;
char tire[6];
void Intensity_Cal(void){
	/*angle of inclination on all the x ,y and z axis*/
	ang_x = atan(myData.x/(sqrt(myData.y * myData.y + myData.z * myData.z))) * 180.0 /PI;
	ang_y = atan(myData.y/(sqrt(myData.x * myData.x + myData.z * myData.z))) * 180.0 /PI;
	ang_z = atan(myData.z/(sqrt(myData.x * myData.x + myData.y * myData.y))) * 180.0 /PI;

	/*formula for calculating the intensity of the brake */
	intensity = sqrt((ang_x + ang_y)*(ang_x + ang_y) + (ang_x - ang_y)*(ang_x - ang_y));

	/*determining the correct tires to apply proper brake*/
	if(ang_x > -1 && ang_x < 1 && ang_y<1 && ang_y>-27&&ang_z <80){
		strcpy(tire,"FRONT");
	}else if(ang_x > -1 && ang_x < 1 && ang_y<27 && ang_y>-1 && ang_z <80){
		strcpy(tire,"BACK_");
	}else if(ang_y > -1 && ang_y < 1 && ang_x<1 && ang_x>-27 && ang_z <80){
		strcpy(tire,"RIGHT");
	}else if(ang_y > -1 && ang_y < 1 && ang_x<27 && ang_x>-1 && ang_z <80){
		strcpy(tire,"LEFT_");
	}else if((ang_x >2 && ang_y>2) || (ang_x <-2 && ang_y<-2)){
		strcpy(tire,"DILTR");
	}else if((ang_x <-2 && ang_y>2) || (ang_x >2 && ang_y<-2)){
		strcpy(tire,"DIRTL");
	}else if(ang_y > -1 && ang_y < 1 && ang_x<1 && ang_x>-1 && ang_z >80){
		strcpy(tire,"STABL");
	}
	strcpy(TxData,tire);
	TxData[6] = intensity;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask01 */
/**
  * @brief  Function implementing the Task01 thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartTask01 */
void StartTask01(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    if(buttonDebounce() == true){
    	xSemaphoreGive(xSemaphore);
    }
    vTaskDelay(2);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Task02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	  xSemaphoreTake(xSemaphore,portMAX_DELAY);
	  while(1){
	  	if(LIS3DSH_PollDRDY(100)==true){
	  		  myData = LIS3DSH_GetDataScaled();
	  		  break;
	  	}
	  }
	  Intensity_Cal();
	  Can_Tx();
	  //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
  }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
