/**
  ******************************************************************************
  * @file    main.c
	* @author  MCU Software Team
	* @Version V1.0.0
  * @Date    21-Oct-2019
  * @brief   main function
  ******************************************************************************
  * This example is used to test uart0,when interrupt received data reaches 10, 
  * it is sent out from serial port.
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"	

/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint8_t *RTU_UART_Data = NULL;
uint64_t count_Us;


//static uint8_t ucRxCompleteFlag = 0;
volatile uint8_t receData;
volatile uint8_t RecIndexLen = 0;

/* Private macros-------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/
UART_HandleTypeDef sUartxHandle = {0};
GPIO_InitTypeDef GPIO_InitStruct1 = {0};
extern uint8_t motor_register[20];



/* Public variables ---------------------------------------------------------*/
PCA_HandleTypeDef	sPcaHandle = {0}; 
PCA_OC_InitTypeDef sPcaOcInit = {0};

void RCC_DelayF(uint32_t mdelay)
{
  __IO uint32_t Delay = mdelay * (24000000 / 8U / 1000000U);
  do 
  {
    __NOP();
  } 
  while (Delay --);
}


static void pwm_init(void)
{
	PWM_gpio_init();
	TimerChannelInit();
}
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();		
	
  /* Configure the system clock to HIRC 24MHz*/
  SystemClock_Config();
			
  /* Peripheral clock enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_I2C_CLK_ENABLE();
  //__HAL_RCC_UART1_CLK_ENABLE();
  __HAL_RCC_PCA_CLK_ENABLE();
  __HAL_RCC_ADC_CLK_ENABLE();
  //HAL_Delay(1000);

  RCC->UNLOCK = RCC_UNLOCK_UNLOCK | (0x2AD5334C << RCC_UNLOCK_KEY_Pos);
	RCC->SWDIOCR = RCC_SWDIOCR_SWDPORT_Pos | (0x5A69 << RCC_SWDIOCR_KEY_Pos);
	RCC->UNLOCK = (0x2AD5334C << RCC_UNLOCK_KEY_Pos) & RCC_UNLOCK_KEY;	


	//UartInit();
  //printf("data");
  i2cInitSlave();
  i2cIRQConfig();
  pwm_init();
  adc_init();
  // printf("aa\n");
  while (1){
  // if(motor_register[6] == MOTOR_FOREWARD){//M4B MOTOR_REVERSE
  //   PCAx->CCAP0H = 255 - motor_register[7];
  //   PCAx->CCAP2H = 255;
  // }else if(motor_register[6] == MOTOR_REVERSE){//M4A MOTOR_FOREWARD
  //   PCAx->CCAP0H = 255;
  //   PCAx->CCAP2H = 255 - motor_register[7];
  // }
  // if(motor_register[4] == MOTOR_REVERSE){//M3A MOTOR_FOREWARD
  //   PCAx->CCAP3H = 255 - motor_register[5]; 
  //   PCAx->CCAP4H = 255;
  // }else if(motor_register[4] == MOTOR_FOREWARD){//M3B MOTOR_REVERSE
  //   PCAx->CCAP3H = 255;
  //   PCAx->CCAP4H = 255 - motor_register[5];
  // }

  if(motor_register[6] == MOTOR_FOREWARD){//M4B MOTOR_REVERSE
    PCAx->CCAP0H = 255;
    PCAx->CCAP2H = 255 - motor_register[7];
  }else if(motor_register[6] == MOTOR_REVERSE){//M4A MOTOR_FOREWARD
    PCAx->CCAP0H = 255 - motor_register[7];
    PCAx->CCAP2H = 255;
  }
  if(motor_register[4] == MOTOR_REVERSE){//M3A MOTOR_FOREWARD
    PCAx->CCAP3H = 255;
    PCAx->CCAP4H = 255 - motor_register[5];
  }else if(motor_register[4] == MOTOR_FOREWARD){//M3B MOTOR_REVERSE
    PCAx->CCAP3H = 255 - motor_register[5]; 
    PCAx->CCAP4H = 255;
  }
   HAL_Delay(10);
	}
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};	
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HIRC;
  RCC_OscInitStruct.HIRCState = RCC_HIRC_ON;
  RCC_OscInitStruct.HIRCCalibrationValue = RCC_HIRCCALIBRATION_8M;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
	
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HIRC;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APBCLKDivider = RCC_PCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }	
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1);
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


/* Private function -------------------------------------------------------*/



