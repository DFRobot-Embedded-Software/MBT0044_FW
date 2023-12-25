#include "DFRobot_Motordriver.h"
#if 1

//GPIO引脚初始化
void PWM_gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  //PD4 
  GPIO_InitStruct.Pin = GPIO_PIN_4;  
  GPIO_InitStruct.Mode = GPIO_MODE_AF;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.OpenDrain = GPIO_PUSHPULL;  
  GPIO_InitStruct.Debounce.Enable = GPIO_DEBOUNCE_DISABLE;
  GPIO_InitStruct.SlewRate = GPIO_SLEW_RATE_HIGH;
  GPIO_InitStruct.DrvStrength = GPIO_DRV_STRENGTH_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1_CH1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);    
  
  //PD6 
  GPIO_InitStruct.Pin = GPIO_PIN_6;  
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1_CH2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);    

  //PA2
  GPIO_InitStruct.Pin = GPIO_PIN_2;  
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1_CH3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);    

  //PC4
  GPIO_InitStruct.Pin = GPIO_PIN_4;  
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1_CH4;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);  

  //PD5
  GPIO_InitStruct.Pin = GPIO_PIN_5;  
  GPIO_InitStruct.Alternate = GPIO_AF8_TIM2_CH4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);  

  //PB5
  GPIO_InitStruct.Pin = GPIO_PIN_5;  
  GPIO_InitStruct.Alternate = GPIO_AF8_TIM2_CH1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);  

  //PC3
  GPIO_InitStruct.Pin = GPIO_PIN_3;  
  GPIO_InitStruct.Alternate = GPIO_AF8_TIM2_CH3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);  

  //PD3
  GPIO_InitStruct.Pin = GPIO_PIN_3;  
  GPIO_InitStruct.Alternate = GPIO_AF8_TIM2_CH2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct); 

  //PA3
  GPIO_InitStruct.Pin = GPIO_PIN_3;  
  GPIO_InitStruct.Alternate = GPIO_AF2_PCA_CH2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);  

  //PC5
  GPIO_InitStruct.Pin = GPIO_PIN_5;  
  GPIO_InitStruct.Alternate = GPIO_AF2_PCA_CH0;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  //PC6
  GPIO_InitStruct.Pin = GPIO_PIN_6;  
  GPIO_InitStruct.Alternate = GPIO_AF2_PCA_CH3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  //PC7
  GPIO_InitStruct.Pin = GPIO_PIN_7;  
  GPIO_InitStruct.Alternate = GPIO_AF2_PCA_CH4;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);



  
  HAL_Delay(1000);

}

//timer配置PWM结构体
TIM_HandleTypeDef sTim1Handle = {0};
TIM_HandleTypeDef sTim2Handle = {0};
TIM_OC_InitTypeDef sTimxOcInitHandle = {0};

extern PCA_HandleTypeDef	sPcaHandle; 
extern PCA_OC_InitTypeDef sPcaOcInit;

/**
 * @brief 这里是对输出脉冲定时器的配置
 * 
 */
void TimerChannelInit(void)
{
  __HAL_RCC_TIM1_CLK_ENABLE();
  sTim1Handle.Instance = TIM1;
  sTim1Handle.Init.Period            = 1000; //计数周期为1ms
  sTim1Handle.Init.Prescaler         = 7; //外部时钟8M,8分频得到1M的时钟
  sTim1Handle.Init.ClockDivision     = 0; 
  sTim1Handle.Init.CounterMode       = TIM_COUNTERMODE_UP; 
  sTim1Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; 

  if (HAL_TIM_PWM_Init(&sTim1Handle) != HAL_OK){
    Error_Handler();
  }
  sTimxOcInitHandle.OCMode = TIM_OCMODE_PWM1; //PWM模式1
  sTimxOcInitHandle.OCPolarity = TIM_OCPOLARITY_HIGH; //输出极性
  sTimxOcInitHandle.OCNPolarity = TIM_OCNPOLARITY_HIGH; //OC1N低电平有效
  sTimxOcInitHandle.OCFastMode = TIM_OCFAST_ENABLE; //输出比较‘0’使能
  sTimxOcInitHandle.OCIdleState = TIM_OCIDLESTATE_RESET; //MOE==0时，如果实现了OC1N，则死区后OC1==0
  sTimxOcInitHandle.OCNIdleState = TIM_OCNIDLESTATE_RESET; //MOE==0时，死区后OC1N==0
  HAL_NVIC_EnableIRQ(TIM1_IRQn);

  __HAL_RCC_TIM2_CLK_ENABLE();
  sTim2Handle.Instance = TIM2;
  sTim2Handle.Init.Period            = 20000; //计数周期为20ms
  sTim2Handle.Init.Prescaler         = 7; //外部时钟8M,8分频得到1M的时钟
  sTim2Handle.Init.ClockDivision     = 0; 
  sTim2Handle.Init.CounterMode       = TIM_COUNTERMODE_UP; 
  sTim2Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; 
	
  if (HAL_TIM_PWM_Init(&sTim2Handle) != HAL_OK){
    Error_Handler();
  }
  sTimxOcInitHandle.OCMode = TIM_OCMODE_PWM1; //PWM模式1
  sTimxOcInitHandle.OCPolarity = TIM_OCPOLARITY_HIGH; //输出极性
  sTimxOcInitHandle.OCNPolarity = TIM_OCNPOLARITY_HIGH; //OC1N低电平有效
  sTimxOcInitHandle.OCFastMode = TIM_OCFAST_ENABLE; //输出比较‘0’使能
  sTimxOcInitHandle.OCIdleState = TIM_OCIDLESTATE_RESET; //MOE==0时，如果实现了OC1N，则死区后OC1==0
  sTimxOcInitHandle.OCNIdleState = TIM_OCNIDLESTATE_RESET; //MOE==0时，死区后OC1N==0
  HAL_NVIC_EnableIRQ(TIM2_IRQn);


  sTimxOcInitHandle.Pulse = 0;
  if(HAL_OK !=(HAL_TIM_PWM_ConfigChannel(&sTim1Handle,&sTimxOcInitHandle, TIM_CHANNEL_4))){
    Error_Handler();
  }
  if(HAL_OK !=(HAL_TIM_PWM_ConfigChannel(&sTim1Handle,&sTimxOcInitHandle, TIM_CHANNEL_3))){
    Error_Handler();
  }
  if(HAL_OK !=(HAL_TIM_PWM_ConfigChannel(&sTim1Handle,&sTimxOcInitHandle, TIM_CHANNEL_2))){
    Error_Handler();
  }
  if(HAL_OK !=(HAL_TIM_PWM_ConfigChannel(&sTim1Handle,&sTimxOcInitHandle, TIM_CHANNEL_1))){
    Error_Handler();
  }    
	HAL_TIM_PWM_Start_IT(&sTim1Handle,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start_IT(&sTim1Handle,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start_IT(&sTim1Handle,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start_IT(&sTim1Handle,TIM_CHANNEL_1);
  
  
  if(HAL_OK !=(HAL_TIM_PWM_ConfigChannel(&sTim2Handle,&sTimxOcInitHandle, TIM_CHANNEL_4))){
    Error_Handler();
  }   
  if(HAL_OK !=(HAL_TIM_PWM_ConfigChannel(&sTim2Handle,&sTimxOcInitHandle, TIM_CHANNEL_3))){
    Error_Handler();
  } 
  if(HAL_OK !=(HAL_TIM_PWM_ConfigChannel(&sTim2Handle,&sTimxOcInitHandle, TIM_CHANNEL_2))){
    Error_Handler();
  } 
  if(HAL_OK !=(HAL_TIM_PWM_ConfigChannel(&sTim2Handle,&sTimxOcInitHandle, TIM_CHANNEL_1))){
    Error_Handler();
  }    
	HAL_TIM_PWM_Start_IT(&sTim2Handle,TIM_CHANNEL_4);
  HAL_TIM_PWM_Start_IT(&sTim2Handle,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start_IT(&sTim2Handle,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start_IT(&sTim2Handle,TIM_CHANNEL_1);

  /*********************** Configure PCA clock source  *******************************/	
  /* Initialize LPTIMx peripheral as follows:
       + Clock = PCLK/8 = 8M/8 = 1MHz
  */		
	sPcaHandle.Instance = PCAx;
	sPcaHandle.Init.ClkSrcSel = PCA_CLOCK_SOURCE_PCLKDIV32;
	sPcaHandle.Init.RunInIdleMode = PCA_IDLEMODE_DISABLE;
	HAL_PCA_OC_Init(&sPcaHandle);

  sPcaOcInit.CompareEnable = PCA_OC_ENABLE;
	sPcaOcInit.PwmEnable = PCA_PWM_ENABLE;
	/* 50% duty cycle, period = (255*2)*(1/1M) = 51ms */
	sPcaOcInit.Period = 255;		
	HAL_PCA_OC_ConfigChannel(&sPcaHandle, &sPcaOcInit, PCA_CHANNEL_0);
  /* PCA output compare enable */	
	HAL_PCA_OC_Start(&sPcaHandle, PCA_CHANNEL_0);

  HAL_PCA_OC_ConfigChannel(&sPcaHandle, &sPcaOcInit, PCA_CHANNEL_2);
  /* PCA output compare enable */	
	HAL_PCA_OC_Start(&sPcaHandle, PCA_CHANNEL_2);

  HAL_PCA_OC_ConfigChannel(&sPcaHandle, &sPcaOcInit, PCA_CHANNEL_3);
  /* PCA output compare enable */	
	HAL_PCA_OC_Start(&sPcaHandle, PCA_CHANNEL_3);

  HAL_PCA_OC_ConfigChannel(&sPcaHandle, &sPcaOcInit, PCA_CHANNEL_4);
  /* PCA output compare enable */	
	HAL_PCA_OC_Start(&sPcaHandle, PCA_CHANNEL_4);
}

extern uint8_t motor_register[20];
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
  
  if(htim->Instance==TIM1){//电机
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)//M2B
    {
      //M1电机正转
      if(motor_register[2] == MOTOR_FOREWARD){
        TIM1->CCR2 = 0;
      //M1电机反转
      }else if(motor_register[2] == MOTOR_REVERSE){
        TIM1->CCR2 = motor_register[3]*4;
      }
    }

    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)//M2A
    {
      //M1电机正转
      if(motor_register[2] == MOTOR_FOREWARD){
				
        TIM1->CCR1 = motor_register[3]*4;
      //M1电机反转
      }else if(motor_register[2] == MOTOR_REVERSE){
        TIM1->CCR1 = 0;
      }
    }

    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)//M1B
    {
      //M1电机正转
      if(motor_register[0] == MOTOR_FOREWARD){
				TIM1->CCR4 = 0;
        
      //M1电机反转
      }else if(motor_register[0] == MOTOR_REVERSE){
        TIM1->CCR4 = motor_register[1]*4;
      }
    }

    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)//M1A
    {
      //M1电机正转
      if(motor_register[0] == MOTOR_FOREWARD){
        
        TIM1->CCR3 = motor_register[1]*4;
      //M1电机反转
      }else if(motor_register[0] == MOTOR_REVERSE){
        TIM1->CCR3 = 0;
      }
    }

  }
  if(htim->Instance==TIM2){//舵机
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)//S1
    {
      uint16_t data = motor_register[8] << 8 | motor_register[9];
      TIM2->CCR4 = data;
    }
  
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)//S2
    {
      uint16_t data = motor_register[10] << 8 | motor_register[11];
      TIM2->CCR2 = data;
    }

    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)//S4
    {
        uint16_t data = motor_register[14] << 8 | motor_register[15];
        TIM2->CCR1 = data;
    }

    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)//S3
    {
        uint16_t data = motor_register[12] << 8 | motor_register[13];
        TIM2->CCR3 = data;
    }
  }
}


// /**
//   * @brief  Overflow callback in non blocking mode 
//   * @param  hpca : PCA handle
//   * @retval None
//   */
// void HAL_PCA_OverflowCallback(PCA_HandleTypeDef *hpca)
// {
  
	
// }


#endif
