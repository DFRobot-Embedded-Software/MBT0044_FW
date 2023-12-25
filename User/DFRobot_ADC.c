#include "DFRobot_ADC.h"

ADC_HandleTypeDef						sAdcHandle = {0};
ADC_ThresholdConfTypeDef 		sAdcThreasHoldInit;
extern uint8_t motor_register[20];
void adc_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // adc l 和 ADC r 端口的电源使能
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;      // 输出模式
  GPIO_InitStruct.OpenDrain = GPIO_OPENDRAIN;    // 推挽输出
  GPIO_InitStruct.Debounce.Enable = GPIO_DEBOUNCE_DISABLE;  // 禁止输入去抖动
  GPIO_InitStruct.SlewRate = GPIO_SLEW_RATE_HIGH;           // 电压转换速率
  GPIO_InitStruct.DrvStrength = GPIO_DRV_STRENGTH_HIGH;     // 驱动强度
  GPIO_InitStruct.Pull = GPIO_NOPULL;                       // 无上下拉
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  /* Configure ADC:
  *						-- continuous mode
  *						-- select channel3 and channel4
  *						-- set ADC_NUMBER
  *						-- software start ADC conversion
  *						-- Channel4 and Channel7 will be converted alternately
  */
  sAdcHandle.Instance = ADCx;
  sAdcHandle.Init.SamplingTime = ADC_SAMPLE_4CYCLE; // 采样周期
  sAdcHandle.Init.ClkSel = ADC_CLOCK_PCLK_DIV128; // ADC时钟分频
  sAdcHandle.Init.SingleContinueMode = ADC_MODE_CONTINUE; // 连续转换模式 
  sAdcHandle.Init.ContinueChannelSel = ADC_CONTINUE_CHANNEL_3; // 多通道选择
  sAdcHandle.Init.NbrOfConversion = CONVERSION_NUM;  // 连续转换次数 ADC_NUMBER
  sAdcHandle.Init.AutoAccumulation = ADC_AUTOACC_DISABLE; // 禁止ADC转换结果自动累加
  sAdcHandle.Init.CircleMode = ADC_MULTICHANNEL_CIRCLE;  // 禁止ADC循环转换模式
  sAdcHandle.Init.ExternalTrigConv1 = ADC_SOFTWARE_START; // 禁用自动触发ADC转换 
  HAL_ADC_Init(&sAdcHandle);
  
  sAdcThreasHoldInit.ITMode = DISABLE;
  sAdcThreasHoldInit.CompareMode = ADC_COMP_THRESHOLD_NONE; // 禁止 ADC比较中断控制
  HAL_ADC_ThresholdConfig(&sAdcHandle, &sAdcThreasHoldInit);
  
  HAL_ADC_Start_IT(&sAdcHandle);  // 启动ADC中断
  HAL_NVIC_EnableIRQ(ADC_IRQn);   // 使能ADC中断
}

/**
  * @brief  Continuous mode channel7 conversion complete callback in non blocking mode 
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_MultiChannel3_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  uint16_t adcData=0;
  adcData = HAL_ADC_GetValue(&sAdcHandle, ADC_CONTINUE_CHANNEL_3);
  motor_register[16] = (adcData >> 8) & 0xff;
  motor_register[17] = adcData & 0xff;
}