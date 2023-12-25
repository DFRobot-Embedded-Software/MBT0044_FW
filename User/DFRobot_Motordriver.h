/* Includes ------------------------------------------------------------------*/
#include "cs32l010_hal.h"


/* Exported constants ---------------------------------------------------------*/
#define PCAx													PCA

//电机状态宏定义
#define MOTOR_REVERSE  0X01//反转
#define MOTOR_FOREWARD  0X00//正转

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

extern void PWM_init(void);
extern void set_MX_direct(uint8_t motor_num,uint8_t direct,uint8_t speed);

extern void PWM_gpio_init(void);
extern void TimerChannelInit(void);

