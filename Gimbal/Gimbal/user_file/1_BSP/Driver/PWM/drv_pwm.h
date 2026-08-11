#ifndef __DRV_PWM_H__
#define __DRV_PWM_H__
#include "main.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_tim.h"


HAL_StatusTypeDef PWM_Init(TIM_HandleTypeDef *htim,uint32_t Channel);

void PWM_Set_Duty(TIM_HandleTypeDef *htim ,uint32_t Channel ,uint32_t Duty);

#endif /*__DRV_PWM_H__*/