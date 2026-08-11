#include "drv_pwm.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_tim.h"
#include <cstdint>

/**
 * @brief 初始化pwm的定时器
 * 
 * @param htim 
 * @param Channel 
 */
HAL_StatusTypeDef PWM_Init(TIM_HandleTypeDef *htim,uint32_t Channel)
{
    HAL_StatusTypeDef ret = HAL_TIM_PWM_Start(htim, Channel);
    return ret;
}


void PWM_Set_Duty(TIM_HandleTypeDef *htim ,uint32_t Channel ,uint32_t Duty)
{
    __HAL_TIM_SET_COMPARE(htim, Channel, Duty);
}

