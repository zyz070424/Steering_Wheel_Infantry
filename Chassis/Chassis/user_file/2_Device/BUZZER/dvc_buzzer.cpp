#include "dvc_buzzer.h"
#include "stm32h7xx_hal_tim.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

/**
 * @brief 蜂鸣器开机提示音：DO-RE-SO 三音节播放
 *
 * 在任务调度开始之前的初始任务中调用。
 */
void Buzzer(void)
{
    PWM_Init(&htim1, TIM_CHANNEL_1);
    
    PWM_Set_Duty(&htim1, TIM_CHANNEL_1, DO);
    vTaskDelay(pdMS_TO_TICKS(250));
    PWM_Set_Duty(&htim1, TIM_CHANNEL_1, RE);
    vTaskDelay(pdMS_TO_TICKS(250));
    PWM_Set_Duty(&htim1, TIM_CHANNEL_1, SO);
    vTaskDelay(pdMS_TO_TICKS(250));
    PWM_Set_Duty(&htim1, TIM_CHANNEL_1, stop);






    
}   