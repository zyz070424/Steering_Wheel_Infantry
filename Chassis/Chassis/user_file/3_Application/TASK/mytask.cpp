#include "mytask.h"

#include "FreeRTOS.h"
#include "app_config.h"
#include "task.h"

namespace
{
constexpr TickType_t CONTROL_TASK_PERIOD_TICKS = pdMS_TO_TICKS(1U);
constexpr float CONTROL_TASK_PERIOD_S = 0.001f;
}

/**
 * @brief FreeRTOS 控制主任务入口
 *
 * 在任务启动时调用 App_Config_Init() 完成全局初始化，
 * 之后以 1ms 周期循环调用 App_Control_Update()。
 *
 * @param argument FreeRTOS 任务参数（未使用）
 */
extern "C" void App_Control_Task(void *argument)
{
    (void)argument;

    App_Config_Init();

    TickType_t last_wake_tick = xTaskGetTickCount();

    for (;;)
    {
        App_Control_Update(CONTROL_TASK_PERIOD_S);
        xTaskDelayUntil(&last_wake_tick, CONTROL_TASK_PERIOD_TICKS);
    }
}
