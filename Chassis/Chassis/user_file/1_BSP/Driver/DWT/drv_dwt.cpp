#include "drv_dwt.h"

/*
 * Cortex-M7 DWT Lock Access Register
 *
 * 某些 Cortex-M7 芯片上，DWT 寄存器可能处于锁定状态，
 * 写入解锁密钥可以提高兼容性。
 */
#define DWT_LAR_ADDRESS       (0xE0001FB0L)
#define DWT_LAR              (*(volatile uint32_t *)DWT_LAR_ADDRESS)
#define DWT_UNLOCK_KEY       (0xC5ACCE55UL)

bool DWT_Init(void)
{
    /*
     * TRCENA：开启跟踪组件，包括 DWT、ITM 等。
     */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /*
     * 解锁 Cortex-M7 的 DWT。
     */
    DWT_LAR = DWT_UNLOCK_KEY;

    /*
     * 先关闭周期计数器，再清零。
     */
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;

    /*
     * 开启周期计数器。
     */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /*
     * 确保前面的寄存器操作完成。
     */
    __DSB();
    __ISB();

    /*
     * 简单检查使能位。
     */
    return (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0;
}

/**
 * @brief 获取 DWT 周期计数器当前值
 * @return 当前周期计数值
 */
uint32_t DWT_GetCycle(void)
{
    return DWT->CYCCNT;
}

uint32_t DWT_GetElapsedCycle(uint32_t start_cycle)
{
    /*
     * uint32_t 无符号减法可以自动处理一次计数器溢出。
     */
    return DWT->CYCCNT - start_cycle;
}

/**
 * @brief 将周期计数转换为微秒
 * @param cycles 周期计数值
 * @return 对应的微秒数
 */
float DWT_CycleToUs(uint32_t cycles)
{
    return ((float)cycles * 1000000.0f) /
           (float)SystemCoreClock;
}

/**
 * @brief 获取从 start_cycle 到当前经过的微秒数
 * @param start_cycle 起始周期计数值
 * @return 经过的微秒数
 */
float DWT_GetElapsedUs(uint32_t start_cycle)
{
    return DWT_CycleToUs(DWT_GetElapsedCycle(start_cycle));
}

void DWT_DelayUs(uint32_t us)
{
    /*
     * 每次最多延时 1 秒，避免等待周期数超过 uint32_t。
     */
    while (us > 0)
    {
        const uint32_t chunk_us =
            (us > 1000000) ? 1000000 : us;

        /*
         * 向上取整，防止实际延时略短。
         */
        const uint32_t wait_cycles =
            (uint32_t)((((uint64_t)SystemCoreClock * chunk_us)
                        + 999999ULL) /
                       1000000LL);

        const uint32_t start_cycle = DWT->CYCCNT;

        while ((uint32_t)(DWT->CYCCNT - start_cycle) <
               wait_cycles)
        {
            /*
             * 忙等待。
             */
        }

        us -= chunk_us;
    }
}