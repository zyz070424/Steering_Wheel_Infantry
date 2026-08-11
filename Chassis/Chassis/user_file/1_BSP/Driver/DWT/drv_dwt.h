#ifndef DRV_DWT_H
#define DRV_DWT_H
#define __DRV_DWT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 初始化 DWT 周期计数器
 *
 * @return true  初始化成功
 * @return false 初始化失败
 */
bool DWT_Init(void);

/**
 * @brief 获取当前 CPU 周期计数
 */
uint32_t DWT_GetCycle(void);

/**
 * @brief 计算从 start_cycle 到当前经过的周期数
 *
 * 即使 CYCCNT 发生一次溢出，该计算仍然有效。
 */
uint32_t DWT_GetElapsedCycle(uint32_t start_cycle);

/**
 * @brief 将 CPU 周期数转换为微秒
 */
float DWT_CycleToUs(uint32_t cycles);

/**
 * @brief 获取从 start_cycle 到当前经过的微秒数
 */
float DWT_GetElapsedUs(uint32_t start_cycle);

/**
 * @brief 微秒级阻塞延时
 *
 * @note 延时过程中 CPU 忙等待，但中断仍然可以响应。
 */
void DWT_DelayUs(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif
