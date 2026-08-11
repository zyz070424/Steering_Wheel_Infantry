#ifndef DRV_FDCAN_H
#define DRV_FDCAN_H

#include <cstdint>

#include "main.h"
#include "stm32h7xx_hal_fdcan.h"


class ClassFDCAN;

using FDCANRxCallback = void (*)(ClassFDCAN *instance);

struct FDCANConfig
{
    FDCAN_HandleTypeDef *fdcan_handle = nullptr;
    uint32_t tx_id = 0;
    uint32_t rx_id = 0;
    FDCANRxCallback rx_callback = nullptr;
    void *owner = nullptr;
};

struct StructFDCANRxFrame
{
    uint32_t id = 0;
    uint8_t data[8]{};
    uint8_t data_length_bytes = 0;
};

class ClassFDCAN
{
public:
    bool Init(const FDCANConfig &config);
    // 同一总线的全部实例完成 Init 后调用一次。
    static bool Start(FDCAN_HandleTypeDef *fdcan_handle);

    static bool Send_Frame(FDCAN_HandleTypeDef *fdcan_handle,
                           uint32_t tx_id,
                           const uint8_t *data,
                           uint8_t data_length_bytes);
    bool Send(const uint8_t *data, uint8_t data_length_bytes);
    const StructFDCANRxFrame &Get_Rx_Frame() const;
    void *Get_Owner() const;

    static void Handle_RxFifo0(FDCAN_HandleTypeDef *fdcan_handle);
public:
    static constexpr uint8_t FDCAN_MAX_INSTANCE_COUNT = 15;
    // 当前 CubeMX 配置为经典 CAN，FIFO 单元长度为 8 字节。
    static constexpr uint8_t FDCAN_MAX_DATA_LENGTH_BYTES = 8;
    static ClassFDCAN *instances_[FDCAN_MAX_INSTANCE_COUNT];
    static uint8_t instance_count_;
private:



    FDCAN_HandleTypeDef *fdcan_handle_ = nullptr;  ///< HAL FDCAN 句柄
    uint32_t tx_id_ = 0;                          ///< 发送 ID
    uint32_t rx_id_ = 0;                          ///< 接收 ID
    FDCANRxCallback rx_callback_ = nullptr;       ///< 接收完成回调
    void *owner_ = nullptr;                       ///< 回调所属对象指针
    StructFDCANRxFrame rx_frame_{};               ///< 最近一次接收帧缓存


};

#endif
