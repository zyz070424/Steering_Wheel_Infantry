#ifndef DVC_FDCAN_COMM_H
#define DVC_FDCAN_COMM_H

#include <cstdint>

#include "drv_fdcan.h"

/**
 * @brief FDCAN COMM 配置结构体
 */
typedef struct
{
    FDCANConfig FDCAN_config{};      ///< FDCAN 基础配置
    uint8_t Tx_Buffer_Length = 0U;   ///< 发送数据长度（不含帧头帧尾）
    uint8_t Rx_Buffer_Length = 0U;   ///< 接收数据长度（不含帧头帧尾）
} FDCANCOMMConfig;

/**
 * @brief FDCAN COMM 设备层：基于 FDCAN 的自定义帧协议通信
 *
 * 协议格式：[HEADER 0xFE] + [数据] + [TAIL 0xEF]，支持多帧拆分发送/拼接接收。
 */
class ClassFDCANCOMM
{
public:
    bool Init(FDCANCOMMConfig &config);
    void FDCANCommSend(uint8_t *data);

    const uint8_t *Get_Rx_Data() const;
    bool Has_New_Data() const;
    void Clear_New_Data();

private:
    static void FDCANCOMMRXCallBack(ClassFDCAN *fdcan);
    void FDCANCommResetRx();

private:
    static constexpr uint8_t FDCAN_MAX_BUFFER_LENGTH = 60U;
    static constexpr uint8_t FDCAN_COMM_HEADER = 0xFEU;
    static constexpr uint8_t FDCAN_COMM_TAIL = 0xEFU;
    static constexpr uint8_t FDCAN_COMM_OFFSET_BYTES = 2U;

    ClassFDCAN fdcan_{};                                            ///< FDCAN 通信实例
    uint8_t Tx_Buffer_[FDCAN_MAX_BUFFER_LENGTH + FDCAN_COMM_OFFSET_BYTES]{};  ///< 发送缓冲区（含帧头帧尾）
    uint8_t Tx_Length_ = 0U;                                         ///< 发送数据净长度

    uint8_t Rx_Buffer_[FDCAN_MAX_BUFFER_LENGTH + FDCAN_COMM_OFFSET_BYTES]{};  ///< 接收拼接缓冲区
    uint8_t RX_Public_Buffer[FDCAN_MAX_BUFFER_LENGTH]{};             ///< 接收完成后的公开数据缓冲区
    uint8_t Rx_Length_ = 0U;                                         ///< 接收数据净长度
    uint8_t Rx_Current_Length_ = 0U;                                 ///< 当前已接收长度
    uint8_t Rx_State_ = 0U;                                          ///< 接收状态机（0=等待帧头, 1=接收中）
    volatile bool Has_New_Data_ = false;                              ///< 新数据到达标志
};

#endif
