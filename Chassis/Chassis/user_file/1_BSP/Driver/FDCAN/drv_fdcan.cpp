/**
 * @file drv_fdcan.cpp
 * @author your name (you@domain.com)
 * @brief FDCAN的底层代码
 * @version 0.1
 * @date 2026-07-31
 * @note 记得代码先注册设备再start FDCAN
 * @copyright Copyright (c) 2026
 * 
 */
#include "drv_fdcan.h"

#include <cstring>

ClassFDCAN *ClassFDCAN::instances_[FDCAN_MAX_INSTANCE_COUNT]{};
uint8_t ClassFDCAN::instance_count_ = 0;
/**
 * @brief 初始化FDCAN实例
 * 
 * @param config FDCAN配置
 * @return true 初始化成功
 * @return false 初始化失败
 */
bool ClassFDCAN::Init(const FDCANConfig &config)
{
    if (instance_count_ >= FDCAN_MAX_INSTANCE_COUNT)
    {
        return false;
    }
    //判断是否重复注册
    for (uint8_t index = 0; index < instance_count_; index++)
    {
        if (instances_[index]->fdcan_handle_ == config.fdcan_handle &&
            instances_[index]->rx_id_ == config.rx_id)
        {
            return false;
        }
    }
    //注册
    fdcan_handle_ = config.fdcan_handle;
    tx_id_ = config.tx_id;
    rx_id_ = config.rx_id;
    rx_callback_ = config.rx_callback;
    owner_ = config.owner;
    instances_[instance_count_++] = this;
    
    return true;
}
/**
 * @brief 启动FDCAN实例兼filter配置
 * 
 * @param fdcan_handle FDCAN句柄
 * @return true 启动成功
 * @return false 启动失败
 */
bool ClassFDCAN::Start(FDCAN_HandleTypeDef *fdcan_handle)
{
    if (fdcan_handle == nullptr)
    {
        return false;
    }

    if (HAL_FDCAN_ConfigGlobalFilter(fdcan_handle,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK ||
        HAL_FDCAN_Start(fdcan_handle) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(fdcan_handle,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                       0) != HAL_OK)
    {
        return false;
    }

    return true;
}
/**
 * @brief FDCAN发送函数
 * 
 * @param data 
 * @param data_length_bytes 
 * @return true 
 * @return false 
 */
bool ClassFDCAN::Send(const uint8_t *data, uint8_t data_length_bytes)
{
    return Send_Frame(fdcan_handle_, tx_id_, data, data_length_bytes);
}

/**
 * @brief 通过指定 FDCAN 句柄发送数据帧（静态方法）
 * @param fdcan_handle HAL FDCAN 句柄
 * @param tx_id 发送 ID
 * @param data 数据指针
 * @param data_length_bytes 数据长度（最大 8 字节）
 * @return true 发送成功，false 发送失败
 */
bool ClassFDCAN::Send_Frame(FDCAN_HandleTypeDef *fdcan_handle,
                            uint32_t tx_id,
                            const uint8_t *data,
                            uint8_t data_length_bytes)
{
    if ((fdcan_handle == nullptr) || (data == nullptr) ||
        (data_length_bytes > FDCAN_MAX_DATA_LENGTH_BYTES) ||
        (HAL_FDCAN_GetTxFifoFreeLevel(fdcan_handle) == 0))
    {
        return false;
    }

    FDCAN_TxHeaderTypeDef tx_header{};
    tx_header.Identifier = tx_id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = data_length_bytes;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(fdcan_handle, &tx_header, data) == HAL_OK;
}
/**
 * @brief 直接获取FDCAN接收帧
 * 
 * @return StructFDCANRxFrame 
 */
const StructFDCANRxFrame &ClassFDCAN::Get_Rx_Frame() const
{
    return rx_frame_;
}
/**
 * @brief 获取FDCAN代码注册实例
 * 
 * @return void* 
 */
void *ClassFDCAN::Get_Owner() const
{
    return owner_;
}

/**
 * @brief 处理FDCAN接收中断
 * 
 * @param fdcan_handle FDCAN句柄
 */
void ClassFDCAN::Handle_RxFifo0(FDCAN_HandleTypeDef *fdcan_handle)
{
    FDCAN_RxHeaderTypeDef rx_header{};
    uint8_t rx_data[FDCAN_MAX_DATA_LENGTH_BYTES]{};

    while (HAL_FDCAN_GetRxFifoFillLevel(fdcan_handle, FDCAN_RX_FIFO0) > 0)
    {
        HAL_FDCAN_GetRxMessage(fdcan_handle, FDCAN_RX_FIFO0, &rx_header, rx_data);

        for (uint8_t index = 0; index < instance_count_; ++index)
        {
            ClassFDCAN *instance = instances_[index];
            if (instance->fdcan_handle_ != fdcan_handle ||instance->rx_id_ != rx_header.Identifier)
            {
                continue;
            }

            instance->rx_frame_.id = rx_header.Identifier;
            instance->rx_frame_.data_length_bytes = static_cast<uint8_t>(rx_header.DataLength);

            std::memcpy(instance->rx_frame_.data,rx_data,instance->rx_frame_.data_length_bytes);
            if (instance->rx_callback_ != nullptr)
            {
                instance->rx_callback_(instance);
            }
            break;
        }
    }
}

/** @brief HAL FDCAN Rx FIFO 0 新消息回调 */
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    ClassFDCAN::Handle_RxFifo0(hfdcan);
}
