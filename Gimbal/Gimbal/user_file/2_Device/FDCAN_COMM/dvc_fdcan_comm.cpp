#include "dvc_fdcan_comm.h"

#include <cstring>
/**
 * @brief 初始化FDCAN_COMM
 * 
 * @param config FDCAN_COMM配置
 * @return true 初始化成功
 * @return false 初始化失败
 */
bool ClassFDCANCOMM::Init(FDCANCOMMConfig &config)
{
    if ((config.Tx_Buffer_Length == 0U) || (config.Tx_Buffer_Length > FDCAN_MAX_BUFFER_LENGTH) ||
        (config.Rx_Buffer_Length == 0U) || (config.Rx_Buffer_Length > FDCAN_MAX_BUFFER_LENGTH))
    {
        return false;
    }

    const FDCANConfig registered_fdcan_config = {
        .fdcan_handle = config.FDCAN_config.fdcan_handle,
        .tx_id = config.FDCAN_config.tx_id,
        .rx_id = config.FDCAN_config.rx_id,
        .rx_callback = FDCANCOMMRXCallBack,
        .owner = this,
    };
    if (!fdcan_.Init(registered_fdcan_config))
    {
        return false;
    }

    Tx_Length_ = config.Tx_Buffer_Length;
    Rx_Length_ = config.Rx_Buffer_Length;
    Has_New_Data_ = false;
    FDCANCommResetRx();
    return true;
}
/**
 * @brief 发送FDCAN_COMM数据
 * 
 * @param data 要发送的数据指针
 */
void ClassFDCANCOMM::FDCANCommSend(uint8_t *data)
{
    const uint8_t tx_frame_length_bytes = Tx_Length_ + FDCAN_COMM_OFFSET_BYTES;

    Tx_Buffer_[0] = FDCAN_COMM_HEADER;
    memcpy(Tx_Buffer_ + 1U, data, Tx_Length_);
    Tx_Buffer_[Tx_Length_ + 1U] = FDCAN_COMM_TAIL;

    for (uint8_t offset_bytes = 0U; offset_bytes < tx_frame_length_bytes; offset_bytes += ClassFDCAN::FDCAN_MAX_DATA_LENGTH_BYTES)
    {
        uint8_t remaining_length_bytes = tx_frame_length_bytes - offset_bytes;
        uint8_t frame_data_length_bytes = remaining_length_bytes > ClassFDCAN::FDCAN_MAX_DATA_LENGTH_BYTES
                ? ClassFDCAN::FDCAN_MAX_DATA_LENGTH_BYTES
                : remaining_length_bytes;


        fdcan_.Send(Tx_Buffer_ + offset_bytes, frame_data_length_bytes);
    }
}
/**
 * @brief 获取接收到的数据缓冲区指针
 * @return 接收数据指针
 */
const uint8_t *ClassFDCANCOMM::Get_Rx_Data() const
{
    return RX_Public_Buffer;
}
/**
 * @brief 查询是否有新数据到达
 * @return true 有新数据，false 无新数据
 */
bool ClassFDCANCOMM::Has_New_Data() const
{
    return Has_New_Data_;
}
/**
 * @brief 清除新数据标志位
 */
void ClassFDCANCOMM::Clear_New_Data()
{
    Has_New_Data_ = false;
}
/**
 * @brief 处理FDCAN_COMM接收回调
 * 
 * @param fdcan FDCAN实例指针
 */
void ClassFDCANCOMM::FDCANCOMMRXCallBack(ClassFDCAN *fdcan)
{
    ClassFDCANCOMM *comm = static_cast<ClassFDCANCOMM *>(fdcan->Get_Owner());
    if (comm == nullptr)
    {
        return;
    }

    const StructFDCANRxFrame &rx_frame = fdcan->Get_Rx_Frame();
    if (comm->Rx_State_ == 0)
    {
        if ((rx_frame.data_length_bytes == 0U) || (rx_frame.data[0] != FDCAN_COMM_HEADER))
        {
            return;
        }

        comm->Rx_State_ = 1U;
    }

    if(comm->Rx_State_ == 1)
    {
        uint8_t rx_frame_length_bytes = comm->Rx_Length_ + FDCAN_COMM_OFFSET_BYTES;
        //长度检测
        if ((comm->Rx_Current_Length_ + rx_frame.data_length_bytes) > rx_frame_length_bytes)
        {
            comm->FDCANCommResetRx();
            return;
        }

        memcpy(comm->Rx_Buffer_ + comm->Rx_Current_Length_, rx_frame.data, rx_frame.data_length_bytes);

        comm->Rx_Current_Length_ += rx_frame.data_length_bytes;

        if (comm->Rx_Current_Length_ != rx_frame_length_bytes)
        {

            return;
        }

        if (comm->Rx_Buffer_[comm->Rx_Length_ + 1U] == FDCAN_COMM_TAIL)
        {
            memcpy(comm->RX_Public_Buffer, comm->Rx_Buffer_ + 1U, comm->Rx_Length_);
            comm->Has_New_Data_ = true;
        }

        comm->FDCANCommResetRx();
    }
}
/**
 * @brief 重置FDCAN_COMM接收缓冲区
 * 
 */
void ClassFDCANCOMM::FDCANCommResetRx()
{
    std::memset(Rx_Buffer_, 0, Rx_Current_Length_);
    Rx_State_ = 0U;
    Rx_Current_Length_ = 0U;
}
