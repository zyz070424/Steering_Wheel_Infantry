#include "dvc_vision.h"
#include <cstddef>
#include <cstdint>

/**
 * @brief 视觉设备初始化
 * 
 * @param Rx_Header 发送帧头
 * @param Rx_Tail   发送帧尾
 */
void ClassVision::Init(uint8_t rx_header, uint8_t rx_tail)
{
    usb_.Init(Vision_Rx_Callback, this);
    Rx_Header_ = rx_header;
    Rx_Tail_ = rx_tail;
}

/**
 * @brief 视觉消息发送
 * 
 * @param Send_Data 发送数据 
 */
void ClassVision::Send(const Vision_UART_Tx_Data &send_data)
{
    uint8_t Tx_Buffer[MANIFOLD_USB_TX_FRAME_LEN]{};
    Vision_UART_Tx_Data tx_data = send_data;

    tx_data.Frame_Header = Rx_Header_;
    tx_data.Frame_Tail = Rx_Tail_;
    memcpy(Tx_Buffer, &tx_data, sizeof(Vision_UART_Tx_Data));

    usb_.Send(Tx_Buffer, MANIFOLD_USB_TX_FRAME_LEN);
}


/**
 * @brief 获取接收到的视觉数据
 * 
 * @return Vision_UART_Rx_Data 接收视觉数据 
 */
Vision_UART_Rx_Data ClassVision::Get_Vision_Rx_Data(void)const
{
    return Rx_Data_;
}

/**
 * @brief 视觉接收中断回调
 * 
 * @param owner 注册指针
 * @param data 接收数据
 * @param data_length_bytes 接收数据长度 
 */
void ClassVision::Vision_Rx_Callback(void *owner,const uint8_t *data,uint32_t data_length_bytes)
{
    if(owner == nullptr ||data == nullptr || data_length_bytes == 0 )
    {
        return;
    }

    ClassVision *vision = static_cast<ClassVision *>(owner);

    for(uint32_t index = 0;index < data_length_bytes;index++)
    {
        const uint8_t byte = data[index];

        if(vision->Rx_Current_Index == 0)
        {
            if(byte == vision->Rx_Header_)
            {
                vision->Rx_Frame_Buffer[vision->Rx_Current_Index++] = byte;
            }
            continue;
        }

        vision->Rx_Frame_Buffer[vision->Rx_Current_Index++] = byte;

        if(vision->Rx_Current_Index != MANIFOLD_USB_RX_FRAME_LEN)
        {
            continue;
        }

        if(vision->Rx_Frame_Buffer[MANIFOLD_USB_RX_FRAME_LEN - 1] == vision->Rx_Tail_)
        {
            // 视觉协议固定为 [帧头][Yaw][Pitch][有效位][帧尾]。
            memcpy(&vision->Rx_Data_.Target_Yaw_Deg, &vision->Rx_Frame_Buffer[1], sizeof(vision->Rx_Data_.Target_Yaw_Deg));
            memcpy(&vision->Rx_Data_.Target_Pitch_Deg, &vision->Rx_Frame_Buffer[1 + sizeof(float)], sizeof(vision->Rx_Data_.Target_Pitch_Deg));
            vision->Rx_Data_.Frame_Header = vision->Rx_Frame_Buffer[0];
            vision->Rx_Data_.Target_Valid = vision->Rx_Frame_Buffer[1 + sizeof(float) + sizeof(float)];
            vision->Rx_Data_.Frame_Tail = vision->Rx_Frame_Buffer[MANIFOLD_USB_RX_FRAME_LEN - 1];
            vision->Rx_Current_Index = 0;
        }
        else if(byte == vision->Rx_Header_)
        {
            // 当前字节本身可能是下一帧的帧头，保留它继续拼帧。
            vision->Rx_Frame_Buffer[0] = byte;
            vision->Rx_Current_Index = 1;
        }
        else
        {
            vision->Rx_Current_Index = 0;
        }
    }
}
