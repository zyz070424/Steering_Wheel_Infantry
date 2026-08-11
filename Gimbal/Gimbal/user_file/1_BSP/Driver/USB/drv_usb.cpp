#include "drv_usb.h"

#include <cstring>

extern "C"
{
extern USBD_HandleTypeDef hUsbDeviceHS;
extern uint8_t UserRxBufferHS[APP_RX_DATA_SIZE];
}

ClassUSB USB_Driver{};
/**
 * @brief 初始化 USB 驱动。
 * @param rx_callback 接收回调函数指针。
 * @param owner 回调函数上下文指针。
 * @return 无。
 */
void ClassUSB::Init(USBRxCallback rx_callback, void *owner)
{
    rx_callback_ = rx_callback;
    owner_ = owner;
    rx_active_buffer_ = UserRxBufferHS;
    rx_ready_buffer_ = nullptr;

    Start_Receive();
}
/**
 * @brief 发送 USB 数据包。
 * @param data 要发送的数据指针。
 * @param data_length_bytes 要发送的数据长度。
 * @return USBTxResult 发送结果。
 * @note 发送失败时，返回 USBTxResult::Error。
 */
USBTxResult ClassUSB::Send(const uint8_t *data, uint16_t data_length_bytes)
{
    if ((data == nullptr) || (data_length_bytes == 0) ||
        (data_length_bytes > USB_TX_BUFFER_SIZE_BYTES))
    {
        return USBTxResult::Error;
    }

    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED)
    {
        return USBTxResult::Busy;
    }

    if (tx_busy_)
    {
        return USBTxResult::Busy;
    }
    tx_busy_ = true;

    std::memcpy(tx_buffer_, data, data_length_bytes);

    const uint8_t usb_result = CDC_Transmit_HS(tx_buffer_, data_length_bytes);
    if (usb_result == USBD_OK)
    {
        return USBTxResult::Ok;
    }

    tx_busy_ = false;

    if (usb_result == USBD_BUSY)
    {
        return USBTxResult::Busy;
    }

    return USBTxResult::Error;
}
/**
 * @brief 处理 USB 接收中断。
 * @param data 接收到的数据指针。
 * @param data_length_bytes 接收到的数据长度。
 * @return 无。
 */
void ClassUSB::Handle_Rx_Irq(uint8_t *data, uint32_t data_length_bytes)
{
    rx_ready_buffer_ = data;
    rx_active_buffer_ = (data == UserRxBufferHS) ? rx_buffer_0_ : UserRxBufferHS;

    // 先把下一包切到另一块内存，再交给上层处理当前数据。
    Start_Receive();

    if (rx_callback_ != nullptr)
    {
        rx_callback_(owner_, rx_ready_buffer_, data_length_bytes);
    }
}
/**
 * @brief USB 发送完成中断处理：清除发送忙标志
 */
void ClassUSB::Handle_Tx_Complete_Irq()
{
    tx_busy_ = false;
}
/**
 * @brief 开始接收 USB 数据包。
 * @return 无。
 */
void ClassUSB::Start_Receive()
{
    USBD_CDC_SetRxBuffer(&hUsbDeviceHS, rx_active_buffer_);
    USBD_CDC_ReceivePacket(&hUsbDeviceHS);
}

/** @brief USB CDC 接收回调入口 */
extern "C" void USB_Rx_Irq_Callback(uint8_t *data, uint32_t data_length_bytes)
{
    USB_Driver.Handle_Rx_Irq(data, data_length_bytes);
}

/** @brief USB CDC 发送完成回调入口 */
extern "C" void USB_Tx_Complete_Irq_Callback(void)
{
    USB_Driver.Handle_Tx_Complete_Irq();
}
