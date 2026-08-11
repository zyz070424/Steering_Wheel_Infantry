#ifndef DRV_USB_H
#define DRV_USB_H

#include <stdint.h>

#ifdef __cplusplus

#include "usbd_cdc_if.h"

using USBRxCallback = void (*)(void *owner,const uint8_t *data, uint32_t data_length_bytes);

enum USBTxResult : uint8_t
{
    Ok,
    Busy,
    Error,
};

class ClassUSB
{
public:
    void Init(USBRxCallback rx_callback, void *owner);
    USBTxResult Send(const uint8_t *data, uint16_t data_length_bytes);

    void Handle_Rx_Irq(uint8_t *data, uint32_t data_length_bytes);
    void Handle_Tx_Complete_Irq();

private:
    static constexpr uint16_t USB_TX_BUFFER_SIZE_BYTES = 256;

    void Start_Receive();

    USBRxCallback rx_callback_ = nullptr;
    void *owner_ = nullptr;

    // UserRxBufferHS 与本缓冲区轮流给 CDC 使用，避免重装接收时覆盖当前包。
    uint8_t rx_buffer_0_[APP_RX_DATA_SIZE]{};
    uint8_t *rx_active_buffer_ = nullptr;
    uint8_t *rx_ready_buffer_ = nullptr;

    // Send 只由一个任务调用；CDC 发送完成中断会清除忙标志。
    // CDC 发送完成前仍会访问这个缓冲区，因此发送数据必须先复制到这里。
    uint8_t tx_buffer_[USB_TX_BUFFER_SIZE_BYTES]{};
    volatile bool tx_busy_ = false;
};

extern ClassUSB USB_Driver;

#endif

#ifdef __cplusplus
extern "C" {
#endif

void USB_Rx_Irq_Callback(uint8_t *data, uint32_t data_length_bytes);
void USB_Tx_Complete_Irq_Callback(void);

#ifdef __cplusplus
}
#endif

#endif
