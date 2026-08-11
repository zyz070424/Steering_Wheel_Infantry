#ifndef __DRV_UART_H__
#define __DRV_UART_H__
#include "main.h"
#include "stm32h7xx_hal_uart.h"


class ClassUART;

using UARTRxCallback = void (*)(void *owner,
                                const uint8_t *data,
                                uint16_t data_length_bytes);
//串口发送模式
typedef enum
{
    USART_TRANSFER_NONE=0,
    USART_TRANSFER_BLOCKING,
    USART_TRANSFER_IT,
    USART_TRANSFER_DMA,
} USART_TRANSFER_MODE;
//串口配置结构体
typedef struct 
{
    uint16_t recv_buff_size = 0;
    UART_HandleTypeDef *uart_handle = nullptr;
    UARTRxCallback rx_callback = nullptr;
    void *owner = nullptr;

}UARTConfig;

class ClassUART
{
public:
    bool Init(const UARTConfig &config);
    bool Send(uint8_t *send_buf,uint16_t send_length,USART_TRANSFER_MODE mode);
    static void Handle_RXIT(UART_HandleTypeDef *huart,uint16_t data_length_bytes);
    static void Handle_RXError(UART_HandleTypeDef *huart);
    static constexpr uint16_t UART_RXBUFFER_LIEMIT = 256;
    static constexpr uint8_t UART_DEVICE_CNT = 6;         //其实一共就5个外设留一个余量
    static ClassUART *instance_[UART_DEVICE_CNT];
    static uint8_t instance_count_;
private:
    bool Start_Rx();

    uint8_t recv_buff_[UART_RXBUFFER_LIEMIT];   ///< DMA 接收缓冲区
    uint16_t recv_buff_size_ = 0;                ///< 有效接收缓冲区大小
    UART_HandleTypeDef *uart_handle_ = nullptr;  ///< HAL UART 句柄
    UARTRxCallback rx_callback_ = nullptr;       ///< 接收完成回调
    void *owner_ = nullptr;                      ///< 回调所属对象指针
private:
    /**
     * @brief 判断串口是否空闲可发送
     * @param huart HAL UART 句柄
     * @return true 空闲，false 忙碌
     */
    bool UARTIsReady(UART_HandleTypeDef * huart);


};

extern "C"  void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);




#endif /*__DRV_UART_H__*/
