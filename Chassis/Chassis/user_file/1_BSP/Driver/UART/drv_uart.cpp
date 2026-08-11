/**
 * @file drv_uart.cpp
 * @author your name (you@domain.com)
 * @brief 串口通信的底层代码
 * @version 0.1
 * @date 2026-07-31
 * @note 默认串口作为一对一通信模块
 * @copyright Copyright (c) 2026
 *
 */
#include "drv_uart.h"
#include "stm32h7xx_hal_uart.h"
#include <cstdint>
#include <stdlib.h>
#include <cstring>
ClassUART *ClassUART::instance_[UART_DEVICE_CNT]{};
uint8_t ClassUART::instance_count_ = 0;
/**
 * @brief 启动串口接收。
 * @return 成功返回 true，失败返回 false。
 * @details
 * 该函数会调用 HAL 库的 `HAL_UARTEx_ReceiveToIdle_DMA()` 函数启动串口接收。
 * 如果启动失败，则返回 false。
 */ 
bool ClassUART::Start_Rx()
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(uart_handle_, recv_buff_, recv_buff_size_) != HAL_OK)
    {
        return false;
    }

    // 关闭dma half transfer中断防止两次进入HAL_UARTEx_RxEventCallback()
    // 这是HAL库的一个设计失误,发生DMA传输完成/半完成以及串口IDLE中断都会触发HAL_UARTEx_RxEventCallback()
    // 我们只希望处理第一种和第三种情况,因此直接关闭DMA半传输中断
    __HAL_DMA_DISABLE_IT(uart_handle_->hdmarx, DMA_IT_HT);
    return true;
}

/**
 * @brief 初始化串口对象。
 * @param config 串口配置结构体。
 * @return 成功返回 true，失败返回 false。
 * @details
 * 该函数会检查配置参数的有效性，并将其保存到内部成员变量中。
 * 如果配置参数无效或串口对象数量超过限制，则返回 false。
 */
bool ClassUART::Init(const UARTConfig &config)
{
    if (instance_count_ >= UART_DEVICE_CNT ||config.recv_buff_size == 0 ||
        config.recv_buff_size > UART_RXBUFFER_LIEMIT ||config.uart_handle == nullptr ||
        config.uart_handle->hdmarx == nullptr)//TODO：记得开DMA Dr16需要
    {
        return false;
    }

    for (uint8_t index = 0; index < instance_count_; index++)
    {
        if(instance_[index]->uart_handle_ == config.uart_handle)
        {
            return false;
        }

    }
    uart_handle_ = config.uart_handle;
    rx_callback_ = config.rx_callback;
    owner_ = config.owner;
    recv_buff_size_ = config.recv_buff_size;
    memset(recv_buff_, 0, recv_buff_size_);

    instance_[instance_count_] = this;
    instance_count_++;

    if (!Start_Rx())
    {
        instance_count_--;
        instance_[instance_count_] = nullptr;
        return false;
    }

    return true;
}

/**
 * @brief 发送数据。
 * @param send_buf 发送缓冲区指针。
 * @param send_length 发送数据长度，单位为字节。
 * @param mode 发送模式，支持阻塞、非阻塞中断和DMA三种模式。
 * @return 成功返回 true，失败返回 false。
 * @details
 * 该函数会根据指定的发送模式调用 HAL 库的相应函数进行数据发送。
 * 如果发送失败，则返回 false。
 */
bool ClassUART::Send(uint8_t *send_buf,uint16_t send_length,USART_TRANSFER_MODE mode)
{   
    if(!UARTIsReady(uart_handle_))
    {
        return false;
    }
    switch(mode)
    {
        case USART_TRANSFER_BLOCKING:
        HAL_UART_Transmit(uart_handle_, send_buf, send_length, 100);
        break;
        case USART_TRANSFER_IT:
        HAL_UART_Transmit_IT(uart_handle_, send_buf, send_length);
        break;
        case USART_TRANSFER_DMA:
        HAL_UART_Transmit_DMA(uart_handle_,send_buf,send_length);
        break;
        default:
        return false;
    }
    return true;
}

/* 串口发送时,gstate会被设为BUSY_TX */
bool ClassUART:: UARTIsReady(UART_HandleTypeDef* uart_handle)
{
    if (uart_handle->gState | HAL_UART_STATE_BUSY_TX)
        return false;
    else
        return true;
}

/**
 * @brief 处理串口接收中断。
 * @param huart 串口句柄指针。
 * @param data_length_bytes 接收到的数据长度，单位为字节。
 * @return 无。
 * @details
 * 该函数会根据串口句柄查找对应的串口对象，并调用其接收回调函数。
 * 如果找不到对应的串口对象，则不做任何处理。
 */
void ClassUART::Handle_RXIT(UART_HandleTypeDef *huart, uint16_t data_length_bytes)
{
    for (uint8_t i = 0; i < instance_count_; ++i)
    {
        if (huart == instance_[i]->uart_handle_)
        {
            if (instance_[i]->rx_callback_ != nullptr)
            {
                // 接收缓冲区在下一次 Start_Rx() 后会被 DMA 复用。
                instance_[i]->rx_callback_(instance_[i]->owner_,
                                           instance_[i]->recv_buff_,
                                           data_length_bytes);
            }

            (void)instance_[i]->Start_Rx();
            return;
        }
    }

}

/**
 * @brief 处理串口接收错误：重启 DMA 接收以恢复通信
 * @param huart HAL UART 句柄
 */
void ClassUART::Handle_RXError(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < instance_count_; ++i)
    {
        if (huart == instance_[i]->uart_handle_)
        {
            (void)instance_[i]->Start_Rx();
            return;
        }
    }
}

/** @brief HAL 串口接收事件回调（IDLE/DMA 完成） */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    ClassUART::Handle_RXIT(huart, Size);
}

/** @brief HAL 串口错误回调 */
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    ClassUART::Handle_RXError(huart);
}
           
