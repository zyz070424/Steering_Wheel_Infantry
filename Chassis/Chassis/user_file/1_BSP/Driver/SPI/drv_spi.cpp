#include "drv_spi.h"

ClassSPI *ClassSPI::instance_[SPI_MAX_INSTANCE_COUNT]{};
uint8_t ClassSPI::instance_count_ = 0;

/**
 * @brief 初始化 SPI 实例，检查参数合法性并注册到全局实例表
 * @param config SPI 配置结构体
 * @return true 初始化成功，false 参数非法或重复注册
 */
bool ClassSPI::Init(const SPIConfig &config)
{
    if (instance_count_ >= SPI_MAX_INSTANCE_COUNT ||
        config.spi_handle == nullptr ||
        config.cs_gpio_port == nullptr ||
        config.cs_pin == 0)
    {
        return false;
    }

    for (uint8_t index = 0; index < instance_count_; ++index)
    {
        if (instance_[index] == this ||
            (instance_[index]->spi_handle_ == config.spi_handle &&
             instance_[index]->cs_gpio_port_ == config.cs_gpio_port &&
             instance_[index]->cs_pin_ == config.cs_pin))
        {
            return false;
        }
    }

    spi_handle_ = config.spi_handle;
    cs_gpio_port_ = config.cs_gpio_port;
    cs_pin_ = config.cs_pin;
    rx_callback_ = config.rx_callback;
    owner_ = config.owner;
    HAL_GPIO_WritePin(cs_gpio_port_, cs_pin_, GPIO_PIN_SET);

    instance_[instance_count_] = this;
    ++instance_count_;
    return true;
}

/**
 * @brief SPI 发送数据，支持阻塞/中断/DMA 三种模式
 * @param send_buf 发送缓冲区指针
 * @param send_length_bytes 发送字节数
 * @param transfer_mode 传输模式
 * @return true 发送成功，false 发送失败
 */
bool ClassSPI::Send(const uint8_t *send_buf,
                    uint16_t send_length_bytes,
                    SPI_TRANSFER_MODE transfer_mode)
{
    if (spi_handle_ == nullptr ||
        send_buf == nullptr ||
        send_length_bytes == 0 ||
        spi_handle_->State != HAL_SPI_STATE_READY ||
        (transfer_mode == SPI_TRANSFER_DMA && spi_handle_->hdmatx == nullptr))
    {
        return false;
    }

    HAL_GPIO_WritePin(cs_gpio_port_, cs_pin_, GPIO_PIN_RESET);

    HAL_StatusTypeDef hal_status = HAL_ERROR;
    switch (transfer_mode)
    {
    case SPI_TRANSFER_BLOCKING:
        hal_status = HAL_SPI_Transmit(spi_handle_,
                                      send_buf,
                                      send_length_bytes,
                                      SPI_DEFAULT_TIMEOUT_MS);
        break;

    case SPI_TRANSFER_IT:
        hal_status = HAL_SPI_Transmit_IT(spi_handle_, send_buf, send_length_bytes);
        break;

    case SPI_TRANSFER_DMA:
        hal_status = HAL_SPI_Transmit_DMA(spi_handle_, send_buf, send_length_bytes);
        break;

    default:
        HAL_GPIO_WritePin(cs_gpio_port_, cs_pin_, GPIO_PIN_SET);
        return false;
    }

    if (hal_status != HAL_OK || transfer_mode == SPI_TRANSFER_BLOCKING)
    {
        HAL_GPIO_WritePin(cs_gpio_port_, cs_pin_, GPIO_PIN_SET);
    }

    return hal_status == HAL_OK;
}

/**
 * @brief SPI 接收数据，支持阻塞/中断/DMA 三种模式
 * @param recv_buf 接收缓冲区指针
 * @param recv_length_bytes 接收字节数
 * @param transfer_mode 传输模式
 * @return true 接收成功，false 接收失败
 */
bool ClassSPI::Receive(uint8_t *recv_buf,
                       uint16_t recv_length_bytes,
                       SPI_TRANSFER_MODE transfer_mode)
{
    if (spi_handle_ == nullptr ||
        recv_buf == nullptr ||
        recv_length_bytes == 0 ||
        spi_handle_->State != HAL_SPI_STATE_READY ||
        (transfer_mode == SPI_TRANSFER_DMA && spi_handle_->hdmarx == nullptr))
    {
        return false;
    }

    recv_buf_ = recv_buf;
    recv_length_bytes_ = recv_length_bytes;
    HAL_GPIO_WritePin(cs_gpio_port_, cs_pin_, GPIO_PIN_RESET);

    HAL_StatusTypeDef hal_status = HAL_ERROR;
    switch (transfer_mode)
    {
    case SPI_TRANSFER_BLOCKING:
        hal_status = HAL_SPI_Receive(spi_handle_,
                                     recv_buf,
                                     recv_length_bytes,
                                     SPI_DEFAULT_TIMEOUT_MS);
        break;

    case SPI_TRANSFER_IT:
        hal_status = HAL_SPI_Receive_IT(spi_handle_, recv_buf, recv_length_bytes);
        break;

    case SPI_TRANSFER_DMA:
        hal_status = HAL_SPI_Receive_DMA(spi_handle_, recv_buf, recv_length_bytes);
        break;

    default:
        HAL_GPIO_WritePin(cs_gpio_port_, cs_pin_, GPIO_PIN_SET);
        return false;
    }

    if (hal_status != HAL_OK || transfer_mode == SPI_TRANSFER_BLOCKING)
    {
        HAL_GPIO_WritePin(cs_gpio_port_, cs_pin_, GPIO_PIN_SET);
        recv_buf_ = nullptr;
        recv_length_bytes_ = 0;
    }

    return hal_status == HAL_OK;
}

/**
 * @brief SPI 全双工收发数据，支持阻塞/中断/DMA 三种模式
 * @param send_buf 发送缓冲区指针
 * @param recv_buf 接收缓冲区指针
 * @param data_length_bytes 数据字节数
 * @param transfer_mode 传输模式
 * @return true 收发成功，false 收发失败
 */
bool ClassSPI::Transmit_Receive(const uint8_t *send_buf,
                                uint8_t *recv_buf,
                                uint16_t data_length_bytes,
                                SPI_TRANSFER_MODE transfer_mode)
{
    if (spi_handle_ == nullptr ||
        send_buf == nullptr ||
        recv_buf == nullptr ||
        data_length_bytes == 0 ||
        spi_handle_->State != HAL_SPI_STATE_READY ||
        (transfer_mode == SPI_TRANSFER_DMA &&
         (spi_handle_->hdmatx == nullptr || spi_handle_->hdmarx == nullptr)))
    {
        return false;
    }

    recv_buf_ = recv_buf;
    recv_length_bytes_ = data_length_bytes;
    HAL_GPIO_WritePin(cs_gpio_port_, cs_pin_, GPIO_PIN_RESET);

    HAL_StatusTypeDef hal_status = HAL_ERROR;
    switch (transfer_mode)
    {
    case SPI_TRANSFER_BLOCKING:
        hal_status = HAL_SPI_TransmitReceive(spi_handle_,
                                             send_buf,
                                             recv_buf,
                                             data_length_bytes,
                                             SPI_DEFAULT_TIMEOUT_MS);
        break;

    case SPI_TRANSFER_IT:
        hal_status = HAL_SPI_TransmitReceive_IT(spi_handle_,
                                                send_buf,
                                                recv_buf,
                                                data_length_bytes);
        break;

    case SPI_TRANSFER_DMA:
        hal_status = HAL_SPI_TransmitReceive_DMA(spi_handle_,
                                                 send_buf,
                                                 recv_buf,
                                                 data_length_bytes);
        break;

    default:
        HAL_GPIO_WritePin(cs_gpio_port_, cs_pin_, GPIO_PIN_SET);
        return false;
    }

    if (hal_status != HAL_OK || transfer_mode == SPI_TRANSFER_BLOCKING)
    {
        HAL_GPIO_WritePin(cs_gpio_port_, cs_pin_, GPIO_PIN_SET);
        recv_buf_ = nullptr;
        recv_length_bytes_ = 0;
    }

    return hal_status == HAL_OK;
}

/**
 * @brief SPI 发送完成事件处理：拉高对应实例的片选引脚
 * @param spi_handle HAL SPI 句柄
 */
void ClassSPI::Handle_Tx_Complete(SPI_HandleTypeDef *spi_handle)
{
    for (uint8_t index = 0; index < instance_count_; ++index)
    {
        ClassSPI *instance = instance_[index];
        if (instance->spi_handle_ == spi_handle && HAL_GPIO_ReadPin(instance->cs_gpio_port_, instance->cs_pin_) == GPIO_PIN_RESET)
        {
            HAL_GPIO_WritePin(instance->cs_gpio_port_, instance->cs_pin_, GPIO_PIN_SET);
            return;
        }
    }
}

/**
 * @brief SPI 接收完成事件处理：拉高片选并触发上层回调
 * @param spi_handle HAL SPI 句柄
 */
void ClassSPI::Handle_Rx_Complete(SPI_HandleTypeDef *spi_handle)
{
    for (uint8_t index = 0; index < instance_count_; ++index)
    {
        ClassSPI *instance = instance_[index];
        if (instance->spi_handle_ == spi_handle && HAL_GPIO_ReadPin(instance->cs_gpio_port_, instance->cs_pin_) == GPIO_PIN_RESET)
        {
            const uint8_t *recv_buf = instance->recv_buf_;
            const uint16_t recv_length_bytes = instance->recv_length_bytes_;

            HAL_GPIO_WritePin(instance->cs_gpio_port_, instance->cs_pin_, GPIO_PIN_SET);
            instance->recv_buf_ = nullptr;
            instance->recv_length_bytes_ = 0;

            if (instance->rx_callback_ != nullptr)
            {
                instance->rx_callback_(instance->owner_, recv_buf, recv_length_bytes);
            }
            return;
        }
    }
}

/** @brief HAL SPI 发送完成回调 */
extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    ClassSPI::Handle_Tx_Complete(hspi);
}

/** @brief HAL SPI 接收完成回调 */
extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    ClassSPI::Handle_Rx_Complete(hspi);
}

/** @brief HAL SPI 全双工收发完成回调 */
extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    ClassSPI::Handle_Rx_Complete(hspi);
}

/** @brief HAL SPI 错误回调：拉高片选防止总线挂死 */
extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    ClassSPI::Handle_Tx_Complete(hspi);
}
