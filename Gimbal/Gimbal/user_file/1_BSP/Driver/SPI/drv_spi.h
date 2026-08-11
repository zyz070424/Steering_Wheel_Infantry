#ifndef DRV_SPI_H
#define DRV_SPI_H

#include <cstdint>

#include "main.h"
#include "stm32h7xx_hal_spi.h"

class ClassSPI;

using SPIRxCallback = void (*)(void *owner,
                               const uint8_t *data,
                               uint16_t data_length_bytes);

typedef enum
{
    SPI_TRANSFER_NONE = 0,
    SPI_TRANSFER_BLOCKING,
    SPI_TRANSFER_IT,
    SPI_TRANSFER_DMA,
} SPI_TRANSFER_MODE;

struct SPIConfig
{
    SPI_HandleTypeDef *spi_handle = nullptr;
    GPIO_TypeDef *cs_gpio_port = nullptr;
    uint16_t cs_pin = 0;
    SPIRxCallback rx_callback = nullptr;
    void *owner = nullptr;
};

class ClassSPI
{
public:
    bool Init(const SPIConfig &config);

    bool Send(const uint8_t *send_buf,
              uint16_t send_length_bytes,
              SPI_TRANSFER_MODE transfer_mode);
    bool Receive(uint8_t *recv_buf,
                 uint16_t recv_length_bytes,
                 SPI_TRANSFER_MODE transfer_mode);
    bool Transmit_Receive(const uint8_t *send_buf,
                          uint8_t *recv_buf,
                          uint16_t data_length_bytes,
                          SPI_TRANSFER_MODE transfer_mode);

    static void Handle_Tx_Complete(SPI_HandleTypeDef *spi_handle);
    static void Handle_Rx_Complete(SPI_HandleTypeDef *spi_handle);

private:
    static constexpr uint8_t SPI_MAX_INSTANCE_COUNT = 4;
    static constexpr uint32_t SPI_DEFAULT_TIMEOUT_MS = 100;

    SPI_HandleTypeDef *spi_handle_ = nullptr;  ///< HAL SPI 句柄
    GPIO_TypeDef *cs_gpio_port_ = nullptr;     ///< 片选 GPIO 端口
    uint16_t cs_pin_ = 0;                      ///< 片选引脚
    SPIRxCallback rx_callback_ = nullptr;      ///< 接收完成回调
    void *owner_ = nullptr;                    ///< 回调所属对象指针
    uint8_t *recv_buf_ = nullptr;              ///< 当前接收缓冲区指针
    uint16_t recv_length_bytes_ = 0;           ///< 当前接收字节数

    static ClassSPI *instance_[SPI_MAX_INSTANCE_COUNT];  ///< 实例指针表
    static uint8_t instance_count_;                       ///< 当前实例计数
};

#endif
