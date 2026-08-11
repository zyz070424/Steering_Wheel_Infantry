
#ifndef __DVC_BMI088_H__
#define __DVC_BMI088_H__

#include <cstdint>

#include "drv_spi.h"

/**
 * @brief BMI088 配置结构体
 */
struct BMI088Config
{
    SPI_HandleTypeDef *spi_handle = nullptr;           ///< HAL SPI 句柄（加速度计和陀螺仪共用）
    GPIO_TypeDef *accelerometer_cs_gpio_port = nullptr;  ///< 加速度计片选 GPIO 端口
    uint16_t accelerometer_cs_pin = 0;                    ///< 加速度计片选引脚
    GPIO_TypeDef *gyroscope_cs_gpio_port = nullptr;       ///< 陀螺仪片选 GPIO 端口
    uint16_t gyroscope_cs_pin = 0;                        ///< 陀螺仪片选引脚
};

/**
 * @brief BMI088 状态结构体
 */
struct BMI088Status
{
    float accelerometer_mps2[3]{};   ///< 三轴加速度 (m/s²)
    float gyroscope_radps[3]{};      ///< 三轴角速度 (rad/s)
    uint32_t sample_sequence = 0;    ///< 采样序列号
    uint32_t last_update_ms = 0;     ///< 上次更新时间戳 (ms)
    bool is_initialized = false;     ///< 初始化标志
};

/**
 * @brief BMI088 六轴 IMU 设备层。
 *
 * 输出统一为传感器坐标系下的 m/s² 与 rad/s，坐标系变换由 App IMU 层负责。
 */
class ClassBMI088
{
public:
    bool Init(const BMI088Config &config);
    bool Update(void);
    BMI088Status Get_Status(void) const;

private:
    bool Init_Accelerometer(void);
    bool Init_Gyroscope(void);
    bool Write_Accelerometer_Register(uint8_t register_address,
                                      uint8_t register_value);
    bool Write_Gyroscope_Register(uint8_t register_address,
                                  uint8_t register_value);
    bool Read_Accelerometer_Registers(uint8_t register_address,
                                      uint8_t *data,
                                      uint8_t data_length_bytes);
    bool Read_Gyroscope_Registers(uint8_t register_address,
                                  uint8_t *data,
                                  uint8_t data_length_bytes);

    ClassSPI accelerometer_spi_;   ///< 加速度计 SPI 实例
    ClassSPI gyroscope_spi_;       ///< 陀螺仪 SPI 实例
    BMI088Status status_{};        ///< 传感器状态
};

#endif /*__DVC_BMI088_H__*/
