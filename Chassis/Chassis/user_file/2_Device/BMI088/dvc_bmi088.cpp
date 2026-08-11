#include "dvc_bmi088.h"

#include "dvc_bmi088_config.h"

namespace
{
constexpr uint8_t BMI088_ACCELEROMETER_READ_DUMMY_BYTES = 1;
constexpr uint8_t BMI088_MAX_ACCELEROMETER_TRANSFER_BYTES = 8;
constexpr uint8_t BMI088_MAX_GYROSCOPE_TRANSFER_BYTES = 7;
constexpr uint8_t BMI088_ACCELEROMETER_DATA_LENGTH_BYTES = 6;
constexpr uint8_t BMI088_GYROSCOPE_DATA_LENGTH_BYTES = 6;
constexpr uint32_t BMI088_RESET_WAIT_MS = 80;

// 解析16位有符号整数
int16_t BMI088_Parse_Int16(const uint8_t *data)
{
    return static_cast<int16_t>(static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8));
}
}
/**
 * @brief 初始化BMI088
 * 
 * @param config 配置结构体
 * @return true 初始化成功
 * @return false 初始化失败
 */
bool ClassBMI088::Init(const BMI088Config &config)
{
    const SPIConfig accelerometer_spi_config = {
        .spi_handle = config.spi_handle,
        .cs_gpio_port = config.accelerometer_cs_gpio_port,
        .cs_pin = config.accelerometer_cs_pin,
    };
    const SPIConfig gyroscope_spi_config = {
        .spi_handle = config.spi_handle,
        .cs_gpio_port = config.gyroscope_cs_gpio_port,
        .cs_pin = config.gyroscope_cs_pin,
        
    };

    if (!accelerometer_spi_.Init(accelerometer_spi_config) || !gyroscope_spi_.Init(gyroscope_spi_config))
    {
        return false;
    }

    if (!Init_Accelerometer() || !Init_Gyroscope())
    {
        return false;
    }

    status_.is_initialized = true;
    return true;
}
/**
 * @brief 更新BMI088状态
 * 
 * @return true 更新成功
 * @return false 更新失败
 */
bool ClassBMI088::Update(void)
{
    if (!status_.is_initialized)
    {
        return false;
    }

    uint8_t accelerometer_data[BMI088_ACCELEROMETER_DATA_LENGTH_BYTES]{};
    uint8_t gyroscope_data[BMI088_GYROSCOPE_DATA_LENGTH_BYTES]{};

    Read_Accelerometer_Registers(BMI088_ACCEL_XOUT_L, accelerometer_data, sizeof(accelerometer_data));
    Read_Gyroscope_Registers(BMI088_GYRO_X_L, gyroscope_data, sizeof(gyroscope_data));


    for (uint8_t axis = 0; axis < 3; ++axis)
    {
        const int16_t accelerometer_raw = BMI088_Parse_Int16(&accelerometer_data[axis * 2]);
        const int16_t gyroscope_raw = BMI088_Parse_Int16(&gyroscope_data[axis * 2]);

        status_.accelerometer_mps2[axis] =
            static_cast<float>(accelerometer_raw) * BMI088_ACCEL_6G_SEN;
        status_.gyroscope_radps[axis] =
            static_cast<float>(gyroscope_raw) * BMI088_GYRO_2000_SEN;
    }

    ++status_.sample_sequence;
    status_.last_update_ms = HAL_GetTick();
    return true;
}

/**
 * @brief 获取 BMI088 当前状态
 * @return BMI088 状态结构体（加速度、角速度、采样序列号等）
 */
BMI088Status ClassBMI088::Get_Status(void) const
{
    return status_;
}

/**
 * @brief 初始化加速度计：软复位 → 验证芯片 ID → 配置电源和量程
 * @return true 初始化成功，false 初始化失败
 */
bool ClassBMI088::Init_Accelerometer(void)
{
    uint8_t chip_id = 0;

    // 加速度计上电后需要一次 SPI 读操作，才会从 I2C 接口切换到 SPI 接口。
    if (!Read_Accelerometer_Registers(BMI088_ACC_CHIP_ID, &chip_id, 1) ||
        !Write_Accelerometer_Register(BMI088_ACC_SOFTRESET,
                                      BMI088_ACC_SOFTRESET_VALUE))
    {
        return false;
    }

    HAL_Delay(BMI088_RESET_WAIT_MS);

    if (!Read_Accelerometer_Registers(BMI088_ACC_CHIP_ID, &chip_id, 1) ||
        chip_id != BMI088_ACC_CHIP_ID_VALUE ||
        !Write_Accelerometer_Register(BMI088_ACC_PWR_CTRL,
                                      BMI088_ACC_ENABLE_ACC_ON))
    {
        return false;
    }

    HAL_Delay(1);

    return Write_Accelerometer_Register(BMI088_ACC_PWR_CONF,
                                        BMI088_ACC_PWR_ACTIVE_MODE) &&
           Write_Accelerometer_Register(BMI088_ACC_CONF,  BMI088_ACC_CONF_MUST_Set |BMI088_ACC_NORMAL | BMI088_ACC_800_HZ) &&
           Write_Accelerometer_Register(BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G);
}

/**
 * @brief 初始化陀螺仪：软复位 → 验证芯片 ID → 配置量程和带宽
 * @return true 初始化成功，false 初始化失败
 */
bool ClassBMI088::Init_Gyroscope(void)
{
    uint8_t chip_id = 0;

    if (!Write_Gyroscope_Register(BMI088_GYRO_SOFTRESET,
                                  BMI088_GYRO_SOFTRESET_VALUE))
    {
        return false;
    }

    HAL_Delay(BMI088_RESET_WAIT_MS);

    if (!Read_Gyroscope_Registers(BMI088_GYRO_CHIP_ID, &chip_id, 1) ||
        chip_id != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return false;
    }

    return Write_Gyroscope_Register(BMI088_GYRO_RANGE,BMI088_GYRO_2000) &&
           Write_Gyroscope_Register(BMI088_GYRO_BANDWIDTH,BMI088_GYRO_BANDWIDTH_MUST_Set|BMI088_GYRO_1000_116_HZ) &&
           Write_Gyroscope_Register(BMI088_GYRO_LPM1,BMI088_GYRO_NORMAL_MODE) &&
           Write_Gyroscope_Register(BMI088_GYRO_CTRL,BMI088_DRDY_OFF);
}

/**
 * @brief 写加速度计寄存器
 * @param register_address 寄存器地址
 * @param register_value 写入值
 * @return true 写入成功，false 写入失败
 */
bool ClassBMI088::Write_Accelerometer_Register(uint8_t register_address, uint8_t register_value)
{
    const uint8_t data[2] = {register_address, register_value};

    return accelerometer_spi_.Send(data,sizeof(data), SPI_TRANSFER_BLOCKING);
}

/**
 * @brief 写陀螺仪寄存器
 * @param register_address 寄存器地址
 * @param register_value 写入值
 * @return true 写入成功，false 写入失败
 */
bool ClassBMI088::Write_Gyroscope_Register(uint8_t register_address, uint8_t register_value)
{
    const uint8_t data[2] = {register_address, register_value};

    return gyroscope_spi_.Send(data,sizeof(data), SPI_TRANSFER_BLOCKING);
}

/**
 * @brief 读加速度计寄存器（SPI 全双工，带 1 字节 dummy）
 * @param register_address 起始寄存器地址
 * @param data 读取数据存放缓冲区
 * @param data_length_bytes 读取字节数
 * @return true 读取成功，false 读取失败
 */
bool ClassBMI088::Read_Accelerometer_Registers(uint8_t register_address, uint8_t *data, uint8_t data_length_bytes)
{
    uint8_t transmit_data[BMI088_MAX_ACCELEROMETER_TRANSFER_BYTES]{};
    uint8_t receive_data[BMI088_MAX_ACCELEROMETER_TRANSFER_BYTES]{};
    const uint8_t transfer_length_bytes = data_length_bytes + BMI088_ACCELEROMETER_READ_DUMMY_BYTES + 1;

    transmit_data[0] = register_address | 0x80;
    if (!accelerometer_spi_.Transmit_Receive(transmit_data,
                                             receive_data,
                                             transfer_length_bytes,
                                             SPI_TRANSFER_BLOCKING))
    {
        return false;
    }

    for (uint8_t index = 0; index < data_length_bytes; ++index)
    {
        data[index] = receive_data[index + BMI088_ACCELEROMETER_READ_DUMMY_BYTES + 1];
    }
    return true;
}

/**
 * @brief 读陀螺仪寄存器（SPI 全双工）
 * @param register_address 起始寄存器地址
 * @param data 读取数据存放缓冲区
 * @param data_length_bytes 读取字节数
 * @return true 读取成功，false 读取失败
 */
bool ClassBMI088::Read_Gyroscope_Registers(uint8_t register_address,
                                            uint8_t *data,
                                            uint8_t data_length_bytes)
{
    uint8_t transmit_data[BMI088_MAX_GYROSCOPE_TRANSFER_BYTES]{};
    uint8_t receive_data[BMI088_MAX_GYROSCOPE_TRANSFER_BYTES]{};
    const uint8_t transfer_length_bytes = data_length_bytes + 1;

    transmit_data[0] = register_address | 0x80;
    if (!gyroscope_spi_.Transmit_Receive(transmit_data,
                                         receive_data,
                                         transfer_length_bytes,
                                         SPI_TRANSFER_BLOCKING))
    {
        return false;
    }

    for (uint8_t index = 0; index < data_length_bytes; ++index)
    {
        data[index] = receive_data[index + 1];
    }
    return true;
}




