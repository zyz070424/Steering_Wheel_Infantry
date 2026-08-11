#ifndef APP_IMU_H
#define APP_IMU_H

#include <cstdint>

#include "dvc_bmi088.h"

/**
 * @brief IMU 传感器轴标识
 */
enum class ImuSensorAxis : uint8_t
{
    X = 0U,  ///< X 轴
    Y = 1U,  ///< Y 轴
    Z = 2U,  ///< Z 轴
};

/**
 * @brief 轴映射关系：将机体轴映射到传感器轴及方向
 */
struct ImuAxisMapping
{
    ImuSensorAxis sensor_axis = ImuSensorAxis::X;  ///< 对应的传感器轴
    int8_t direction = 1;                           ///< 方向（+1 或 -1）
};

/**
 * @brief IMU 配置结构体
 */
struct ImuConfig
{
    ClassBMI088 *bmi088 = nullptr;  ///< BMI088 设备指针
    ImuAxisMapping body_axis_mappings[3] = {  ///< 机体轴到传感器轴的映射
        {ImuSensorAxis::X, 1},
        {ImuSensorAxis::Y, 1},
        {ImuSensorAxis::Z, 1},
    };
};

/**
 * @brief IMU 状态结构体
 */
struct ImuStatus
{
    float accelerometer_body_mps2[3]{};  ///< 机体坐标系加速度 (m/s²)
    float gyroscope_body_radps[3]{};     ///< 机体坐标系角速度 (rad/s)
    float roll_deg = 0.0f;               ///< 横滚角 (deg)
    float pitch_deg = 0.0f;              ///< 俯仰角 (deg)
    float yaw_deg = 0.0f;                ///< 偏航角 (deg)
    uint32_t sample_sequence = 0;         ///< 采样序列号
    bool is_valid = false;                ///< 姿态解算是否有效
};

/**
 * @brief IMU 应用层：在此完成坐标系转换并向 QEKF 提供物理量。
 */
class ClassImu
{
public:
    bool Init(const ImuConfig &config);
    bool Update(float dt_s);
    ImuStatus Get_Status(void) const;

private:
    void Apply_BMI088_Frame(const BMI088Status &bmi088_status);

    ClassBMI088 *bmi088_ = nullptr;  ///< BMI088 设备指针
    ImuConfig config_{};             ///< 轴映射配置
    ImuStatus status_{};             ///< IMU 状态
};

#endif
