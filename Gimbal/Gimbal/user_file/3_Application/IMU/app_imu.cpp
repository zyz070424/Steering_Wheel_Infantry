#include "app_imu.h"

#include "alg_quaternion_ekf.h"

/**
 * @brief 初始化 IMU 应用层，校验坐标轴映射配置合法性
 * @param config IMU 配置结构体，包含 BMI088 设备指针和轴映射关系
 * @return true 初始化成功，false 配置非法
 */
bool ClassImu::Init(const ImuConfig &config)
{
    if (config.bmi088 == nullptr)
    {
        return false;
    }

    bool is_sensor_axis_used[3]{};
    for (uint8_t body_axis = 0U; body_axis < 3U; ++body_axis)
    {
        const ImuAxisMapping &mapping = config.body_axis_mappings[body_axis];
        const uint8_t sensor_axis = static_cast<uint8_t>(mapping.sensor_axis);

        if (sensor_axis >= 3U ||
            (mapping.direction != 1 && mapping.direction != -1) ||
            is_sensor_axis_used[sensor_axis])
        {
            return false;
        }

        is_sensor_axis_used[sensor_axis] = true;
    }

    bmi088_ = config.bmi088;
    config_ = config;
    return true;
}

/**
 * @brief 周期更新 IMU 状态：坐标系转换 + 四元数 EKF 姿态解算
 * @param dt_s 控制周期 (s)
 * @return true 更新成功，false 传感器未就绪或数据未更新
 */
bool ClassImu::Update(float dt_s)
{
    if (bmi088_ == nullptr || dt_s <= 0.0f)
    {
        return false;
    }

    const BMI088Status bmi088_status = bmi088_->Get_Status();
    if (!bmi088_status.is_initialized ||
        bmi088_status.sample_sequence == status_.sample_sequence)
    {
        return false;
    }

    Apply_BMI088_Frame(bmi088_status);
    IMU_QuaternionEKF_Update(status_.gyroscope_body_radps[0],
                             status_.gyroscope_body_radps[1],
                             status_.gyroscope_body_radps[2],
                             status_.accelerometer_body_mps2[0],
                             status_.accelerometer_body_mps2[1],
                             status_.accelerometer_body_mps2[2],
                             dt_s);

    status_.roll_deg = QEKF_INS.Roll;
    status_.pitch_deg = QEKF_INS.Pitch;
    status_.yaw_deg = QEKF_INS.Yaw;
    status_.is_valid = true;
    return true;
}

/**
 * @brief 获取 IMU 当前状态
 * @return IMU 状态结构体
 */
ImuStatus ClassImu::Get_Status(void) const
{
    return status_;
}

/**
 * @brief 将 BMI088 传感器坐标系数据映射到机体坐标系
 * @param bmi088_status BMI088 传感器状态
 */
void ClassImu::Apply_BMI088_Frame(const BMI088Status &bmi088_status)
{
    for (uint8_t body_axis = 0U; body_axis < 3U; ++body_axis)
    {
        const ImuAxisMapping &mapping = config_.body_axis_mappings[body_axis];
        const uint8_t sensor_axis = static_cast<uint8_t>(mapping.sensor_axis);
        const float direction = static_cast<float>(mapping.direction);

        status_.accelerometer_body_mps2[body_axis] =
            direction * bmi088_status.accelerometer_mps2[sensor_axis];
        status_.gyroscope_body_radps[body_axis] =
            direction * bmi088_status.gyroscope_radps[sensor_axis];
    }

    status_.sample_sequence = bmi088_status.sample_sequence;
}
