#include "app_chassis.h"

#include <cmath>

#include "common_math.h"

/**
 * @brief 初始化底盘，包括四个轮组电机和偏航跟随 PID
 * @param config 底盘配置结构体
 * @return true 初始化成功，false 初始化失败
 */
bool ClassChassis::Init(const ChassisConfig &config)
{
    config_ = config;

    for (uint8_t index = 0; index < 4; index++)
    {
        wheel_djimotors_[index].Init(config_.wheel_djimotor_configs[index]);
        steer_djimotors_[index].Init(config_.steer_djimotor_configs[index]);
    }

    yaw_follow_pid_.Init(config_.yaw_follow_pid_config);
    status_.state = CHASSIS_STATE_DISABLED;
    status_.mode = CHASSIS_DISABLE;
    return true;
}

/**
 * @brief 使能底盘，开启所有轮组电机
 */
void ClassChassis::Enable()
{
    for (uint8_t index = 0; index < 4; index++)
    {
        wheel_djimotors_[index].Enable();
        steer_djimotors_[index].Enable();
    }

    status_.state = CHASSIS_STATE_ACTIVE;
}

/**
 * @brief 停止底盘，关闭所有轮组电机
 */
void ClassChassis::Disable()
{
    for (uint8_t index = 0; index < 4; index++)
    {
        wheel_djimotors_[index].Stop();
        steer_djimotors_[index].Stop();
    }

    status_.state = CHASSIS_STATE_DISABLED;
}

/**
 * @brief 设置底盘运行模式
 * @param mode 底盘模式枚举值
 */
void ClassChassis::Set_Mode(ChassisMode mode)
{
    status_.mode = mode;
}

/**
 * @brief 设置底盘平移速度（以云台坐标系为基准）
 * @param vx_mps x 方向速度 (m/s)
 * @param vy_mps y 方向速度 (m/s)
 */
void ClassChassis::Set_Velocity(float vx_mps, float vy_mps)
{
    status_.vx_mps = vx_mps;
    status_.vy_mps = vy_mps;
}

/**
 * @brief 设置底盘当前偏航角
 * @param yaw_deg 偏航角 (deg)
 */
void ClassChassis::Set_Chassis_Yaw_Deg(float yaw_deg)
{
    status_.chassis_yaw_deg = yaw_deg;
}

/**
 * @brief 设置云台当前偏航角
 * @param yaw_deg 偏航角 (deg)
 */
void ClassChassis::Set_Gimbal_Yaw_Deg(float yaw_deg)
{
    status_.gimbal_yaw_deg = yaw_deg;
}

/**
 * @brief 底盘周期更新函数，完成运动学逆解算并下发电机目标
 *
 * 根据当前模式计算旋转角速度 wz，将云台坐标系速度转换到车体坐标系后，
 * 通过麦克纳姆轮逆运动学求解各轮转速和转向角。
 *
 * @param dt_s 控制周期 (s)
 */
void ClassChassis::Update(float dt_s)
{
    switch (status_.mode)
    {
    case CHASSIS_DISABLE:
        Disable();
        return;

    case CHASSIS_NORMAL:
        status_.wz_radps = 0.0f;
        break;

    case CHASSIS_FOLLOW:
    {
        float target_yaw_deg = status_.chassis_yaw_deg + AngleWrapDeg(status_.gimbal_yaw_deg - status_.chassis_yaw_deg);
        status_.wz_radps = yaw_follow_pid_.Calculate(status_.chassis_yaw_deg, target_yaw_deg, dt_s);
        break;
    }

    case CHASSIS_SPIN:
        status_.wz_radps = config_.spin_wz_radps;
        break;

    default:
        break;
    }

    if (status_.state == CHASSIS_STATE_DISABLED)
    {
        return;
    }

    bool chassis_static = std::fabs(status_.vx_mps) < config_.static_speed_threshold_mps &&
                           std::fabs(status_.vy_mps) < config_.static_speed_threshold_mps &&
                           std::fabs(status_.wz_radps) < config_.static_wz_threshold_radps;

    // Cmd 中的 vx、vy 始终以云台朝向为坐标系；小陀螺时旋转到车体坐标系后再做逆解。
    float gimbal_relative_yaw_rad = DegToRad(AngleWrapDeg(status_.gimbal_yaw_deg - status_.chassis_yaw_deg));
    float chassis_vx_mps = std::cos(gimbal_relative_yaw_rad) * status_.vx_mps - std::sin(gimbal_relative_yaw_rad) * status_.vy_mps;
    float chassis_vy_mps = std::sin(gimbal_relative_yaw_rad) * status_.vx_mps + std::cos(gimbal_relative_yaw_rad) * status_.vy_mps;

    for (uint8_t index = 0; index < 4; index++)
    {
        if (chassis_static)
        {
            status_.wheel_target_speed_mps[index] = 0.0f;
            status_.steer_target_deg[index] = (index == 0 || index == 3) ? 45.0f : -45.0f;
        }
        else
        {
            float wheel_vx_mps = chassis_vx_mps - status_.wz_radps * config_.wheel_position_y_m[index];
            float wheel_vy_mps = chassis_vy_mps + status_.wz_radps * config_.wheel_position_x_m[index];
            status_.wheel_target_speed_mps[index] = std::sqrt(wheel_vx_mps * wheel_vx_mps + wheel_vy_mps * wheel_vy_mps);
            status_.steer_target_deg[index] = RadToDeg(std::atan2(wheel_vy_mps, wheel_vx_mps)) * config_.steer_gear_ratio;
        }

        float steer_current_deg = steer_djimotors_[index].Get_Status().total_angle_deg;
        float steer_motor_target_deg = status_.steer_target_deg[index] + config_.steer_zero_offset_deg[index];
        status_.steer_target_deg[index] = steer_current_deg + AngleWrapDeg(steer_motor_target_deg - steer_current_deg);

        float wheel_target_speed_degps = RadToDeg(status_.wheel_target_speed_mps[index] / config_.wheel_radius_m) * config_.wheel_gear_ratio;
        wheel_djimotors_[index].Set_Ref(wheel_target_speed_degps);
        steer_djimotors_[index].Set_Ref(status_.steer_target_deg[index]);
    }
}

/**
 * @brief 获取底盘当前状态
 * @return 底盘状态结构体引用
 */
const ChassisStatus &ClassChassis::Get_Status() const
{
    return status_;
}
