#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include "alg_pid.h"
#include "dvc_djimotor.h"
#include "interboard_frame.h"

/**
 * @brief 底盘状态枚举
 */
typedef enum
{
    CHASSIS_STATE_DISABLED = 0,  ///< 底盘未使能
    CHASSIS_STATE_ACTIVE,        ///< 底盘运行中
} ChassisState;

/**
 * @brief 底盘配置结构体
 */
struct ChassisConfig
{
    DJIMotorConfig wheel_djimotor_configs[4]{};   ///< 四个驱动轮电机配置
    DJIMotorConfig steer_djimotor_configs[4]{};   ///< 四个转向电机配置
    PID_Config yaw_follow_pid_config{};           ///< 偏航跟随 PID 配置
    float steer_zero_offset_deg[4]{};             ///< 转向电机零位偏移角 (deg)

    float wheel_position_x_m[4] = {0.20f, 0.20f, -0.20f, -0.20f};  ///< 轮组 X 坐标 (m)
    float wheel_position_y_m[4] = {0.20f, -0.20f, 0.20f, -0.20f};  ///< 轮组 Y 坐标 (m)
    float wheel_radius_m = 0.076f;                                    ///< 驱动轮半径 (m)
    float wheel_gear_ratio = 19.0f;                                   ///< 驱动轮减速比
    float steer_gear_ratio = 1.0f;                                    ///< 转向减速比
    float spin_wz_radps = 6.0f;                                       ///< 小陀螺模式旋转角速度 (rad/s)
    float static_speed_threshold_mps = 0.01f;                         ///< 静止判定速度阈值 (m/s)
    float static_wz_threshold_radps = 0.01f;                          ///< 静止判定角速度阈值 (rad/s)
};

/**
 * @brief 底盘状态结构体
 */
struct ChassisStatus
{
    ChassisState state = CHASSIS_STATE_DISABLED;  ///< 底盘使能状态
    ChassisMode mode = CHASSIS_DISABLE;           ///< 底盘运动模式
    float vx_mps = 0.0f;                          ///< x 方向速度 (m/s)
    float vy_mps = 0.0f;                          ///< y 方向速度 (m/s)
    float wz_radps = 0.0f;                        ///< 旋转角速度 (rad/s)
    float chassis_yaw_deg = 0.0f;                 ///< 底盘偏航角 (deg)
    float gimbal_yaw_deg = 0.0f;                  ///< 云台偏航角 (deg)
    float wheel_target_speed_mps[4]{};            ///< 各驱动轮目标转速 (m/s)
    float steer_target_deg[4]{};                  ///< 各转向电机目标角度 (deg)
};

/**
 * @brief 底盘应用层：麦克纳姆轮四轮独立驱动 + 四轮独立转向
 */
class ClassChassis
{
public:
    bool Init(const ChassisConfig &config);

    void Enable();
    void Disable();

    void Set_Mode(ChassisMode mode);
    void Set_Velocity(float vx_mps, float vy_mps);
    void Set_Chassis_Yaw_Deg(float yaw_deg);
    void Set_Gimbal_Yaw_Deg(float yaw_deg);
    void Update(float dt_s);

    const ChassisStatus &Get_Status() const;

private:
    DJIMotor wheel_djimotors_[4];       ///< 四个驱动轮电机实例
    DJIMotor steer_djimotors_[4];       ///< 四个转向电机实例
    Class_PID yaw_follow_pid_;          ///< 偏航跟随 PID 控制器
    ChassisConfig config_{};            ///< 底盘配置
    ChassisStatus status_{};            ///< 底盘状态
};

#endif
