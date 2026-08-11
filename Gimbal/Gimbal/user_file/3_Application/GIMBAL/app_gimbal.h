#ifndef APP_GIMBAL_H
#define APP_GIMBAL_H

#include "app_imu.h"
#include "dvc_djimotor.h"
#include "dvc_dmmotor.h"
#include "dvc_vision.h"
#include "interboard_frame.h"
/**
 * @brief 云台状态枚举
 */
typedef enum
{
    GIMBAL_STATE_DISABLED = 0,  ///< 云台未使能
    GIMBAL_STATE_ACTIVE,        ///< 云台运行中
} GimbalState;

/**
 * @brief 偏航反馈源
 */
typedef enum
{
    GIMBAL_YAW_FEEDBACK_ENCODER = 0,  ///< 编码器反馈
    GIMBAL_YAW_FEEDBACK_IMU,          ///< IMU 反馈
} GimbalYawFeedback;

/**
 * @brief 云台配置结构体
 */
struct GimbalConfig
{
    DJIMotorConfig yaw_djimotor_config{};            ///< 偏航 GM6020 电机配置
    DMMotorConfig pitch_dmmotor_config{};            ///< 俯仰 DM4310 电机配置
    ImuConfig imu_config{};                          ///< IMU 配置
    ClassVision *vision = nullptr;                   ///< 视觉模块指针
    GimbalYawFeedback yaw_feedback = GIMBAL_YAW_FEEDBACK_ENCODER;  ///< 偏航反馈源
    float pitch_position_kp_nmprad = 1.0f;           ///< 俯仰位置环 Kp (N·m/rad)
    float pitch_position_kd_nmsprad = 1.0f;          ///< 俯仰位置环 Kd (N·m·s/rad)
    float pitch_gravity_comp_torque_nm = 1.0f;       ///< 俯仰重力补偿力矩 (N·m)
};

/**
 * @brief 云台状态结构体
 */
struct GimbalStatus
{
    GimbalState state = GIMBAL_STATE_DISABLED;                ///< 云台使能状态
    GimbalYawFeedback yaw_feedback = GIMBAL_YAW_FEEDBACK_ENCODER;  ///< 当前偏航反馈源
    GimbalMode mode = GIMBAL_MANUAL;                          ///< 运行模式（手动/自瞄）
    uint8_t vision_target_valid = 0U;                         ///< 视觉目标是否有效
    float yaw_deg = 0.0f;                                     ///< 当前偏航角 (deg)
    float pitch_rad = 0.0f;                                   ///< 当前俯仰角 (rad)
    float yaw_target_deg = 0.0f;                              ///< 目标偏航角 (deg)
    float pitch_target_rad = 0.0f;                            ///< 目标俯仰角 (rad)
};

/**
 * @brief 云台应用层：偏航 DJI 电机 + 俯仰达妙电机 + IMU + 视觉自瞄
 */
class ClassGimbal
{
public:
    bool Init(const GimbalConfig &config);
    void Enable(void);
    void Disable(void);
    void Set_Yaw_Feedback(GimbalYawFeedback feedback);
    void Set_Mode(GimbalMode mode);
    void Set_Target_Yaw_Deg(float target_yaw_deg);
    void Set_Target_Pitch_Rad(float target_pitch_rad);
    void Update(float dt_s);
    const GimbalStatus &Get_Status(void) const;

private:
    DJIMotor yaw_djimotor_{};                     ///< 偏航 GM6020 电机实例
    DMMotor pitch_dmmotor_{};                     ///< 俯仰 DM4310 电机实例
    ClassImu imu_{};                              ///< IMU 实例
    ClassVision *vision_ = nullptr;               ///< 视觉模块指针
    GimbalStatus status_{};                       ///< 云台状态
    float yaw_imu_deg_ = 0.0f;                   ///< IMU 累计偏航角（多圈）(deg)
    float yaw_imu_raw_deg_ = 0.0f;               ///< 上次 IMU 原始偏航角 (deg)
    float pitch_position_kp_nmprad_ = 1.0f;      ///< 俯仰位置环 Kp (N·m/rad)
    float pitch_position_kd_nmsprad_ = 1.0f;     ///< 俯仰位置环 Kd (N·m·s/rad)
    float pitch_gravity_comp_torque_nm_ = 1.0f;  ///< 俯仰重力补偿力矩 (N·m)
};

#endif
