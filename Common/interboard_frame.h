#ifndef INTERBOARD_FRAME_H
#define INTERBOARD_FRAME_H

#include <cstdint>

// 底盘模式在底盘 App 内决定 wz 的来源。
typedef enum
{
    CHASSIS_DISABLE = 0,
    CHASSIS_NORMAL,
    CHASSIS_FOLLOW,
    CHASSIS_SPIN,
} ChassisMode;

// 射击模式只描述业务意图，摩擦轮和拨弹盘由安装所在板的 Cmd 应用。
typedef enum
{
    SHOOT_DISABLE = 0,
    SHOOT_SINGLE,
    SHOOT_CONTINUOUS,
} ShootMode;

// 云台模式属于遥控器业务命令，必须经 CAN2 同步到云台板。
typedef enum
{
    GIMBAL_MANUAL = 0,
    GIMBAL_VISION,
} GimbalMode;

// 遥控器所在板生成的业务命令。另一块板只消费这些物理量，不接触 DR16 原始数据。
struct ControlCommand
{
    float gimbal_yaw_speed_degps = 0.0f;
    float gimbal_pitch_speed_radps = 0.0f;
    float chassis_vx_mps = 0.0f;
    float chassis_vy_mps = 0.0f;
    uint16_t shot_sequence = 0U;
    uint8_t gimbal_enable = 0U;
    uint8_t shoot_trigger = 0U;
    GimbalMode gimbal_mode = GIMBAL_MANUAL;
    ChassisMode chassis_mode = CHASSIS_DISABLE;
    ShootMode shoot_mode = SHOOT_DISABLE;
    uint8_t reserved = 0U;
};

// 云台板回传给底盘的姿态反馈，底盘跟随和小陀螺只使用这里的物理量。
struct GimbalFeedback
{
    float yaw_deg = 0.0f;
    float pitch_rad = 0.0f;
};

// CAN2 双向周期帧。两边均以相同布局编译，大小受 FDCANCOMM 的 60 字节缓冲限制。
struct InterboardFrame
{
    ControlCommand command{};
    GimbalFeedback gimbal_feedback{};
};

static_assert(sizeof(InterboardFrame) <= 60U, "InterboardFrame exceeds FDCANCOMM buffer");

#endif
