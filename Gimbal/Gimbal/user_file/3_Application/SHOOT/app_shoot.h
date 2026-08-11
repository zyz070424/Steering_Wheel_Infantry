#ifndef APP_SHOOT_H
#define APP_SHOOT_H

#include "dvc_djimotor.h"

/**
 * @brief 射击状态枚举
 */
typedef enum
{
    SHOOT_STATE_DISABLED = 0,  ///< 射击未使能
    SHOOT_STATE_ACTIVE,        ///< 射击运行中
} ShootState;

/**
 * @brief 射击配置结构体
 */
struct ShootConfig
{
    DJIMotorConfig left_friction_djimotor_config{};    ///< 左摩擦轮电机配置
    DJIMotorConfig right_friction_djimotor_config{};   ///< 右摩擦轮电机配置
    float left_friction_target_speed_rpm = 0.0f;       ///< 左摩擦轮目标转速 (RPM)
    float right_friction_target_speed_rpm = 0.0f;      ///< 右摩擦轮目标转速 (RPM)
};

/**
 * @brief 射击状态结构体
 */
struct ShootStatus
{
    ShootState state = SHOOT_STATE_DISABLED;           ///< 射击使能状态
    float left_friction_target_speed_rpm = 0.0f;       ///< 左摩擦轮目标转速 (RPM)
    float right_friction_target_speed_rpm = 0.0f;      ///< 右摩擦轮目标转速 (RPM)
};

/**
 * @brief 射击应用层：左右摩擦轮 DJI 电机控制
 */
class ClassShoot
{
public:
    bool Init(const ShootConfig &config);
    void Enable(void);
    void Disable(void);
    void Set_Target_Friction_Speed_Rpm(float left_friction_target_speed_rpm, float right_friction_target_speed_rpm);
    void Update(float dt_s);
    const ShootStatus &Get_Status(void) const;

private:
    DJIMotor left_friction_djimotor_{};   ///< 左摩擦轮电机实例
    DJIMotor right_friction_djimotor_{};  ///< 右摩擦轮电机实例
    ShootStatus status_{};                ///< 射击状态
};

#endif
