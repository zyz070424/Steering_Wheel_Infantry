#include "app_shoot.h"

namespace
{
constexpr float SHOOT_RPM_TO_DEGPS = 6.0f;
}
/**
 * @brief 初始化射击类
 * 
 * @param config 射击配置
 * @return true 
 * @return false 
 */
bool ClassShoot::Init(const ShootConfig &config)
{
    left_friction_djimotor_.Init(config.left_friction_djimotor_config);
    right_friction_djimotor_.Init(config.right_friction_djimotor_config);

    status_.left_friction_target_speed_rpm = config.left_friction_target_speed_rpm;
    status_.right_friction_target_speed_rpm = config.right_friction_target_speed_rpm;
    return true;
}
/**
 * @brief 启用射击类
 * 
 */
void ClassShoot::Enable(void)
{
    left_friction_djimotor_.Enable();
    right_friction_djimotor_.Enable();
    status_.state = SHOOT_STATE_ACTIVE;
}
/**
 * @brief 停用射击类
 * 
 */ 
void ClassShoot::Disable(void)
{
    left_friction_djimotor_.Stop();
    right_friction_djimotor_.Stop();
    status_.state = SHOOT_STATE_DISABLED;
}
/**
 * @brief 设置射击类的目标速度
 * 
 * @param left_friction_target_speed_rpm 左摩擦电机目标速度
 * @param right_friction_target_speed_rpm 右摩擦电机目标速度
 */
void ClassShoot::Set_Target_Friction_Speed_Rpm(float left_friction_target_speed_rpm, float right_friction_target_speed_rpm)
{
    status_.left_friction_target_speed_rpm = left_friction_target_speed_rpm;
    status_.right_friction_target_speed_rpm = right_friction_target_speed_rpm;
}
/**
 * @brief 更新射击类的状态
 * 
 * @param dt_s 时间间隔
 */
void ClassShoot::Update(float dt_s)
{
    switch (status_.state)
    {
    case SHOOT_STATE_DISABLED:
        break;

    case SHOOT_STATE_ACTIVE:
        left_friction_djimotor_.Set_Ref(status_.left_friction_target_speed_rpm * SHOOT_RPM_TO_DEGPS);
        right_friction_djimotor_.Set_Ref(status_.right_friction_target_speed_rpm * SHOOT_RPM_TO_DEGPS);
        break;
    }
}
/**
 * @brief 获取射击类的状态
 * 
 * @return ShootStatus& 射击类的状态
 */
const ShootStatus &ClassShoot::Get_Status(void) const
{
    return status_;
}
