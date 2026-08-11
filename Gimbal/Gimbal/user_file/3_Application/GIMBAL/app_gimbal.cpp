#include "app_gimbal.h"

#include <cmath>

#include "common_math.h"
/**
 * @brief 初始化云台
 * 
 * @param config 云台配置
 * @return true 初始化成功
 * @return false 初始化失败
 */
bool ClassGimbal::Init(const GimbalConfig &config)
{
    DJIMotorConfig yaw_djimotor_config = config.yaw_djimotor_config;

    yaw_djimotor_config.controller.other_angle_feedback_ptr = &yaw_imu_deg_;

    imu_.Init(config.imu_config);
    yaw_djimotor_.Init(yaw_djimotor_config);
    pitch_dmmotor_.Init(config.pitch_dmmotor_config);

    vision_ = config.vision;
    status_.yaw_feedback = config.yaw_feedback;
    pitch_position_kp_nmprad_ = config.pitch_position_kp_nmprad;
    pitch_position_kd_nmsprad_ = config.pitch_position_kd_nmsprad;
    pitch_gravity_comp_torque_nm_ = config.pitch_gravity_comp_torque_nm;
    return true;
}
/**
 * @brief 使能云台
 * 
 */
void ClassGimbal::Enable(void)
{
    yaw_djimotor_.Enable();
    pitch_dmmotor_.Enable();
    Set_Yaw_Feedback(status_.yaw_feedback);
    status_.pitch_target_rad = pitch_dmmotor_.Get_Status().position_rad;
    status_.state = GIMBAL_STATE_ACTIVE;
}
/**
 * @brief 禁用云台
 * 
 */
void ClassGimbal::Disable(void)
{
    yaw_djimotor_.Stop();
    pitch_dmmotor_.Stop();
    status_.state = GIMBAL_STATE_DISABLED;
}
/**
 * @brief 设置云台反馈模式
 * 
 * @param feedback 反馈模式
 */
void ClassGimbal::Set_Yaw_Feedback(GimbalYawFeedback feedback)
{
    status_.yaw_feedback = feedback;

    if (feedback == GIMBAL_YAW_FEEDBACK_IMU)
    {
        status_.yaw_target_deg = yaw_imu_deg_;
        yaw_djimotor_.Change_Angle_Feed(IMU_Feedback, status_.yaw_target_deg);
    }
    else
    {
        status_.yaw_target_deg = yaw_djimotor_.Get_Status().total_angle_deg;
        yaw_djimotor_.Change_Angle_Feed(Encoder_Feedback, status_.yaw_target_deg);
    }
}
/**
 * @brief 设置云台控制模式标志位
 *
 * @param mode 手动控制或视觉自瞄
 */
void ClassGimbal::Set_Mode(GimbalMode mode)
{
    status_.mode = mode;
}
/**
 * @brief 设置云台目标角度
 * 
 * @param target_yaw_deg 目标角度（度）
 */
void ClassGimbal::Set_Target_Yaw_Deg(float target_yaw_deg)
{
    float yaw_current_deg;

    if (status_.yaw_feedback == GIMBAL_YAW_FEEDBACK_IMU)
    {
        yaw_current_deg = yaw_imu_deg_;
    }
    else
    {
        yaw_current_deg = yaw_djimotor_.Get_Status().total_angle_deg;
    }

    status_.yaw_target_deg = yaw_current_deg + AngleWrapDeg(target_yaw_deg - yaw_current_deg);
}
/**
 * @brief 设置云台目标角度
 * 
 * @param target_pitch_rad 目标角度（弧度）
 */
void ClassGimbal::Set_Target_Pitch_Rad(float target_pitch_rad)
{
    status_.pitch_target_rad = target_pitch_rad;
}
/**
 * @brief 更新云台状态
 * 
 * @param dt_s 时间步长（秒）
 */
void ClassGimbal::Update(float dt_s)
{
    float yaw_imu_raw_deg;

    imu_.Update(dt_s);
    yaw_imu_raw_deg = imu_.Get_Status().yaw_deg;
    yaw_imu_deg_ += AngleWrapDeg(yaw_imu_raw_deg - yaw_imu_raw_deg_);
    yaw_imu_raw_deg_ = yaw_imu_raw_deg;
    status_.yaw_deg = status_.yaw_feedback == GIMBAL_YAW_FEEDBACK_IMU ? yaw_imu_deg_ : yaw_djimotor_.Get_Status().total_angle_deg;
    status_.pitch_rad = pitch_dmmotor_.Get_Status().position_rad;

    if (status_.mode == GIMBAL_VISION && vision_ != nullptr)
    {
        Vision_UART_Rx_Data vision_rx_data = vision_->Get_Vision_Rx_Data();

        status_.vision_target_valid = vision_rx_data.Target_Valid;
        if (vision_rx_data.Target_Valid != 0U)
        {
            Set_Target_Yaw_Deg(vision_rx_data.Target_Yaw_Deg);
            Set_Target_Pitch_Rad(DegToRad(vision_rx_data.Target_Pitch_Deg));
        }
    }
    else
    {
        status_.vision_target_valid = 0U;
    }

    switch (status_.state)
    {
    case GIMBAL_STATE_DISABLED:
        break;

    case GIMBAL_STATE_ACTIVE:
    {
        DMMotorMitCommand pitch_command{};

        yaw_djimotor_.Set_Ref(status_.yaw_target_deg);

        pitch_command.position_rad = status_.pitch_target_rad;
        pitch_command.velocity_radps = 0.0f;
        pitch_command.kp_nmprad = pitch_position_kp_nmprad_;
        pitch_command.kd_nmsprad = pitch_position_kd_nmsprad_;
        //后面的补偿都放在这个地方
        pitch_command.torque_ff_nm = pitch_gravity_comp_torque_nm_ * std::cos(pitch_dmmotor_.Get_Status().position_rad);
        pitch_dmmotor_.Send_Mit_Command(pitch_command);
        break;
    }

    }

    if (vision_ != nullptr)
    {
        Vision_UART_Tx_Data vision_tx_data{};

        vision_tx_data.Yaw_Deg = yaw_imu_deg_;
        vision_tx_data.Pitch_Deg = imu_.Get_Status().pitch_deg;
        vision_->Send(vision_tx_data);
    }
}
/**
 * @brief 获取云台状态
 * 
 * @return GimbalStatus 云台状态
 */
const GimbalStatus &ClassGimbal::Get_Status(void) const
{
    return status_;
}
