#include "app_cmd.h"

#include <cstring>

/**
 * @brief 初始化底盘 Cmd 调度器
 * @note  Cmd 只保存各 App 和设备的引用，不保存 PID 或电机控制逻辑。
 *
 * @param config Cmd 初始化配置
 * @return true 初始化成功
 * @return false 初始化失败
 */
bool ClassCmd::Init(const CmdConfig &config)
{
    chassis_ = config.chassis;
    imu_ = config.imu;
    feed_rotor_ = config.feed_rotor;
    dr16_ = config.dr16;
    interboard_comm_ = config.interboard_comm;
    max_yaw_speed_degps_ = config.max_yaw_speed_degps;
    max_pitch_speed_radps_ = config.max_pitch_speed_radps;
    max_chassis_vx_mps_ = config.max_chassis_vx_mps;
    max_chassis_vy_mps_ = config.max_chassis_vy_mps;
    continuous_shot_interval_s_ = config.continuous_shot_interval_s;
    return true;
}

/**
 * @brief 设置云台业务模式标志位
 * @note  遥控器安装在底盘板时，调用后会通过 CAN2 同步给云台板。
 *
 * @param gimbal_mode 手动或视觉自瞄模式
 */
void ClassCmd::Set_Gimbal_Mode(GimbalMode gimbal_mode)
{
    status_.command.gimbal_mode = gimbal_mode;
}

/**
 * @brief 底盘 Cmd 周期更新函数
 * @note  遥控器所在板生成 ControlCommand，另一块板只从 CAN2 取该命令；
 *        跟随 PID、舵轮逆解和电机闭环全部留在底盘 App 内。
 *
 * @param dt_s 距上次调用的时间间隔 (s)
 */
void ClassCmd::Update(float dt_s)
{
#if REMOTE_OWNER_BOARD == BOARD_CHASSIS
    if (dr16_->Process(&dr16_data_))
    {
        // 两个摇杆同时生成云台和底盘业务命令，CAN2 只传递业务量而不传 DR16 原始帧。
        status_.command.gimbal_yaw_speed_degps = dr16_data_.right_x * max_yaw_speed_degps_;
        status_.command.gimbal_pitch_speed_radps = dr16_data_.right_y * max_pitch_speed_radps_;
        status_.command.chassis_vx_mps = dr16_data_.left_y * max_chassis_vx_mps_;
        status_.command.chassis_vy_mps = dr16_data_.left_x * max_chassis_vy_mps_;
        status_.command.gimbal_enable = 1U;

        // 左三档拨杆：上小陀螺，中底盘跟随云台，下底盘失能。
        switch (dr16_data_.raw_s1)
        {
        case DR16_SWITCH_UP:
            status_.command.chassis_mode = CHASSIS_SPIN;
            break;

        case DR16_SWITCH_MIDDLE:
            status_.command.chassis_mode = CHASSIS_FOLLOW;
            break;

        default:
            status_.command.chassis_mode = CHASSIS_DISABLE;
            break;
        }

        // 右三档拨杆选择射击模式；拨弹盘在哪一块板由预编译宏决定。
        switch (dr16_data_.raw_s2)
        {
        case DR16_SWITCH_UP:
            status_.command.shoot_mode = SHOOT_CONTINUOUS;
            break;

        case DR16_SWITCH_MIDDLE:
            status_.command.shoot_mode = SHOOT_SINGLE;
            break;

        default:
            status_.command.shoot_mode = SHOOT_DISABLE;
            break;
        }

        // 此处假定左拨轮下拨后的归一化值为负；实物相反时只改比较符号。
        status_.command.shoot_trigger = dr16_data_.left_trigger <= -0.5f ? 1U : 0U;
    }
#else
    // 遥控器装在云台板时，底盘板只使用 CAN2 发来的业务命令。
    if (interboard_comm_->Has_New_Data())
    {
        std::memcpy(&interboard_frame_, interboard_comm_->Get_Rx_Data(), sizeof(interboard_frame_));
        status_.command = interboard_frame_.command;
        interboard_comm_->Clear_New_Data();
    }
#endif

#if REMOTE_OWNER_BOARD == BOARD_CHASSIS
    // 发射序列号只能在遥控器所在板生成，避免两块板对同一次触发重复计数。
    if (status_.command.shoot_mode == SHOOT_SINGLE)
    {
        continuous_shot_elapsed_s_ = 0.0f;

        if (status_.command.shoot_trigger != 0U && last_shoot_trigger_ == 0U)
        {
            ++status_.command.shot_sequence;
        }
    }
    else if (status_.command.shoot_mode == SHOOT_CONTINUOUS)
    {
        if (status_.command.shoot_trigger != 0U)
        {
            continuous_shot_elapsed_s_ += dt_s;

            if (continuous_shot_elapsed_s_ >= continuous_shot_interval_s_)
            {
                ++status_.command.shot_sequence;
                continuous_shot_elapsed_s_ = 0.0f;
            }
        }
        else
        {
            continuous_shot_elapsed_s_ = 0.0f;
        }
    }
    else
    {
        continuous_shot_elapsed_s_ = 0.0f;
    }

    last_shoot_trigger_ = status_.command.shoot_trigger;
#endif

    imu_->Update(dt_s);
    chassis_->Set_Mode(status_.command.chassis_mode);
    chassis_->Set_Velocity(status_.command.chassis_vx_mps, status_.command.chassis_vy_mps);
    chassis_->Set_Chassis_Yaw_Deg(imu_->Get_Status().yaw_deg);
    chassis_->Set_Gimbal_Yaw_Deg(interboard_frame_.gimbal_feedback.yaw_deg);

    if (status_.command.chassis_mode != CHASSIS_DISABLE && chassis_->Get_Status().state == CHASSIS_STATE_DISABLED)
    {
        chassis_->Enable();
    }

#if FEED_ROTOR_OWNER_BOARD == BOARD_CHASSIS
    if (status_.command.shoot_mode != SHOOT_DISABLE)
    {
        if (feed_rotor_->Get_Status().state == FEED_ROTOR_STATE_DISABLED)
        {
            feed_rotor_->Enable();
        }

        feed_rotor_->Set_Shot_Sequence(status_.command.shot_sequence);
    }
    else if (feed_rotor_->Get_Status().state != FEED_ROTOR_STATE_DISABLED)
    {
        feed_rotor_->Disable();
    }
#endif

    chassis_->Update(dt_s);

#if FEED_ROTOR_OWNER_BOARD == BOARD_CHASSIS
    feed_rotor_->Update(dt_s);
#endif

    // 底盘板回传最近收到的业务命令；云台反馈字段由云台板填充。
    interboard_frame_.command = status_.command;
    interboard_comm_->FDCANCommSend(reinterpret_cast<uint8_t *>(&interboard_frame_));

    // 本板 8 个 DJI 电机只在此处统一计算和发送一次。
    DJIMotor::Control_All(dt_s);
}

/**
 * @brief 获取 Cmd 当前状态
 *
 * @return const CmdStatus& Cmd 状态结构体的常量引用
 */
const CmdStatus &ClassCmd::Get_Status(void) const
{
    return status_;
}
