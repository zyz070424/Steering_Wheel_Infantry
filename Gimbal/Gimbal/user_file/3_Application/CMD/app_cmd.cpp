#include "app_cmd.h"

#include <cstring>

/**
 * @brief 初始化 Cmd 调度器
 * @note  Cmd 只保存各 App 和设备的引用，不保存 PID 或电机控制逻辑。
 *
 * @param config Cmd 初始化配置
 * @return true 初始化成功
 * @return false 初始化失败
 */
bool ClassCmd::Init(const CmdConfig &config)
{
    gimbal_ = config.gimbal;
    shoot_ = config.shoot;
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
 * @note  遥控器所在板调用后，Cmd 会在 CAN2 帧中同步该标志位。
 *
 * @param gimbal_mode 手动或视觉自瞄模式
 */
void ClassCmd::Set_Gimbal_Mode(GimbalMode gimbal_mode)
{
    status_.command.gimbal_mode = gimbal_mode;
}

/**
 * @brief Cmd 周期更新函数
 * @note  调度顺序如下：
 *        - 遥控器所在板：DR16 转换为 ControlCommand，并通过 CAN2 同步给另一块板
 *        - 非遥控器所在板：从 CAN2 获取同一份 ControlCommand
 *        - 按命令切换本板 App 的使能状态，并更新各 App
 *        - 所有 DJI 电机只在本函数末尾调用一次 Control_All()
 *
 * @param dt_s 距上次调用的时间间隔 (s)
 */
void ClassCmd::Update(float dt_s)
{
#if REMOTE_OWNER_BOARD == BOARD_GIMBAL
    if (dr16_->Process(&dr16_data_))
    {
        // 两个摇杆直接给出云台和底盘的速度命令；云台 App 再把速度积分为位置目标。
        status_.command.gimbal_yaw_speed_degps = dr16_data_.right_x * max_yaw_speed_degps_;
        status_.command.gimbal_pitch_speed_radps = dr16_data_.right_y * max_pitch_speed_radps_;
        status_.command.chassis_vx_mps = dr16_data_.left_y * max_chassis_vx_mps_;
        status_.command.chassis_vy_mps = dr16_data_.left_x * max_chassis_vy_mps_;
        status_.command.gimbal_enable = 1U;

        // 左三档拨杆只选择底盘模式；PID 跟随和小陀螺的 wz 都由底盘 App 计算。
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

        // 右三档拨杆选择射击模式；射击机构具体安装在哪块板不影响这个业务命令。
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

        // 此处假定左拨轮下拨后归一化值为负；若实物方向相反，只改比较符号。
        if(dr16_data_.left_trigger < -0.5)
        {
            status_.command.shoot_trigger = 1;
        }
        else 
        {
            status_.command.shoot_trigger = 0;
        }

    }
#else
    // 遥控器装在底盘板时，云台板只取 CAN2 发来的业务命令。
    if (interboard_comm_->Has_New_Data())
    {
        std::memcpy(&interboard_frame_, interboard_comm_->Get_Rx_Data(), sizeof(interboard_frame_));
        status_.command = interboard_frame_.command;
        interboard_comm_->Clear_New_Data();
    }
#endif

    // 发射序列号只能在遥控器所在板生成，避免两块板对同一次触发重复计数。
#if REMOTE_OWNER_BOARD == BOARD_GIMBAL
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

    gimbal_->Set_Mode(status_.command.gimbal_mode);

    if (status_.command.gimbal_enable != 0U)
    {
        if (gimbal_->Get_Status().state == GIMBAL_STATE_DISABLED)
        {
            gimbal_->Enable();
        }

        gimbal_->Set_Target_Yaw_Deg(gimbal_->Get_Status().yaw_target_deg + status_.command.gimbal_yaw_speed_degps * dt_s);
        gimbal_->Set_Target_Pitch_Rad(gimbal_->Get_Status().pitch_target_rad + status_.command.gimbal_pitch_speed_radps * dt_s);
    }
    else if (gimbal_->Get_Status().state == GIMBAL_STATE_ACTIVE)
    {
        gimbal_->Disable();
    }

    if (status_.command.shoot_mode != SHOOT_DISABLE)
    {
        if (shoot_->Get_Status().state == SHOOT_STATE_DISABLED)
        {
            shoot_->Enable();
        }
    }
    else if (shoot_->Get_Status().state == SHOOT_STATE_ACTIVE)
    {
        shoot_->Disable();
    }

#if FEED_ROTOR_OWNER_BOARD == BOARD_GIMBAL
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

    gimbal_->Update(dt_s);
    shoot_->Update(dt_s);

#if FEED_ROTOR_OWNER_BOARD == BOARD_GIMBAL
    feed_rotor_->Update(dt_s);
#endif

    // 遥控器所在板拥有 command 字段，云台板拥有 gimbal_feedback 字段。
    interboard_frame_.command = status_.command;
    interboard_frame_.gimbal_feedback.yaw_deg = gimbal_->Get_Status().yaw_deg;
    interboard_frame_.gimbal_feedback.pitch_rad = gimbal_->Get_Status().pitch_rad;
    interboard_comm_->FDCANCommSend(reinterpret_cast<uint8_t *>(&interboard_frame_));

    // Control_All 会计算并发送本板全部 DJI 电机，不能在每个 App 内重复调用。
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
