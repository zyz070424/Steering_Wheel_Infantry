#include "app_feed_rotor.h"

#include "common_math.h"

/**
 * @brief 初始化拨盘
 * @note  从配置结构体中读取电机配置并初始化拨弹电机，
 *        同时拷贝拨弹角度、堵转检测阈值、回退参数等控制参数
 *
 * @param config 拨盘配置结构体，包含电机配置与拨弹控制参数
 * @return true  初始化成功
 * @return false 初始化失败
 */
bool ClassFeedRotor::Init(const FeedRotorConfig &config)
{
    feed_djimotor_.Init(config.feed_djimotor_config);
    one_ammo_angle_deg_ = config.one_ammo_angle_deg;
    jam_torque_threshold_raw_ = config.jam_torque_threshold_raw;
    jam_duration_s_ = config.jam_duration_s;
    backoff_angle_deg_ = config.backoff_angle_deg;
    backoff_done_error_deg_ = config.backoff_done_error_deg;
    return true;
}

/**
 * @brief 启用拨盘
 * @note  启动拨弹电机，将当前电机角度锁定为目标角度，
 *        同步已处理的发射序列号，并重置堵转计时器
 */
void ClassFeedRotor::Enable(void)
{
    feed_djimotor_.Enable();
    status_.target_angle_deg = feed_djimotor_.Get_Status().total_angle_deg;
    handled_shot_sequence_ = status_.shot_sequence;
    status_.jam_elapsed_s = 0.0f;
    status_.state = FEED_ROTOR_STATE_NORMAL;
}

/**
 * @brief 停用拨盘
 * @note  停止拨弹电机，重置堵转计时器并将状态机切换到 DISABLED
 */
void ClassFeedRotor::Disable(void)
{
    feed_djimotor_.Stop();
    status_.jam_elapsed_s = 0.0f;
    status_.state = FEED_ROTOR_STATE_DISABLED;
}

/**
 * @brief 设置发射弹丸序列号
 * @note  由外部射击模块传入，增量部分将在 Update() 中驱动拨盘转动对应角度
 *
 * @param shot_sequence 累计发射弹丸数
 */
void ClassFeedRotor::Set_Shot_Sequence(uint16_t shot_sequence)
{
    status_.shot_sequence = shot_sequence;
}

/**
 * @brief 拨盘周期更新函数
 * @note  状态机驱动，需以固定频率调用。三个状态的逻辑如下：
 *        - DISABLED：仅转发控制指令，不做任何计算
 *        - NORMAL：  检测发射增量并累加目标角度，监测力矩判断堵转，
 *                    若堵转则切换到 BACKOFF 状态
 *        - BACKOFF： 电机反向回退一定角度，误差进入容限后恢复 NORMAL
 *
 * @param dt_s 距上次调用的时间间隔 (s)
 */
void ClassFeedRotor::Update(float dt_s)
{
    const DJIMotor_Status &feed_status = feed_djimotor_.Get_Status();
    float angle_error_deg;

    switch (status_.state)
    {
    case FEED_ROTOR_STATE_DISABLED:
        break;

    case FEED_ROTOR_STATE_NORMAL:
        /* 计算新增发射弹丸数，累加目标角度 */
        if (status_.shot_sequence != handled_shot_sequence_)
        {
            uint16_t shot_count = static_cast<uint16_t>(status_.shot_sequence - handled_shot_sequence_);

            status_.target_angle_deg += static_cast<float>(shot_count) * one_ammo_angle_deg_;
            handled_shot_sequence_ = status_.shot_sequence;
        }

        /* 堵转检测：力矩超限则累加计时，否则清零 */
        if (feed_status.torque_raw >= jam_torque_threshold_raw_ || feed_status.torque_raw <= -jam_torque_threshold_raw_)
        {
            status_.jam_elapsed_s += dt_s;
        }
        else
        {
            status_.jam_elapsed_s = 0.0f;
        }

        /* 堵转持续超时，设置回退目标并切换状态 */
        if (status_.jam_elapsed_s >= jam_duration_s_)
        {
            status_.target_angle_deg = feed_status.total_angle_deg - Sign(one_ammo_angle_deg_) * backoff_angle_deg_;
            status_.jam_elapsed_s = 0.0f;
            status_.state = FEED_ROTOR_STATE_BACKOFF;
        }

        feed_djimotor_.Set_Ref(status_.target_angle_deg);
        break;

    case FEED_ROTOR_STATE_BACKOFF:
        /* 计算回退角度误差，进入容限后恢复正常状态 */
        angle_error_deg = status_.target_angle_deg - feed_status.total_angle_deg;

        if (angle_error_deg <= backoff_done_error_deg_ && angle_error_deg >= -backoff_done_error_deg_)
        {
            status_.state = FEED_ROTOR_STATE_NORMAL;
        }

        feed_djimotor_.Set_Ref(status_.target_angle_deg);
        break;
    }
}

/**
 * @brief 获取拨盘当前运行状态
 *
 * @return const FeedRotorStatus& 拨盘状态结构体的常量引用
 */
const FeedRotorStatus &ClassFeedRotor::Get_Status(void) const
{
    return status_;
}
