#ifndef __APP_FEED_ROTOR_H__
#define __APP_FEED_ROTOR_H__

#include "dvc_djimotor.h"

/** @brief 拨盘状态枚举 */
typedef enum
{
    FEED_ROTOR_STATE_DISABLED = 0, ///< 停用状态，电机停止运转
    FEED_ROTOR_STATE_NORMAL,       ///< 正常状态，接受发射指令驱动拨弹
    FEED_ROTOR_STATE_BACKOFF,      ///< 堵转回退状态，检测到堵转后反转释放卡弹
} FeedRotorState;

/** @brief 拨盘配置结构体 */
struct FeedRotorConfig
{
    DJIMotorConfig feed_djimotor_config{};      ///< 拨弹电机配置
    float one_ammo_angle_deg = 45.0f;           ///< 拨出一发弹丸所需的角度增量 (°)
    float jam_torque_threshold_raw = 5000.0f;   ///< 堵转力矩检测阈值 (原始值)
    float jam_duration_s = 0.2f;                ///< 力矩超限持续多久判定为堵转 (s)
    float backoff_angle_deg = 15.0f;            ///< 堵转后回退的角度 (°)
    float backoff_done_error_deg = 3.0f;        ///< 回退完成的角度误差容限 (°)
};

/** @brief 拨盘运行状态结构体 */
struct FeedRotorStatus
{
    FeedRotorState state = FEED_ROTOR_STATE_DISABLED; ///< 当前拨盘状态
    uint16_t shot_sequence = 0U;                      ///< 累计发射弹丸序列号
    float target_angle_deg = 0.0f;                    ///< 电机目标角度 (°)
    float jam_elapsed_s = 0.0f;                       ///< 力矩超限已持续时间 (s)
};

class ClassFeedRotor
{
public:
    bool Init(const FeedRotorConfig &config);
    void Enable(void);
    void Disable(void);
    void Set_Shot_Sequence(uint16_t shot_sequence);
    void Update(float dt_s);
    const FeedRotorStatus &Get_Status(void) const;

private:
    DJIMotor feed_djimotor_{};                  ///< 拨弹电机实例
    FeedRotorStatus status_{};                  ///< 拨盘运行状态
    uint16_t handled_shot_sequence_ = 0U;       ///< 已处理的发射序列号，用于检测增量
    float one_ammo_angle_deg_ = 45.0f;          ///< 拨出一发弹丸所需的角度增量 (°)
    float jam_torque_threshold_raw_ = 5000.0f;  ///< 堵转力矩检测阈值 (原始值)
    float jam_duration_s_ = 0.2f;               ///< 力矩超限持续多久判定为堵转 (s)
    float backoff_angle_deg_ = 15.0f;           ///< 堵转后回退的角度 (°)
    float backoff_done_error_deg_ = 3.0f;       ///< 回退完成的角度误差容限 (°)
};

#endif
