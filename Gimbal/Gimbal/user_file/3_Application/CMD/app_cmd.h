#ifndef APP_CMD_H
#define APP_CMD_H

#include "app_feed_rotor.h"
#include "app_gimbal.h"
#include "app_shoot.h"
#include "dvc_dr16.h"
#include "dvc_fdcan_comm.h"
#include "interboard_frame.h"
#include "robot_config.h"

/** @brief Cmd 初始化配置结构体 */
struct CmdConfig
{
    ClassGimbal *gimbal = nullptr;                 ///< 云台 App
    ClassShoot *shoot = nullptr;                   ///< 摩擦轮 App
    ClassFeedRotor *feed_rotor = nullptr;          ///< 拨弹盘 App
    ClassDR16 *dr16 = nullptr;                     ///< DR16，仅遥控器在本板时使用
    ClassFDCANCOMM *interboard_comm = nullptr;     ///< CAN2 板间通信设备
    float max_yaw_speed_degps = 180.0f;            ///< 右摇杆满量程对应 yaw 速度 (deg/s)
    float max_pitch_speed_radps = 1.0f;            ///< 右摇杆满量程对应 pitch 速度 (rad/s)
    float max_chassis_vx_mps = 2.0f;               ///< 左摇杆纵向满量程对应底盘速度 (m/s)
    float max_chassis_vy_mps = 2.0f;               ///< 左摇杆横向满量程对应底盘速度 (m/s)
    float continuous_shot_interval_s = 0.1f;       ///< 连发时两发弹丸的时间间隔 (s)
};

/** @brief Cmd 运行状态结构体 */
struct CmdStatus
{
    ControlCommand command{};                      ///< 当前控制周期实际使用的业务命令
};

class ClassCmd
{
public:
    bool Init(const CmdConfig &config);
    void Set_Gimbal_Mode(GimbalMode gimbal_mode);
    void Update(float dt_s);
    const CmdStatus &Get_Status(void) const;

private:
    ClassGimbal *gimbal_ = nullptr;                ///< 云台 App 指针
    ClassShoot *shoot_ = nullptr;                  ///< 摩擦轮 App 指针
    ClassFeedRotor *feed_rotor_ = nullptr;         ///< 拨弹盘 App 指针
    ClassDR16 *dr16_ = nullptr;                    ///< 本板 DR16 设备指针
    ClassFDCANCOMM *interboard_comm_ = nullptr;    ///< CAN2 板间通信设备指针
    DR16_DataTypeDef dr16_data_{};                 ///< 本板解析后的 DR16 数据
    InterboardFrame interboard_frame_{};           ///< 周期发送和接收的板间数据帧
    CmdStatus status_{};                           ///< Cmd 当前运行状态
    float max_yaw_speed_degps_ = 180.0f;           ///< yaw 最大给定速度 (deg/s)
    float max_pitch_speed_radps_ = 1.0f;           ///< pitch 最大给定速度 (rad/s)
    float max_chassis_vx_mps_ = 2.0f;              ///< 底盘纵向最大给定速度 (m/s)
    float max_chassis_vy_mps_ = 2.0f;              ///< 底盘横向最大给定速度 (m/s)
    float continuous_shot_interval_s_ = 0.1f;      ///< 连发发弹间隔 (s)
    float continuous_shot_elapsed_s_ = 0.0f;       ///< 当前连发已持续时间 (s)
    uint8_t last_shoot_trigger_ = 0U;              ///< 上一周期的发射触发状态
};

#endif
