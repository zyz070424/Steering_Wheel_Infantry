#ifndef DVC_DMMOTOR_H
#define DVC_DMMOTOR_H

#include <cstdint>

#include "drv_fdcan.h"
#include "dvc_motor_config.h"

/**
 * @brief 达妙电机类型
 */
typedef enum
{
    DMMOTOR_TYPE_NONE = 0,  ///< 未指定
    DM_J4310 = 1,           ///< J4310 电机
} DMMotor_type_e;

/**
 * @brief 达妙电机控制指令
 */
typedef enum
{
    DM_CMD_MOTOR_MODE = 0xFC,     ///< 进入电机模式
    DM_CMD_RESET_MODE = 0xFD,     ///< 退出电机模式
    DM_CMD_ZERO_POSITION = 0xFE,  ///< 设置零位
    DM_CMD_CLEAR_ERROR = 0xFB,    ///< 清除错误
} DMMotor_Mode_e;

/**
 * @brief MIT 控制指令结构体
 */
struct DMMotorMitCommand
{
    float position_rad = 0.0f;    ///< 目标位置 (rad)
    float velocity_radps = 0.0f;  ///< 目标速度 (rad/s)
    float kp_nmprad = 0.0f;       ///< 位置增益 (N·m/rad)
    float kd_nmsprad = 0.0f;      ///< 速度增益 (N·m·s/rad)
    float torque_ff_nm = 0.0f;    ///< 前馈力矩 (N·m)
};

/**
 * @brief 达妙电机状态结构体
 */
struct DMMotorStatus
{
    uint8_t motor_id = 0;                ///< 电机 ID
    uint8_t fault_code = 0;              ///< 故障码
    float position_rad = 0.0f;           ///< 当前位置 (rad)
    float velocity_radps = 0.0f;         ///< 当前速度 (rad/s)
    float torque_nm = 0.0f;              ///< 当前力矩 (N·m)
    uint8_t mos_temperature_c = 0;       ///< MOS 管温度 (℃)
    uint8_t rotor_temperature_c = 0;     ///< 转子温度 (℃)
    bool is_feedback_valid = false;       ///< 反馈数据是否有效
};

/**
 * @brief 达妙电机配置结构体
 */
struct DMMotorConfig
{
    CanMotorInitConfig base{};                   ///< CAN 电机基础配置
    DMMotor_type_e motor_type = DMMOTOR_TYPE_NONE;  ///< 电机类型
};

/**
 * @brief 达妙电机设备驱动层
 */
class DMMotor
{
public:
    bool Init(const DMMotorConfig &config);
    bool Enable();
    bool Stop();
    bool Set_Zero_Position();
    bool Clear_Error();
    bool Send_Mit_Command(const DMMotorMitCommand &command);
    const DMMotorStatus &Get_Status() const;

private:
    bool Send_Mode_Command(DMMotor_Mode_e command);
    void Handle_FDCAN_Feedback(const StructFDCANRxFrame &rx_frame);

    static void FDCAN_Rx_Callback(ClassFDCAN *fdcan);
    static uint16_t Float_To_Uint(float value,
                                  float min_value,
                                  float max_value,
                                  uint8_t bit_count);
    static float Uint_To_Float(uint16_t value,
                               float min_value,
                               float max_value,
                               uint8_t bit_count);

    ClassFDCAN fdcan_{};                                          ///< FDCAN 通信实例
    DMMotorStatus status_{};                                      ///< 电机状态
    DMMotor_type_e motor_type_ = DMMOTOR_TYPE_NONE;               ///< 电机型号
    Motor_Reverse_Flag_e motor_reverse_flag_ = MOTOR_DIRECTION_NORMAL;          ///< 电机方向
    Feedback_Reverse_Flag_e feedback_reverse_flag_ = FEEDBACK_DIRECTION_NORMAL;  ///< 反馈方向
    Motor_Working_Type_e working_type_ = MOTOR_STOP;              ///< 工作状态
    bool is_initialized_ = false;                                  ///< 初始化标志
};

#endif /* DVC_DMMOTOR_H */
