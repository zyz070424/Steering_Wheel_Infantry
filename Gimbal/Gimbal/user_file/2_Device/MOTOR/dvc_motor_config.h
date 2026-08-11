#ifndef __DVC_MOTOR_CONFIG_H__
#define __DVC_MOTOR_CONFIG_H__

#include "alg_pid.h"
#include "drv_fdcan.h"

/**
 * @brief 闭环控制类型（位掩码组合）
 */
typedef enum
{
    OPEN_LOOP = 0,              ///< 开环
    SPEED_LOOP = 0b0001,        ///< 速度环
    ANGLE_LOOP = 0b0010,        ///< 角度环
    CURRENT_LOOP = 0b0100,      ///< 电流环

    ANGLE_AND_SPEED_LOOP = 0b0011,    ///< 角度 + 速度串级
    SPEED_AND_CURRENT_LOOP = 0b0101,  ///< 速度 + 电流串级
    ALL_LOOP = 0b0111,                ///< 全闭环
} Closeloop_Type_e;

/**
 * @brief 电机工作状态
 */
typedef enum
{
    MOTOR_STOP = 0,      ///< 停止
    MOTOR_ENABLED = 1,   ///< 使能
} Motor_Working_Type_e;

/**
 * @brief 角度反馈源
 */
typedef enum
{
    Encoder_Feedback = 0,  ///< 编码器反馈
    IMU_Feedback = 1,      ///< IMU 反馈
} Angle_Feedback_Source_e;

/**
 * @brief 电机方向标志
 */
typedef enum
{
    MOTOR_DIRECTION_NORMAL = 0,   ///< 正向
    MOTOR_DIRECTION_REVERSE = 1,  ///< 反向
} Motor_Reverse_Flag_e;

/**
 * @brief 反馈方向标志
 */
typedef enum
{
    FEEDBACK_DIRECTION_NORMAL = 0,   ///< 正向
    FEEDBACK_DIRECTION_REVERSE = 1,  ///< 反向
} Feedback_Reverse_Flag_e;

/**
 * @brief 电机方向配置
 */
struct MotorDirectionConfig
{
    Motor_Reverse_Flag_e motor_reverse_flag = MOTOR_DIRECTION_NORMAL;          ///< 电机方向
    Feedback_Reverse_Flag_e feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL;  ///< 反馈方向
};

/**
 * @brief 电机 CAN 总线配置
 */
struct MotorCanConfig
{
    FDCAN_HandleTypeDef *fdcan_handle = nullptr;  ///< HAL FDCAN 句柄
    uint32_t tx_id = 0;                           ///< 发送 ID（电机 ID）
    uint32_t rx_id = 0;                           ///< 接收 ID（由 Init 自动计算）
};

/**
 * @brief 电机基础初始化配置
 */
struct MotorInitConfig
{
    MotorDirectionConfig direction{};  ///< 方向配置
};

/**
 * @brief CAN 电机初始化配置
 */
struct CanMotorInitConfig
{
    MotorInitConfig motor{};  ///< 电机基础配置
    MotorCanConfig can{};     ///< CAN 总线配置
};

/**
 * @brief 电机控制环路配置
 */
struct MotorControlConfig
{
    Closeloop_Type_e close_loop_type = OPEN_LOOP;             ///< 闭环类型
    Closeloop_Type_e outer_loop_type = ANGLE_LOOP;            ///< 外环类型
    Angle_Feedback_Source_e angle_feedback_source = Encoder_Feedback;  ///< 角度反馈源
};

/**
 * @brief 电机 PID 控制器初始化配置
 */
struct MotorControllerInitConfig
{
    const float *other_angle_feedback_ptr = nullptr;  ///< 外部角度反馈指针（如 IMU）
    PID_Config current_pid{};                         ///< 电流环 PID 参数
    PID_Config speed_pid{};                           ///< 速度环 PID 参数
    PID_Config angle_pid{};                           ///< 角度环 PID 参数
};

#endif /*__DVC_MOTOR_CONFIG_H__*/
