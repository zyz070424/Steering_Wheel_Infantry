#ifndef DVC_DJIMOTOR_H
#define DVC_DJIMOTOR_H

#include <cstdint>

#include "alg_pid.h"
#include "drv_fdcan.h"
#include "dvc_motor_config.h"

/**
 * @brief DJI 电机类型
 */
typedef enum
{
    DJIMOTOR_TYPE_NONE = 0,  ///< 未指定
    GM6020,                  ///< GM6020 云台电机
    M3508,                   ///< M3508 减速电机
} DIJMotor_Type_e;

/**
 * @brief DJI 电机配置结构体
 */
struct DJIMotorConfig
{
    CanMotorInitConfig base{};                        ///< CAN 电机基础配置
    DIJMotor_Type_e motor_type = DJIMOTOR_TYPE_NONE;  ///< 电机类型
    MotorControlConfig control{};                     ///< 控制环路配置
    MotorControllerInitConfig controller{};           ///< PID 控制器配置
};

/**
 * @brief DJI 电机状态结构体
 */
typedef struct
{
    uint16_t encoder_cnt = 0;         ///< 编码器原始计数值 (0-8191)
    float angle_single_deg = 0.0f;    ///< 单圈角度 (deg)
    float total_angle_deg = 0.0f;     ///< 累计角度（支持多圈）(deg)
    float speed_degps = 0.0f;         ///< 转速 (deg/s)
    int16_t torque_raw = 0;           ///< 原始力矩反馈值
    uint8_t temperature_c = 0;        ///< 电机温度 (℃)
    int16_t output_torque_cmd = 0;    ///< 输出力矩控制命令
    bool is_feedback_valid = false;    ///< 反馈数据是否有效
} DJIMotor_Status;

/**
 * @brief DJI 电机设备驱动层
 *
 * 支持 GM6020 和 M3508 两种电机，通过 FDCAN 总线通信，
 * 内部实现多级 PID 串级控制和编码器多圈累计。
 */
class DJIMotor
{
public:
    bool Init(const DJIMotorConfig &config);
    void Enable();
    void Stop();
    void Set_Ref(float ref);
    void Control(float dt_s);
    static void Control_All(float dt_s);

    void Change_Angle_Feed(Angle_Feedback_Source_e source, float ref);

    DIJMotor_Type_e Get_Motor_Type() const;
    const DJIMotor_Status &Get_Status() const;
    // TODO: 在后续电机控制任务中，把各电机命令按 CAN 分组后统一发送。
    int16_t Get_Output_torque_Cmd() const;

private:
    static constexpr uint8_t DJI_MOTOR_MAX_ID = 8;
    static constexpr uint8_t DJI_MOTOR_GROUP_COUNT_PER_BUS = 3;
    static constexpr uint8_t DJI_MOTOR_MAX_FDCAN_BUS_COUNT = 3;
    static constexpr uint8_t DJI_MOTOR_SENDER_GROUP_COUNT =  DJI_MOTOR_GROUP_COUNT_PER_BUS * DJI_MOTOR_MAX_FDCAN_BUS_COUNT;

    struct DJIMotor_Sender_Group
    {
        FDCAN_HandleTypeDef *fdcan_handle = nullptr;
        uint32_t tx_id = 0;
        uint8_t tx_data[ClassFDCAN::FDCAN_MAX_DATA_LENGTH_BYTES]{};
        bool is_enabled = false;
    };

    static void FDCAN_Rx_Callback(ClassFDCAN *fdcan);
    bool Register_Sender_Group(FDCANConfig *fdcan_config,
                               DIJMotor_Type_e motor_type);

    void Handle_FDCAN_Feedback(const StructFDCANRxFrame &rx_frame);

    static DJIMotor *motor_instances_[ClassFDCAN::FDCAN_MAX_INSTANCE_COUNT];
    static uint8_t motor_instance_count_;
    static DJIMotor_Sender_Group sender_groups_[DJI_MOTOR_SENDER_GROUP_COUNT];

    ClassFDCAN fdcan_{};                                  ///< FDCAN 通信实例
    Class_PID current_pid_{};                             ///< 电流环 PID 控制器
    Class_PID speed_pid_{};                               ///< 速度环 PID 控制器
    Class_PID angle_pid_{};                               ///< 角度环 PID 控制器

    DJIMotor_Status DJIMotor_status_{};                   ///< 电机状态
    DIJMotor_Type_e motor_type_ = DJIMOTOR_TYPE_NONE;     ///< 电机型号
    Closeloop_Type_e close_loop_type_ = OPEN_LOOP;        ///< 闭环类型
    Closeloop_Type_e outer_loop_type_ = ANGLE_LOOP;       ///< 外环类型
    Motor_Working_Type_e working_type_ = MOTOR_STOP;      ///< 工作状态
    Angle_Feedback_Source_e angle_feedback_source_ = Encoder_Feedback;  ///< 角度反馈源
    Motor_Reverse_Flag_e motor_reverse_flag_ = MOTOR_DIRECTION_NORMAL;          ///< 电机方向
    Feedback_Reverse_Flag_e feedback_reverse_flag_ = FEEDBACK_DIRECTION_NORMAL;  ///< 反馈方向
    const float *other_angle_feedback_ptr_ = nullptr;     ///< 外部角度反馈指针（如 IMU）

    uint8_t sender_group_index_ = 0;  ///< 所属发送组索引
    uint8_t message_num_ = 0;         ///< 在发送帧中的字节位置编号
    float ref_ = 0.0f;                ///< 控制参考值

    int32_t encoder_round_count_ = 0;   ///< 编码器圈数累计
    uint16_t last_encoder_cnt_ = 0;     ///< 上次编码器计数值（用于溢出检测）
    bool is_initialized_ = false;        ///< 初始化标志
};

#endif /* DVC_DJIMOTOR_H */
