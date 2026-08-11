#include "dvc_djimotor.h"

#include "common_math.h"

DJIMotor *DJIMotor::motor_instances_[ClassFDCAN::FDCAN_MAX_INSTANCE_COUNT]{};
uint8_t DJIMotor::motor_instance_count_ = 0;
DJIMotor::DJIMotor_Sender_Group DJIMotor::sender_groups_[DJI_MOTOR_SENDER_GROUP_COUNT]{};

namespace
{
constexpr uint16_t DJI_ENCODER_COUNTS_PER_REV = 8192;         // DJI 电机编码器每转脉冲数
constexpr uint16_t DJI_ENCODER_HALF_COUNTS_PER_REV = DJI_ENCODER_COUNTS_PER_REV / 2; //用于判断编码器计数器是否发生了翻转
constexpr float DJI_ENCODER_DEG_PER_COUNT = 360.0f / static_cast<float>(DJI_ENCODER_COUNTS_PER_REV); // DJI 电机编码器每转角度数
constexpr float DJI_RPM_TO_DEGPS = 6.0f; // DJI 电机转速单位转换，1 RPM = 6 deg/s
}
/**
 * @brief 初始化电机对象。
 * @param config 电机配置结构体。
 * @return 成功返回 true，失败返回 false。
 * @details
 * 该函数会检查配置参数的有效性，并将其保存到内部成员变量中。
 * 如果配置参数无效，则返回 false。
 */
bool DJIMotor::Init(const DJIMotorConfig &config)
{
    FDCANConfig registered_fdcan_config{};

    registered_fdcan_config.fdcan_handle = config.base.can.fdcan_handle;
    registered_fdcan_config.tx_id = config.base.can.tx_id;
    registered_fdcan_config.rx_id = config.base.can.rx_id;
    registered_fdcan_config.rx_callback = FDCAN_Rx_Callback;
    registered_fdcan_config.owner = this;

    if ((registered_fdcan_config.fdcan_handle == nullptr) || (config.motor_type == DJIMOTOR_TYPE_NONE) || (motor_instance_count_ >= ClassFDCAN::FDCAN_MAX_INSTANCE_COUNT))
    {
        return false;
    }

    if (!Register_Sender_Group(&registered_fdcan_config, config.motor_type))
    {
        return false;
    }

    if (!fdcan_.Init(registered_fdcan_config))
    {
        return false;
    }

    current_pid_.Init(config.controller.current_pid);
    speed_pid_.Init(config.controller.speed_pid);
    angle_pid_.Init(config.controller.angle_pid);


    motor_type_ = config.motor_type;

    close_loop_type_ = config.control.close_loop_type;
    outer_loop_type_ = config.control.outer_loop_type;

    motor_reverse_flag_ = config.base.motor.direction.motor_reverse_flag;
    feedback_reverse_flag_ = config.base.motor.direction.feedback_reverse_flag;

    angle_feedback_source_ = config.control.angle_feedback_source;
    other_angle_feedback_ptr_ = config.controller.other_angle_feedback_ptr;

    is_initialized_ = true;
    motor_instances_[motor_instance_count_++] = this;
    sender_groups_[sender_group_index_].is_enabled = true;
    return true;
}
/**
 * @brief 使能大疆电机
 */
void DJIMotor::Enable()
{
    working_type_ = MOTOR_ENABLED;
}
/**
 * @brief 停止大疆电机，清零输出扭矩命令
 * @todo 考虑重置 PID 控制器
 */
void DJIMotor::Stop()
{
    working_type_ = MOTOR_STOP;
    DJIMotor_status_.output_torque_cmd = 0;
}
/**
 * @brief 设置电机控制参考值（角度/速度，取决于闭环类型）
 * @param ref 参考值
 */
void DJIMotor::Set_Ref(float ref)
{
    ref_ = ref;
}
/**
 * @brief 切换角度反馈源并设置新的参考值
 * @param source 反馈源（编码器或 IMU）
 * @param ref 参考角度值
 */
void DJIMotor::Change_Angle_Feed(Angle_Feedback_Source_e source, float ref)
{
    angle_feedback_source_ = source;
    ref_ = ref;
}
/**
 * @brief 单电机 PID 闭环控制：根据闭环类型逐级串联角度/速度/电流环
 * @param dt_s 控制周期 (s)
 */
void DJIMotor::Control(float dt_s)
{
    bool need_angle_feedback;
    bool need_speed_feedback;
    bool need_current_feedback;
    float pid_measure;
    float pid_ref;

    if ((is_initialized_ == false) || (working_type_ != MOTOR_ENABLED))
    {
        DJIMotor_status_.output_torque_cmd = 0;
        return;
    }
    need_angle_feedback = ((close_loop_type_ & ANGLE_LOOP) != 0) && (outer_loop_type_ == ANGLE_LOOP);
    need_speed_feedback = ((close_loop_type_ & SPEED_LOOP) != 0) && ((outer_loop_type_ & (ANGLE_LOOP | SPEED_LOOP)) != 0);
    need_current_feedback = (close_loop_type_ & CURRENT_LOOP) != 0;

    if ((need_angle_feedback || need_speed_feedback || need_current_feedback) && (DJIMotor_status_.is_feedback_valid == false))
    {
        DJIMotor_status_.output_torque_cmd = 0;
        return;
    }

    if (need_angle_feedback && (angle_feedback_source_ == IMU_Feedback) && (other_angle_feedback_ptr_ == nullptr))
    {
        DJIMotor_status_.output_torque_cmd = 0;
        return;
    }

    pid_ref = ref_;

    if (motor_reverse_flag_ == MOTOR_DIRECTION_REVERSE)
    {
        pid_ref = -pid_ref;
    }
    
    if (need_angle_feedback)
    {
        if (angle_feedback_source_ == IMU_Feedback)
        {
            pid_measure = *other_angle_feedback_ptr_;
        }
        else
        {
            pid_measure = DJIMotor_status_.total_angle_deg;
        }
        pid_ref = angle_pid_.Calculate(pid_measure, pid_ref, dt_s);
    }

    if (need_speed_feedback)
    {
        pid_measure = DJIMotor_status_.speed_degps;
        pid_ref = speed_pid_.Calculate(pid_measure, pid_ref, dt_s);
    }

    if (need_current_feedback)
    {
        pid_measure = static_cast<float>(DJIMotor_status_.torque_raw);
        pid_ref = current_pid_.Calculate(pid_measure, pid_ref, dt_s);
    }

    if (feedback_reverse_flag_ == FEEDBACK_DIRECTION_REVERSE)
    {
        pid_ref = -pid_ref;
    }

    DJIMotor_status_.output_torque_cmd = FloatToInt16Sat(pid_ref);
}


/**
 * @brief 控制所有电机。
 * @param dt_s 时间间隔，单位为秒。
 * @details
 * 该函数会遍历所有注册的电机实例，调用每个电机的 `Control()` 方法，更新电机状态并发送控制命令。
 */
void DJIMotor::Control_All(float dt_s)
{
    for (uint8_t index = 0; index < motor_instance_count_; ++index)
    {
        DJIMotor *motor = motor_instances_[index];
        DJIMotor_Sender_Group &sender_group = sender_groups_[motor->sender_group_index_];
        uint16_t output_cmd;

        motor->Control(dt_s);
        output_cmd = static_cast<uint16_t>(motor->DJIMotor_status_.output_torque_cmd);
        sender_group.tx_data[2 * motor->message_num_] = static_cast<uint8_t>(output_cmd >> 8);
        sender_group.tx_data[2 * motor->message_num_ + 1] = static_cast<uint8_t>(output_cmd & 0x00FF);
    }

    for (uint8_t index = 0; index < DJI_MOTOR_SENDER_GROUP_COUNT; ++index)
    {
        DJIMotor_Sender_Group &sender_group = sender_groups_[index];

        if (sender_group.is_enabled)
        {
            ClassFDCAN::Send_Frame(sender_group.fdcan_handle,
                                   sender_group.tx_id,
                                   sender_group.tx_data,
                                   ClassFDCAN::FDCAN_MAX_DATA_LENGTH_BYTES);
        }
    }
}

/**
 * @brief 获取电机类型
 * @return 电机类型枚举值
 */
DIJMotor_Type_e DJIMotor::Get_Motor_Type() const
{
    return motor_type_;
}

/**
 * @brief 获取电机当前状态
 * @return 电机状态结构体引用
 */
const DJIMotor_Status &DJIMotor::Get_Status() const
{
    return DJIMotor_status_;
}
/**
 * @brief 获取电机输出扭矩命令。
 * @return 电机输出扭矩命令，单位为原始值。
 * @details
 * 该函数返回电机的当前输出扭矩命令，用于发送给电机驱动器。
 */
int16_t DJIMotor::Get_Output_torque_Cmd() const
{
    return DJIMotor_status_.output_torque_cmd;
}
/**
 * @brief 注册电机发送组。
 * @param fdcan_config FDCAN 配置结构体指针。
 * @param motor_type 电机类型。
 * @return true 如果注册成功，否则返回 false。
 * @details
 * 该函数将电机发送组注册到 FDCAN 配置中，用于发送电机状态和控制命令。
 */
bool DJIMotor::Register_Sender_Group(FDCANConfig *fdcan_config, DIJMotor_Type_e motor_type)
{
    uint8_t motor_id;
    uint8_t message_num;
    uint8_t sender_group_index = DJI_MOTOR_SENDER_GROUP_COUNT;
    uint32_t sender_id;

    if ((fdcan_config->tx_id == 0) || (fdcan_config->tx_id > DJI_MOTOR_MAX_ID))
    {
        return false;
    }
    motor_id = static_cast<uint8_t>(fdcan_config->tx_id);

    if (motor_type == M3508)
    {
        fdcan_config->rx_id = 0x200 + motor_id;
        sender_id = motor_id <= 4 ? 0x200 : 0x1FF;
    }
    else if (motor_type == GM6020)
    {
        fdcan_config->rx_id = 0x204 + motor_id;
        sender_id = motor_id <= 4 ? 0x1FF : 0x2FF;
    }
    else
    {
        return false;
    }
    message_num = (motor_id - 1) % 4;
    //遍历检查
    for (uint8_t index = 0; index < DJI_MOTOR_SENDER_GROUP_COUNT; ++index)
    {
        //查早已经有的
        if ((sender_groups_[index].fdcan_handle == fdcan_config->fdcan_handle) && (sender_groups_[index].tx_id == sender_id))
        {
            sender_group_index = index;
            break;
        }
        //注册新的
        if ((sender_group_index == DJI_MOTOR_SENDER_GROUP_COUNT) && (sender_groups_[index].fdcan_handle == nullptr))
        {
            sender_group_index = index;
        }
    }

    if (sender_group_index == DJI_MOTOR_SENDER_GROUP_COUNT)
    {
        return false;
    }

    sender_groups_[sender_group_index].fdcan_handle = fdcan_config->fdcan_handle;
    sender_groups_[sender_group_index].tx_id = sender_id;
    sender_group_index_ = sender_group_index;
    message_num_ = message_num;
    return true;
}

/**
 * @brief FDCAN 接收回调：将 FDCAN 实例映射回电机对象并解析反馈帧
 * @param fdcan FDCAN 实例指针
 */
void DJIMotor::FDCAN_Rx_Callback(ClassFDCAN *fdcan)
{
    DJIMotor *motor = static_cast<DJIMotor *>(fdcan->Get_Owner());

    if (motor != nullptr)
    {
        motor->Handle_FDCAN_Feedback(fdcan->Get_Rx_Frame());
    }
}
/**
 * @brief 处理 FDCAN 接收的反馈帧。
 * @param rx_frame 接收到的 FDCAN 帧。
 * @details
 * 该函数会解析接收到的 FDCAN 帧，并更新电机状态结构体中的相关字段。
 * 如果接收到的帧长度不足 7 字节，则忽略该帧。
 */
void DJIMotor::Handle_FDCAN_Feedback(const StructFDCANRxFrame &rx_frame)
{
    uint16_t encoder_cnt;
    int16_t speed_rpm;
    int32_t encoder_delta_cnt;
    float total_angle_deg;

    if (rx_frame.data_length_bytes < 7)
    {
        return;
    }

    encoder_cnt = static_cast<uint16_t>((static_cast<uint16_t>(rx_frame.data[0]) << 8) |
                                        static_cast<uint16_t>(rx_frame.data[1]));
    speed_rpm = static_cast<int16_t>((static_cast<uint16_t>(rx_frame.data[2]) << 8) |
                                     static_cast<uint16_t>(rx_frame.data[3]));

    if (DJIMotor_status_.is_feedback_valid)
    {
        encoder_delta_cnt = static_cast<int32_t>(encoder_cnt) -  static_cast<int32_t>(last_encoder_cnt_);
        if (encoder_delta_cnt > static_cast<int32_t>(DJI_ENCODER_HALF_COUNTS_PER_REV))
        {
            --encoder_round_count_;
        }
        else if (encoder_delta_cnt < -static_cast<int32_t>(DJI_ENCODER_HALF_COUNTS_PER_REV))
        {
            ++encoder_round_count_;
        }
    }

    last_encoder_cnt_ = encoder_cnt;
    total_angle_deg = static_cast<float>(encoder_round_count_) * 360.0f +  static_cast<float>(encoder_cnt) * DJI_ENCODER_DEG_PER_COUNT;

    DJIMotor_status_.encoder_cnt = encoder_cnt;
    DJIMotor_status_.angle_single_deg = static_cast<float>(encoder_cnt) * DJI_ENCODER_DEG_PER_COUNT;
    DJIMotor_status_.total_angle_deg = total_angle_deg;
    DJIMotor_status_.speed_degps = static_cast<float>(speed_rpm) * DJI_RPM_TO_DEGPS;
    DJIMotor_status_.torque_raw = static_cast<int16_t>( (static_cast<uint16_t>(rx_frame.data[4]) << 8) |static_cast<uint16_t>(rx_frame.data[5]));
    DJIMotor_status_.temperature_c = rx_frame.data[6];
    DJIMotor_status_.is_feedback_valid = true;
}
