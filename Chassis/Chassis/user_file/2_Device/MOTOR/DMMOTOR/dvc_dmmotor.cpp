#include "dvc_dmmotor.h"

#include "common_math.h"

namespace
{
constexpr float DM_J4310_P_MIN_RAD = -12.5f;
constexpr float DM_J4310_P_MAX_RAD = 12.5f;
constexpr float DM_J4310_V_MIN_RADPS = -45.0f;
constexpr float DM_J4310_V_MAX_RADPS = 45.0f;
constexpr float DM_J4310_KP_MIN_NMPRAD = 0.0f;
constexpr float DM_J4310_KP_MAX_NMPRAD = 500.0f;
constexpr float DM_J4310_KD_MIN_NMSPRAD = 0.0f;
constexpr float DM_J4310_KD_MAX_NMSPRAD = 5.0f;
constexpr float DM_J4310_T_MIN_NM = -18.0f;
constexpr float DM_J4310_T_MAX_NM = 18.0f;
}

/**
 * @brief 初始化达妙电机，注册 FDCAN 通信实例
 * @param config 达妙电机配置结构体
 * @return true 初始化成功，false 参数非法
 */
bool DMMotor::Init(const DMMotorConfig &config)
{
    FDCANConfig registered_fdcan_config{};

    registered_fdcan_config.fdcan_handle = config.base.can.fdcan_handle;
    registered_fdcan_config.tx_id = config.base.can.tx_id;
    registered_fdcan_config.rx_id = config.base.can.rx_id;
    registered_fdcan_config.rx_callback = FDCAN_Rx_Callback;
    registered_fdcan_config.owner = this;

    if ((registered_fdcan_config.fdcan_handle == nullptr) || (config.motor_type == DMMOTOR_TYPE_NONE))
    {
        return false;
    }

    if (!fdcan_.Init(registered_fdcan_config))
    {
        return false;
    }

    motor_type_ = config.motor_type;

    motor_reverse_flag_ = config.base.motor.direction.motor_reverse_flag;
    feedback_reverse_flag_ = config.base.motor.direction.feedback_reverse_flag;
    
    is_initialized_ = true;
    return true;
}

/**
 * @brief 使能达妙电机（发送 MOTOR_MODE 指令）
 * @return true 始终返回 true
 */
bool DMMotor::Enable()
{
    Send_Mode_Command(DM_CMD_MOTOR_MODE);
    working_type_ = MOTOR_ENABLED;
    return true;
}

/**
 * @brief 停止达妙电机（发送 RESET_MODE 指令）
 * @return true 始终返回 true
 */
bool DMMotor::Stop()
{
    Send_Mode_Command(DM_CMD_RESET_MODE);
    working_type_ = MOTOR_STOP;
    return true;
}

/**
 * @brief 设置电机当前位置为零位
 * @return true 发送成功，false 发送失败
 */
bool DMMotor::Set_Zero_Position()
{
    return Send_Mode_Command(DM_CMD_ZERO_POSITION);
}

/**
 * @brief 清除电机错误状态
 * @return true 发送成功，false 发送失败
 */
bool DMMotor::Clear_Error()
{
    return Send_Mode_Command(DM_CMD_CLEAR_ERROR);
}

/**
 * @brief 发送 MIT 控制指令（位置/速度/Kp/Kd/前馈力矩）
 * @param command MIT 控制指令结构体
 * @return true 发送成功，false 未初始化或未使能
 */
bool DMMotor::Send_Mit_Command(const DMMotorMitCommand &command)
{
    uint8_t tx_data[ClassFDCAN::FDCAN_MAX_DATA_LENGTH_BYTES]{};
    float position_rad = command.position_rad;
    float velocity_radps = command.velocity_radps;
    float torque_ff_nm = command.torque_ff_nm;

    if (!is_initialized_ || (working_type_ != MOTOR_ENABLED))
    {
        return false;
    }

    if (motor_reverse_flag_ == MOTOR_DIRECTION_REVERSE)
    {
        position_rad = -position_rad;
        velocity_radps = -velocity_radps;
        torque_ff_nm = -torque_ff_nm;
    }

    const uint16_t position_uint = Float_To_Uint(position_rad, DM_J4310_P_MIN_RAD, DM_J4310_P_MAX_RAD, 16);
    const uint16_t velocity_uint = Float_To_Uint(velocity_radps, DM_J4310_V_MIN_RADPS, DM_J4310_V_MAX_RADPS, 12);
    const uint16_t kp_uint = Float_To_Uint(command.kp_nmprad, DM_J4310_KP_MIN_NMPRAD, DM_J4310_KP_MAX_NMPRAD, 12);
    const uint16_t kd_uint = Float_To_Uint(command.kd_nmsprad, DM_J4310_KD_MIN_NMSPRAD, DM_J4310_KD_MAX_NMSPRAD, 12);
    const uint16_t torque_uint = Float_To_Uint(torque_ff_nm, DM_J4310_T_MIN_NM, DM_J4310_T_MAX_NM, 12);

    tx_data[0] = static_cast<uint8_t>(position_uint >> 8);
    tx_data[1] = static_cast<uint8_t>(position_uint & 0x00FF);
    tx_data[2] = static_cast<uint8_t>(velocity_uint >> 4);
    tx_data[3] = static_cast<uint8_t>((velocity_uint & 0x000F) << 4) | static_cast<uint8_t>(kp_uint >> 8);
    tx_data[4] = static_cast<uint8_t>(kp_uint & 0x00FF);
    tx_data[5] = static_cast<uint8_t>(kd_uint >> 4);
    tx_data[6] = static_cast<uint8_t>((kd_uint & 0x000F) << 4) | static_cast<uint8_t>(torque_uint >> 8);
    tx_data[7] = static_cast<uint8_t>(torque_uint & 0x00FF);

    return fdcan_.Send(tx_data, ClassFDCAN::FDCAN_MAX_DATA_LENGTH_BYTES);
}

/**
 * @brief 获取达妙电机当前状态
 * @return 电机状态结构体引用
 */
const DMMotorStatus &DMMotor::Get_Status() const
{
    return status_;
}

/**
 * @brief 发送电机模式控制指令（使能/停止/零位/清错）
 * @param command 模式指令枚举
 * @return true 发送成功，false 发送失败
 */
bool DMMotor::Send_Mode_Command(DMMotor_Mode_e command)
{
    uint8_t tx_data[ClassFDCAN::FDCAN_MAX_DATA_LENGTH_BYTES]{};

    for (uint8_t index = 0; index < ClassFDCAN::FDCAN_MAX_DATA_LENGTH_BYTES - 1; ++index)
    {
        tx_data[index] = 0xFF;
    }
    tx_data[ClassFDCAN::FDCAN_MAX_DATA_LENGTH_BYTES - 1] = static_cast<uint8_t>(command);
    return fdcan_.Send(tx_data, ClassFDCAN::FDCAN_MAX_DATA_LENGTH_BYTES);
}
/**
 * @brief 处理FDCAN反馈数据
 * 
 * @param rx_frame FDCAN接收数据帧
 */
void DMMotor::Handle_FDCAN_Feedback(const StructFDCANRxFrame &rx_frame)
{
    if (rx_frame.data_length_bytes < ClassFDCAN::FDCAN_MAX_DATA_LENGTH_BYTES)
    {
        return;
    }

    const uint16_t position_uint = static_cast<uint16_t>((static_cast<uint16_t>(rx_frame.data[1]) << 8) | static_cast<uint16_t>(rx_frame.data[2]));
    const uint16_t velocity_uint = static_cast<uint16_t>((static_cast<uint16_t>(rx_frame.data[3]) << 4) | (static_cast<uint16_t>(rx_frame.data[4]) >> 4));
    const uint16_t torque_uint = static_cast<uint16_t>(((static_cast<uint16_t>(rx_frame.data[4]) & 0x000F) << 8) | static_cast<uint16_t>(rx_frame.data[5]));
    float position_rad;
    float velocity_radps;
    float torque_nm;

    position_rad = Uint_To_Float(position_uint, DM_J4310_P_MIN_RAD, DM_J4310_P_MAX_RAD, 16);
    velocity_radps = Uint_To_Float(velocity_uint, DM_J4310_V_MIN_RADPS, DM_J4310_V_MAX_RADPS,12);
    torque_nm = Uint_To_Float(torque_uint, DM_J4310_T_MIN_NM, DM_J4310_T_MAX_NM,12);

    if (feedback_reverse_flag_ == FEEDBACK_DIRECTION_REVERSE)
    {
        position_rad = -position_rad;
        velocity_radps = -velocity_radps;
        torque_nm = -torque_nm;
    }

    status_.motor_id = rx_frame.data[0] & 0x0F;
    status_.fault_code = rx_frame.data[0] >> 4;
    status_.position_rad = position_rad;
    status_.velocity_radps = velocity_radps;
    status_.torque_nm = torque_nm;
    status_.mos_temperature_c = rx_frame.data[6];
    status_.rotor_temperature_c = rx_frame.data[7];
    status_.is_feedback_valid = true;
}
/**
 * @brief FDCAN 接收回调：将 FDCAN 实例映射回电机对象并处理反馈
 * @param fdcan FDCAN 实例指针
 */
void DMMotor::FDCAN_Rx_Callback(ClassFDCAN *fdcan)
{
    DMMotor *motor = static_cast<DMMotor *>(fdcan->Get_Owner());

    if (motor != nullptr)
    {
        motor->Handle_FDCAN_Feedback(fdcan->Get_Rx_Frame());
    }
}



/**
 * @brief  将浮点数转换为无符号整数
 * @param value 浮点数值
 * @param min_value 最小值
 * @param max_value 最大值
 * @param bit_count 位数
 * @return 无符号整数
 */
uint16_t DMMotor::Float_To_Uint(float value,float min_value, float max_value,  uint8_t bit_count)
{
    const uint32_t max_uint = (1L << bit_count) - 1L;
    const float clamped_value = Clamp(value, min_value, max_value);
    const float scaled_value =(clamped_value - min_value) * static_cast<float>(max_uint) / (max_value - min_value);

    return static_cast<uint16_t>(scaled_value + 0.5f);
}
/**
 * @brief  将无符号整数转换为浮点数
 * @param value 无符号整数值
 * @param min_value 最小值
 * @param max_value 最大值
 * @param bit_count 位数
 * @return 浮点数
 */
float DMMotor::Uint_To_Float(uint16_t value,float min_value, float max_value, uint8_t bit_count)
{
    const uint32_t max_uint = (1L << bit_count) - 1L;

    return static_cast<float>(value) * (max_value - min_value) / static_cast<float>(max_uint) + min_value;
}
