#include "app_config.h"

#include "fdcan.h"
#include "spi.h"
#include "usart.h"

namespace
{
ClassBMI088 bmi088;
ClassImu imu;
ClassDR16 dr16;
ClassFDCANCOMM interboard_comm;
ClassChassis chassis;
ClassFeedRotor feed_rotor;
}

ClassCmd cmd_app;

/**
 * @brief 初始化底盘板全部手写模块
 * @note  四个 M3508 驱动轮注册到 CAN1，四个 GM6020 转向电机注册到 CAN3；
 *        CAN2 只负责与云台交换业务命令和云台姿态。
 */
void App_Config_Init(void)
{
    BMI088Config bmi088_config{};
    ImuConfig imu_config{};
    ChassisConfig chassis_config{};
    FDCANCOMMConfig interboard_config{};
    PID_Config angle_pid_config{};
    PID_Config speed_pid_config{};
    PID_Config yaw_follow_pid_config{};
    CmdConfig cmd_config{};

    bmi088_config.spi_handle = &hspi2;
    bmi088_config.accelerometer_cs_gpio_port = CS1_ACCLE_GPIO_Port;
    bmi088_config.accelerometer_cs_pin = CS1_ACCLE_Pin;
    bmi088_config.gyroscope_cs_gpio_port = CS2_GYRO_GPIO_Port;
    bmi088_config.gyroscope_cs_pin = CS2_GYRO_Pin;
    bmi088.Init(bmi088_config);

    imu_config.bmi088 = &bmi088;
    imu.Init(imu_config);

    angle_pid_config.Kp = 1.0f;
    angle_pid_config.out_min = -1000.0f;
    angle_pid_config.out_max = 1000.0f;
    angle_pid_config.integral_min = -1000.0f;
    angle_pid_config.integral_max = 1000.0f;

    speed_pid_config.Kp = 1.0f;
    speed_pid_config.out_min = -16000.0f;
    speed_pid_config.out_max = 16000.0f;
    speed_pid_config.integral_min = -16000.0f;
    speed_pid_config.integral_max = 16000.0f;

    // 跟随 PID 的输入为角度误差，输出为底盘角速度给定 (rad/s)。
    yaw_follow_pid_config.Kp = 0.05f;
    yaw_follow_pid_config.out_min = -6.0f;
    yaw_follow_pid_config.out_max = 6.0f;
    yaw_follow_pid_config.integral_min = -6.0f;
    yaw_follow_pid_config.integral_max = 6.0f;
    chassis_config.yaw_follow_pid_config = yaw_follow_pid_config;

    // 最终转向电机目标 = 车轮期望角 * 减速比 + 零位偏置；实测后填写四个模块的偏置。
    chassis_config.steer_zero_offset_deg[0] = 0.0f;
    chassis_config.steer_zero_offset_deg[1] = 0.0f;
    chassis_config.steer_zero_offset_deg[2] = 0.0f;
    chassis_config.steer_zero_offset_deg[3] = 0.0f;

    // 四个 M3508 驱动轮位于 CAN1，速度环直接接收电机输出轴角速度 (deg/s)。
    for (uint8_t index = 0; index < 4; index++)
    {
        chassis_config.wheel_djimotor_configs[index].base.can.fdcan_handle = &hfdcan1;
        chassis_config.wheel_djimotor_configs[index].base.can.tx_id = index + 1U;
        chassis_config.wheel_djimotor_configs[index].motor_type = M3508;
        chassis_config.wheel_djimotor_configs[index].control.close_loop_type = SPEED_LOOP;
        chassis_config.wheel_djimotor_configs[index].control.outer_loop_type = SPEED_LOOP;
        chassis_config.wheel_djimotor_configs[index].controller.speed_pid = speed_pid_config;
    }

    // 四个 GM6020 转向电机位于 CAN3，采用编码器角度外环和速度内环。
    for (uint8_t index = 0; index < 4; index++)
    {
        chassis_config.steer_djimotor_configs[index].base.can.fdcan_handle = &hfdcan3;
        chassis_config.steer_djimotor_configs[index].base.can.tx_id = index + 1U;
        chassis_config.steer_djimotor_configs[index].motor_type = GM6020;
        chassis_config.steer_djimotor_configs[index].control.close_loop_type = ANGLE_AND_SPEED_LOOP;
        chassis_config.steer_djimotor_configs[index].control.outer_loop_type = ANGLE_LOOP;
        chassis_config.steer_djimotor_configs[index].control.angle_feedback_source = Encoder_Feedback;
        chassis_config.steer_djimotor_configs[index].controller.angle_pid = angle_pid_config;
        chassis_config.steer_djimotor_configs[index].controller.speed_pid = speed_pid_config;
    }

    chassis.Init(chassis_config);

#if FEED_ROTOR_OWNER_BOARD == BOARD_CHASSIS
    // 下供弹时，拨弹盘使用 CAN1 的 M3508 ID5；与四个驱动轮的 ID1~4 不冲突。
    FeedRotorConfig feed_rotor_config{};
    feed_rotor_config.feed_djimotor_config.base.can.fdcan_handle = &hfdcan1;
    feed_rotor_config.feed_djimotor_config.base.can.tx_id = 5U;
    feed_rotor_config.feed_djimotor_config.motor_type = M3508;
    feed_rotor_config.feed_djimotor_config.control.close_loop_type = ANGLE_AND_SPEED_LOOP;
    feed_rotor_config.feed_djimotor_config.control.outer_loop_type = ANGLE_LOOP;
    feed_rotor_config.feed_djimotor_config.controller.angle_pid = angle_pid_config;
    feed_rotor_config.feed_djimotor_config.controller.speed_pid = speed_pid_config;
    feed_rotor.Init(feed_rotor_config);
#endif

    // 底盘发 0x302、收 0x301；CAN2 与云台 GM6020 共线但 CAN ID 不冲突。
    interboard_config.FDCAN_config.fdcan_handle = &hfdcan2;
    interboard_config.FDCAN_config.tx_id = 0x302U;
    interboard_config.FDCAN_config.rx_id = 0x301U;
    interboard_config.Tx_Buffer_Length = sizeof(InterboardFrame);
    interboard_config.Rx_Buffer_Length = sizeof(InterboardFrame);
    interboard_comm.Init(interboard_config);

#if REMOTE_OWNER_BOARD == BOARD_CHASSIS
    // 仅遥控器装在底盘板时才占用 UART7 接收 DR16。
    DR16Config dr16_config{};
    dr16_config.uartconfig.uart_handle = &huart7;
    dr16_config.uartconfig.recv_buff_size = DR16_FRAME_LEN;
    dr16.Init(dr16_config);
#endif

    cmd_config.chassis = &chassis;
    cmd_config.imu = &imu;
    cmd_config.feed_rotor = &feed_rotor;
    cmd_config.dr16 = &dr16;
    cmd_config.interboard_comm = &interboard_comm;
    cmd_app.Init(cmd_config);

    ClassFDCAN::Start(&hfdcan1);
    ClassFDCAN::Start(&hfdcan2);
    ClassFDCAN::Start(&hfdcan3);
}

/**
 * @brief 底盘板控制周期入口
 * @note  由任务以固定周期调用；BMI088 先刷新数据，随后 Cmd 分发业务命令。
 *
 * @param dt_s 控制周期 (s)
 */
void App_Control_Update(float dt_s)
{
    bmi088.Update();
    cmd_app.Update(dt_s);
}
