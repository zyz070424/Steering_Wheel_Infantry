#include "app_config.h"

#include "fdcan.h"
#include "spi.h"
#include "usart.h"

namespace
{
ClassBMI088 bmi088;
ClassVision vision;
ClassDR16 dr16;
ClassFDCANCOMM interboard_comm;
ClassGimbal gimbal;
ClassShoot shoot;
ClassFeedRotor feed_rotor;
}

ClassCmd cmd_app;

/**
 * @brief 初始化云台板全部手写模块
 * @note  电机、IMU、DR16 和板间通信均在这里完成设备绑定；
 *        最后再统一启动已注册的 CAN1、CAN2。
 */
void App_Config_Init(void)
{
    BMI088Config bmi088_config{};
    GimbalConfig gimbal_config{};
    ShootConfig shoot_config{};
    FeedRotorConfig feed_rotor_config{};
    FDCANCOMMConfig interboard_config{};
    DR16Config dr16_config{};
    PID_Config angle_pid_config{};
    PID_Config speed_pid_config{};
    CmdConfig cmd_config{};

    bmi088_config.spi_handle = &hspi2;
    bmi088_config.accelerometer_cs_gpio_port = CS1_ACCLE_GPIO_Port;
    bmi088_config.accelerometer_cs_pin = CS1_ACCLE_Pin;
    bmi088_config.gyroscope_cs_gpio_port = CS2_GYRO_GPIO_Port;
    bmi088_config.gyroscope_cs_pin = CS2_GYRO_Pin;
    bmi088.Init(bmi088_config);

    // 视觉 USB 虚拟串口帧格式：[0xA5][yaw_deg][pitch_deg][valid][0x5A]。
    vision.Init(0xA5U, 0x5AU);

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

    // Pitch DM4310 位于 CAN1，DM MIT 命令与 DJI 电机发送 ID 不冲突。
    gimbal_config.pitch_dmmotor_config.base.can.fdcan_handle = &hfdcan1;
    gimbal_config.pitch_dmmotor_config.base.can.tx_id = 0x001U;
    gimbal_config.pitch_dmmotor_config.base.can.rx_id = 0x001U;
    gimbal_config.pitch_dmmotor_config.motor_type = DM_J4310;

    // Yaw GM6020 位于 CAN2，角度外环和速度内环参数暂用便于验收的 1。
    gimbal_config.yaw_djimotor_config.base.can.fdcan_handle = &hfdcan2;
    gimbal_config.yaw_djimotor_config.base.can.tx_id = 1U;
    gimbal_config.yaw_djimotor_config.motor_type = GM6020;
    gimbal_config.yaw_djimotor_config.control.close_loop_type = ANGLE_AND_SPEED_LOOP;
    gimbal_config.yaw_djimotor_config.control.outer_loop_type = ANGLE_LOOP;
    gimbal_config.yaw_djimotor_config.control.angle_feedback_source = IMU_Feedback;
    gimbal_config.yaw_djimotor_config.controller.angle_pid = angle_pid_config;
    gimbal_config.yaw_djimotor_config.controller.speed_pid = speed_pid_config;
    gimbal_config.yaw_feedback = GIMBAL_YAW_FEEDBACK_IMU;
    gimbal_config.imu_config.bmi088 = &bmi088;
    gimbal_config.vision = &vision;
    gimbal.Init(gimbal_config);

    // 两个摩擦轮与上供弹拨弹盘均挂在 CAN1，电机 ID 必须与 Pitch DM4310 区分。
    shoot_config.left_friction_djimotor_config.base.can.fdcan_handle = &hfdcan1;
    shoot_config.left_friction_djimotor_config.base.can.tx_id = 1U;
    shoot_config.left_friction_djimotor_config.motor_type = M3508;
    shoot_config.left_friction_djimotor_config.control.close_loop_type = SPEED_LOOP;
    shoot_config.left_friction_djimotor_config.control.outer_loop_type = SPEED_LOOP;
    shoot_config.left_friction_djimotor_config.controller.speed_pid = speed_pid_config;
    shoot_config.left_friction_target_speed_rpm = 3000.0f;

    shoot_config.right_friction_djimotor_config = shoot_config.left_friction_djimotor_config;
    shoot_config.right_friction_djimotor_config.base.can.tx_id = 2U;
    shoot_config.right_friction_target_speed_rpm = -3000.0f;
    shoot.Init(shoot_config);

#if FEED_ROTOR_OWNER_BOARD == BOARD_GIMBAL
    feed_rotor_config.feed_djimotor_config.base.can.fdcan_handle = &hfdcan1;
    feed_rotor_config.feed_djimotor_config.base.can.tx_id = 3U;
    feed_rotor_config.feed_djimotor_config.motor_type = M3508;
    feed_rotor_config.feed_djimotor_config.control.close_loop_type = ANGLE_AND_SPEED_LOOP;
    feed_rotor_config.feed_djimotor_config.control.outer_loop_type = ANGLE_LOOP;
    feed_rotor_config.feed_djimotor_config.controller.angle_pid = angle_pid_config;
    feed_rotor_config.feed_djimotor_config.controller.speed_pid = speed_pid_config;
    feed_rotor.Init(feed_rotor_config);
#endif

    // CAN2 上使用 0x301/0x302 与底盘交换固定大小的业务帧，避开 GM6020 的反馈 ID。
    interboard_config.FDCAN_config.fdcan_handle = &hfdcan2;
    interboard_config.FDCAN_config.tx_id = 0x301U;
    interboard_config.FDCAN_config.rx_id = 0x302U;
    interboard_config.Tx_Buffer_Length = sizeof(InterboardFrame);
    interboard_config.Rx_Buffer_Length = sizeof(InterboardFrame);
    interboard_comm.Init(interboard_config);

#if REMOTE_OWNER_BOARD == BOARD_GIMBAL
    // UART7 的 RX 引脚连接 DR16。原始遥控数据绝不通过 CAN2 传给底盘。
    dr16_config.uartconfig.uart_handle = &huart7;
    dr16_config.uartconfig.recv_buff_size = DR16_FRAME_LEN;
    dr16.Init(dr16_config);
#endif

    cmd_config.gimbal = &gimbal;
    cmd_config.shoot = &shoot;
    cmd_config.feed_rotor = &feed_rotor;
    cmd_config.dr16 = &dr16;
    cmd_config.interboard_comm = &interboard_comm;
    cmd_app.Init(cmd_config);

    ClassFDCAN::Start(&hfdcan1);
    ClassFDCAN::Start(&hfdcan2);
}

/**
 * @brief 云台板控制周期入口
 * @note  由任务以固定周期调用；所有 App 的 Update 和 DJI 电机发送均由 Cmd 完成。
 *
 * @param dt_s 控制周期 (s)
 */
void App_Control_Update(float dt_s)
{
    bmi088.Update();
    cmd_app.Update(dt_s);
}
