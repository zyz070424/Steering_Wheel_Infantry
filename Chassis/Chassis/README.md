# 舵轮底盘工程

本工程是考核中的底盘板工程，与 `../../Gimbal/Gimbal` 独立编译；两块板只共享 `../../Common` 的通信契约和安装位置宏。

## 分层

- `Core/`、`Drivers/`：CubeMX 生成的 HAL 库和启动代码，本次不改动。
- `user_file/1_BSP`：CAN、UART、SPI、PID、EKF 等基础封装。
- `user_file/2_Device`：DJI 电机、BMI088、DR16、CAN2 通信等具体设备。
- `user_file/3_Application`：`CHASSIS` 负责运动学和跟随 PID，`CMD` 负责把遥控器或 CAN2 命令分发给 App，`CONFIG` 只负责实例和硬件绑定。

依赖方向固定为 `Application -> Device -> BSP -> HAL`；`CMD` 不写电机 PID，`CHASSIS` 不读取 DR16 原始帧。

## CAN 分配

| 总线 | 设备 | ID |
| --- | --- | --- |
| CAN1 | 四个驱动 M3508 | 1~4 |
| CAN1 | 拨弹 M3508（仅下供弹） | 5 |
| CAN2 | 云台板互联 | 底盘发 `0x302`，收 `0x301` |
| CAN3 | 四个转向 GM6020 | 1~4 |

## 控制逻辑

`ClassCmd::Update()` 得到统一的 `ControlCommand` 后：

1. 更新底盘 IMU，取底盘 yaw；
2. 接收云台通过 CAN2 回传的 yaw；
3. `ClassChassis` 按模式计算 `wz`，并将云台坐标系的 `vx/vy` 转为车体坐标系；
4. 四轮逆解得到驱动速度和转向角；静止时转向角锁为 X 形；
5. `DJIMotor::Control_All()` 在 Cmd 的末尾只调用一次。

左拨杆上/中/下依次是小陀螺、底盘跟随云台、底盘失能。`CHASSIS_NORMAL` 也保留为无自转模式，供上层程序直接调用 `Set_Mode()` 使用。

小陀螺时云台仍锁定自身 IMU 角度，底盘将平移命令从云台坐标系旋转到车体坐标系，因此头部方向不随底盘转动而改变。

## 预编译切换

修改共享文件 `../../Common/robot_config.h` 后，**云台和底盘工程必须一起重新编译**：

```cpp
#define REMOTE_OWNER_BOARD BOARD_GIMBAL   // 或 BOARD_CHASSIS
#define FEED_ROTOR_OWNER_BOARD BOARD_GIMBAL  // 或 BOARD_CHASSIS
```

遥控器所在板是唯一生成 `shot_sequence` 的板；另一块板只消费 CAN2 中的业务命令。因此切换安装位置不会导致同一发弹被计数两次。

## 接入任务

Cube 初始化完成后调用一次 `App_Config_Init()`；固定周期任务中调用：

```cpp
App_Control_Update(0.001f);
```

实机上两块板 IMU 的 yaw 零点需在车体朝向一致时建立，底盘跟随 PID 才能直接比较两块板的 yaw。
