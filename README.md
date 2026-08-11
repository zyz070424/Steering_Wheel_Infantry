# 舵轮步兵考核工程

本仓库用于 STM32H723VET6 舵轮步兵考核。系统由一块云台板、一块底盘板和一份 Common 通信契约组成：

```text
Final_Assessment/
├─ Gimbal/Gimbal/      云台 STM32 独立工程
├─ Chassis/Chassis/    底盘 STM32 独立工程
└─ Common/             两板共同编译的宏和 CAN2 业务帧
```

两块板可以单独编译、独立烧录。跨板信息只能经过 CAN2 的 `InterboardFrame`。

## 1. 考核功能概览

| 考核功能                 | 当前实现                                      |
| -------------------- | ----------------------------------------- |
| 云台 Pitch：DM4310 CAN1 | MIT 位置控制，带简单余弦重力补偿                        |
| 云台 Yaw：GM6020 CAN2   | PID 位置闭环，可选 IMU 或编码器反馈                    |
| 云台位置目标               | 目标角度做就近转位，避免跨越整圈                          |
| 底盘驱动                 | 四个 M3508 CAN1，速度闭环                        |
| 底盘转向                 | 四个 GM6020 CAN3，位置外环、速度内环                  |
| 舵轮运动学                | 四轮逆解、云台坐标系平移、静止 X 锁轮                      |
| 底盘模式                 | 失能、普通、跟随、小陀螺                              |
| 云台/底盘通信              | CAN2 双向 `ControlCommand + GimbalFeedback` |
| 遥控器位置切换              | `REMOTE_OWNER_BOARD` 预编译开关                |
| 拨弹盘位置切换              | `FEED_ROTOR_OWNER_BOARD` 预编译开关            |
| 摩擦轮与拨弹盘              | 使能、单发、连发；拨弹盘具有堵转回退                        |
| 视觉自瞄                 | USB CDC 虚拟串口接收目标，业务标志位启动                  |

> 参数均为便于检查框架而填入的初始值。实机使用前需要重新标定 PID、零位、轮距和方向。

## 2. 分层与依赖方向

每块板采用同一套层级，依赖只能向下：

```text
APP (3_Application)
        ↓
Device (2_Device)
        ↓
BSP / Algorithm (1_BSP)
        ↓
HAL / CubeMX (Core, Drivers)
```

| 目录                        | 责任                               |
| ------------------------- | -------------------------------- |
| `user_file/1_BSP`         | FDCAN、UART、USB、SPI、PID、EKF、通用数学  |
| `user_file/2_Device`      | DJI/DM 电机、BMI088、DR16、视觉、CAN2 通信 |
| `user_file/3_Application` | 云台、底盘、拨弹、摩擦轮、CMD 调度、实例配置         |
| `Core`、`Drivers`          | CubeMX/HAL 生成代码，不放业务控制逻辑         |

`CMD` 是本板的调度入口：它把 DR16 或 CAN2 接收的业务命令送给各 App；App 内部才计算 PID、运动学和电机目标。`CMD` 不直接写 PID，`CHASSIS` 不直接解析 DR16，`GIMBAL` 不直接处理 USB 拼帧。

## 3. 工程入口与运行周期

两块板都在 `user_file/3_Application/TASK/mytask.cpp` 中提供同一个 `App_Control_Task`。CubeMX 的 `StartDefaultTask` 完成 USB 初始化后进入该任务；任务先执行一次 `App_Config_Init()`，随后以 1 ms 固定周期调用 `App_Control_Update(0.001f)`。

任务使用 `xTaskDelayUntil()` 按绝对 tick 调度，避免 `vTaskDelay(1)` 把本周期的运行时间累计到下一周期。两工程的 FreeRTOS tick 均为 1000 Hz，CubeMX 中 `defaultTask` 的栈已设为 256 words（1 KB）。

不要把 Cmd、云台、底盘或 CAN 通信拆成多个同时访问 App 对象的任务；CAN、USB、DR16 仍由中断更新 Device 缓存，唯一控制任务负责读取缓存并运行 App。入口内部的顺序为：

```text
刷新 BMI088
→ 更新 IMU 姿态
→ 获取 DR16 或 CAN2 的 ControlCommand
→ 更新云台 / 底盘 / 发射 App
→ 发送 CAN2 业务帧
→ 本板所有 DJI 电机统一 Control_All()
```

## 4. 云台板

### 4.1 电机与 CAN 分配

| 总线   | 设备             | CAN ID  | 控制方式                  |
| ---- | -------------- | ------- | --------------------- |
| CAN1 | Pitch DM4310   | `0x001` | MIT 位置控制              |
| CAN2 | Yaw GM6020     | 1       | 角度 PID 外环 + 速度 PID 内环 |
| CAN1 | 左摩擦轮 M3508     | 1       | 速度闭环                  |
| CAN1 | 右摩擦轮 M3508     | 2       | 速度闭环                  |
| CAN1 | 拨弹盘 M3508（上供弹） | 3       | 角度外环 + 速度内环           |

DM4310 与 M3508 虽同为 CAN1 的 `ID 1`，但使用的协议不同；本工程按相应设备协议分别发送。

### 4.2 云台 App

`ClassGimbal` 维护当前姿态和位置目标：

- Yaw 反馈可选编码器或 IMU；当前配置使用 IMU。
- `Set_Target_Yaw_Deg()` 会把目标转换为离当前角最近的一圈。
- Pitch 直接发送 DM MIT 位置命令。
- Pitch 前馈为 `gravity_comp_torque * cos(pitch_angle)`，用于抵消俯仰轴重力矩。

云台状态：

| 状态                      | 行为                      |
| ----------------------- | ----------------------- |
| `GIMBAL_STATE_DISABLED` | 两轴停止输出                  |
| `GIMBAL_STATE_ACTIVE`   | Yaw 位置闭环，Pitch MIT 位置控制 |

### 4.3 视觉自瞄

视觉设备位于 `2_Device/VISION`，使用 USB CDC 虚拟串口。云台 App 只读取已解析的目标，不处理 USB 接收细节。

视觉业务标志在 Common 的 `ControlCommand::gimbal_mode` 中：

```cpp
cmd_app.Set_Gimbal_Mode(GIMBAL_VISION);  // 开启视觉跟随
cmd_app.Set_Gimbal_Mode(GIMBAL_MANUAL);  // 回到遥控器手动目标
```

必须在**遥控器所在板**调用该函数；Cmd 会把该标志经 CAN2 发给云台板。当前没有强制绑定某个遥控器按键，后续可自行选择键位或组合开关。

视觉接收帧格式为：

```text
[0xA5][target_yaw_deg(float)][target_pitch_deg(float)][target_valid(uint8)][0x5A]
```

- `target_yaw_deg`、`target_pitch_deg` 是云台 IMU 坐标系中的绝对角度，单位为 `deg`。
- `target_valid != 0` 时，云台直接把这两个值写入 yaw/pitch 位置目标。
- 目标无效时保留上一帧位置目标。

云台回传给视觉的帧格式为：

```text
[0xA5][yaw_deg(float)][pitch_deg(float)][0x5A]
```

回传内容来自云台 IMU，以便视觉和云台使用同一角度参考系。

## 5. 底盘板

### 5.1 电机与 CAN 分配

| 总线   | 设备             | CAN ID              | 控制方式                  |
| ---- | -------------- | ------------------- | --------------------- |
| CAN1 | 四个驱动 M3508     | 1~4                 | 速度闭环                  |
| CAN1 | 拨弹盘 M3508（下供弹） | 5                   | 角度外环 + 速度内环           |
| CAN3 | 四个转向 GM6020    | 1~4                 | 角度 PID 外环 + 速度 PID 内环 |
| CAN2 | 云台板通信          | 发 `0x302`，收 `0x301` | `InterboardFrame`     |

### 5.2 舵轮坐标与逆解

`ClassChassis` 接收的 `vx_mps`、`vy_mps` 始终以云台朝向为坐标系。先根据云台 yaw 与底盘 yaw 的差值旋转到车体坐标系，再对四个模块做逆解：

```text
wheel_vx = chassis_vx - wz × wheel_y
wheel_vy = chassis_vy + wz × wheel_x
wheel_speed = sqrt(wheel_vx² + wheel_vy²)
steer_angle = atan2(wheel_vy, wheel_vx)
```

这使小陀螺时仍可按云台方向平移；云台头部朝向保持不随底盘自转而改变。

### 5.3 底盘模式

| 模式                | 行为                            |
| ----------------- | ----------------------------- |
| `CHASSIS_DISABLE` | 所有底盘电机停止                      |
| `CHASSIS_NORMAL`  | 不额外产生 `wz`，保留给上层直接选择          |
| `CHASSIS_FOLLOW`  | 以云台 yaw 为目标，PID 输出 `wz_radps` |
| `CHASSIS_SPIN`    | 使用配置中的固定 `spin_wz_radps` 自转   |

当前 DT7 左三档拨杆的对应关系：上为小陀螺，中为跟随，下为失能。`CHASSIS_NORMAL` 已实现，但还未分配到遥控器输入；若需要完整模式选择，可添加一个组合键或业务标志位。

### 5.4 静止锁轮与转向零位

当 `vx`、`vy` 和 `wz` 都接近 0 时，四个转向模块锁为 X 形：

```text
模块 0 / 3：+45°
模块 1 / 2：-45°
```

每个转向模块都有独立零位偏置：

```cpp
chassis_config.steer_zero_offset_deg[0] = 0.0f;
chassis_config.steer_zero_offset_deg[1] = 0.0f;
chassis_config.steer_zero_offset_deg[2] = 0.0f;
chassis_config.steer_zero_offset_deg[3] = 0.0f;
```

实际下发关系为：

```text
转向电机目标角 = 车轮期望角 × 转向减速比 + steer_zero_offset_deg
```

标定方法：将某个车轮摆到定义的机械 `0°`，读取该 GM6020 的当前编码器总角度，填入对应偏置。这样后续任意车轮期望角都会相对正确的安装零位计算。

## 6. DR16 输入与发射逻辑

当前遥控器业务映射：

| DT7 输入 | 业务命令                       | 作用            |
| ------ | -------------------------- | ------------- |
| 左摇杆上下  | `chassis_vx_mps`           | 前进 / 后退       |
| 左摇杆左右  | `chassis_vy_mps`           | 左移 / 右移       |
| 右摇杆左右  | `gimbal_yaw_speed_degps`   | 云台 yaw 速度目标   |
| 右摇杆上下  | `gimbal_pitch_speed_radps` | 云台 pitch 速度目标 |
| 左三档上   | `CHASSIS_SPIN`             | 小陀螺           |
| 左三档中   | `CHASSIS_FOLLOW`           | 底盘跟随云台        |
| 左三档下   | `CHASSIS_DISABLE`          | 底盘失能          |
| 右三档上   | `SHOOT_CONTINUOUS`         | 连发            |
| 右三档中   | `SHOOT_SINGLE`             | 单发            |
| 右三档下   | `SHOOT_DISABLE`            | 发射机构失能        |
| 左拨轮向下  | `shoot_trigger = 1`        | 触发发弹          |

### 6.1 摩擦轮和拨弹盘

- 射击模式不是 `SHOOT_DISABLE` 时，云台摩擦轮保持设定速度。
- 单发模式只在拨轮下拨沿增加一次 `shot_sequence`。
- 连发模式在拨轮持续下拨时，按 `continuous_shot_interval_s` 周期增加序列号。
- 拨弹盘看到新的 `shot_sequence` 后，累加一发对应角度。
- 若拨弹电机力矩超过阈值且持续超时，则转入 `BACKOFF`，回退一定角度后恢复正常。

`shot_sequence` 只能由遥控器所在板生成，另一块板只消费它。这是为了在拨弹盘跨板安装时避免同一次触发被记两次。

## 7. CAN2 双板通信

两板使用 Common 中定义的 `InterboardFrame`：

```text
InterboardFrame
├─ ControlCommand
│  ├─ 云台速度、使能、视觉模式
│  ├─ 底盘 vx / vy / mode
│  └─ 射击模式、触发、shot_sequence
└─ GimbalFeedback
   ├─ yaw_deg
   └─ pitch_rad
```

CAN2 业务帧 ID：

| 方向      | CAN ID  | 主要用途              |
| ------- | ------- | ----------------- |
| 云台 → 底盘 | `0x301` | 默认情况下发送遥控器命令和云台姿态 |
| 底盘 → 云台 | `0x302` | 遥控器在底盘时发送业务命令     |

### 7.1 默认：DR16 在云台板

```text
DR16
 ↓
云台 Cmd 生成 ControlCommand
 ↓ CAN2 0x301
底盘 Cmd → 底盘运动学 / 跟随 PID
```

同一帧也包含云台 yaw，因此底盘能完成跟随和小陀螺坐标变换。

### 7.2 切换：DR16 在底盘板

```text
DR16
 ↓
底盘 Cmd 生成 ControlCommand
 ↓ CAN2 0x302
云台 Cmd → 云台 / 发射机构

云台姿态反馈
 ↓ CAN2 0x301
底盘 Cmd → 跟随 PID / 坐标变换
```

## 8. 预编译安装位置切换

所有位置选择只修改 Common 中的一个文件：

```text
Common/robot_config.h
```

```cpp
#define REMOTE_OWNER_BOARD BOARD_GIMBAL      // 或 BOARD_CHASSIS
#define FEED_ROTOR_OWNER_BOARD BOARD_GIMBAL  // 或 BOARD_CHASSIS
```

| 宏                        | `BOARD_GIMBAL`      | `BOARD_CHASSIS`     |
| ------------------------ | ------------------- | ------------------- |
| `REMOTE_OWNER_BOARD`     | DR16 接在云台板          | DR16 接在底盘板          |
| `FEED_ROTOR_OWNER_BOARD` | 上供弹，拨弹盘在云台 CAN1 ID3 | 下供弹，拨弹盘在底盘 CAN1 ID5 |

修改宏后，**云台与底盘工程必须使用同一份 Common 一起重新编译**，否则 `InterboardFrame` 的业务解释可能不一致。

已验证的编译组合：

1. 默认：DR16、拨弹盘都在云台板；
2. 切换：DR16、拨弹盘都在底盘板。

## 9. 编译

两个工程分别进入自己的根目录编译：

```powershell
cd Gimbal/Gimbal
cmake --build build/Debug

cd ../../Chassis/Chassis
cmake --build build/Debug
```

当前两工程均已使用 STM32 工具链完成编译。请使用各工程原本的 Cube/CMake 配置，不要把两个工程合并为一个工程。

## 10参考代码

BSP层与Device层主要参考岳麓框架

APP层参考了中科大

Algorithm里面的 Matrix搬运的是https://github.com/PX4/PX4-Matrix.git Middlewares/Third_Party/PX4-Matrix（虽然没有用上）

Algorithm里面的四元数解算和卡尔曼滤波参考的是https://github.com/WangHongxi2001/RoboMaster-C-Board-INS-Example
