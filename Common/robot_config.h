#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// 两块板编译时共用的安装位置开关。
#define BOARD_GIMBAL  1
#define BOARD_CHASSIS 2

// 修改这两个宏后，云台工程和底盘工程使用同一组配置重新编译。
#ifndef REMOTE_OWNER_BOARD
#define REMOTE_OWNER_BOARD BOARD_GIMBAL
#endif

#ifndef FEED_ROTOR_OWNER_BOARD
#define FEED_ROTOR_OWNER_BOARD BOARD_GIMBAL
#endif

#endif
