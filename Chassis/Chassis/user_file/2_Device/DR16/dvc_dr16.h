#ifndef __DVC_DR16_H__
#define __DVC_DR16_H__

#include "drv_uart.h"

#define DR16_FRAME_LEN      18

#define DR16_SWITCH_UP      1
#define DR16_SWITCH_DOWN    2
#define DR16_SWITCH_MIDDLE  3

#define DR16_KEY_FREE       0
#define DR16_KEY_PRESSED    1

#define DR16_ROCKER_OFFSET  1024
#define DR16_ROCKER_RANGE   1320

#define DR16_KEY_W      0
#define DR16_KEY_S      1
#define DR16_KEY_A      2
#define DR16_KEY_D      3
#define DR16_KEY_SHIFT  4
#define DR16_KEY_CTRL   5
#define DR16_KEY_Q      6
#define DR16_KEY_E      7
#define DR16_KEY_R      8
#define DR16_KEY_F      9
#define DR16_KEY_G      10
#define DR16_KEY_Z      11
#define DR16_KEY_X      12
#define DR16_KEY_C      13
#define DR16_KEY_V      14
#define DR16_KEY_B      15

typedef enum
{
    DR16_SWITCH_STATUS_UP = 0,
    DR16_SWITCH_STATUS_MIDDLE,
    DR16_SWITCH_STATUS_DOWN,
    DR16_SWITCH_STATUS_TRIG_UP_MIDDLE,
    DR16_SWITCH_STATUS_TRIG_MIDDLE_UP,
    DR16_SWITCH_STATUS_TRIG_MIDDLE_DOWN,
    DR16_SWITCH_STATUS_TRIG_DOWN_MIDDLE,
} DR16_Switch_Status_TypeDef;

typedef enum
{
    DR16_KEY_STATUS_FREE = 0,
    DR16_KEY_STATUS_PRESSED,
    DR16_KEY_STATUS_TRIG_FREE_PRESSED,
    DR16_KEY_STATUS_TRIG_PRESSED_FREE,
} DR16_Key_Status_TypeDef;

/**
 * @brief DR16 应用层解析结果。
 * @details
 * 保存归一化摇杆值、鼠标值、原始拨杆与按键状态，以及边沿触发后的状态机结果。
 */
#pragma pack(1)
typedef struct DR16_DataTypeDef
{
    float right_x;  /**< 右摇杆 X，归一化到 [-1, 1] */
    float right_y;  /**< 右摇杆 Y，归一化到 [-1, 1] */
    float left_x;   /**< 左摇杆 X，归一化到 [-1, 1] */
    float left_y;   /**< 左摇杆 Y，归一化到 [-1, 1] */
    float left_trigger; /**< 左触发杆归一化到 [0, 1] */
    float mouse_x;  /**< 鼠标 X，归一化到 [-1, 1] */
    float mouse_y;  /**< 鼠标 Y，归一化到 [-1, 1] */
    float mouse_z;  /**< 鼠标 Z，归一化到 [-1, 1] */
    uint8_t raw_s1; /**< 左拨杆原始值 */
    uint8_t raw_s2; /**< 右拨杆原始值 */
    uint8_t prev_raw_s1; /**< 上一周期左拨杆原始值 */
    uint8_t prev_raw_s2; /**< 上一周期右拨杆原始值 */
    uint8_t raw_mouse_l; /**< 鼠标左键原始值 */
    uint8_t raw_mouse_r; /**< 鼠标右键原始值 */
    uint8_t prev_raw_mouse_l; /**< 上一周期鼠标左键原始值 */
    uint8_t prev_raw_mouse_r; /**< 上一周期鼠标右键原始值 */
    uint16_t raw_key;        /**< 当前键盘 16 位原始位图 */
    uint16_t prev_raw_key;   /**< 上一周期键盘 16 位原始位图 */
    DR16_Switch_Status_TypeDef left_switch;   /**< 左拨杆状态机输出 */
    DR16_Switch_Status_TypeDef right_switch;  /**< 右拨杆状态机输出 */
    DR16_Key_Status_TypeDef mouse_left;       /**< 鼠标左键状态机输出 */
    DR16_Key_Status_TypeDef mouse_right;      /**< 鼠标右键状态机输出 */
    DR16_Key_Status_TypeDef key[16];          /**< 16 个键盘键状态机输出 */
} DR16_DataTypeDef;
#pragma pack()

typedef struct
{
    UARTConfig uartconfig;
} DR16Config;

class ClassDR16
{
public:
    bool Init(const DR16Config &config);
    bool Process(DR16_DataTypeDef *dr16);

private:
    ClassUART uart_;
    DR16_Key_Status_TypeDef Dr16_;

    // 仅供 UART 接收回调写入；后续由 Process 按需读取。
    uint8_t frame_buffer_[2][DR16_FRAME_LEN]{};
    volatile uint8_t read_index_ = 0u;
    volatile uint8_t write_index_ = 1u;
    volatile uint8_t has_new_frame_ = 0u;

private:
    static void DR16_Rx_Callback(void *owner,
                                 const uint8_t *data,
                                 uint16_t data_length_bytes);
    uint8_t SanitizeSwitch(uint8_t sw) const;
    void JudgeSwitch(DR16_Switch_Status_TypeDef *sw, uint8_t now, uint8_t prev);
    void JudgeKey(DR16_Key_Status_TypeDef *key, uint8_t now, uint8_t prev);
    void Update(DR16_DataTypeDef *dr16);
};

#endif /*__DVC_DR16_H__*/
