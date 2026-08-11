#include "dvc_dr16.h"
#include "drv_uart.h"
#include <cstdint>
#include <cstring>
#include "common_math.h"

bool ClassDR16::Init(const DR16Config &config)
{
    UARTConfig registeredconfig;
    //进行通信设备绑定
    registeredconfig.owner = this;
    registeredconfig.uart_handle = config.uartconfig.uart_handle;
    registeredconfig.recv_buff_size = config.uartconfig.recv_buff_size;
    registeredconfig.rx_callback = DR16_Rx_Callback;
    if(!uart_.Init(registeredconfig))
    {
        return false;
    }
    return true;

}

/**
 * @brief 将非法拨杆值修正到中位。
 * @param sw 输入拨杆原始值。
 * @return 合法化后的拨杆值。
 */
uint8_t ClassDR16::SanitizeSwitch(uint8_t sw) const
{
    if ((sw == DR16_SWITCH_UP) || (sw == DR16_SWITCH_MIDDLE) || (sw == DR16_SWITCH_DOWN))
    {
        return sw;
    }

    return DR16_SWITCH_MIDDLE;
}

/**
 * @brief 根据当前值和上一值判断拨杆状态。
 * @param sw 输出拨杆状态指针。
 * @param now 当前拨杆原始值。
 * @param prev 上一周期拨杆原始值。
 * @return 无。
 */
void ClassDR16::JudgeSwitch(DR16_Switch_Status_TypeDef *sw, uint8_t now, uint8_t prev)
{
    now = SanitizeSwitch(now);
    prev = SanitizeSwitch(prev);

    switch (prev)
    {
    case DR16_SWITCH_UP:
        if (now == DR16_SWITCH_UP)
        {
            *sw = DR16_SWITCH_STATUS_UP;
        }
        else if (now == DR16_SWITCH_MIDDLE)
        {
            *sw = DR16_SWITCH_STATUS_TRIG_UP_MIDDLE;
        }
        else
        {
            *sw = DR16_SWITCH_STATUS_TRIG_MIDDLE_DOWN;
        }
        break;

    case DR16_SWITCH_DOWN:
        if (now == DR16_SWITCH_DOWN)
        {
            *sw = DR16_SWITCH_STATUS_DOWN;
        }
        else if (now == DR16_SWITCH_MIDDLE)
        {
            *sw = DR16_SWITCH_STATUS_TRIG_DOWN_MIDDLE;
        }
        else
        {
            *sw = DR16_SWITCH_STATUS_TRIG_MIDDLE_UP;
        }
        break;

    case DR16_SWITCH_MIDDLE:
    default:
        if (now == DR16_SWITCH_UP)
        {
            *sw = DR16_SWITCH_STATUS_TRIG_MIDDLE_UP;
        }
        else if (now == DR16_SWITCH_DOWN)
        {
            *sw = DR16_SWITCH_STATUS_TRIG_MIDDLE_DOWN;
        }
        else
        {
            *sw = DR16_SWITCH_STATUS_MIDDLE;
        }
        break;
    }
}

/**
 * @brief 根据当前值和上一值判断按键状态。
 * @param key 输出按键状态指针。
 * @param now 当前按键原始值。
 * @param prev 上一周期按键原始值。
 * @return 无。
 */
void ClassDR16::JudgeKey(DR16_Key_Status_TypeDef *key, uint8_t now, uint8_t prev)
{
    if (prev == DR16_KEY_FREE)
    {
        if (now == DR16_KEY_FREE)
        {
            *key = DR16_KEY_STATUS_FREE;
        }
        else
        {
            *key = DR16_KEY_STATUS_TRIG_FREE_PRESSED;
        }
    }
    else
    {
        if (now == DR16_KEY_FREE)
        {
            *key = DR16_KEY_STATUS_TRIG_PRESSED_FREE;
        }
        else
        {
            *key = DR16_KEY_STATUS_PRESSED;
        }
    }
}


/**
 * @brief 处理一帧 DR16 原始数据并更新应用层数据结构。
 * @param dr16 DR16 应用层数据结构指针。
 * @return true 已处理一帧新数据。
 * @return false 当前没有新数据。
 */
bool ClassDR16::Process(DR16_DataTypeDef *dr16)
{
    uint8_t raw[DR16_FRAME_LEN]{};
    uint16_t ch0;
    uint16_t ch1;
    uint16_t ch2;
    uint16_t ch3;
    uint16_t ch4;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    float rocker_denom;

    if (dr16 == NULL)
    {
        return false;
    }

    if (has_new_frame_ == 0U)
    {
        return false;
    }

    const uint8_t read_index = read_index_;
    has_new_frame_ = 0U;
    __DMB();
    std::memcpy(raw, frame_buffer_[read_index], DR16_FRAME_LEN);

    ch0 = ((uint16_t)raw[0] | ((uint16_t)(raw[1] & 0x07u) << 8)) & 0x07FFu;
    ch1 = (((uint16_t)raw[1] >> 3) | ((uint16_t)(raw[2] & 0x3Fu) << 5)) & 0x07FFu;
    ch2 = (((uint16_t)raw[2] >> 6) | ((uint16_t)raw[3] << 2) | ((uint16_t)(raw[4] & 0x01u) << 10)) & 0x07FFu;
    ch3 = (((uint16_t)raw[4] >> 1) | ((uint16_t)(raw[5] & 0x0Fu) << 7)) & 0x07FFu;

    dr16->raw_s1 = (raw[5] >> 6) & 0x03u;
    dr16->raw_s2 = (raw[5] >> 4) & 0x03u;

    mouse_x = (int16_t)((uint16_t)raw[6] | ((uint16_t)raw[7] << 8));
    mouse_y = (int16_t)((uint16_t)raw[8] | ((uint16_t)raw[9] << 8));
    mouse_z = (int16_t)((uint16_t)raw[10] | ((uint16_t)raw[11] << 8));

    dr16->raw_mouse_l = raw[12];
    dr16->raw_mouse_r = raw[13];
    dr16->raw_key = (uint16_t)raw[14] | ((uint16_t)raw[15] << 8);

    ch4 = ((uint16_t)raw[16] | ((uint16_t)(raw[17] & 0x07u) << 8)) & 0x07FFu; //左侧棘轮

    rocker_denom = DR16_ROCKER_RANGE / 2.0f;

    dr16->right_x = Clamp((ch0 - DR16_ROCKER_OFFSET) / rocker_denom, -1.0f, 1.0f);
    dr16->right_y = Clamp((ch1 - DR16_ROCKER_OFFSET) / rocker_denom, -1.0f, 1.0f);
    dr16->left_x = Clamp((ch2 - DR16_ROCKER_OFFSET) / rocker_denom, -1.0f, 1.0f);
    dr16->left_y = Clamp((ch3 - DR16_ROCKER_OFFSET) / rocker_denom, -1.0f, 1.0f);
    dr16->left_trigger = Clamp((ch4 - DR16_ROCKER_OFFSET) / rocker_denom, -1.0f, 1.0f);


    dr16->mouse_x = Clamp(mouse_x / 32768.0f, -1.0f, 1.0f);
    dr16->mouse_y = Clamp(mouse_y / 32768.0f, -1.0f, 1.0f);
    dr16->mouse_z = Clamp(mouse_z / 32768.0f, -1.0f, 1.0f);

   
    //更新按键状态和拨杆状态，上面更新的遥感和获得初始值
    Update(dr16);
    return true;
}

/**
 * @brief 更新按键状态和拨杆状态
 * @param dr16 DR16应用层数据结构指针。
 * @return 无。
 * @novel 该回调为项目自定义回调，不是 HAL / Cube / 官方库默认回调入口。
 */
void ClassDR16::Update(DR16_DataTypeDef *dr16)
{
    uint8_t i;
    uint8_t now_bit;
    uint8_t prev_bit;

    if (dr16 == NULL)
    {
        return;
    }

    JudgeSwitch(&dr16->left_switch, dr16->raw_s1, dr16->prev_raw_s1);
    JudgeSwitch(&dr16->right_switch, dr16->raw_s2, dr16->prev_raw_s2);

    JudgeKey(&dr16->mouse_left, dr16->raw_mouse_l, dr16->prev_raw_mouse_l);
    JudgeKey(&dr16->mouse_right, dr16->raw_mouse_r, dr16->prev_raw_mouse_r);

    for (i = 0u; i < 16u; i++)
    {
        now_bit = (dr16->raw_key >> i) & 0x01u;
        prev_bit = (dr16->prev_raw_key >> i) & 0x01u;
        JudgeKey(&dr16->key[i], now_bit, prev_bit);
    }

    dr16->prev_raw_s1 = dr16->raw_s1;
    dr16->prev_raw_s2 = dr16->raw_s2;
    dr16->prev_raw_mouse_l = dr16->raw_mouse_l;
    dr16->prev_raw_mouse_r = dr16->raw_mouse_r;
    dr16->prev_raw_key = dr16->raw_key;
}

/**
 * @brief 保存 UART 接收到的一帧 DR16 原始数据，并交换双缓冲索引。
 */
void ClassDR16::DR16_Rx_Callback(void *owner, const uint8_t *data,  uint16_t data_length_bytes)
{
    ClassDR16 *dr16 = static_cast<ClassDR16 *>(owner);

    if ((dr16 == nullptr) || (data == nullptr) || (data_length_bytes < DR16_FRAME_LEN))
    {
        return;
    }

    const uint8_t write_index = dr16->write_index_;
    std::memcpy(dr16->frame_buffer_[write_index], data, DR16_FRAME_LEN);

    __DMB();
    dr16->write_index_ = dr16->read_index_;
    dr16->read_index_ = write_index;
    dr16->has_new_frame_ = 1;
}
