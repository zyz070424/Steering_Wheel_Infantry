#ifndef __DVC_VISION_H__
#define __DVC_VISION_H__
#include "drv_usb.h"
#include <cstdint>


#define MANIFOLD_USB_RX_FRAME_LEN (1u + sizeof(float) + sizeof(float) + 1u + 1u)
#define MANIFOLD_USB_TX_FRAME_LEN (1u + sizeof(float) + sizeof(float) + 1u)
#define MANIFOLD_USB_RX_DEBUG_RAW_MAX USB_TX_BUFFER_SIZE_BYTES




/**
 * @brief 视觉识别敌方颜色枚举。
 */
enum Enum_Manifold_Enemy_Color
{
    Manifold_Enemy_Color_RED = 0,
    Manifold_Enemy_Color_BLUE,
};

/**
 * @brief 视觉识别敌方目标编号枚举。
 */
enum Enum_Manifold_Enemy_ID
{
    Manifold_Enemy_ID_NONE_0 = 0,
    Manifold_Enemy_ID_HERO_1 = 1,
    Manifold_Enemy_ID_ENGINEER_2 = 2,
    Manifold_Enemy_ID_INFANTRY_3 = 3,
    Manifold_Enemy_ID_INFANTRY_4 = 4,
    Manifold_Enemy_ID_INFANTRY_5 = 5,
    Manifold_Enemy_ID_SENTRY_7 = 7,
    Manifold_Enemy_ID_OUTPOST = 8,
    Manifold_Enemy_ID_RUNE = 9,
};

/**
 * @brief 视觉链路哨兵模式枚举。
 */
enum Enum_Manifold_Sentry_Mode
{
    Manifold_Sentry_Mode_DISABLE = 0,
    Manifold_Sentry_Mode_ENABLE,
};

/**
 * @brief USB 接收目标帧。
 */
#pragma pack(1)
typedef struct
{
    uint8_t Frame_Header;                 /**< 帧头 */
    float Taget_Yaw;                      /**< 目标偏航角 */
    float Taget_Pitch;                    /**< 目标俯仰角 */
    uint8_t Target_Valid;                 /**< 当前目标是否有效 */
    uint8_t Frame_Tail;                   /**< 帧尾 */
    enum Enum_Manifold_Enemy_ID Enemy_ID; /**< 当前目标敌方编号 */
} Vision_UART_Rx_Data;

#pragma pack()
/**
 * @brief USB 发送姿态帧。
 */

 #pragma pack(1)
typedef struct
{
    uint8_t Frame_Header; /**< 帧头 */
    float Yaw;            /**< 输出偏航角 */
    float Pitch;          /**< 输出俯仰角 */
    uint8_t Frame_Tail;   /**< 帧尾 */
} Vision_UART_Tx_Data;
#pragma pack()


/**
 * @class Class_Manifold
 * @brief Manifold 视觉链路管理对象。
 * @details
 * 负责 Manifold 协议初始化、USB 流式拼帧、目标数据解析和姿态数据回传。
 */
class ClassVision
{
public:
    void Init(uint8_t Rx_Header,uint8_t Rx_Tail);
    void Send(const Vision_UART_Tx_Data &Send_Data);
    Vision_UART_Rx_Data Get_Vision_Rx_Data(void)const;
private:
    static void Vision_Rx_Callback(void *owner,const uint8_t *data,uint32_t data_length_bytes);
    


private:
    uint8_t Rx_Frame_Buffer[MANIFOLD_USB_RX_FRAME_LEN];/**< 流式接收拼帧缓存 */
    uint8_t Rx_Current_Index = 0;                                  /**< 当前流式接收索引 */
    ClassUSB usb_{};
    Vision_UART_Rx_Data Rx_Data_{};
    uint8_t Rx_Header_ = 0;
    uint8_t Rx_Tail_   = 0; 

};
#endif /*__DVC_VISION_H__*/