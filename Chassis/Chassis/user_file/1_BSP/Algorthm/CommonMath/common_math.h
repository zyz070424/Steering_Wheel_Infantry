#ifndef __COMMON_MATH_H__
#define __COMMON_MATH_H__

#include <cmath>
#include <cstdint>

/**
 * @brief  限幅函数，将值限制在指定范围内。
 * @param value 输入值。
 * @param min_value 最小值。
 * @param max_value 最大值。
 * @return 限制后的值。
 */
float Clamp(float value, float min_value, float max_value);

/**
 * @brief  绝对值限幅，当绝对值超过限制时，保持符号并限制绝对值。
 * @param value 输入值。
 * @param limit 绝对值限制（正数）。
 * @return 限制后的值。
 */
float AbsLimit(float value, float limit);

/**
 * @brief  符号函数，返回输入值的符号。
 * @param value 输入值。
 * @return 正数返回1.0f，负数返回-1.0f，零返回0.0f。
 */
float Sign(float value);

/**
 * @brief  角度归一化（度），将角度归一化到 [-180, 180] 范围。
 * @param deg 输入角度（度）。
 * @return 归一化后的角度。
 */
float AngleWrapDeg(float deg);

/**
 * @brief  角度归一化（弧度），将角度归一化到 [-π, π] 范围。
 * @param rad 输入角度（弧度）。
 * @return 归一化后的角度。
 */
float AngleWrapRad(float rad);

/**
 * @brief  度转弧度。
 * @param deg 角度（度）。
 * @return 对应的弧度值。
 */
float DegToRad(float deg);

/**
 * @brief  弧度转度。
 * @param rad 弧度值。
 * @return 对应的角度（度）。
 */
float RadToDeg(float rad);

/**
 * @brief  将相位包裹到 [0, 2*pi) 区间。
 * @param rad 待包裹相位，单位：rad。
 * @return 包裹后的相位。
 */
float WrapPhaseto0To2Pi(float rad);

/**
 * @brief  一阶低通滤波器，对输入信号进行平滑处理。
 * @param input 当前输入值。
 * @param prev_output 上一次的输出值。
 * @param alpha 滤波系数（0~1），越小滤波效果越强。
 * @return 滤波后的输出值。
 */
float FirstOrderLPF(float input, float prev_output, float alpha);

/**
 * @brief  斜率限制，限制输出值相对于上一次输出的最大变化率。
 * @param target 目标值。
 * @param prev_output 上一次的输出值。
 * @param max_delta 最大允许的变化量。
 * @return 限制变化率后的输出值。
 */
float SlewLimit(float target, float prev_output, float max_delta);

/**
 * @brief  判断浮点数是否为有限值。
 * @param value 输入值。
 * @return 有限值返回 true，否则返回 false。
 */
bool IsFiniteFloat(float value);


/**
 * @brief  安全除法，避免除零错误。
 * @param numerator 被除数。
 * @param denominator 除数。
 * @param default_value 除数为零时的默认返回值。
 * @return 除法结果，除数为零时返回默认值。
 */
float SafeDivide(float numerator, float denominator, float default_value = 0.0f);

/**
 * @brief  范围检查，判断值是否在指定范围内。
 * @param value 输入值。
 * @param min_value 最小值。
 * @param max_value 最大值。
 * @return 在范围内返回true，否则返回false。
 */
bool RangeCheck(float value, float min_value, float max_value);

/**
 * @brief  将浮点数转换为 int16_t，溢出时做饱和截断。
 * @param  value 输入浮点数。
 * @return 饱和截断后的 int16_t 值。
 */
int16_t FloatToInt16Sat(float value);

/**
 * @brief  将浮点数转换为 uint16_t，溢出时做饱和截断。
 * @param  value 输入浮点数。
 * @return 饱和截断后的 uint16_t 值。
 */
uint16_t FloatToUint16Sat(float value);

/**
 * @brief  输出值转换为零值。
 * @param value 输入值。
 * @return 零值。
 */
int16_t OutputToZero(float value);

/**
 * @brief  将两个 uint8_t 拼接为一个 uint16_t（大端序，high 为高字节）。
 * @param high 高字节。
 * @param low  低字节。
 * @return 拼接后的 uint16_t 值。
 */
uint16_t Uint8ToUint16(uint8_t high, uint8_t low);

/**
 * @brief  将一个 uint16_t 拆分为两个 uint8_t（大端序，high 为高字节）。
 * @param value 输入的 uint16_t 值。
 * @param high  输出的高字节。
 * @param low   输出的低字节。
 */
void Uint16ToUint8(uint16_t value, uint8_t &high, uint8_t &low);

#endif /* __COMMON_MATH_H__ */
