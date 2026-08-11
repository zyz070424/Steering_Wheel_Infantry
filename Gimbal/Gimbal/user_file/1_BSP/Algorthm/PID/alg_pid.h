/**
 * @file alg_pid.h
 * @brief PID 控制器类定义。
 * @details
 * 本文件定义 PID 配置结构体 `PID_Config` 以及 PID 控制器类 `Class_PID`。
 * 使用时先填充 `PID_Config`，再调用 `Class_PID::Init()` 完成一次性配置。
 * 运行时仅需调用 `Class_PID::Calculate()` 即可获得控制输出。
 */
#ifndef ALG_PID_H
#define ALG_PID_H

#ifdef __cplusplus

/**
 * @struct PID_Config
 * @brief PID 控制器完整配置。
 * @details
 * 包含 PID 增益、限幅参数、各功能开关及其附属参数。
 * 在调用 `Class_PID::Init()` 前填充此结构体即可完成全部配置。
 */
typedef struct
{
    /* ---- PID 增益 ---- */
    float Kp;                        /**< 比例系数 */
    float Ki;                        /**< 积分系数 */
    float Kd;                        /**< 微分系数 */
    float FeedForward;               /**< 前馈系数 */

    /* ---- 输出 / 积分限幅 ---- */
    float out_min;                   /**< 输出最小限幅 */
    float out_max;                   /**< 输出最大限幅 */
    float integral_min;              /**< 积分项最小限幅 */
    float integral_max;              /**< 积分项最大限幅 */

    /* ---- 目标值限幅 ---- */
    bool  target_limit_enable;       /**< 是否启用目标值限幅 */
    float target_limit_min;          /**< 目标值下限 */
    float target_limit_max;          /**< 目标值上限 */

    /* ---- 变速积分 ---- */
    bool  integral_separation_enable; /**< 是否启用变速积分 */
    float integral_separation_threshold_A; /**< 变速积分强抑制阈值 */
    float integral_separation_threshold_B; /**< 变速积分弱抑制阈值 */

    /* ---- 微分先行 ---- */
    bool  differential_enable;       /**< 是否对输入值启用微分先行 */

    /* ---- 死区 ---- */
    bool  deadband_enable;           /**< 是否启用死区 */
    float deadband;                  /**< 死区阈值 */

    /* ---- 摩擦补偿 ---- */
    bool  friction_comp_enable;      /**< 是否启用摩擦补偿 */
    float friction_fc;               /**< 库仑摩擦补偿系数 */
    float friction_bv;               /**< 粘性摩擦补偿系数 */
    float friction_w_eps;            /**< 摩擦补偿平滑参数 */
    bool  friction_use_target_omega; /**< 摩擦补偿是否使用目标速度 */

    /* ---- 重力补偿 ---- */
    bool  gravity_comp_enable;       /**< 是否启用重力补偿 */
    float gravity_comp_kg;           /**< 重力补偿系数 */
    bool  gravity_comp_reverse;      /**< 是否反向输出重力补偿 */

    /* ---- 输出低通整形 ---- */
    bool  output_filter_enable;      /**< 是否启用输出低通整形 */
    float output_filter_tau_s;       /**< 输出低通时间常数，单位秒 */

    /* ---- 输出斜率限制 ---- */
    bool  output_slew_enable;        /**< 是否启用输出斜率限制 */
    float output_slew_rate;          /**< 输出最大变化率，单位每秒 */

} PID_Config;

/**
 * @class Class_PID
 * @brief PID 控制器对象。
 * @details
 * 该对象维护 PID 运行时状态以及控制计算能力。
 * 外部通过 `PID_Config` 完成配置，运行时只需调用 `Calculate()`。
 */
class Class_PID
{
public:
    /* ---- 运行时输出（每次 Calculate 后可读取） ---- */
    float P_out;                     /**< 本次计算得到的比例输出 */
    float I_out;                     /**< 本次计算得到的积分输出 */
    float D_out;                     /**< 本次计算得到的微分输出 */
    float FeedForward_out;           /**< 本次计算得到的前馈输出 */
    float Friction_Compensation_out; /**< 本次计算得到的摩擦补偿输出 */
    float Gravity_Compensation_out;  /**< 本次计算得到的重力补偿输出 */
    float output_raw;                /**< 当前总输出原始值（限幅/整形前） */
    float output_limited;            /**< 当前总输出限幅值（整形前） */
    float output;                    /**< 当前总输出 */
    float target;                    /**< 当前控制目标值 */
    float Input;                     /**< 当前输入值 */
    float error;                     /**< 当前误差 */
    float integral;                  /**< 当前积分累计值 */

    void Init(PID_Config config);
    float Calculate(float input_value, float target_value, float dt_value,
                    float gravity_angle_deg = 0.0f);

private:
    /* ---- 内部辅助 ---- */
    float GetFriction_Compensation();
    float GetGravity_Compensation(float angle_deg);

    /* ---- PID 增益 ---- */
    float Kp;
    float Ki;
    float Kd;
    float FeedForward;

    /* ---- 输出 / 积分限幅 ---- */
    float out_min;
    float out_max;
    float integral_min;
    float integral_max;

    /* ---- 目标值限幅 ---- */
    bool  target_limit_enable;
    float target_limit_min;
    float target_limit_max;

    /* ---- 变速积分 ---- */
    bool  integral_separation_enable;
    float integral_separation_threshold_A;
    float integral_separation_threshold_B;

    /* ---- 微分先行 ---- */
    bool  differential_enable;

    /* ---- 死区 ---- */
    bool  deadband_enable;
    float deadband;

    /* ---- 摩擦补偿 ---- */
    bool  friction_comp_enable;
    float friction_fc;
    float friction_bv;
    float friction_w_eps;
    bool  friction_use_target_omega;

    /* ---- 重力补偿 ---- */
    bool  gravity_comp_enable;
    float gravity_comp_kg;
    bool  gravity_comp_reverse;

    /* ---- 输出低通整形 ---- */
    bool  output_filter_enable;
    float output_filter_tau_s;

    /* ---- 输出斜率限制 ---- */
    bool  output_slew_enable;
    float output_slew_rate;

    /* ---- 内部状态 ---- */
    float prev_target;
    float prev_input;
    float prev_error;
    bool  output_shaper_inited;
    float output_shaper_state;
    float dt;
};

#endif

#endif /* ALG_PID_H */
