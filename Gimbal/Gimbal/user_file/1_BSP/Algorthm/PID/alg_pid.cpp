/**
 * @file alg_pid.cpp
 * @brief PID 控制器实现。
 * @details
 * 本文件实现 `Class_PID` 的成员函数。
 */
#include "alg_pid.h"
#include "common_math.h"

namespace
{
constexpr float PID_OUTPUT_SLEW_RELEASE_GAIN = 8.0f;
constexpr float PID_MIN_DT_S = 1e-6f;
constexpr float PID_MIN_FRICTION_OMEGA = 1e-6f;

/**
 * @brief 检查浮点数是否为有限值，非有限值返回零。
 * @param value 待检查的浮点数。
 * @return 有限值原样返回，否则返回 0.0f。
 */
float PID_Finite_Or_Zero(float value)
{
    return IsFiniteFloat(value) ? value : 0.0f;
}

} /* namespace */

/**
 * @brief 计算摩擦补偿输出。
 * @return 当前摩擦补偿输出值。
 * @details
 * 补偿模型为 `fc * tanh(omega / w_eps) + bv * omega`。
 * 当未启用摩擦补偿时，直接返回零。
 */
float Class_PID::GetFriction_Compensation()
{
    float omega;
    float w_eps;

    if (friction_comp_enable == false)
    {
        return 0.0f;
    }

    omega = friction_use_target_omega ? target : Input;
    w_eps = fabsf(PID_Finite_Or_Zero(friction_w_eps));
    if (w_eps < PID_MIN_FRICTION_OMEGA)
    {
        w_eps = PID_MIN_FRICTION_OMEGA;
    }

    return PID_Finite_Or_Zero(friction_fc * tanhf(omega / w_eps) +
                               friction_bv * omega);
}

/**
 * @brief 计算重力补偿输出。
 * @param angle_deg 当前姿态角，单位度。
 * @return 当前重力补偿输出值。
 * @details
 * 补偿模型为 `kg * sin(theta)`。
 * 当未启用重力补偿时，直接返回零。
 */
float Class_PID::GetGravity_Compensation(float angle_deg)
{
    float gravity_ff;

    if (gravity_comp_enable == false)
    {
        return 0.0f;
    }

    gravity_ff = PID_Finite_Or_Zero(gravity_comp_kg) *
                 sinf(DegToRad(PID_Finite_Or_Zero(angle_deg)));
    if (gravity_comp_reverse != false)
    {
        gravity_ff = -gravity_ff;
    }

    return PID_Finite_Or_Zero(gravity_ff);
}

/**
 * @brief 从配置结构体初始化 PID 对象。
 * @param config PID 配置结构体。
 * @return 无。
 * @details
 * 将配置参数复制到内部成员，并清空全部运行时状态。
 * 对需要交换的限幅参数（min > max）会自动交换。
 */
void Class_PID::Init(PID_Config config)
{
    /* ---- 清空运行时状态 ---- */
    P_out = 0.0f;
    I_out = 0.0f;
    D_out = 0.0f;
    FeedForward_out = 0.0f;
    Friction_Compensation_out = 0.0f;
    Gravity_Compensation_out = 0.0f;

    output_raw = 0.0f;
    output_limited = 0.0f;
    output = 0.0f;
    target = 0.0f;
    Input = 0.0f;
    error = 0.0f;
    integral = 0.0f;

    prev_target = 0.0f;
    prev_input = 0.0f;
    prev_error = 0.0f;
    output_shaper_inited = false;
    output_shaper_state = 0.0f;
    dt = 0.0f;

    /* ---- 读取配置 ---- */
    Kp = PID_Finite_Or_Zero(config.Kp);
    Ki = PID_Finite_Or_Zero(config.Ki);
    Kd = PID_Finite_Or_Zero(config.Kd);
    FeedForward = PID_Finite_Or_Zero(config.FeedForward);

    integral_min = PID_Finite_Or_Zero(config.integral_min);
    integral_max = PID_Finite_Or_Zero(config.integral_max);
    if (integral_min > integral_max)
    {
        const float temp = integral_min;
        integral_min = integral_max;
        integral_max = temp;
    }

    out_min = PID_Finite_Or_Zero(config.out_min);
    out_max = PID_Finite_Or_Zero(config.out_max);
    if (out_min > out_max)
    {
        const float temp = out_min;
        out_min = out_max;
        out_max = temp;
    }

    target_limit_enable = config.target_limit_enable;
    target_limit_min = PID_Finite_Or_Zero(config.target_limit_min);
    target_limit_max = PID_Finite_Or_Zero(config.target_limit_max);
    if (target_limit_min > target_limit_max)
    {
        const float temp = target_limit_min;
        target_limit_min = target_limit_max;
        target_limit_max = temp;
    }

    integral_separation_enable = config.integral_separation_enable;
    integral_separation_threshold_A = fabsf(PID_Finite_Or_Zero(config.integral_separation_threshold_A));
    integral_separation_threshold_B = fabsf(PID_Finite_Or_Zero(config.integral_separation_threshold_B));
    if (integral_separation_threshold_A < integral_separation_threshold_B)
    {
        const float temp = integral_separation_threshold_A;
        integral_separation_threshold_A = integral_separation_threshold_B;
        integral_separation_threshold_B = temp;
    }

    differential_enable = config.differential_enable;

    deadband_enable = config.deadband_enable;
    deadband = fabsf(PID_Finite_Or_Zero(config.deadband));

    friction_comp_enable = config.friction_comp_enable;
    friction_fc = PID_Finite_Or_Zero(config.friction_fc);
    friction_bv = PID_Finite_Or_Zero(config.friction_bv);
    friction_w_eps = fabsf(PID_Finite_Or_Zero(config.friction_w_eps));
    if (friction_w_eps < PID_MIN_FRICTION_OMEGA)
    {
        friction_w_eps = PID_MIN_FRICTION_OMEGA;
    }
    friction_use_target_omega = config.friction_use_target_omega;

    gravity_comp_enable = config.gravity_comp_enable;
    gravity_comp_kg = PID_Finite_Or_Zero(config.gravity_comp_kg);
    gravity_comp_reverse = config.gravity_comp_reverse;

    output_filter_enable = config.output_filter_enable;
    output_filter_tau_s = PID_Finite_Or_Zero(config.output_filter_tau_s);
    if (output_filter_tau_s <= 0.0f)
    {
        output_filter_tau_s = 0.0f;
    }

    output_slew_enable = config.output_slew_enable;
    output_slew_rate = fabsf(PID_Finite_Or_Zero(config.output_slew_rate));
}

/**
 * @brief 执行一次 PID 控制计算。
 * @param input_value 当前输入值。
 * @param target_value 当前目标值。
 * @param dt_value 当前控制周期，单位为秒。
 * @param gravity_angle_deg 当前重力补偿姿态角，单位为度，默认为 0。
 * @return 本次计算得到的控制输出。
 * @details
 * 该接口会按顺序执行目标限幅、误差计算、死区处理、积分更新、
 * 微分计算、前馈补偿、摩擦补偿、重力补偿以及输出整形与限幅。
 */
float Class_PID::Calculate(float input_value, float target_value, float dt_value,
                           float gravity_angle_deg)
{
    bool deadband_active;
    float integral_coef = 1.0f;
    float integral_candidate;
    float output_unsat_candidate;
    float output_unsat;
    float output_limited;
    float output_shaped;
    float alpha;
    float delta;
    float delta_max;
    float output_no_extra_comp;

    if ((!IsFiniteFloat(input_value)) || (!IsFiniteFloat(target_value)) ||
        (!IsFiniteFloat(dt_value)) || (dt_value <= 0.0f))
    {
        P_out = 0.0f;
        I_out = 0.0f;
        D_out = 0.0f;
        FeedForward_out = 0.0f;
        Friction_Compensation_out = 0.0f;
        Gravity_Compensation_out = 0.0f;
        Input = 0.0f;
        prev_input = 0.0f;
        target = 0.0f;
        prev_target = 0.0f;
        error = 0.0f;
        prev_error = 0.0f;
        integral = 0.0f;
        output_raw = 0.0f;
        output_limited = 0.0f;
        output = 0.0f;
        output_shaper_state = 0.0f;
        output_shaper_inited = true;
        dt = 0.0f;
        return 0.0f;
    }

    if (dt_value < PID_MIN_DT_S)
    {
        dt_value = PID_MIN_DT_S;
    }

    dt = dt_value;
    Input = input_value;
    target = target_value;

    if (target_limit_enable)
    {
        target = Clamp(target, target_limit_min, target_limit_max);
    }

    error = target - Input;

    deadband_active = deadband_enable && (fabsf(error) <= deadband);
    P_out = 0.0f;
    I_out = 0.0f;
    D_out = 0.0f;
    FeedForward_out = FeedForward * ((target - prev_target) / dt);
    Friction_Compensation_out = GetFriction_Compensation();
    Gravity_Compensation_out = GetGravity_Compensation(gravity_angle_deg);

    if (deadband_active == false)
    {
        P_out = Kp * error;

        if (integral_separation_enable)
        {
            const float abs_err = fabsf(error);

            if (abs_err >= integral_separation_threshold_A)
            {
                integral_coef = 0.0f;
            }
            else if ((abs_err > integral_separation_threshold_B) &&
                     (integral_separation_threshold_A > integral_separation_threshold_B))
            {
                integral_coef = (integral_separation_threshold_A - abs_err) /
                                (integral_separation_threshold_A - integral_separation_threshold_B);
            }
        }

        if (differential_enable)
        {
            D_out = -Kd * ((Input - prev_input) / dt);
        }
        else
        {
            D_out = Kd * ((error - prev_error) / dt);
        }

        integral_candidate = Clamp(integral + error * dt * integral_coef,
                                   integral_min,
                                   integral_max);
        output_no_extra_comp = P_out + Ki * integral_candidate + D_out + FeedForward_out;
        output_unsat_candidate = output_no_extra_comp + Friction_Compensation_out +
                                 Gravity_Compensation_out;

        if (!(((output_unsat_candidate > out_max) && (error > 0.0f)) ||
              ((output_unsat_candidate < out_min) && (error < 0.0f))))
        {
            integral = integral_candidate;
        }

        I_out = Ki * integral;
    }

    output_unsat = P_out + I_out + D_out + FeedForward_out +
                   Friction_Compensation_out + Gravity_Compensation_out;
    output_raw = output_unsat;
    output_limited = Clamp(output_unsat, out_min, out_max);

    this->output_limited = output_limited;

    if (output_filter_enable || output_slew_enable)
    {
        if (output_shaper_inited == false)
        {
            output_shaper_state = output_limited;
            output_shaper_inited = true;
        }

        output_shaped = output_limited;
        if (output_filter_enable && (output_filter_tau_s > 0.0f))
        {
            alpha = dt / (output_filter_tau_s + dt);
            alpha = Clamp(alpha, 0.0f, 1.0f);

            output_shaped = output_shaper_state + alpha * (output_limited - output_shaper_state);
        }

        if (output_slew_enable && (output_slew_rate > 0.0f))
        {
            delta = output_shaped - output_shaper_state;
            delta_max = output_slew_rate * dt;
            if ((output_shaped * output_shaper_state < 0.0f) ||
                (fabsf(output_shaped) < fabsf(output_shaper_state)))
            {
                delta_max *= PID_OUTPUT_SLEW_RELEASE_GAIN;
            }

            delta = AbsLimit(delta, delta_max);

            output_shaped = output_shaper_state + delta;
        }

        output_shaped = Clamp(output_shaped, out_min, out_max);

        output = output_shaped;
        output_shaper_state = output_shaped;
    }
    else
    {
        output = output_limited;
        output_shaper_state = output;
        output_shaper_inited = true;
    }

    prev_error = error;
    prev_target = target;
    prev_input = Input;

    return output;
}
