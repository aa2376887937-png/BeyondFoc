#pragma once

#include "FOC_math.h"

/*==============================================================================
 * PI 控制器 —— 纯算法层
 *
 *   电流环的被控对象是一阶系统 R + Ls，用 PI 就够了，不需要 D 项
 *   （一阶系统没有超调倾向，加 D 只会放大高频噪声）。
 *============================================================================*/

class Pi_Controller
{
public:
	Pi_Controller(float kp_ = 0.0f, float ki_ = 0.0f);

	void Set_Kp_Ki(float kp_, float ki_);
	void Set_Limit(float min_, float max_);		// 输出限幅
	void Set_Integral_Limit(float min_, float max_);	// 积分项单独限幅（抗饱和）
	void Set_Integral_Separation(float error_band_);	// |误差| > 该值时暂停积分（0=关闭）
	void Reset();								// 清积分项

	// 位置式 / 增量式切换（调试对比用）
	//   false = 位置式：out = Kp·e + ∫Ki·e        （默认）
	//   true  = 增量式：out += Kp·(e−e_prev) + Ki·e
	// 切换时会把状态对齐到当前输出，做到无扰切换。
	void Set_Incremental(bool on_);
	bool Is_Incremental() const { return incremental; }

	float Process(float error, float dt);		// 一个控制周期，梯形积分 + 抗饱和
	float Get_Integral() const { return integral; }
	float Get_Kp() const { return kp; }
	float Get_Ki() const { return ki; }

	// 反算抗积分饱和（back-calculation）：
	// 限幅发生在 Process 之外（电压圆），把 v_sat − v_pi 喂回来，
	// 积分按 (Kb/Ki)·Δv 回退，与 dt 无关。
	void Back_Calc(float sat_diff, float dt_)
	{
		(void)dt_;
		constexpr float kBack = 0.5f;

		if (sat_diff == 0.0f) return;

		if (incremental)
		{
			// 增量式的状态就是输出本身，直接回退输出
			out_prev += kBack * sat_diff;
		}
		else
		{
			integral += (kBack / (ki + 1.0e-6f)) * sat_diff;
		}
	}

	// 按带宽整定电流环参数（零极点对消）：
	//     kp = ω * L，ki = ω * R
	static void Calc_Current_Gains(float w_bandwidth, float r, float l, float *kp_, float *ki_);

private:
	float kp = 0.0f;
	float ki = 0.0f;
	float integral = 0.0f;
	float prev_error = 0.0f;
	float out_min = -1e30f;
	float out_max = 1e30f;
	float int_min = -1e30f;		// 积分项自己的限幅
	float int_max = 1e30f;
	float int_sep = 0.0f;		// 积分分离阈值（0=关闭）
	bool incremental = false;	// false=位置式 true=增量式（调试用）
	float out_prev = 0.0f;		// 增量式的输出状态 / 位置式的上一拍输出
};

/*------------------------------------------------------------------------------
 * 一个控制周期：梯形积分 + 条件积分抗饱和
 *
 * 电流环每个节拍要调 d/q 两次，放在头文件里强制内联，省掉 10kHz/20kHz
 * 中断里的两次函数调用。算法与 FOC_pi.cpp 里的说明完全一致。
 *------------------------------------------------------------------------------*/
inline __attribute__((always_inline)) float Pi_Controller::Process(float error, float dt)
{
	if (dt <= 0.0f) dt = 1e-6f;

	// 积分分离（连续权重）：误差接近 0 时积分全速工作，
	// 误差接近 int_sep 时权重平滑降到 0；不会像硬开关那样在阈值处跳一下。
	float int_weight = 1.0f;

	if (int_sep > 0.0f)
	{
		const float r = fabsf(error) / int_sep;

		// 用 !(r < 1) 而不是 r >= 1：NaN 时也走"权重=0"，不会把 NaN 传进积分
		if (!(r < 1.0f))
		{
			int_weight = 0.0f;
		}
		else
		{
			// smoothstep：两端斜率为 0，切换更干净
			int_weight = 1.0f - (3.0f * r * r - 2.0f * r * r * r);
		}
	}

	const float delta = ki * 0.5f * dt * (error + prev_error) * int_weight;

	// ---- 增量式：out += Kp·Δe + Ki·∫e ----
	// 状态是输出本身，限幅直接就抗饱和；积分项是增量，天然不会累积过头。
	if (incremental)
	{
		const float du = kp * (error - prev_error) + delta;

		float out = out_prev + du;

		if (out > out_max) out = out_max;
		else if (out < out_min) out = out_min;

		out_prev = out;
		prev_error = error;

		return out;
	}

	// ---- 位置式 ----
	const float new_integral = integral + delta;

	float out = kp * error + new_integral;

	if (out > out_max)
	{
		out = out_max;
		if (delta <= 0.0f) integral = new_integral;	// 只允许"往下退"的积分
	}
	else if (out < out_min)
	{
		out = out_min;
		if (delta >= 0.0f) integral = new_integral;	// 只允许"往上退"的积分
	}
	else
	{
		integral = new_integral;
	}

	// 积分限幅 = [int_min,int_max] 和 [out_min,out_max] 的交集：
	// 没单独设积分限幅时（电流环）就等于输出限幅，和原来的行为一致；
	// 设了更小的积分限幅（转速环）就按更小的来。
	const float i_lo = (int_min > out_min) ? int_min : out_min;
	const float i_hi = (int_max < out_max) ? int_max : out_max;

	if (integral > i_hi) integral = i_hi;
	else if (integral < i_lo) integral = i_lo;

	out_prev = out;
	prev_error = error;

	return out;
}
