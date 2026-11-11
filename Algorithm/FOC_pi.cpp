#include "FOC_pi.h"

Pi_Controller::Pi_Controller(float kp_, float ki_)
	: kp(kp_)
	, ki(ki_)
{
}

void Pi_Controller::Set_Kp_Ki(float kp_, float ki_)
{
	kp = kp_;
	ki = ki_;
}

void Pi_Controller::Set_Limit(float min_, float max_)
{
	if (min_ > max_)
	{
		float tmp = min_;
		min_ = max_;
		max_ = tmp;
	}

	out_min = min_;
	out_max = max_;
}

void Pi_Controller::Set_Integral_Limit(float min_, float max_)
{
	if (min_ > max_)
	{
		float tmp = min_;
		min_ = max_;
		max_ = tmp;
	}

	int_min = min_;
	int_max = max_;

	if (integral > int_max) integral = int_max;
	if (integral < int_min) integral = int_min;
}

void Pi_Controller::Set_Integral_Separation(float error_band_)
{
	int_sep = fabsf(error_band_);
}

void Pi_Controller::Set_Incremental(bool on_)
{
	if (on_ == incremental) return;

	if (on_)
	{
		// 位置式 → 增量式：out_prev 在位置式里每拍都记着实际输出，
		// 直接拿来当增量式的状态即可（饱和时也是对的，不会跳）。
	}
	else
	{
		// 增量式 → 位置式：反推积分项，保证下一次输出不变
		integral = out_prev - kp * prev_error;
	}

	incremental = on_;
}

void Pi_Controller::Reset()
{
	integral = 0.0f;
	prev_error = 0.0f;
	out_prev = 0.0f;
}

/*------------------------------------------------------------------------------
 * 一个控制周期：梯形积分 + 条件积分抗饱和
 *
 * 实现已移到 FOC_pi.h 内联（电流环每个节拍要调 d/q 两次）。
 * 参考框架的离散 PI：
 *   Io += T*Ki/2 * (e[k] + e[k-1])
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 * 电流环参数整定
 *
 * 被控对象：I(s)/V(s) = 1/(Ls + R)
 * PI 写成 kp*(s + ki/kp)/s，令零点 ki/kp = R/L 抵消被控对象极点，
 * 开环传递函数变成 kp/(L*s)，穿越频率（带宽）ω = kp/L，于是：
 *     kp = ω * L        ki = ω * R
 *------------------------------------------------------------------------------*/
void Pi_Controller::Calc_Current_Gains(float w_bandwidth, float r, float l, float *kp_, float *ki_)
{
	if (kp_ != nullptr) *kp_ = w_bandwidth * l;
	if (ki_ != nullptr) *ki_ = w_bandwidth * r;
}
