#include "FOC_position.h"

/*==============================================================================
 * 开环角度发生器
 *============================================================================*/
open_loop_sensor::open_loop_sensor()
{
	Set_Angle(0.0f);
	Set_Speed(0.0f);
}

void open_loop_sensor::Set_Speed(float speed_)
{
	speed = speed_;
}

void open_loop_sensor::Set_Angle(float angle_)
{
	angle = foc_math::Norm_Angle(angle_);
}

void open_loop_sensor::Position_Update(float dt)
{
	angle = foc_math::Norm_Angle(angle + speed * dt);
}

/*==============================================================================
 * 滑模观测器
 *============================================================================*/
smo_sensor::smo_sensor()
{
	Reset();

	// 锁相环缺省参数：带宽几百 rad/s，比电流环慢很多，够用且不容易被噪声带跑
	Set_Pll_Gain(200.0f, 20000.0f);
}

void smo_sensor::Reset()
{
	i_hat.alpha = 0.0f;
	i_hat.beta = 0.0f;
	emf.alpha = 0.0f;
	emf.beta = 0.0f;
	pll_integral = 0.0f;
	pll_angle = 0.0f;
	angle = 0.0f;
	speed = 0.0f;
	valid = false;
}

void smo_sensor::Set_Motor_Params(float r_, float l_, float flux_)
{
	if (l_ <= 0.0f) return;

	r = r_;
	l = l_;
	flux = flux_;
}

void smo_sensor::Set_Gain_Auto(float w_elec_max)
{
	auto_w_max = fabsf(w_elec_max);
	auto_tune = true;
	tuned = false;
}

/*------------------------------------------------------------------------------
 * 自动整定（第一次调用 Position_Update 时按实际 dt 算）
 *
 *   1) 等效增益 g = k/φ
 *        g >> R          才有精度
 *        g < 2L/dt - R   否则离散化后观测器发散
 *   2) 边界层 φ
 *        φ > |e|max/g    保证不饱和、输出平滑
 *------------------------------------------------------------------------------*/
void smo_sensor::Auto_Tune(float dt)
{
	// 离散稳定性上限留 80% 余量
	float g_max = 0.8f * (2.0f * l / dt - r);
	if (g_max < r) g_max = r + 0.1f;

	float g_use = (g_max > 6.0f) ? 6.0f : g_max;

	const float emf_max = fabsf(auto_w_max) * flux;

	phi = 2.0f * emf_max / g_use;				// 留 2 倍余量
	if (phi < 0.1f) phi = 0.1f;
	k = g_use * phi;

	tuned = true;
}

void smo_sensor::Set_Sliding_Gain(float k_)
{
	k = k_;
	auto_tune = false;
	tuned = true;
}

void smo_sensor::Set_Boundary_Layer(float phi_)
{
	phi = phi_;
	auto_tune = false;
	tuned = true;
}

void smo_sensor::Set_Emf_Filter(float factor_)
{
	if (factor_ <= 0.0f || factor_ > 1.0f) return;

	emf_filter = factor_;
}

void smo_sensor::Set_Pll_Gain(float kp_, float ki_)
{
	pll_kp = kp_;
	pll_ki = ki_;
}

void smo_sensor::Set_Emf_Min(float emf_min_)
{
	emf_min = emf_min_;
}

void smo_sensor::Set_Angle_Compensate(bool enable_)
{
	angle_comp = enable_;
}

/*------------------------------------------------------------------------------
 * 饱和函数：|error| <= φ 时线性输出，超过就饱和
 * φ = 0 时退化成 sign（纯滑模，抖振大但抗扰最强）
 *------------------------------------------------------------------------------*/
float smo_sensor::Sat(float error)
{
	if (phi <= 0.0f)
	{
		return (error >= 0.0f) ? 1.0f : -1.0f;
	}

	float s = error / phi;
	if (s > 1.0f) s = 1.0f;
	else if (s < -1.0f) s = -1.0f;

	return s;
}

void smo_sensor::Update_Input(const foc_math::AlphaBeta &v_ab, const foc_math::AlphaBeta &i_ab)
{
	v_in = v_ab;
	i_in = i_ab;
}

/*------------------------------------------------------------------------------
 * 一个控制周期：电流观测器 -> 等效反电势 -> 锁相环出角度
 *------------------------------------------------------------------------------*/
void smo_sensor::Position_Update(float dt)
{
	if (dt <= 0.0f || l <= 0.0f) return;

	if (auto_tune && !tuned)
	{
		Auto_Tune(dt);
	}

	// ---- 1. 电流观测器 + 滑模控制律 ----
	const float z_alpha = k * Sat(i_hat.alpha - i_in.alpha);
	const float z_beta  = k * Sat(i_hat.beta  - i_in.beta);

	// L*dî/dt = v - R*î - z
	i_hat.alpha += (v_in.alpha - r * i_hat.alpha - z_alpha) * dt / l;
	i_hat.beta  += (v_in.beta  - r * i_hat.beta  - z_beta)  * dt / l;

	// ---- 2. 低通取出等效控制量 ----
	emf.alpha = foc_math::Lp_Filter(emf.alpha, z_alpha, emf_filter);
	emf.beta  = foc_math::Lp_Filter(emf.beta,  z_beta,  emf_filter);

	const float emf_mag = sqrtf(emf.alpha * emf.alpha + emf.beta * emf.beta);
	valid = (emf_mag > emf_min);

	// ---- 3. 锁相环 ----
	float err = 0.0f;
	if (emf_mag > 1e-6f)
	{
		err = (-emf.alpha * cosf(pll_angle) - emf.beta * sinf(pll_angle)) / emf_mag;
	}

	pll_integral += pll_ki * err * dt;
	pll_integral = foc_math::Limit(pll_integral, -SMO_SPEED_MAX, SMO_SPEED_MAX);
	speed = foc_math::Limit(pll_kp * err + pll_integral, -SMO_SPEED_MAX, SMO_SPEED_MAX);
	pll_angle = foc_math::Norm_Angle(pll_angle + speed * dt);

	// ---- 4. 低通相位滞后补偿 ----
	if (angle_comp)
	{
		const float tau = dt * (1.0f - emf_filter) / emf_filter;
		angle = foc_math::Norm_Angle(pll_angle + atanf(speed * tau));
	}
	else
	{
		angle = pll_angle;
	}
}
