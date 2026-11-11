#include "FOC_control.h"

Foc_Controller::Foc_Controller()
{
	Set_Pi(kp, ki);
	Set_Speed_Pi(0.0f, 0.0f, 0.0f);
	Reset_State();
}

/*------------------------------------------------------------------------------
 * 电流环：一个控制周期的全部计算
 *------------------------------------------------------------------------------*/
Foc_Controller::Output Foc_Controller::Step(const Input &input_)
{
	Output out;
	out.duty = {0.5f, 0.5f, 0.5f};
	out.i_dq = i_dq;
	out.v_dq = {vd, vq};
	out.v_pi_dq = {vd_pi, vq_pi};
	out.mod = mod;
	out.sc = sc;
	out.active = false;

	// 用 !(x > 0) 而不是 x <= 0：NaN 时比较结果都是 false，
	// 否则 v_bus = NaN 会一路传到 SVPWM，再被 (uint32_t) 转成 0，
	// 相当于三路上桥全开，非常危险。
	if (!(input_.dt > 0.0f) || !(input_.v_bus > 0.0f))
	{
		out.mod = {0.0f, 0.0f};
		out.v_dq = {0.0f, 0.0f};
		out.v_pi_dq = {0.0f, 0.0f};
		return out;
	}

	// 未使能：直接输出零矢量，**不要**先算 sin/cos / Clarke / Park。
	// 这段每 50us 都要跑一次，没使能时还在中断里做三角函数纯属浪费：
	// 20kHz 中断一旦超过 50us，FreeRTOS 任务（串口上报）就会被饿死，
	// 表现出来就是"切到非开环模式/一开串口，串口就不发数据了"。
	if (!enable)
	{
		Reset_State();
		i_dq = {0.0f, 0.0f};
		out.i_dq = i_dq;
		out.duty = {0.5f, 0.5f, 0.5f};
		out.mod = {0.0f, 0.0f};
		out.v_dq = {0.0f, 0.0f};
		out.v_pi_dq = {0.0f, 0.0f};
		out.sc = sc;
		return out;
	}

	// 1. 电流：三相 -> αβ -> dq
	const foc_math::AlphaBeta i_ab = foc_math::Clarke(input_.i_abc);
	i_abc_meas = input_.i_abc;			// 死区补偿直接复用，省一遍逆变换
	sc = foc_math::Sin_Cos(input_.angle);
	i_dq = foc_math::Park(i_ab, sc);

	// 2. 电流只有瞬时值：解耦前馈和死区补偿都用本拍采样，
	//    不再做低通（滤波的 ~0.5ms 滞后会让加减速时的耦合电压补不干净）。

	// 4. PI：电流误差 -> 电压指令
	if (voltage_override)
	{
		// 参数辨识测试：直接给 dq 电压，绕过 PI
		vd = ov_vd;
		vq = ov_vq;
		vd_pi = 0.0f;
		vq_pi = 0.0f;
	}
	else
	{
		// 电流指令统一限幅：MIT 力矩、转速环输出、上位机电流指令都走这里，
		// 防止某一环（比如高 Kp 阻抗）把电流指令推到硬件限流之外。
		const float id_ref = foc_math::Limit_Abs(id_target, iq_limit);
		const float iq_ref = foc_math::Limit_Abs(iq_target + iq_cog, iq_limit);

		vd = pi_d.Process(id_ref - i_dq.d, input_.dt);
		vq = pi_q.Process(iq_ref - i_dq.q, input_.dt);
		vd_pi = vd;
		vq_pi = vq;

		// 5. 解耦前馈：vd = R*id + Ld*did/dt - ωe*Lq*iq
		//    用本拍瞬时电流：电流是同步采样，纹波在采样点上是确定的；
		//    滤波会带 ~0.5ms 滞后，加减速时反而补不干净
		if (decoupling == DECOUPLING_CROSS || decoupling == DECOUPLING_CROSS_BEMF)
		{
			vd -= i_dq.q * input_.speed_elec * lq;
			vq += i_dq.d * input_.speed_elec * ld;
		}
		if (decoupling == DECOUPLING_BEMF || decoupling == DECOUPLING_CROSS_BEMF)
		{
			vq += input_.speed_elec * flux;
		}

		// 6. 电压圆限幅（d 轴优先）
		Limit_Voltage(input_.v_bus);
	}

	// 7. 逆 Park + 归一化：dq 电压 -> αβ 调制度
	const foc_math::Dq v_dq_out = {vd, vq};
	const foc_math::AlphaBeta mod_ab = foc_math::Inv_Park(v_dq_out, sc);
	const float voltage_normalize = foc_math::SQRT3 / input_.v_bus;

	mod.alpha = mod_ab.alpha * voltage_normalize;
	mod.beta = mod_ab.beta * voltage_normalize;

	// 8. 死区补偿（可选）
	if (dead_time_enable)
	{
		Dead_Time_Compensate(mod.alpha, mod.beta, input_.v_bus);
	}

	// 9. SVPWM -> 三相占空比（period = 1 时直接得到 0~1 的占空比）
	duty = foc_math::Svpwm(mod, 1.0f);

	// 10. 兜底：任何 NaN/Inf/越界都退回零矢量，
	//     否则 Set_Duty 里 (uint32_t) 转换会把 NaN 变成 0，三路上桥全开
	if (!(duty.a >= 0.0f && duty.a <= 1.0f)) duty.a = 0.5f;
	if (!(duty.b >= 0.0f && duty.b <= 1.0f)) duty.b = 0.5f;
	if (!(duty.c >= 0.0f && duty.c <= 1.0f)) duty.c = 0.5f;

	out.duty = duty;
	out.i_dq = i_dq;
	out.v_dq = {vd, vq};
	out.v_pi_dq = {vd_pi, vq_pi};
	out.mod = mod;
	out.sc = sc;
	out.active = true;

	return out;
}

/*------------------------------------------------------------------------------
 * 转速环：放在 1kHz 任务里跑（比电流环慢一个数量级）
 *------------------------------------------------------------------------------*/
void Foc_Controller::Speed_Loop(float rpm_feedback, float dt)
{
	if (!enable || !speed_loop_enable) return;

	if (dt <= 0.0f) dt = dt_speed;

	// 目标斜坡：速度突变时先把目标按 kSpeedTargetAccel 平滑地推过去，
	// 避免 PI 输出（iq）瞬间跳变，引起电流/机械的啸叫。
	// 使能后的第一拍从当前实际转速起步，不会从 0 硬拉到目标。
	if (!speed_ramp_ready)
	{
		speed_target_ramped = rpm_feedback;
		speed_ramp_ready = true;
	}

	if (speed_target_accel > 0.0f)
	{
		const float step = speed_target_accel * dt;
		const float err = speed_target - speed_target_ramped;

		if (err > step) speed_target_ramped += step;
		else if (err < -step) speed_target_ramped -= step;
		else speed_target_ramped = speed_target;
	}
	else
	{
		speed_target_ramped = speed_target;
	}

	// 速度误差 -> iq 指令。id 指令一般保持 0，需要弱磁时再用 Set_Target 改成负值
	// 加速度前馈直接叠在 PI 输出上，再一起限幅
	iq_target = pi_speed.Process(speed_target_ramped - rpm_feedback, dt) + iq_ff;
	iq_target = foc_math::Limit_Abs(iq_target, speed_limit);
}

/*------------------------------------------------------------------------------
 * 电压圆限幅 + 反算抗积分饱和（back-calculation）
 *
 * d 轴优先：弱磁和效率都靠 d 轴，电压不够时先保证 d 轴，
 * q 轴只能用剩下的电压，满足 vd² + vq² ≤ max²。
 *
 * 抗积分饱和：积分器按 Kb·T·(v_sat − v_pi) 回退，直接看"实际输出与
 * PI 输出的差"，不依赖限幅数值——q 轴真实限幅是 √(max²−vd²)，
 * 旧的"按 max 限积分"在 vd 大时会让积分虚高，退饱和慢。
 *------------------------------------------------------------------------------*/
void Foc_Controller::Limit_Voltage(float v_bus_)
{
	const float max_v_mag = v_bus_ * foc_math::ONE_BY_SQRT3 * max_duty * overmod_factor * thermal_derate;

	// 1. 限幅前先留一份 PI 净输出
	const float vd_pi_raw = vd;
	const float vq_pi_raw = vq;

	// 2. d 轴优先
	vd = foc_math::Limit_Abs(vd, max_v_mag);

	// 3. q 轴用剩下的电压
	const float rest = max_v_mag * max_v_mag - vd * vd;
	const float max_vq = (rest > 0.0f) ? sqrtf(rest) : 0.0f;

	vq = foc_math::Limit_Abs(vq, max_vq);

	// 4. 反算抗饱和：被削掉多少，积分就退回多少（与 dt 无关）
	pi_d.Back_Calc(vd - vd_pi_raw, 0.0f);
	pi_q.Back_Calc(vq - vq_pi_raw, 0.0f);
}

/*------------------------------------------------------------------------------
 * 死区补偿
 *
 * 补偿量 = 死区时间 / PWM周期，方向跟电流方向一致。
 * 用的是本拍 ADC 采样的瞬时相电流（i_abc_meas），不滤波、不做 z⁻¹ 延迟：
 * 死区畸变是逐拍发生的，滤波会带进相位滞后，低电流过零时更容易补错方向。
 * 过零附近的符号抖动由 Dead_Time_Voltage() 的三次多项式平滑处理。
 *------------------------------------------------------------------------------*/
void Foc_Controller::Dead_Time_Compensate(float &mod_alpha_, float &mod_beta_, float v_bus_)
{
	if (!(v_bus_ > 1.0f)) return;

	// 复用 Step 里存的相电流（每拍已算好），省掉一次 Inv_Park/Inv_Clarke
	const foc_math::Abc &i_abc = i_abc_meas;

	foc_math::Abc vc;
	vc.a = Dead_Time_Voltage(i_abc.a, v_bus_);
	vc.b = Dead_Time_Voltage(i_abc.b, v_bus_);
	vc.c = Dead_Time_Voltage(i_abc.c, v_bus_);

	const foc_math::AlphaBeta vc_ab = foc_math::Clarke(vc);

	// 电压 -> 调制度
	const float k = foc_math::SQRT3 / v_bus_;

	mod_alpha_ += vc_ab.alpha * k;
	mod_beta_ += vc_ab.beta * k;
}

/*------------------------------------------------------------------------------
 * 死区补偿（多项式过渡）
 *
 * 每相的补偿电压曲线（以一阶导连续的 smoothstep 为基础）：
 *
 *   Vmax ┤                  ●━━━━━━━━━━  ← 饱和段
 *        │               ╱
 *        │            ╱        ← 过渡段（kOrder=3 用 3x²−2x³，C¹）
 *        │         ╱
 *       0┤━━━━━━●                          ●━━━━━ ... 每相独立
 *        │      ╱ ╲
 *        │   ╱      ╲
 *  −Vmax ┤╱           ●━━━━━━━━━━━━
 *        └─────┬──────┬──────┬──────
 *             −knee   0     +knee
 *
 *   |i| ≤ knee   V = Vmax·sign(i)·poly(|i|/knee)
 *   |i| >  knee   V = Vmax·sign(i)
 *
 * 连续性对比（x = |i|/knee）：
 *   kOrder=3  三次 3x²−2x³：    x=0 和 x=1 处 C¹（值、一阶导连续）
 *   kOrder=5  五次 10x³−15x⁴+6x⁵：x=0 和 x=1 处 C²（二阶导也连续，真样条）
 *
 * 注意：kOrder=3 在两个端点的二阶导仍跳变（±6Vmax/knee²）；
 *       kOrder=5 过零建立更慢（同 knee 下 0.05A 处补偿量更低）。
 *------------------------------------------------------------------------------*/
constexpr uint8_t kDtCompOrder = 3;		// 3=三次（C¹）

float Foc_Controller::Dead_Time_Voltage(float current_, float v_bus_) const
{
	const float a = fabsf(current_);

	if (a < 1.0e-6f) return 0.0f;

	float v_dead = v_bus_ * (dead_time_us * 1.0e-6f) * pwm_freq;
	// 只补死区造成的伏秒缺失：ΔV = Vbus·Td·fs·sign(i)。
	// v_max = v_dead + dt_comp_drop，其中附加项默认 0（见 app_config.h）。
	const float v_max = v_dead + dt_comp_drop;

	const float sgn = (current_ >= 0.0f) ? 1.0f : -1.0f;

	if (a >= dt_comp_knee)
	{
		return dt_comp_sign * sgn * v_max;				// 饱和段
	}

	const float x = a / dt_comp_knee;

	float s;

	if (kDtCompOrder == 5)
	{
		// 五次：x³(10−15x+6x²)，x=0/x=1 处一阶、二阶导全为 0
		const float x3 = x * x * x;
		s = x3 * (10.0f - 15.0f * x + 6.0f * x * x);
	}
	else
	{
		// 三次：3x²−2x³
		const float x2 = x * x;
		s = 3.0f * x2 - 2.0f * x2 * x;
	}

	return dt_comp_sign * sgn * v_max * s;
}

float Foc_Controller::Speed_To_Rpm(float speed_elec_) const
{
	if (pole_pairs == 0) return 0.0f;

	return speed_elec_ * 60.0f / (foc_math::TWO_PI * (float)pole_pairs);
}

void Foc_Controller::Reset_State()
{
	pi_d.Reset();
	pi_q.Reset();
	vd = 0.0f;
	vq = 0.0f;
	vd_pi = 0.0f;
	vq_pi = 0.0f;
	mod = {0.0f, 0.0f};
	duty = {0.5f, 0.5f, 0.5f};
}

/*==============================================================================
 * 参数配置
 *============================================================================*/
void Foc_Controller::Set_Enable(bool enable_)
{
	// 切换使能状态时清掉积分和滤波状态，避免带着上一次的残留突然输出。
	// 由 App 在任务上下文调用，ISR 只读 enable 标志。
	enable = enable_;
	Reset_State();
}

void Foc_Controller::Set_Target(float id_target_, float iq_target_)
{
	id_target = id_target_;
	iq_target = iq_target_;

	speed_loop_enable = false;			// 直接给电流指令，就把转速环关掉
}

void Foc_Controller::Set_Id_Target(float id_target_)
{
	id_target = id_target_;
}

void Foc_Controller::Set_Speed_Target(float rpm_target_)
{
	speed_target = rpm_target_;
	speed_loop_enable = true;
}

/*------------------------------------------------------------------------------
 * 力矩指令：τ -> iq
 *
 * MIT 阻抗模式直接给力矩，不经过转速环。Kt = 1.5·极对数·ψf。
 * 用 !(tau == tau) 挡住 NaN，避免 NaN 一路传到 PI 再进 SVPWM。
 *------------------------------------------------------------------------------*/
void Foc_Controller::Set_Torque_Target(float tau_)
{
	const float kt = Get_Kt();

	if (!(kt > 0.0f) || !(tau_ == tau_))
	{
		iq_target = 0.0f;
		speed_loop_enable = false;
		return;
	}

	iq_target = foc_math::Limit_Abs(tau_ / kt, iq_limit);
	speed_loop_enable = false;
}

void Foc_Controller::Set_Iq_Limit(float iq_max_)
{
	if (iq_max_ > 0.0f) iq_limit = iq_max_;
}

float Foc_Controller::Get_Kt() const
{
	return 1.5f * (float)pole_pairs * flux * kt_scale;
}

void Foc_Controller::Set_Kt_Scale(float scale_)
{
	if (!(scale_ > 0.1f)) scale_ = 0.1f;
	if (scale_ > 10.0f) scale_ = 10.0f;

	kt_scale = scale_;
}

void Foc_Controller::Reset_Speed_Pi()
{
	pi_speed.Reset();
	speed_ramp_ready = false;		// 下一拍从实际转速重新起步
}

void Foc_Controller::Set_Speed_Target_Accel(float rpm_per_s)
{
	speed_target_accel = (rpm_per_s > 0.0f) ? rpm_per_s : 0.0f;
}

void Foc_Controller::Set_Pi(float kp_, float ki_)
{
	kp = kp_;
	ki = ki_;

	pi_d.Set_Kp_Ki(kp, ki);
	pi_q.Set_Kp_Ki(kp, ki);
}

void Foc_Controller::Set_Pi_Dq(float kp_d_, float kp_q_, float ki_)
{
	kp = kp_d_;
	ki = ki_;

	pi_d.Set_Kp_Ki(kp_d_, ki_);
	pi_q.Set_Kp_Ki(kp_q_, ki_);
}

/*------------------------------------------------------------------------------
 * 在线整定：d/q 两轴各自的 Kp、Ki 独立写
 *   只改增益，不动积分项，所以可以在使能状态下直接调，不会因为改参数清积分
 *   产生的扰动。上位机每 50ms 重发一次，这里重复调用是幂等的。
 *------------------------------------------------------------------------------*/
void Foc_Controller::Set_Pi_Dq4(float kp_d_, float ki_d_, float kp_q_, float ki_q_)
{
	kp = kp_d_;
	ki = ki_d_;

	pi_d.Set_Kp_Ki(kp_d_, ki_d_);
	pi_q.Set_Kp_Ki(kp_q_, ki_q_);
}

void Foc_Controller::Set_Pi_By_Bandwidth(float w_bandwidth, float r_, float l_)
{
	float kp_;
	float ki_;

	Pi_Controller::Calc_Current_Gains(w_bandwidth, r_, l_, &kp_, &ki_);
	Set_Pi(kp_, ki_);
}

void Foc_Controller::Set_Pi_Dq_By_Bandwidth(float w_bandwidth, float r_, float ld_, float lq_)
{
	float kp_d_;
	float kp_q_;
	float ki_;

	Pi_Controller::Calc_Current_Gains(w_bandwidth, r_, ld_, &kp_d_, &ki_);
	Pi_Controller::Calc_Current_Gains(w_bandwidth, r_, lq_, &kp_q_, &ki_);

	Set_Pi_Dq(kp_d_, kp_q_, ki_);
}

void Foc_Controller::Set_Speed_Pi(float kp_, float ki_, float limit_)
{
	pi_speed.Set_Kp_Ki(kp_, ki_);
	pi_speed.Set_Limit(-limit_, limit_);

	if (limit_ > 0.0f) speed_limit = limit_;
}

void Foc_Controller::Set_Speed_Integral_Limit(float limit_)
{
	if (limit_ <= 0.0f) return;

	pi_speed.Set_Integral_Limit(-limit_, limit_);
}

void Foc_Controller::Set_Speed_Integral_Separation(float error_band_)
{
	pi_speed.Set_Integral_Separation(error_band_);
}

void Foc_Controller::Set_Motor_Params(float r_, float ld_, float lq_, float flux_)
{
	r = r_;
	ld = ld_;
	lq = lq_;
	flux = flux_;
}

void Foc_Controller::Set_Pole_Pairs(uint8_t pole_pairs_)
{
	if (pole_pairs_ == 0) return;

	pole_pairs = pole_pairs_;
}

void Foc_Controller::Set_Thermal_Derate(float ratio_)
{
	if (ratio_ < 0.05f) ratio_ = 0.05f;
	if (ratio_ > 1.0f) ratio_ = 1.0f;

	thermal_derate = ratio_;
}

void Foc_Controller::Adjust_Ki_D_Only(float ki_)
{
	if (!(ki_ > 0.0f)) return;
	pi_d.Set_Kp_Ki(pi_d.Get_Kp(), ki_);
}

void Foc_Controller::Adjust_Ki_Q_Only(float ki_)
{
	if (!(ki_ > 0.0f)) return;
	pi_q.Set_Kp_Ki(pi_q.Get_Kp(), ki_);
}

void Foc_Controller::Set_Voltage_Limit(float max_duty_, float overmod_factor_)
{
	if (max_duty_ <= 0.0f || max_duty_ > 1.0f || overmod_factor_ <= 0.0f) return;

	max_duty = max_duty_;
	overmod_factor = overmod_factor_;
}

void Foc_Controller::Set_Decoupling(Decoupling type_)
{
	decoupling = type_;
}

void Foc_Controller::Set_Voltage_Override(bool enable_, float vd_, float vq_)
{
	voltage_override = enable_;
	ov_vd = vd_;
	ov_vq = vq_;
}

void Foc_Controller::Set_Dead_Time_Comp(float dead_time_us_, bool enable_)
{
	dead_time_us = dead_time_us_;
	dead_time_enable = enable_;
}

void Foc_Controller::Set_Dead_Time_Volt(float drop_, float knee_, float sign_)
{
	dt_comp_drop = (drop_ > 0.0f) ? drop_ : 0.0f;
	dt_comp_knee = (knee_ > 0.001f) ? knee_ : 0.001f;
	dt_comp_sign = (sign_ < 0.0f) ? -1.0f : 1.0f;
}

void Foc_Controller::Set_Pi_Incremental(bool on_)
{
	pi_d.Set_Incremental(on_);
	pi_q.Set_Incremental(on_);
	pi_speed.Set_Incremental(on_);
}

void Foc_Controller::Set_Pwm_Freq(float freq_)
{
	if (freq_ <= 0.0f) return;

	pwm_freq = freq_;
}
