#include "control_loop.h"
#include "RC_io.h"
#include "RC_prof.h"
#include "app_config.h"

namespace app
{
	namespace
	{
		// 在线整定用的安全限幅：防止上位机误输入把电流环推到不稳定
		float Clamp_Gain(float v_, float lo_, float hi_)
		{
			if (!(v_ >= lo_)) return lo_;		// 同时挡住 NaN
			if (v_ > hi_) return hi_;
			return v_;
		}

		/*--------------------------------------------------------------------------
		 * ADRC 离散 TD 的最速控制函数 fhan
		 *
		 *   x1: 位置误差（规划位置 - 目标）
		 *   x2: 规划速度
		 *   r : 最大加速度 (rad/s²)
		 *   h : 积分步长 (s)
		 *
		 * 输出是"最速下落曲线"的加速度：先以 +r 加速，接近目标时以 -r 减速，
		 * 不会超调，也不会像一阶低通那样拖尾。
		 *------------------------------------------------------------------------*/
		float Fhan(float x1, float x2, float r, float h)
		{
			const float d = r * h;
			const float d0 = h * d;
			const float y = x1 + h * x2;
			const float a0 = sqrtf(d * d + 8.0f * r * fabsf(y));

			float a;
			if (fabsf(y) <= d0)
			{
				a = x2 + y / h;
			}
			else
			{
				a = x2 + 0.5f * (a0 - d) * (y >= 0.0f ? 1.0f : -1.0f);
			}

			if (fabsf(a) <= d)
			{
				return -r * ((a >= 0.0f) ? 1.0f : -1.0f);
			}

			return -r * a / d;
		}
	}

	ControlLoop::ControlLoop(adc::Adc &adc_, pwm::Pwm &pwm_, tim::Tim &tick_, uint8_t pole_pairs_,
							 framework::SpscRing<Command, 8> &commands_,
							 framework::Latest<ControlSnapshot> &snapshot_out_,
							 framework::Latest<PowerSnapshot> &power_in_)
		: adc::AdcInjectedHandler(&adc_)		// 注入组完成回调
		, tim::TimHandler(&tick_)				// tim 风格：构造时挂到 1kHz 定时器
		, adc(adc_)
		, pwm(pwm_)
		, commands(commands_)
		, snapshot_out(snapshot_out_)
		, power_in(power_in_)
		, open_loop(pole_pairs_)
		, pole_pairs(pole_pairs_)
	{
		position = &open_loop_position;
	}

	void ControlLoop::Init()
	{
		// 三角函数查表必须先建好：20kHz 中断里 Sin_Cos 每次都要用
		foc_math::Init_Lut();

		// 控制器参数（双闭环）
		controller.Set_Motor_Params(config::kMotorR, config::kMotorLd, config::kMotorLq, config::kMotorFlux);
		controller.Set_Pole_Pairs(pole_pairs);
		controller.Set_Kt_Scale(config::kKtScale);

		// 和 Runtime 构造函数里的初始化保持一致
		controller.Set_Speed_Pi(config::kSpeedPiKp, config::kSpeedPiKi, config::kSpeedPiLimit);
		controller.Set_Speed_Integral_Limit(config::kSpeedPiIntegralLimit);
		controller.Set_Speed_Integral_Separation(config::kSpeedPiIntegralSeparation);
		controller.Set_Voltage_Limit(0.95f, 1.0f);
		controller.Set_Iq_Limit(config::kIqLimit);
		controller.Set_Decoupling(Foc_Controller::DECOUPLING_CROSS_BEMF);
		controller.Set_Pwm_Freq(pwm.Get_Freq());
		controller.Set_Dead_Time_Comp(config::kDeadTimeUs, config::kDeadTimeCompEnable);
		controller.Set_Dead_Time_Volt(config::kDtCompDrop, config::kDtCompKnee, config::kDtCompSign);

		// 位置环是 PD，Kp/Kd 已在成员默认值里从 config 初始化

		// 电流方向：默认三相都为正。若上电后 iq 反馈方向与指令相反，
		// 在这里改符号，不要去改算法。
		current.Set_Sign(config::kCurrentSignA, config::kCurrentSignB, config::kCurrentSignC);

		if (encoder != nullptr)
		{
			encoder->Set_Pole_Pairs(pole_pairs);

			// 上电默认电角度零位：实测标定值（139.57°）。
			// 这样不用每次上电都重新标定；重新标定会覆盖 RAM 里的值。
			encoder->Set_Electrical_Offset(config::kElecAngleOffsetDeg * (foc_math::PI / 180.0f));
		}

		// TIM1 更新中断 -> 软件启动 ADC1 注入组；ADC EOC -> 电流环
		const float f_pwm = pwm.Get_Freq();
		const float adc_hz = (f_pwm > 0.0f) ? f_pwm : 20000.0f;

		// 电流 20kHz（可降频），转速 1kHz（和编码器速度更新率一致），位置 800Hz
		// 如果开了电流环降频，后面的分频和 dt 都按“电流环实际节拍”算
		current_decim = (config::kCurrentLoopDecim == 0) ? 1 : config::kCurrentLoopDecim;
		current_tick = 0;

		const float loop_hz = adc_hz / (float)current_decim;
		const float speed_div = loop_hz / (float)config::kSpeedLoopHz;
		const float position_div = loop_hz / (float)config::kPositionLoopHz;
		speed_decim = (speed_div < 1.0f) ? 1 : (uint8_t)(speed_div + 0.5f);
		position_decim = (position_div < 1.0f) ? 1 : (uint8_t)(position_div + 0.5f);

		dt = 1.0f / loop_hz;

		// 转速环/位置环的实际周期 = 分频数 / 电流环节拍。
		// 降频到 10kHz 时 10000/4000 只能取整到 3 分频（实际 3.33kHz），
		// 如果再按 1/4000 算 dt，积分增益会错 20%，必须用真实周期。
		speed_dt = (float)speed_decim / loop_hz;
		position_dt = (float)position_decim / loop_hz;

		// 位置环 D 项的一阶低通系数（双线性近似）
		{
			const float w = foc_math::TWO_PI * config::kPositionKdFilterHz * position_dt;
			pos_d_alpha = w / (1.0f + w);

			if (pos_d_alpha > 1.0f) pos_d_alpha = 1.0f;
			else if (pos_d_alpha < 0.0f) pos_d_alpha = 0.0f;
		}

		// 电流环带宽受实际节拍限制：默认不超过采样率的 1/20。
		// 10kHz 节拍时自动把 1kHz 带宽降到 500Hz，否则采样延迟会把相位裕度吃光，
		// 表现就是"电流环很抖、跟踪震荡"。
		const float bw_limit = loop_hz * foc_math::TWO_PI * config::kCurrentBwLoopRatio;
		const float bw = (config::kCurrentLoopBw < bw_limit) ? config::kCurrentLoopBw : bw_limit;
		controller.Set_Pi_Dq_By_Bandwidth(bw, config::kMotorR, config::kMotorLd, config::kMotorLq);

		// 记录按带宽算出来的默认增益：上位机取消在线整定时恢复这一组
		pi_kp_d0 = controller.Get_Kp_D();
		pi_ki_d0 = controller.Get_Ki_D();
		pi_kp_q0 = controller.Get_Kp_Q();
		pi_ki_q0 = controller.Get_Ki_Q();

		v_bus = config::kDefaultBusVoltage;

		if (config::kUseSmo)
		{
			// 滑模观测器：先旁路运行（不参与控制），每周期照常估计角度
			smo.Set_Motor_Params(config::kMotorR, config::kMotorLd, config::kMotorFlux);
			smo.Set_Gain_Auto(2.0f * 3.14159f * 400.0f);		// 最高电角速度 400Hz
			smo.Set_Emf_Filter(0.2f);
			smo.Set_Pll_Gain(300.0f, 30000.0f);
		}

		// ADC1 注入组启动一次：硬件触发（TIM1 TRGO=OC4REF）下这次调用
		// 只做"使能 JEOC 中断 + 等 TRGO"，每个 PWM 周期由硬件自动触发转换
		adc.Adc_Injected_It_Start();
	}

	void ControlLoop::Set_Bus_Voltage(float v_bus_)
	{
		if (v_bus_ > 5.0f && v_bus_ < 60.0f)
		{
			v_bus = v_bus_;
		}
	}

	/*------------------------------------------------------------------------------
	 * 控制任务（1kHz）
	 *------------------------------------------------------------------------------*/
	void ControlLoop::Speed_Loop_Process(float dt_task)
	{
		// 1. 消费通信任务送来的命令
		Command cmd;
		while (commands.Try_Pop(cmd))
		{
			Apply_Command(cmd);
		}

		// 2. 消费监控任务的母线电压/温度快照
		PowerSnapshot power;
		if (power_in.Try_Read(power) && power.valid)
		{
			v_bus = power.v_bus;

			// NTC 有效（开路/短路时 Ntc_To_Temp 返回 -273）才更新
			if (power.motor_temp > -100.0f) motor_temp_latched = power.motor_temp;
			if (power.driver_temp > -100.0f) driver_temp_latched = power.driver_temp;
		}

		// 2.5 过温状态机 + Rs 温补（1kHz 足够，温度本来就是慢变量）
		Thermal_Protect();
		Update_Rs_Temp_Comp(false);
		Update_Flux_Estimate();

		// 3. 电角度标定中：只等转子稳定，不跑转速环
		if (cal_active)
		{
			Update_Angle_Calibration();
			Publish_Snapshot();
			return;
		}

		// 4. 测试信号
		if (test_type == 3 || test_type == 4)
		{
			// 两种扫频都在中断里生成和解调（1kHz 任务给不出高频正弦），
			// 这里只负责把当前频点报给上位机。
			const bool sweep_on = (test_type == 3) ? sweep_isr_active : speed_sweep_active;
			test_freq_now = sweep_on ? sweep_freq : 0.0f;
		}
		else
		{
			// 固定频率正弦（test_type 1/2）也在 4kHz 速度环节拍里生成：
			// 1kHz 任务给速度环的是 250µs 阶梯参考，频率越高台阶越大。
			// 这里只清上报值。
			test_freq_now = 0.0f;
		}

		// 5. 转速环在 ADC 中断里执行，这里只发布遥测快照
		Publish_Snapshot();
	}

	/*------------------------------------------------------------------------------
	 * 过温状态机（1kHz 任务里跑）
	 *
	 *   正常 →(绕组 >80°C / 驱动 >90°C)→ 降流（80~100°C 线性降到 30%）
	 *        →(绕组 >100°C)→ 停机
	 *   全部带 8°C 回差，避免在阈值附近反复横跳
	 *   NTC 开路/短路读出 -273°C，按 25°C 处理（不动作）
	 *------------------------------------------------------------------------------*/
	void ControlLoop::Thermal_Protect()
	{
		const float t_mot = motor_temp_latched;
		const float t_drv = driver_temp_latched;

		// NTC 异常值兜底
		const bool mot_ok = (t_mot > -50.0f && t_mot < 200.0f);
		const bool drv_ok = (t_drv > -50.0f && t_drv < 200.0f);

		switch (thermal_state)
		{
		case 0:		// 正常
			if ((mot_ok && t_mot >= config::kMotorTempStopC + config::kTempRecoverHystC)
				|| (drv_ok && t_drv >= config::kDriverTempStopC + config::kTempRecoverHystC))
			{
				thermal_state = 2;
				Set_Output_Enable(false);
				controller.Set_Enable(false);
			}
			else if (mot_ok && t_mot >= config::kMotorTempDerateC)
			{
				thermal_state = 1;
			}
			derate_ratio = 1.0f;
			break;

		case 1:		// 降流：80→100°C 线性压到 30%
			{
				const float span = config::kMotorTempStopC - config::kMotorTempDerateC;
				const float over = (mot_ok ? t_mot : 25.0f) - config::kMotorTempDerateC;
				float r = 1.0f - (over / span) * (1.0f - config::kTempDerateMinRatio);

				if (r > 1.0f) r = 1.0f;
				if (r < config::kTempDerateMinRatio) r = config::kTempDerateMinRatio;

				derate_ratio = r;

				if (!mot_ok || t_mot >= config::kMotorTempStopC
					|| (drv_ok && t_drv >= config::kDriverTempStopC))
				{
					thermal_state = 2;
					Set_Output_Enable(false);
					controller.Set_Enable(false);
				}
				else if (mot_ok && t_mot < config::kMotorTempDerateC - config::kTempRecoverHystC)
				{
					thermal_state = 0;
					derate_ratio = 1.0f;
				}
			}
			break;

		case 2:		// 停机：等温度降下来，上位机命令会重新使能
			derate_ratio = config::kTempDerateMinRatio;

			if ((!mot_ok || t_mot < config::kMotorTempStopC - config::kTempRecoverHystC)
				&& (!drv_ok || t_drv < config::kDriverTempStopC - config::kTempRecoverHystC))
			{
				thermal_state = 0;
				derate_ratio = 1.0f;
			}
			break;
		}

		// 降流：把电压圆按比例缩（等效于全部电流限幅缩小）
		controller.Set_Thermal_Derate(derate_ratio);
	}

	/*------------------------------------------------------------------------------
	 * Rs 温度补偿：Ki = ω·Rs(T)
	 *
	 * 只在比例变化超过 2% 时才写 PI，避免 1kHz 里反复写同一个值。
	 * Kp 不动（Kp=ω·L，L 随温度变化小）；写 Ki 用 Set_Kp_Ki 保持 Kp。
	 *------------------------------------------------------------------------------*/
	void ControlLoop::Update_Rs_Temp_Comp(bool force_reset_)
	{
		static float r_ratio_last = 1.0f;

		float t = motor_temp_latched;

		if (!(t > config::kRsTempMinC && t < config::kRsTempMaxC))
		{
			t = config::kRsTempRefC;		// NTC 异常，退回标称
		}

		float ratio = 1.0f + config::kRsTempCoef * (t - config::kRsTempRefC);

		if (ratio > config::kRsCompMaxRatio) ratio = config::kRsCompMaxRatio;
		if (ratio < 2.0f - config::kRsCompMaxRatio) ratio = 2.0f - config::kRsCompMaxRatio;

		if (force_reset_) r_ratio_last = -1.0f;

		if (fabsf(ratio - r_ratio_last) < 0.02f) return;		// 变化 <2% 不动

		r_ratio_last = ratio;

		if (pi_tuned) return;			// 上位机在线整定时不要覆盖

		const float ki_new = config::kCurrentLoopBw * config::kMotorR * ratio;

		controller.Adjust_Ki_D_Only(ki_new);
		controller.Adjust_Ki_Q_Only(ki_new);
	}

	/*------------------------------------------------------------------------------
	 * 在线磁链辨识（方法一：q 轴稳态反电势）
	 *
	 *   vq = R·iq + ωe·Ld·id + ωe·ψf
	 *   ψf = (vq − R·iq − ωe·Ld·id) / ωe
	 *
	 * 只在"高速、有负载、电压未饱和、没在扫频"时更新。1kHz 调用，
	 * 结果用 1Hz 低通，所以对单个坏点不敏感。
	 *------------------------------------------------------------------------------*/
	void ControlLoop::Update_Flux_Estimate()
	{
		if (!controller.Get_Enable()) { psi_state = 1; return; }
		if (test_type != 0) { psi_state = 2; return; }	// 扫频/辨识期间工况不平稳
		if (encoder == nullptr || position != encoder) { psi_state = 3; return; }

		const float we = encoder->Get_Speed();		// 电角速度 (rad/s)
		if (fabsf(we) < config::kFluxEstMinElecW) { psi_state = 4; return; }

		const foc_math::Dq i = controller.Get_Current_Dq();
		const foc_math::Dq v = controller.Get_Voltage_Dq();

		if (fabsf(i.q) < config::kFluxEstMinIq) { psi_state = 5; return; }

		// 电压圆饱和时 vq 被削顶，估出来会偏小，直接跳过
		const float v_mag = sqrtf(v.d * v.d + v.q * v.q);
		const float v_lim = v_bus * foc_math::ONE_BY_SQRT3 * 0.95f * derate_ratio * 0.9f;
		if (v_mag > v_lim) { psi_state = 6; return; }

		// Rs 温补：优先用双频 L 辨识时顺带测出的 R（带测量时的温度基准），
		// 没有测过就用标称值。
		float t = motor_temp_latched;
		if (!(t > config::kRsTempMinC && t < config::kRsTempMaxC))
		{
			t = config::kRsTempRefC;
		}

		float ratio = 1.0f + config::kRsTempCoef * (t - r_ident_temp);
		if (ratio > config::kRsCompMaxRatio) ratio = config::kRsCompMaxRatio;
		if (ratio < 2.0f - config::kRsCompMaxRatio) ratio = 2.0f - config::kRsCompMaxRatio;

		const float r = r_ident * ratio;

		// 指令电压已含死区补偿，可以认为等于实际相电压
		const float psi_raw = (v.q - r * i.q - we * config::kMotorLd * i.d) / we;

		const float lo = config::kFluxEstMinRatio * config::kMotorFlux;
		const float hi = config::kFluxEstMaxRatio * config::kMotorFlux;

		if (!(psi_raw > lo && psi_raw < hi)) { psi_state = 7; return; }

		psi_state = 0;

		// 1Hz 一阶低通（tau 远大于 1ms，直接用小角度近似）
		const float a = foc_math::TWO_PI * config::kFluxEstLpfHz * 0.001f;
		psi_est += a * (psi_raw - psi_est);
	}

	/*------------------------------------------------------------------------------
	 * MIT 阻抗：4kHz（和转速环节拍一致）
	 *
	 *   τ = Kp·(θd − θ) + Kd·(ωd − ω) + τff
	 *   然后 controller.Set_Torque_Target(τ) -> iq = τ/Kt -> 20kHz 电流环
	 *
	 * 注意这里不经过转速环：速度环的积分会把外力当成误差吃掉，
	 * 背驱透明性就没了。MIT 要的就是"外力直接改变位置"。
	 *------------------------------------------------------------------------------*/
	void ControlLoop::Mit_Step(float dt)
	{
		if (!mit_active || encoder == nullptr || !(dt > 0.0f)) return;

		const float p = encoder->Get_Mech_Angle_Rad();					// 连续机械角 (rad)
		const float v = encoder->Get_Mech_Rpm() * (foc_math::TWO_PI / 60.0f);	// 机械角速度 (rad/s)

		// 目标位置斜率限制（0 = 直接跟随上位机目标）
		if (config::kMitPosSlewRadS > 0.0f)
		{
			const float step = config::kMitPosSlewRadS * dt;
			const float err = mit_pos_des - mit_pos_slew;

			if (err > step) mit_pos_slew += step;
			else if (err < -step) mit_pos_slew -= step;
			else mit_pos_slew = mit_pos_des;
		}
		else
		{
			mit_pos_slew = mit_pos_des;
		}

		// Kp/Kd 软启动：进模式瞬间不要从 0 直接跳到目标刚度
		if (mit_gain_scale < 1.0f)
		{
			const float ramp = config::kMitGainRampMs * 0.001f;

			mit_gain_scale += (ramp > 0.0f) ? (dt / ramp) : 1.0f;
			if (mit_gain_scale > 1.0f) mit_gain_scale = 1.0f;
		}

		float perr = mit_pos_slew - p;
		perr = foc_math::Limit_Abs(perr, config::kMitPosErrMax);

		const float kp = mit_kp * mit_gain_scale;
		const float kd = mit_kd * mit_gain_scale;

		float tau = kp * perr + kd * (mit_vel_des - v) + mit_tau_ff;

		// 力矩限幅：上位机给的值、电流上限、温度降流三者取最小
		const float kt = controller.Get_Kt();
		float tau_lim = mit_tau_limit;

		if (kt > 0.0f)
		{
			const float tau_hw = kt * config::kIqLimit * derate_ratio;

			if (tau_hw < tau_lim) tau_lim = tau_hw;
		}

		tau = foc_math::Limit_Abs(tau, tau_lim);

		// 任何 NaN 都不许进电流环
		if (!(tau == tau)) tau = 0.0f;

		mit_tau_cmd = tau;
		controller.Set_Torque_Target(tau * config::kTorqueSign);
	}

	/*------------------------------------------------------------------------------
	 * TIM2 1kHz 中断（和旧 bench 一样在中断里更新，避免任务抖动）
	 *------------------------------------------------------------------------------*/
	void ControlLoop::Tim_It_Process()
	{
		if (mode == MODE_OPEN_LOOP || mode == MODE_SCOPE)
		{
			const foc_math::Abc duty = open_loop.Step(0.001f);
			pwm.Set_Duty(duty.a, duty.b, duty.c);		// 波形观察时 MOE 是关的，只写比较值
		}
	}

	/*------------------------------------------------------------------------------
	 * 扫频解调的一步：累加 -> 推进相位 -> 满一个频点就出结果并切到下一个频点
	 *
	 *   feedback  电流环填 iq (A)，转速环填转速 (rpm)
	 *   dt        电流环 50µs（20kHz），转速环 250µs（4kHz）
	 *   is_speed  true = 转速环扫频
	 *------------------------------------------------------------------------------*/
	void ControlLoop::Sweep_Advance(float feedback, float sc_sin, float sc_cos, float dt, bool is_speed)
	{
		if (sweep_cycles >= config::kSweepSettleCycles)
		{
			sweep_i_acc += feedback * sc_sin;
			sweep_q_acc += feedback * sc_cos;
			sweep_acc_n++;
		}

		sweep_phase += foc_math::TWO_PI * sweep_freq * dt;

		if (sweep_phase >= foc_math::TWO_PI)
		{
			sweep_phase -= foc_math::TWO_PI;
			sweep_cycles++;
		}

		const uint16_t meas_cycles = is_speed ? speed_sweep_meas_cycles : sweep_meas_cycles;

		if (sweep_cycles < (uint16_t)(config::kSweepSettleCycles + meas_cycles)) return;

		const float n = (float)sweep_acc_n;
		const float amp_ref = fabsf(test_amp);

		if (n > 4.0f && amp_ref > 1.0e-6f)
		{
			const float mag = 2.0f * sqrtf(sweep_i_acc * sweep_i_acc + sweep_q_acc * sweep_q_acc) / n;

			sweep_res_mag_db = 20.0f * log10f(mag / amp_ref + 1.0e-9f);
			sweep_res_phase_deg = atan2f(sweep_q_acc, sweep_i_acc) * (180.0f / foc_math::PI);
		}
		else
		{
			sweep_res_mag_db = -120.0f;
			sweep_res_phase_deg = 0.0f;
		}

		sweep_res_freq = sweep_freq;
		sweep_res_valid = true;

		sweep_point++;

		if (sweep_point >= config::kSweepPoints)
		{
			if (is_speed)
			{
				speed_sweep_active = false;
				controller.Set_Speed_Target(test_center_speed);
			}
			else
			{
				sweep_isr_active = false;
				controller.Set_Target(0.0f, 0.0f);
			}
			return;
		}

		const float f0 = is_speed ? config::kSweepSpeedFStart : config::kSweepCurrentFStart;
		const float f1 = is_speed ? config::kSweepSpeedFEnd : config::kSweepCurrentFEnd;
		const float t = (float)sweep_point / (float)(config::kSweepPoints - 1);

		sweep_freq = f0 * powf(f1 / f0, t);

		float mc = config::kSweepMinMeasureS * sweep_freq;
		if (mc < (float)config::kSweepSettleCycles) mc = (float)config::kSweepSettleCycles;

		if (is_speed) speed_sweep_meas_cycles = (uint16_t)(mc + 0.999f);
		else sweep_meas_cycles = (uint16_t)(mc + 0.999f);

		sweep_phase = 0.0f;
		sweep_cycles = 0;
		sweep_i_acc = 0.0f;
		sweep_q_acc = 0.0f;
		sweep_acc_n = 0;
	}

	/*------------------------------------------------------------------------------
	 * 中位数（n <= kLSineRepeat）
	 *------------------------------------------------------------------------------*/
	float ControlLoop::Median_Of(float *a, uint8_t n)
	{
		if (n == 0) return 0.0f;

		for (uint8_t i = 1; i < n; i++)
		{
			const float v = a[i];
			int8_t j = (int8_t)(i - 1);

			while (j >= 0 && a[j] > v)
			{
				a[j + 1] = a[j];
				j--;
			}

			a[j + 1] = v;
		}

		if (n & 1u) return a[n / 2];

		return 0.5f * (a[n / 2 - 1] + a[n / 2]);
	}

	/*------------------------------------------------------------------------------
	 * Ld/Lq 正弦阻抗辨识：一个控制周期的一步
	 *
	 *   给 d（或 q）轴注入 V = kLSineVolt·sin(ωt)，电流稳态后用同相位
	 *   sin/cos 做 I/Q 解调，取基波幅值：
	 *       I_amp = 2·sqrt(I²+Q²)/N，Z = V/I，L = sqrt(Z²-R²)/ω
	 *------------------------------------------------------------------------------*/
	void ControlLoop::L_Sine_Advance(float id, float iq, float sc_sin, float sc_cos)
	{
		if (l_sine_cycles >= config::kLSineSettleCycles)
		{
			const float fb = (l_sine_axis == 0) ? id : iq;

			l_sine_i += fb * sc_sin;
			l_sine_q += fb * sc_cos;
			l_sine_n++;
		}

		l_sine_phase += foc_math::TWO_PI * l_sine_f * dt;

		if (l_sine_phase >= foc_math::TWO_PI)
		{
			l_sine_phase -= foc_math::TWO_PI;
			l_sine_cycles++;
		}

		if (l_sine_cycles < (uint16_t)(config::kLSineSettleCycles + config::kLSineMeasCycles)) return;

		// 本次频点的阻抗
		const float n = (float)l_sine_n;
		float z = 0.0f;

		if (n > 10.0f)
		{
			const float i_amp = 2.0f * sqrtf(l_sine_i * l_sine_i + l_sine_q * l_sine_q) / n;

			if (i_amp > 0.01f) z = config::kLSineVolt / i_amp;
		}

		// 复位累加器，准备下一个频点/下一次
		l_sine_i = 0.0f;
		l_sine_q = 0.0f;
		l_sine_n = 0;
		l_sine_phase = 0.0f;
		l_sine_cycles = 0;

		// ---- 低频点测完：换到高频点，重复计数不前进 ----
		if (l_freq_idx == 0 && z > 0.0f)
		{
			l_z1 = z;
			l_freq_idx = 1;
			l_sine_f = config::kLSineF2;
			return;
		}

		// ---- 高频点测完：双频解出 L 和 R ----
		if (l_freq_idx == 1)
		{
			const float w1 = foc_math::TWO_PI * config::kLSineF1;
			const float w2 = foc_math::TWO_PI * config::kLSineF2;
			const float denom = w2 * w2 - w1 * w1;
			const float dz = z * z - l_z1 * l_z1;

			float l = 0.0f;
			float r = 0.0f;

			if (denom > 0.0f && dz > 0.0f)
			{
				l = sqrtf(dz / denom);

				// 低频点反推 R：R² = Z1² − (ω1·L)²
				const float wl = w1 * l;
				if (l_z1 * l_z1 > wl * wl) r = sqrtf(l_z1 * l_z1 - wl * wl);
			}

			if (l > 0.0f && l < 0.01f)
			{
				if (l_sine_axis == 0 && l_cnt_d < config::kLSineRepeat) l_samp_d[l_cnt_d++] = l;
				else if (l_sine_axis == 1 && l_cnt_q < config::kLSineRepeat) l_samp_q[l_cnt_q++] = l;

				// 电阻只在 d 轴记录一次（q 轴有转矩，转子可能微动）
				if (l_sine_axis == 0 && r > 0.0f && r < 2.0f && r_cnt < config::kLSineRepeat)
				{
					r_samp[r_cnt++] = r;
				}
			}
		}

		l_freq_idx = 0;
		l_sine_f = config::kLSineF1;

		if (++l_rep < config::kLSineRepeat)
		{
			// 同一轴继续测
		}
		else if (l_sine_axis == 0)
		{
			l_sine_axis = 1;
			l_rep = 0;
		}
		else
		{
			l_ld = Median_Of(l_samp_d, l_cnt_d);
			l_lq = Median_Of(l_samp_q, l_cnt_q);

			if (r_cnt > 0)
			{
				r_ident = Median_Of(r_samp, r_cnt);
				r_ident_temp = motor_temp_latched;
			}

			l_state = 0;
			controller.Set_Voltage_Override(false, 0.0f, 0.0f);
		}
	}

	/*------------------------------------------------------------------------------
	 * 齿槽补偿：建表 / 注入
	 *------------------------------------------------------------------------------*/
	void ControlLoop::Cogging_Start()
	{
		for (uint16_t i = 0; i < config::kCogBins; i++)
		{
			cog_sum[i] = 0.0f;
			cog_cnt[i] = 0;
			cog_table[i] = 0;
		}

		cog_valid = false;
		cog_learning = true;
		cog_angle_acc = 0.0f;
		cog_last_angle = (encoder != nullptr) ? encoder->Get_Mech_Angle_Wrapped_Rad() : 0.0f;

		controller.Set_Speed_Target(config::kCogLearnRpm);
	}

	void ControlLoop::Cogging_Apply()
	{
		float iq = 0.0f;

		if (cog_enabled && cog_valid && encoder != nullptr &&
			(speed_loop || position_loop || mode == MODE_MIT))
		{
			float a = encoder->Get_Mech_Angle_Wrapped_Rad();
			if (a < 0.0f) a += foc_math::TWO_PI;

			int bin = (int)(a * ((float)config::kCogBins / foc_math::TWO_PI));
			if (bin < 0) bin = 0;
			else if (bin >= (int)config::kCogBins) bin = (int)config::kCogBins - 1;

			// 表里存的是"维持匀速所需的电流"，补偿取反；
			// cog_invert 是调试用的反号开关
			iq = (cog_invert ? 1.0f : -1.0f) * (float)cog_table[bin] * 0.001f;
		}

		controller.Set_Iq_Cogging(iq);
	}

	void ControlLoop::Cogging_Learn(float iq_fb)
	{
		if (!cog_learning || encoder == nullptr) return;

		const float a = encoder->Get_Mech_Angle_Wrapped_Rad();

		// 累计转过角度（绝对值），用于判圈数
		float d = a - cog_last_angle;
		if (d > foc_math::PI) d -= foc_math::TWO_PI;
		else if (d < -foc_math::PI) d += foc_math::TWO_PI;

		cog_angle_acc += fabsf(d);
		cog_last_angle = a;

		float an = (a < 0.0f) ? (a + foc_math::TWO_PI) : a;
		int bin = (int)(an * ((float)config::kCogBins / foc_math::TWO_PI));
		if (bin < 0) bin = 0;
		else if (bin >= (int)config::kCogBins) bin = (int)config::kCogBins - 1;

		cog_sum[bin] += iq_fb;
		if (cog_cnt[bin] < 0xFFFFu) cog_cnt[bin]++;

		if (cog_angle_acc >= foc_math::TWO_PI * config::kCogLearnRevs)
		{
			Cogging_Finish();
		}
	}

	void ControlLoop::Cogging_Finish()
	{
		cog_learning = false;

		float sum = 0.0f;
		uint32_t n = 0;

		for (uint16_t i = 0; i < config::kCogBins; i++)
		{
			sum += cog_sum[i];
			n += cog_cnt[i];
		}

		const float mean = (n > 0u) ? (sum / (float)n) : 0.0f;

		for (uint16_t i = 0; i < config::kCogBins; i++)
		{
			float v = (cog_cnt[i] > 0u) ? (cog_sum[i] / (float)cog_cnt[i] - mean) : 0.0f;

			v = foc_math::Limit_Abs(v, config::kCogMaxA);
			cog_table[i] = (int16_t)(v * 1000.0f);
		}

		cog_valid = true;
		controller.Set_Speed_Target(0.0f);
	}

	/*------------------------------------------------------------------------------
	 * 电流采样中断（20kHz）
	 *
	 *   这里是"电流生产者"的唯一消费者：
	 *     读 ADC 注入结果 -> 电流处理 -> 位置更新 -> 控制器 -> 写 PWM
	 *------------------------------------------------------------------------------*/
	void ControlLoop::Adc_InjectedConvCplt_Process()
	{
		// ADC1 注入组：rank1=IC(PA0)，rank2=IB(PA1)，rank3=IA(PA2)
		// 先读 JDR：JEOC 已经把三通道结果锁存好了，越早读，电流值越早
		// 进入控制运算（编码器帧消费有 CRC+PLL，约 2~3µs，放它后面会让
		// 采样→计算的路径白白多出这段时间）。
		foc_current::Raw raw;
		raw.c = (uint16_t)adc.Get_Injected(ADC_INJECTED_RANK_1);
		raw.b = (uint16_t)adc.Get_Injected(ADC_INJECTED_RANK_2);
		raw.a = (uint16_t)adc.Get_Injected(ADC_INJECTED_RANK_3);

		// 编码器帧在 20kHz 电流环中断里直接消费：速度估计最高 16kHz 更新，
		// 比原来放在 4kHz 监控任务里快 4 倍。Monitor 只负责看门狗和快照发布。
		if (frame_src != nullptr && encoder != nullptr)
		{
			encoder::Frame frame;
			if (frame_src->Try_Take_Frame(frame))
			{
				encoder->Consume(frame);
			}
		}

		// 采样和零偏累加永远按注入频率（20kHz）跑，校准/遥测的时间基准不受降频影响
		const bool current_ready = current.Update(raw);
		const bool open = (mode == MODE_OPEN_LOOP || mode == MODE_SCOPE);

		// 零偏校准期间输出零电压；开环模式下占空比由 1kHz 任务写，这里不抢 CCR
		if (!current_ready)
		{
			if (!open) pwm.Set_Duty(0.5f, 0.5f, 0.5f);
			return;
		}

		// 开环/波形观察时，占空比由 1kHz 控制任务写，这里绝不抢 CCR
		if (open)
		{
			return;
		}

		// 控制器没使能：只需要把三相拉到 50%（线电压 0），
		// 不要在 20kHz 中断里再跑 sin/cos、Park、PI。
		// 这段省下来的时间就是留给 FreeRTOS 任务的，否则串口上报会被饿死。
		if (!controller.Get_Enable())
		{
			pwm.Set_Duty(0.5f, 0.5f, 0.5f);
			return;
		}

		// 电流环降频：没轮到的拍直接返回，CCR 保持上一拍的值。
		// CPU 吃紧（比如 Debug/-O0）时把 kCurrentLoopDecim 设成 2，中断负载直接减半，
		// 代价是 PWM 的比较值更新率从 20kHz 变成 10kHz（电流纹波略大）。
		if (current_decim > 1)
		{
			if (++current_tick < current_decim) return;
			current_tick = 0;
		}

		// 转速环 / 位置环按“电流环节拍”分频
		const bool speed_due = (++speed_tick >= speed_decim);
		if (speed_due) speed_tick = 0;

		const bool position_due = (++position_tick >= position_decim);
		if (position_due) position_tick = 0;

		const foc_math::Abc i_abc = current.Get_Current();
		const foc_math::AlphaBeta i_ab = foc_math::Clarke(i_abc);

		// 电压用上一周期的调制度反算（v = mod * Vbus/√3），这是最接近实际输出的信息
		const float v_scale = v_bus * foc_math::ONE_BY_SQRT3;
		const foc_math::AlphaBeta v_ab = {
			last_mod.alpha * v_scale,
			last_mod.beta * v_scale
		};

		position->Update_Input(v_ab, i_ab);
		position->Position_Update(dt);

		if (config::kUseSmo && position != &smo)
		{
			// 旁路观测器也跟着更新（不参与控制）
			smo.Update_Input(v_ab, i_ab);
			smo.Position_Update(dt);
		}

		Foc_Controller::Input input;
		input.i_abc = i_abc;
		// 编码器作为角度源时用"最近一帧 + 电角速度外推"的角度：
		// 监控任务 1kHz 才更新角度，直接用会有最多 1ms 的滞后，
		// 高转速下就是几十度的 dq 失配（id<0 时会变成负阻尼）。
		input.angle = (position == encoder)
						? encoder->Get_Predicted_Angle(DWT->CYCCNT, dt * 1.5f)	// 含影子寄存器延迟
						: position->Get_Angle();
		input.speed_elec = position->Get_Speed();
		input.v_bus = v_bus;
		input.dt = dt;

		// 位置环 800Hz：TD 轨迹规划 + PD + 速度前馈 -> 速度目标（多圈）
		if (position_due && position_loop && !cal_active && encoder != nullptr && position == encoder)
		{
			const float actual = encoder->Get_Mech_Angle_Rad();	// 连续机械角，可多圈

			const bool test_sine = (test_type == 2 && test_freq > 0.0f);

			// ---- 1. 轨迹：正弦测试直接给目标，其余走规划器 ----
			float target = target_position_rad;

			if (test_sine)
			{
				// TD 的加速度上限会削平正弦，跟踪测试必须绕开它。
				// 目标由 test_phase 现算（相位在 4kHz 节拍里推进），
				// 解析微分作为速度前馈：d/dt[amp·sin(ωt)] = amp·ω·cos(ωt)
				const float amp_rad = test_amp * (foc_math::PI / 180.0f);
				const float w = foc_math::TWO_PI * test_freq;
				const foc_math::SinCos sc = foc_math::Sin_Cos(test_phase);

				target = test_center_pos + amp_rad * sc.sin;
				pos_td_x1 = target;
				pos_td_x2 = amp_rad * w * sc.cos;
				// 解析二阶导：d²/dt²[amp·sin(ωt)] = -amp·ω²·sin(ωt)
				pos_td_a = -amp_rad * w * w * sc.sin;
			}
			else if (config::kPositionPlanEnable)
			{
				/*--------------------------------------------------------------
				 * 最速控制：bang-bang + 相位平面开关线
				 *
				 *   开关函数  σ = d − v·|v| / (2·a_max)
				 *   σ > 0 → a = +a_max（还没到刹车点，继续加速）
				 *   σ < 0 → a = −a_max（速度太快，必须减速）
				 *   σ = 0 就是开关线  v|v| = 2·a_max·d
				 *
				 * 加速度永远只有 ±a_max 两态，没有中间过渡；v_max 只是
				 * 长位移时的速度上限（短位移根本碰不到，速度曲线是三角形）。
				 *
				 * 离散实现下参考轨迹会在终点附近轻微越过目标，越过即落位，
				 * 避免开关线两边来回翻。
				 *--------------------------------------------------------------*/
				const float d = target - pos_td_x1;
				const float v = pos_td_x2;
				const float a_max = plan_accel;

				const float sigma = d - v * fabsf(v) / (2.0f * a_max);
				float a_cmd = (sigma > 0.0f) ? a_max : -a_max;

				// 速度上限：只对长位移的匀速段生效
				if ((v >= plan_vmax && a_cmd > 0.0f) ||
					(v <= -plan_vmax && a_cmd < 0.0f))
				{
					a_cmd = 0.0f;
				}

				pos_td_x1 += v * position_dt + 0.5f * a_cmd * position_dt * position_dt;
				pos_td_x2 = v + a_cmd * position_dt;
				pos_td_a = a_cmd;

				if (pos_td_x2 > plan_vmax) pos_td_x2 = plan_vmax;
				else if (pos_td_x2 < -plan_vmax) pos_td_x2 = -plan_vmax;

				// 越过目标 → 落位（开关线在离散化下的终点处理）
				const float d_after = target - pos_td_x1;
				if ((d > 0.0f && d_after <= 0.0f) || (d < 0.0f && d_after >= 0.0f) ||
					fabsf(d) < 1.0e-6f)
				{
					pos_td_x1 = target;
					pos_td_x2 = 0.0f;
					pos_td_a = 0.0f;
				}
			}
			else if (config::kPositionTdEnable)
			{
				const float fh = Fhan(pos_td_x1 - target, pos_td_x2, config::kPositionTdAccel, position_dt);
				pos_td_x1 += position_dt * pos_td_x2;
				pos_td_x2 += position_dt * fh;
				pos_td_a = fh;
			}
			else
			{
				pos_td_x1 = target;
				pos_td_x2 = 0.0f;
				pos_td_a = 0.0f;
			}

			// ---- 2. 误差 + 带回差的小死区 ----
			float pos_err = pos_td_x1 - actual;

			const float db_enter = config::kPositionDeadbandEnterDeg * (foc_math::PI / 180.0f);
			const float db_exit = config::kPositionDeadbandExitDeg * (foc_math::PI / 180.0f);

			// 死区只用于"静态锁位"：正弦跟踪或轨迹还在走的时候不启用，
			// 否则误差被清零会让 P 项突然消失，速度指令出现台阶。
			if (test_sine || (fabsf(pos_td_x2) > 0.5f))
			{
				pos_db_hold = false;
			}
			else if (pos_db_hold)
			{
				if (fabsf(pos_err) > db_exit) pos_db_hold = false;
				else pos_err = 0.0f;
			}
			else if (fabsf(pos_err) < db_enter)
			{
				pos_db_hold = true;
				pos_err = 0.0f;
			}

			// ---- 3. D 项一阶低通（压掉编码器速度量化噪声） ----
			const float mech_speed = encoder->Get_Mech_Rpm() * (foc_math::TWO_PI / 60.0f);
			pos_d_filt += pos_d_alpha * (mech_speed - pos_d_filt);

			// ---- 4. PD + 速度前馈 ----
			// D 作用在"速度误差"上：跟踪上以后 D 为 0，不会吃掉前馈；
			// 只有实际偏离规划速度时才产生阻尼。
			const float vel_err = pos_d_filt - pos_td_x2;

			float speed_ref = position_kp * pos_err
							  - position_kd * vel_err
							  + config::kPositionSpeedFf * pos_td_x2;

			// 同时挡住 NaN 和越界
			if (!(speed_ref == speed_ref)) speed_ref = 0.0f;
			else if (speed_ref > config::kPositionSpeedLimit) speed_ref = config::kPositionSpeedLimit;
			else if (speed_ref < -config::kPositionSpeedLimit) speed_ref = -config::kPositionSpeedLimit;

			controller.Set_Speed_Target(speed_ref * 60.0f / foc_math::TWO_PI);

			// 加速度前馈：iq_ff = J·a / Kt（机械域，单位 A）
			// 加速段直接给转速环预支转矩，减少位置滞后；减速段为负。
			const float iq_ff = config::kPositionAccelFf
								* config::kMotorJ * pos_td_a / config::kMotorKt;
			controller.Set_Iq_Feedforward(iq_ff);
		}

		// ---- 固定频率正弦：在 4kHz 速度环节拍上生成 ----
		// 原来由 1kHz 任务生成，速度环看到的是 250µs 阶梯参考，
		// 频率一高台阶就明显（50Hz 时 20 点/周期），闭环会跟着抖。
		// 相位只在这里推进（1kHz 任务不再动 test_phase），保证唯一写者。
		if (speed_due && test_freq > 0.0f && (test_type == 1 || test_type == 2))
		{
			test_phase += foc_math::TWO_PI * test_freq * speed_dt;

			if (test_phase >= foc_math::TWO_PI) test_phase -= foc_math::TWO_PI;

			if (test_type == 1)
			{
				controller.Set_Speed_Target(test_center_speed + test_amp * foc_math::Sin_Cos(test_phase).sin);
			}
		}

		// ---- 转速环扫频：4kHz 生成激励（1kHz 任务给不出 800Hz 正弦）----
		foc_math::SinCos speed_sweep_sc = {0.0f, 0.0f};
		const bool speed_sweep_now = (speed_due && speed_sweep_active);

		if (speed_sweep_now)
		{
			speed_sweep_sc = foc_math::Sin_Cos(sweep_phase);
			cap_exc_speed = test_center_speed + test_amp * speed_sweep_sc.sin;
			controller.Set_Speed_Target(cap_exc_speed);
		}

		// 转速环 4kHz：读取 Monitor 任务更新的最新编码器速度
		if (speed_due && speed_loop && !cal_active && encoder != nullptr && position == encoder)
		{
			controller.Speed_Loop(encoder->Get_Mech_Rpm(), speed_dt);
		}

		// MIT 阻抗 4kHz：力矩级控制，旁路上面的转速环。
		// mode 是 volatile，命令任务写、ISR 读；只要 mode 还不是 MIT 就不会进。
		if (speed_due && mode == MODE_MIT && !cal_active)
		{
			if (encoder != nullptr && position == encoder && encoder->Is_Valid())
			{
				Mit_Step(speed_dt);
			}
			else
			{
				// 编码器失效不能继续保留上一拍力矩，立即清零
				mit_active = false;
				controller.Set_Torque_Target(0.0f);
			}
		}

		if (speed_sweep_now && encoder != nullptr)
		{
			Sweep_Advance(encoder->Get_Mech_Rpm(), speed_sweep_sc.sin, speed_sweep_sc.cos, speed_dt, true);
		}

		// Ld/Lq 正弦阻抗辨识：d/q 轴注入 1kHz 正弦电压，稳态测电流幅值
		foc_math::SinCos l_sc = {0.0f, 0.0f};
		const bool l_now = (l_state == 1);

		if (l_now)
		{
			// 测试过程中也要盯住转子：q 轴注入会产生转矩，
			// 一旦转起来反电势会污染 Lq 的测量
			const float rpm_now = (encoder != nullptr) ? encoder->Get_Mech_Rpm() : 0.0f;

			if (fabsf(rpm_now) > 30.0f)
			{
				l_state = 0;
				controller.Set_Voltage_Override(false, 0.0f, 0.0f);
			}
			else
			{
				l_sc = foc_math::Sin_Cos(l_sine_phase);

				const float v = config::kLSineVolt * l_sc.sin;

				if (l_sine_axis == 0) controller.Set_Voltage_Override(true, v, 0.0f);
				else controller.Set_Voltage_Override(true, 0.0f, v);
			}
		}

		// ---- 电流环扫频：20kHz 生成激励 ----
		// 1kHz 任务生成不了 1kHz 正弦（Nyquist 只有 500Hz），
		// 所以激励和测量都放在这个 20kHz 中断里。
		foc_math::SinCos sweep_sc = {0.0f, 0.0f};
		const bool sweep_now = sweep_isr_active;

		if (sweep_now)
		{
			sweep_sc = foc_math::Sin_Cos(sweep_phase);
			cap_exc_q = test_amp * sweep_sc.sin;
			controller.Set_Target(0.0f, cap_exc_q);
		}

		// 齿槽补偿：按当前机械角查表，叠加到 iq 指令（在 Step 之前）
		Cogging_Apply();

		const Foc_Controller::Output output = controller.Step(input);

		last_mod = output.mod;
		pwm.Set_Duty(output.duty.a, output.duty.b, output.duty.c);

		// 齿槽学习：按位置把实际 iq 累加进对应的角度 bin
		Cogging_Learn(output.i_dq.q);

		// ---- 板内捕获：按控制节拍记录 4 路信号 ----
		// 电流扫频/普通捕获按 20kHz，转速扫频按 4kHz（和速度环节拍一致）
		if (capture.armed)
		{
			if (test_type == 4)
			{
				if (speed_due)
				{
					const float spd = (encoder != nullptr) ? encoder->Get_Mech_Rpm() : 0.0f;
					capture.Push(cap_exc_speed, spd, controller.Get_Iq_Target(), output.i_dq.q);
				}
			}
			else if (test_type == 3)
			{
				capture.Push(cap_exc_q, output.i_dq.q, output.i_dq.d, output.v_dq.q);
			}
			else
			{
				const float spd = (encoder != nullptr) ? encoder->Get_Mech_Rpm() : 0.0f;
				capture.Push(controller.Get_Iq_Target(), output.i_dq.q, spd, output.i_dq.d);
			}
		}

		// ---- 电流环扫频：与激励同相位的 I/Q 同步解调 ----
		if (sweep_now)
		{
			Sweep_Advance(output.i_dq.q, sweep_sc.sin, sweep_sc.cos, dt, false);
		}

		// L 辨识：稳态 I/Q 解调
		if (l_now)
		{
			L_Sine_Advance(output.i_dq.d, output.i_dq.q, l_sc.sin, l_sc.cos);
		}
	}

	/*------------------------------------------------------------------------------
	 * 命令落地（只在控制任务上下文执行）
	 *------------------------------------------------------------------------------*/
	void ControlLoop::Apply_Command(const Command &cmd_)
	{
		last_cmd_ms = HAL_GetTick();

		// 记录上一条命令的测试类型：固定频率正弦结束时要把速度目标
		// 从"最后一个正弦值"拉回用户设定值，否则电机会停在正弦的残留转速上
		// （20Hz 命令流里 last_speed_target 没变，不会自动重写目标）。
		const uint8_t prev_test_type = test_type;

		// 先落地幅值/中心值：ISR 里的扫频激励会立刻用到 test_amp
		test_amp = cmd_.test_amp;
		test_freq = cmd_.test_freq;
		test_center_speed = cmd_.speed_rpm;
		test_center_pos = cmd_.position_deg * (foc_math::PI / 180.0f);

		// 最速规划的限幅/加速度：上位机每次下发，0 = 保持固件当前值
		if (cmd_.plan_vmax > 0.0f)
		{
			plan_vmax = Clamp_Gain(cmd_.plan_vmax, 0.1f, 300.0f);
		}
		if (cmd_.plan_accel > 0.0f)
		{
			plan_accel = Clamp_Gain(cmd_.plan_accel, 1.0f, 5000.0f);
		}

		if (cmd_.test_type != test_type)
		{
			test_phase = 0.0f;			// 开始/切换时相位归零

			// 停掉上一种测试
			sweep_isr_active = false;
			speed_sweep_active = false;

			if (l_state != 0)
			{
				l_state = 0;
				controller.Set_Voltage_Override(false, 0.0f, 0.0f);
			}

			sweep_point = 0;
			sweep_cycles = 0;
			sweep_freq = 0.0f;

			if (cmd_.test_type == 3 || cmd_.test_type == 4)
			{
				const bool is_speed = (cmd_.test_type == 4);

				sweep_freq = is_speed ? config::kSweepSpeedFStart : config::kSweepCurrentFStart;
				sweep_phase = 0.0f;
				sweep_cycles = 0;
				sweep_i_acc = 0.0f;
				sweep_q_acc = 0.0f;
				sweep_acc_n = 0;
				sweep_res_valid = false;
				sweep_res_freq = 0.0f;
				sweep_res_mag_db = 0.0f;
				sweep_res_phase_deg = 0.0f;

				float mc = config::kSweepMinMeasureS * sweep_freq;
				if (mc < (float)config::kSweepSettleCycles) mc = (float)config::kSweepSettleCycles;

				if (is_speed)
				{
					speed_sweep_meas_cycles = (uint16_t)(mc + 0.999f);
					speed_sweep_active = true;
				}
				else
				{
					sweep_meas_cycles = (uint16_t)(mc + 0.999f);
					sweep_isr_active = true;
				}
			}
			else if (cmd_.test_type == 5)
			{
				// Lq 会被反电势 ωe·ψf 污染，必须静止测量
				const float rpm_now = (encoder != nullptr) ? encoder->Get_Mech_Rpm() : 0.0f;

				if (fabsf(rpm_now) < 15.0f)
				{
					l_ld = 0.0f;
					l_lq = 0.0f;
					l_rep = 0;
					l_cnt_d = 0;
					l_cnt_q = 0;
					r_cnt = 0;
					l_sine_axis = 0;
					l_freq_idx = 0;
					l_z1 = 0.0f;
					l_sine_f = config::kLSineF1;
					l_sine_cycles = 0;
					l_sine_phase = 0.0f;
					l_sine_i = 0.0f;
					l_sine_q = 0.0f;
					l_sine_n = 0;
					l_state = 1;			// 启动 Ld/Lq 双频阻抗辨识
				}
			}
		}

		test_type = cmd_.test_type;

		// 控制器的使能/目标/位置指针会被 20kHz ADC 中断读取，
		// 这里用临界区保证 ISR 看不到"改到一半"的状态（和旧代码一致）。
		taskENTER_CRITICAL();

		if (cmd_.calibrate_angle)
		{
			Start_Angle_Calibration();
			taskEXIT_CRITICAL();
			return;
		}

		// 标定期间忽略普通控制命令，通信任务会继续重发，标定结束后自然恢复
		if (cal_active)
		{
			taskEXIT_CRITICAL();
			return;
		}

		if (cmd_.calibrate)
		{
			// 零偏校准必须满足：功率级关断、电机静止、电流为 0。
			// 这里先主动停掉控制器和 MOE，再开始采样；
			// Start_Calibration 内部还会丢掉前 20ms 的采样，避开残余电流。
			controller.Set_Enable(false);
			controller.Set_Target(0.0f, 0.0f);
			requested_enable = false;
			speed_loop = false;
			position_loop = false;
			last_mod = {0.0f, 0.0f};
			Set_Output_Enable(false);

			current.Start_Calibration();
		}

		// 电流环 PI 在线整定：只改增益、不动积分项，使能状态下也能直接调。
		// 限幅顺便挡住 NaN / 误输入。
		if (cmd_.tune_pi)
		{
			controller.Set_Pi_Dq4(Clamp_Gain(cmd_.kp_d, 0.0f, 10.0f),
								  Clamp_Gain(cmd_.ki_d, 0.0f, 50000.0f),
								  Clamp_Gain(cmd_.kp_q, 0.0f, 10.0f),
								  Clamp_Gain(cmd_.ki_q, 0.0f, 50000.0f));
			pi_tuned = true;
		}
		else if (pi_tuned)
		{
			// 上位机取消在线整定：恢复到上电时按带宽算出来的那组参数
			controller.Set_Pi_Dq4(pi_kp_d0, pi_ki_d0, pi_kp_q0, pi_ki_q0);
			pi_tuned = false;
		}

		// PI 位置式 / 增量式切换（调试开关，只在变化时切一次，
		// 切换内部会把状态对齐，不会产生输出台阶）
		if (cmd_.pi_incremental != pi_incremental)
		{
			pi_incremental = cmd_.pi_incremental;
			controller.Set_Pi_Incremental(pi_incremental);
		}

		// 板内捕获：命令上升沿触发一次（上位机只发一帧）
		if (cmd_.capture && !last_capture_cmd)		{
			// 自适应抽取：按当前扫频频率算，目标覆盖 10 个周期，
			// 最长 5 秒（电流 20kHz → D≤97，转速 4kHz → D≤19）。
			const uint16_t cap_fs = (test_type == 4) ? 4000u : 20000u;

			float f = 0.0f;
			if ((test_type == 3 || test_type == 4) && sweep_freq > 0.0f)
			{
				f = sweep_freq;
			}

			uint16_t d = 1;

			if (f > 0.01f)
			{
				const float target = 10.0f * (float)cap_fs
								   / (f * (float)CaptureBuffer::POINTS);
				if (target > 1.0f) d = (uint16_t)(target + 0.5f);
			}

			const uint16_t dmax = (uint16_t)(5.0f * (float)cap_fs
											/ (float)CaptureBuffer::POINTS);
			if (d > dmax) d = dmax;
			if (d < 1u) d = 1u;

			capture.Arm(d, cap_fs);
		}
		last_capture_cmd = cmd_.capture;

		// Ld/Lq 双频辨识：命令上升沿触发一次（和 test_type 解耦）
		if (cmd_.l_ident && !last_l_ident_cmd)
		{
			const float rpm_now = (encoder != nullptr) ? encoder->Get_Mech_Rpm() : 0.0f;

			if (fabsf(rpm_now) < 15.0f && l_state == 0)
			{
				l_ld = 0.0f;
				l_lq = 0.0f;
				l_rep = 0;
				l_cnt_d = 0;
				l_cnt_q = 0;
				r_cnt = 0;
				l_sine_axis = 0;
				l_freq_idx = 0;
				l_z1 = 0.0f;
				l_sine_f = config::kLSineF1;
				l_sine_cycles = 0;
				l_sine_phase = 0.0f;
				l_sine_i = 0.0f;
				l_sine_q = 0.0f;
				l_sine_n = 0;
				l_state = 1;
			}
		}
		last_l_ident_cmd = cmd_.l_ident;

		// 齿槽补偿：建表上升沿触发一次；使能是电平
		cog_enabled = cmd_.cog_enable;
		cog_invert = cmd_.cog_invert;

		if (cmd_.cog_learn && !last_cog_learn_cmd)
		{
			Cogging_Start();
		}
		last_cog_learn_cmd = cmd_.cog_learn;

		// 学习期间锁住 8rpm，别被 20Hz 命令流覆盖
		if (cog_learning)
		{
			controller.Set_Speed_Target(config::kCogLearnRpm);
		}

		// 转速环 PI 在线整定
		if (cmd_.tune_speed)
		{
			controller.Set_Speed_Pi(Clamp_Gain(cmd_.speed_kp, 0.0f, 10.0f),
									Clamp_Gain(cmd_.speed_ki, 0.0f, 1000.0f),
									config::kSpeedPiLimit);
			speed_tuned = true;
		}
		else if (speed_tuned)
		{
			controller.Set_Speed_Pi(config::kSpeedPiKp, config::kSpeedPiKi, config::kSpeedPiLimit);
			speed_tuned = false;
		}

		// 位置环 PD 在线整定
		if (cmd_.tune_pos)
		{
			position_kp = Clamp_Gain(cmd_.pos_kp, 0.0f, 1000.0f);
			position_kd = Clamp_Gain(cmd_.pos_kd, 0.0f, 1000.0f);
			pos_tuned = true;
		}
		else if (pos_tuned)
		{
			position_kp = config::kPositionKp;
			position_kd = config::kPositionKd;
			pos_tuned = false;
		}

		if (cmd_.reset_angle && config::kEncoderEnable && encoder != nullptr)
		{
			encoder->Request_Reset_Angle();
		}

		const bool open = (cmd_.mode == MODE_OPEN_LOOP || cmd_.mode == MODE_SCOPE);
		const bool mode_changed = (cmd_.mode != mode);

		if (open)
		{
			// 开环强拖 / 波形观察：控制器彻底退出，占空比由 open_loop 接管
			open_loop.Set_Active(true);
			open_loop.Set_Target(cmd_.speed_rpm, cmd_.iq_a);

			controller.Set_Enable(false);
			speed_loop = false;
			position_loop = false;
			mit_active = false;		// 下次切回 MIT 必须重新用当前位置初始化目标
			last_mod = {0.0f, 0.0f};

			// 波形观察强制不使能；强拖跟随上位机
			Set_Output_Enable(cmd_.enable && (cmd_.mode == MODE_OPEN_LOOP));
			requested_enable = cmd_.enable;		// 保持状态一致，切回闭环时不会误判
			io::Led_Set(2, true);

			mode = cmd_.mode;
			taskEXIT_CRITICAL();
			return;
		}

		open_loop.Set_Active(false);
		io::Led_Set(2, false);

		// d 轴指令：任何闭环模式都接受（弱磁、参数辨识、d 轴电流环调试都用得上）
		controller.Set_Id_Target(cmd_.id_a);

		const bool encoder_ready = config::kEncoderEnable && encoder != nullptr && encoder->Is_Valid();
		const bool use_position_loop = (cmd_.mode == MODE_POSITION) && encoder_ready;
		const bool use_speed_loop = ((cmd_.mode == MODE_ENCODER) || use_position_loop) && encoder_ready;
		const bool use_mit = (cmd_.mode == MODE_MIT) && encoder_ready;

		// 编码器有效时，电流环也使用编码器角度。
		// 电流模式如果继续用固定开环角，电流一大转子一动，d/q 坐标系就不跟着转子，
		// 会表现为电流越大越容易振荡/啸叫。
		//
		// 这里必须每帧都重新判断，不能只在 mode_changed 时切：
		// 电角度标定结束时 position 被临时切回 open_loop_position（固定角 0），
		// 而标定结束时 mode 也已经是 MODE_CURRENT，下一条命令的 mode_changed 为 false，
		// 结果标定完继续用电流模式时一直用固定角 0 —— 给 id/iq 都只是把转子吸住不转。
		if (encoder_ready)
		{
			position = encoder;
		}
		else if (position != &open_loop_position)
		{
			position = &open_loop_position;
			open_loop_position.Set_Speed(0.0f);
		}

		last_encoder_valid = encoder_ready;

		if (use_position_loop)
		{
			// 多圈位置：目标不再归一化，直接用绝对机械角（deg → rad）
			target_position_rad = cmd_.position_deg * (foc_math::PI / 180.0f);

			if (mode_changed || !position_loop)
			{
				controller.Reset_Speed_Pi();
				controller.Set_Speed_Target(0.0f);	// 位置环 800Hz 会覆盖

				// 轨迹规划从当前位置启动：否则一进位置模式 z1=0，目标会瞬移
				pos_td_x1 = encoder->Get_Mech_Angle_Rad();
				pos_td_x2 = 0.0f;
				pos_td_a = 0.0f;
				pos_d_filt = encoder->Get_Mech_Rpm() * (foc_math::TWO_PI / 60.0f);
				pos_db_hold = false;
			}

			position_loop = true;
			speed_loop = true;
			mit_active = false;
		}
		else if (use_speed_loop)
		{
			position_loop = false;
			mit_active = false;
			controller.Set_Iq_Feedforward(0.0f);	// 非位置模式不保留加速度前馈

			// 速度正弦刚结束的那一条命令：强制把目标写回 cmd_.speed_rpm。
			// 否则 speed_target 停在最后一个正弦值（最多 ±test_amp），
			// 而 last_speed_target 仍等于 cmd_.speed_rpm，不会触发重写。
			const bool sine_just_ended = (prev_test_type == 1 && cmd_.test_type != 1);

			if (mode_changed || !speed_loop || last_speed_target != cmd_.speed_rpm || sine_just_ended)
			{
				controller.Set_Speed_Target(cmd_.speed_rpm);

				// 只在进入转速环/正弦结束重新同步时清一次积分；
				// 目标值微调不重置积分
				if (mode_changed || !speed_loop || sine_just_ended)
				{
					controller.Reset_Speed_Pi();
				}

				last_speed_target = cmd_.speed_rpm;
			}

			speed_loop = true;
		}
		else if (use_mit)
		{
			// MIT 阻抗：力矩级，关掉转速/位置环，4kHz 的 Mit_Step 直接写 iq。
			position_loop = false;
			speed_loop = false;
			controller.Set_Iq_Feedforward(0.0f);

			// 参数限幅挡住 NaN / 误输入，避免一个错误增益把电机推到硬件限流
			mit_kp = Clamp_Gain(cmd_.mit_kp, 0.0f, config::kMitMaxKp);
			mit_kd = Clamp_Gain(cmd_.mit_kd, 0.0f, config::kMitMaxKd);
			// 速度/前馈遇到 NaN 或越界直接回 0：不能用 Clamp_Gain 回下限，
			// 否则一条坏命令会变成 -300rad/s 或 -1N·m。
			mit_vel_des = (cmd_.mit_vel_des > -config::kMitVelMax
						   && cmd_.mit_vel_des < config::kMitVelMax)
						  ? cmd_.mit_vel_des : 0.0f;
			mit_tau_ff = (cmd_.mit_tau_ff > -config::kMitTauFfMax
						  && cmd_.mit_tau_ff < config::kMitTauFfMax)
						 ? cmd_.mit_tau_ff : 0.0f;

			float tlim = (cmd_.mit_tau_limit > 0.0f) ? cmd_.mit_tau_limit : config::kMitDefaultTauLimit;
			mit_tau_limit = Clamp_Gain(tlim, 0.0f, config::kMitTauLimitMax);

			if (!cmd_.enable)
			{
				mit_active = false;
			}
			else if (mode_changed || !mit_active)
			{
				// 刚进 MIT：目标先取当前位置，避免上位机默认目标 0 让转子猛弹一下。
				// 下一条命令（50ms 后）再跟随上位机给的目标。
				mit_pos_des = encoder->Get_Mech_Angle_Rad();
				mit_pos_slew = mit_pos_des;
				mit_gain_scale = 0.0f;
				mit_active = true;
			}
			else
			{
				float pdes = cmd_.mit_pos_des;

				// 目标角必须是有限值；坏值就保持上一拍目标
				if (!(pdes > -1.0e6f && pdes < 1.0e6f)) pdes = mit_pos_des;

				mit_pos_des = pdes;
			}
		}
		else
		{
			position_loop = false;
			speed_loop = false;
			mit_active = false;
			controller.Set_Iq_Feedforward(0.0f);	// 非位置模式不保留加速度前馈

			// 转速/位置/MIT 模式但编码器无效：不要拿旧的 iq 继续跑，直接给 0
			const float fallback_iq =
				(cmd_.mode == MODE_ENCODER || cmd_.mode == MODE_POSITION
				 || cmd_.mode == MODE_MIT) ? 0.0f : cmd_.iq_a;
			controller.Set_Target(cmd_.id_a, fallback_iq);
		}

		// 从开环切回闭环时也要重新使能控制器
		if (mode_changed || requested_enable != cmd_.enable)
		{
			requested_enable = cmd_.enable;
			controller.Set_Enable(cmd_.enable);

			// 重新使能转速环时清掉上一次停机前的积分
			if (use_speed_loop && cmd_.enable)
			{
				controller.Reset_Speed_Pi();
			}

			// MIT 重新使能：增益软启动从头开始
			if (use_mit && cmd_.enable)
			{
				controller.Reset_Speed_Pi();
				mit_gain_scale = 0.0f;
			}

			// 开环期间没有更新过 last_mod，切回闭环时从零矢量开始
			last_mod = {0.0f, 0.0f};
		}

		// 零偏校准期间强制关输出：此时采到的必须是真的 0 电流。
		// 校准做完 calibrating 会自己变 false，控制自然恢复。
		Set_Output_Enable(cmd_.enable && !current.Is_Calibrating());

		mode = cmd_.mode;

		taskEXIT_CRITICAL();
	}

	/*------------------------------------------------------------------------------
	 * 电角度零位标定
	 *
	 *   1. 用开环角度 0，给 d 轴一个小电流，iq=0
	 *   2. 等转子被吸住并稳定
	 *   3. 读编码器原始电角度，offset = -raw
	 *   4. 写入编码器，之后 angle = raw + offset
	 *------------------------------------------------------------------------------*/
	void ControlLoop::Start_Angle_Calibration()
	{
		const bool encoder_ready = config::kEncoderEnable && encoder != nullptr && encoder->Is_Valid();

		if (!encoder_ready)
		{
			cal_status = 3;			// 失败：没有编码器反馈
			cal_active = false;
			controller.Set_Target(0.0f, 0.0f);
			controller.Set_Enable(false);
			requested_enable = false;
			speed_loop = false;
			position_loop = false;
			mit_active = false;
			open_loop.Set_Active(false);
			position = &open_loop_position;
			open_loop_position.Set_Speed(0.0f);
			Set_Output_Enable(false);
			mode = MODE_CURRENT;
			io::Led_Set(2, false);
			return;
		}

		cal_active = true;
		cal_status = 1;				// 标定中
		cal_ticks = 0;
		cal_point = 0;				// 多点标定：从第 0 个角度开始

		open_loop.Set_Active(false);
		mode = MODE_CURRENT;
		speed_loop = false;
		position_loop = false;
		mit_active = false;

		position = &open_loop_position;
		open_loop_position.Set_Angle(0.0f);
		open_loop_position.Set_Speed(0.0f);

		// 从 0 开始，稍后在 Update_Angle_Calibration 里按斜坡加到位，
		// 避免 0→0.5A 的阶跃把电流环激起振荡（会啸叫、转子抖）
		controller.Set_Target(0.0f, 0.0f);
		controller.Set_Enable(true);
		requested_enable = true;

		last_mod = {0.0f, 0.0f};
		Set_Output_Enable(true);

		// MOE 没打开（例如 Break 被触发）时，转子不会被吸住，直接判失败
		if (!pwm.Output_Is_Enable())
		{
			cal_status = 3;
			cal_active = false;
			controller.Set_Target(0.0f, 0.0f);
			controller.Set_Enable(false);
			requested_enable = false;
			speed_loop = false;
			position_loop = false;
			mit_active = false;
			open_loop.Set_Active(false);
			position = &open_loop_position;
			open_loop_position.Set_Speed(0.0f);
			Set_Output_Enable(false);
			mode = MODE_CURRENT;
			io::Led_Set(2, false);
			return;
		}

		io::Led_Set(2, true);
	}

	void ControlLoop::Update_Angle_Calibration()
	{
		if (!cal_active) return;

		if (config::kEncoderEnable == false || encoder == nullptr || !encoder->Is_Valid())
		{
			taskENTER_CRITICAL();
			Finish_Angle_Calibration(false);
			taskEXIT_CRITICAL();
			return;
		}

		// 电流零偏校准期间 d 轴电流还没建立，等它完成后再开始计时
		if (current.Is_Calibrating())
		{
			cal_ticks = 0;
			return;
		}

		const uint16_t ramp_ms = (config::kAngleCalRampMs == 0) ? 1 : config::kAngleCalRampMs;
		const uint16_t settle_ms = config::kAngleCalSettleMs;

		// 第 0 个点：先做电流斜坡再等稳定；后面的点电流已经建立，只等角度步进后稳定
		const uint16_t wait_ms = (cal_point == 0) ? (uint16_t)(ramp_ms + settle_ms) : settle_ms;

		if (cal_ticks < wait_ms)
		{
			float id_cmd = config::kAngleCalCurrent;

			if (cal_point == 0 && cal_ticks < ramp_ms)
			{
				id_cmd = config::kAngleCalCurrent * (float)cal_ticks / (float)ramp_ms;
			}

			controller.Set_Target(id_cmd, 0.0f);
			cal_ticks++;
			return;
		}

		// 本点已稳定：定子磁场在 theta，转子 d 轴应与之对齐，
		// 所以 offset = theta - 编码器原始电角度
		const float theta = (float)cal_point * (foc_math::TWO_PI / (float)config::kAngleCalPoints);
		const float raw_elec = encoder->Get_Electrical_Angle_Raw();
		cal_off[cal_point] = foc_math::Norm_Angle(theta - raw_elec);

		cal_point++;
		cal_ticks = 0;

		if (cal_point < config::kAngleCalPoints)
		{
			// 定子磁场步进到下一个点，等转子跟过去
			open_loop_position.Set_Angle((float)cal_point * (foc_math::TWO_PI / (float)config::kAngleCalPoints));
			return;
		}

		// 所有点采完：矢量平均（offset 在 ±π 附近时也不会算错）
		float sum_cos = 0.0f;
		float sum_sin = 0.0f;

		for (uint8_t i = 0; i < config::kAngleCalPoints; i++)
		{
			sum_cos += cosf(cal_off[i]);
			sum_sin += sinf(cal_off[i]);
		}

		const float mean = atan2f(sum_sin, sum_cos);

		// 一致性检查：各点应基本相等。离散过大说明有的点没对齐、
		// 编码器偏心/非线性，或者方向/极对数不对，此时不要覆盖旧 offset。
		float spread = 0.0f;

		for (uint8_t i = 0; i < config::kAngleCalPoints; i++)
		{
			const float d = fabsf(foc_math::Norm_Angle(cal_off[i] - mean));
			if (d > spread) spread = d;
		}

		if (spread > (15.0f * (foc_math::PI / 180.0f)))
		{
			taskENTER_CRITICAL();
			Finish_Angle_Calibration(false);
			taskEXIT_CRITICAL();
			return;
		}

		encoder->Set_Electrical_Offset(mean);

		taskENTER_CRITICAL();
		Finish_Angle_Calibration(true);
		taskEXIT_CRITICAL();
	}

	void ControlLoop::Finish_Angle_Calibration(bool ok_)
	{
		controller.Set_Target(0.0f, 0.0f);
		controller.Set_Enable(false);

		requested_enable = false;
		speed_loop = false;
		position_loop = false;
		mit_active = false;
		cal_point = 0;
		open_loop.Set_Active(false);
		position = &open_loop_position;
		open_loop_position.Set_Speed(0.0f);
		last_mod = {0.0f, 0.0f};

		Set_Output_Enable(false);
		io::Led_Set(2, false);

		mode = MODE_CURRENT;
		cal_active = false;
		cal_status = ok_ ? 2 : 3;	// 2=完成 3=失败
	}

	void ControlLoop::Set_Output_Enable(bool on_)
	{
		if (on_ == output_enable) return;

		output_enable = on_;

		if (on_) pwm.Output_Enable();
		else pwm.Output_Disable();
	}

	/*------------------------------------------------------------------------------
	 * 遥测快照（控制任务 1kHz）
	 *------------------------------------------------------------------------------*/
	void ControlLoop::Publish_Snapshot()
	{
		ControlSnapshot snapshot;
		snapshot.valid = true;

		const bool open = (mode == MODE_OPEN_LOOP || mode == MODE_SCOPE);

		foc_math::Abc current_avg;
		foc_math::Abc raw;
		foc_math::Dq i_dq;
		foc_math::Dq v_dq;
		foc_math::Abc duty;
		float speed_rpm;
		float speed_target_rpm = 0.0f;
		float pos_target_deg = 0.0f;
		float test_freq_now = 0.0f;
		float vd_now = 0.0f;
		float vq_now = 0.0f;
		float ld_h = 0.0f;
		float lq_h = 0.0f;
		float angle;
		float iq_target = 0.0f;
		float id_target = 0.0f;
		float pi_kp_d = 0.0f;
		float pi_ki_d = 0.0f;
		float pi_kp_q = 0.0f;
		float pi_ki_q = 0.0f;
		float speed_kp = 0.0f;
		float speed_ki = 0.0f;
		float pos_kp = 0.0f;
		float pos_kd = 0.0f;
		uint16_t offs_a = 0;
		uint16_t offs_b = 0;
		uint16_t offs_c = 0;

		// 控制器/开环角度由 ADC/TIM2 中断更新，遥测读取时加临界区，
		// 避免读到半更新的 duty/电流，导致上位机看到“SVPWM 计算错误”。
		taskENTER_CRITICAL();
		current_avg = current.Take_Average();
		raw = current.Get_Current_Raw();
		i_dq = controller.Get_Current_Dq();
		v_dq = controller.Get_Voltage_Dq();
		pi_kp_d = controller.Get_Kp_D();
		pi_ki_d = controller.Get_Ki_D();
		pi_kp_q = controller.Get_Kp_Q();
		pi_ki_q = controller.Get_Ki_Q();
		iq_target = controller.Get_Iq_Target();
		id_target = controller.Get_Id_Target();
		// MIT 模式没有转速/位置环，遥测里把 MIT 目标映射到这两个字段，
		// 上位机现有的“角度/转速参考线”直接就能看到 MIT 目标。
		if (mode == MODE_MIT)
		{
			speed_target_rpm = mit_vel_des * 60.0f / foc_math::TWO_PI;
			pos_target_deg = mit_pos_des * (180.0f / foc_math::PI);
		}
		else
		{
			speed_target_rpm = controller.Get_Speed_Target();
			pos_target_deg = target_position_rad * (180.0f / foc_math::PI);
		}
		test_freq_now = this->test_freq_now;
		vd_now = controller.Get_Voltage_Dq().d;
		vq_now = controller.Get_Voltage_Dq().q;

		// 扫频时，vd/vq 两个字段借用来回传板内解调结果：
		//   vd = 幅值 (dB)，vq = 相位 (度)
		// 上位机按 test_freq_now 去重收集每个频点。
		if (test_type == 3 || test_type == 4)
		{
			const bool sweep_on = (test_type == 3) ? sweep_isr_active : speed_sweep_active;

			test_freq_now = sweep_on
							? (sweep_res_valid ? sweep_res_freq : sweep_freq)
							: 0.0f;

			if (sweep_res_valid)
			{
				vd_now = sweep_res_mag_db;
				vq_now = sweep_res_phase_deg;
			}
			else
			{
				vd_now = 999.0f;		// 结果未就绪的哨兵值
				vq_now = 999.0f;
			}
		}
		ld_h = l_ld;
		lq_h = l_lq;
		speed_kp = controller.Get_Speed_Kp();
		speed_ki = controller.Get_Speed_Ki();
		pos_kp = position_kp;
		pos_kd = position_kd;

		const foc_math::Abc io = current.Get_Offset();
		offs_a = (uint16_t)(io.a + 0.5f);
		offs_b = (uint16_t)(io.b + 0.5f);
		offs_c = (uint16_t)(io.c + 0.5f);

		if (open)
		{
			duty = open_loop.Get_Duty();
			speed_rpm = open_loop.Get_Speed_Rpm();
			angle = open_loop.Get_Angle();
		}
		else
		{
			duty = controller.Get_Duty();
			speed_rpm = (encoder != nullptr && position == encoder)
							? encoder->Get_Mech_Rpm()
							: controller.Speed_To_Rpm(position->Get_Speed());
			angle = position->Get_Angle();
		}
		taskEXIT_CRITICAL();

		snapshot.current = current_avg;
		snapshot.raw = raw;
		snapshot.i_dq = i_dq;
		snapshot.v_dq = v_dq;
		snapshot.v_bus = v_bus;
		snapshot.duty = duty;
		snapshot.speed_rpm = speed_rpm;
		snapshot.angle = angle;
		snapshot.iq_target = iq_target;
		snapshot.id_target = id_target;
		snapshot.speed_target_rpm = speed_target_rpm;
		snapshot.pos_target_deg = pos_target_deg;
		snapshot.test_freq_now = test_freq_now;
		snapshot.vd = vd_now;
		snapshot.vq = vq_now;
		snapshot.ld_h = ld_h;
		snapshot.lq_h = lq_h;
		snapshot.speed_kp = speed_kp;
		snapshot.speed_ki = speed_ki;
		snapshot.pos_kp = pos_kp;
		snapshot.pos_kd = pos_kd;
		snapshot.tau_cmd = controller.Get_Torque_Target();
		snapshot.mit_pos_des = mit_pos_des;
		snapshot.mit_vel_des = mit_vel_des;
		snapshot.mit_kp = mit_kp;
		snapshot.mit_kd = mit_kd;
		snapshot.mit_tau_ff = mit_tau_ff;
		snapshot.kp_d = pi_kp_d;
		snapshot.ki_d = pi_ki_d;
		snapshot.kp_q = pi_kp_q;
		snapshot.ki_q = pi_ki_q;
		snapshot.offs_a = offs_a;
		snapshot.offs_b = offs_b;
		snapshot.offs_c = offs_c;
		snapshot.motor_temp = motor_temp_latched;
		snapshot.driver_temp = driver_temp_latched;
		snapshot.thermal_state = thermal_state;
		snapshot.psi_est_mwb = psi_est * 1000.0f;
		snapshot.psi_state = psi_state;
		snapshot.cog_state = (cog_valid ? 1u : 0u) | (cog_learning ? 2u : 0u) | (cog_enabled ? 4u : 0u);

		if (open)
		{
			// 强拖时电流环被跳过，用当前角度现算一份 dq 给上位机看
			const foc_math::AlphaBeta i_ab = foc_math::Clarke(snapshot.current);
			const foc_math::SinCos sc = foc_math::Sin_Cos(snapshot.angle);
			snapshot.i_dq = foc_math::Park(i_ab, sc);
		}

		// 和 DWT 计数器一起放在临界区里取，避免和 ADC 中断的 prof_isr_end 打架，
		// 否则偶尔会读到一个巨大值（UI 上显示成几百 us）
		taskENTER_CRITICAL();
		prof_take(&snapshot.isr_avg_x10, &snapshot.isr_max_x10);
		taskEXIT_CRITICAL();

		snapshot.output_enable = pwm.Output_Is_Enable();
		snapshot.open_loop = open;
		snapshot.speed_loop = speed_loop;
		snapshot.calibrating = current.Is_Calibrating();
		snapshot.mode = mode;
		snapshot.cal_status = cal_status;
		snapshot.cmd_ok = ((uint32_t)(HAL_GetTick() - last_cmd_ms) < 200u);

		snapshot_out.Publish(snapshot);
	}
}
