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

		// 和 Runtime 构造函数里的初始化保持一致
		controller.Set_Speed_Pi(config::kSpeedPiKp, config::kSpeedPiKi, config::kSpeedPiLimit);
		controller.Set_Speed_Integral_Limit(config::kSpeedPiIntegralLimit);
		controller.Set_Speed_Integral_Separation(config::kSpeedPiIntegralSeparation);
		controller.Set_Voltage_Limit(0.95f, 1.0f);
		controller.Set_Decoupling(Foc_Controller::DECOUPLING_CROSS_BEMF);
		controller.Set_Pwm_Freq(pwm.Get_Freq());
		controller.Set_Dead_Time_Comp(config::kDeadTimeUs, config::kDeadTimeCompEnable);

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

		// ADC1 注入组不在这里启动：由 TIM1 更新中断软件启动
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

		// 2. 消费监控任务的母线电压快照
		PowerSnapshot power;
		if (power_in.Try_Read(power) && power.valid)
		{
			v_bus = power.v_bus;
		}

		// 3. 电角度标定中：只等转子稳定，不跑转速环
		if (cal_active)
		{
			Update_Angle_Calibration();
			Publish_Snapshot();
			return;
		}

		// 4. 测试信号（1kHz 生成）
		if ((test_type == 3 || test_type == 4) && sweep_point < config::kSweepPoints)
		{
			// ---- 扫频：对数步进，每个频点停留 kSweepCyclesPerPoint 个周期 ----
			test_phase += foc_math::TWO_PI * sweep_freq * dt_task;

			if (test_phase >= foc_math::TWO_PI)
			{
				test_phase -= foc_math::TWO_PI;

				if (++sweep_cycles >= config::kSweepCyclesPerPoint)
				{
					sweep_cycles = 0;
					sweep_point++;

					if (sweep_point < config::kSweepPoints)
					{
						const float f0 = (test_type == 3) ? config::kSweepCurrentFStart : config::kSweepSpeedFStart;
						const float f1 = (test_type == 3) ? config::kSweepCurrentFEnd : config::kSweepSpeedFEnd;
						const float t = (float)sweep_point / (float)(config::kSweepPoints - 1);
						sweep_freq = f0 * powf(f1 / f0, t);
					}
				}
			}

			const float s = foc_math::Sin_Cos(test_phase).sin;

			if (test_type == 3)
			{
				controller.Set_Target(0.0f, test_amp * s);			// 电流环：iq 注入
			}
			else
			{
				controller.Set_Speed_Target(test_center_speed + test_amp * s);
			}

			test_freq_now = sweep_freq;
		}
		else if ((test_type == 3 || test_type == 4) && sweep_point >= config::kSweepPoints)
		{
			// 扫频完成：撤掉激励，但保持 test_type 不变，
			// 等上位机看到 test_freq_now=0 后主动下发 0，避免重复启动。
			if (test_type == 3) controller.Set_Target(0.0f, 0.0f);
			else controller.Set_Speed_Target(test_center_speed);

			test_freq_now = 0.0f;
		}
		else if ((test_type == 1 || test_type == 2) && test_freq > 0.0f)
		{
			// ---- 固定频率正弦 ----
			test_phase += foc_math::TWO_PI * test_freq * dt_task;

			if (test_phase >= foc_math::TWO_PI) test_phase -= foc_math::TWO_PI;

			const float s = foc_math::Sin_Cos(test_phase).sin;

			if (test_type == 1)
			{
				controller.Set_Speed_Target(test_center_speed + test_amp * s);
			}
			else if (test_type == 2)
			{
				target_position_rad = test_center_pos + test_amp * (foc_math::PI / 180.0f) * s;
			}

			test_freq_now = 0.0f;
		}
		else
		{
			test_freq_now = 0.0f;
		}

		// 5. 转速环在 ADC 中断里执行，这里只发布遥测快照
		Publish_Snapshot();
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
	 * 电流采样中断（20kHz）
	 *
	 *   这里是"电流生产者"的唯一消费者：
	 *     读 ADC 注入结果 -> 电流处理 -> 位置更新 -> 控制器 -> 写 PWM
	 *------------------------------------------------------------------------------*/
	void ControlLoop::Adc_InjectedConvCplt_Process()
	{
		// ADC1 注入组：rank1=IC(PA0)，rank2=IB(PA1)，rank3=IA(PA2)
		foc_current::Raw raw;
		raw.c = (uint16_t)adc.Get_Injected(ADC_INJECTED_RANK_1);
		raw.b = (uint16_t)adc.Get_Injected(ADC_INJECTED_RANK_2);
		raw.a = (uint16_t)adc.Get_Injected(ADC_INJECTED_RANK_3);

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
						? encoder->Get_Predicted_Angle(DWT->CYCCNT)
						: position->Get_Angle();
		input.speed_elec = position->Get_Speed();
		input.v_bus = v_bus;
		input.dt = dt;

		// 位置环 800Hz：TD 轨迹规划 + PD + 速度前馈 -> 速度目标（多圈）
		if (position_due && position_loop && !cal_active && encoder != nullptr && position == encoder)
		{
			const float actual = encoder->Get_Mech_Angle_Rad();	// 连续机械角，可多圈

			const bool test_sine = (test_type == 2 && test_freq > 0.0f);

			// ---- 1. 轨迹：正弦测试直接用目标 + 解析微分，其它情况用 TD ----
			float target = target_position_rad;

			if (test_sine)
			{
				// TD 的加速度上限会削平正弦，跟踪测试必须绕开它。
				// 解析微分作为速度前馈：d/dt[amp·sin(ωt)] = amp·ω·cos(ωt)
				pos_td_x1 = target;
				pos_td_x2 = test_amp * (foc_math::PI / 180.0f)
							* (foc_math::TWO_PI * test_freq)
							* foc_math::Sin_Cos(test_phase).cos;
			}
			else if (config::kPositionTdEnable)
			{
				const float fh = Fhan(pos_td_x1 - target, pos_td_x2, config::kPositionTdAccel, position_dt);
				pos_td_x1 += position_dt * pos_td_x2;
				pos_td_x2 += position_dt * fh;
			}
			else
			{
				pos_td_x1 = target;
				pos_td_x2 = 0.0f;
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
		}

		// 转速环 4kHz：读取 Monitor 任务更新的最新编码器速度
		if (speed_due && speed_loop && !cal_active && encoder != nullptr && position == encoder)
		{
			controller.Speed_Loop(encoder->Get_Mech_Rpm(), speed_dt);
		}

		// Ld/Lq 阶跃辨识：直接给 dq 电压，绕过电流环 PI
		if (l_state == 1)
		{
			controller.Set_Voltage_Override(true, config::kLStepVoltD, 0.0f);
		}
		else if (l_state == 3)
		{
			controller.Set_Voltage_Override(true, 0.0f, config::kLStepVoltQ);
		}
		else if (l_state != 0)
		{
			controller.Set_Voltage_Override(true, 0.0f, 0.0f);
		}

		const Foc_Controller::Output output = controller.Step(input);

		last_mod = output.mod;
		pwm.Set_Duty(output.duty.a, output.duty.b, output.duty.c);

		// L 辨识状态机：电压施加一拍后测电流变化
		if (l_state != 0)
		{
			switch (l_state)
			{
			case 1:		// d 轴阶跃
				if (l_tick == 0)
				{
					l_i0 = output.i_dq.d;
					l_tick = 1;
				}
				else
				{
					const float di = fabsf(output.i_dq.d - l_i0);
					const float l = (di > 0.10f) ? (config::kLStepVoltD * dt / di) : 0.0f;
					l_ld = (l > 0.0f && l < 0.01f) ? l : 0.0f;		// 只接受 0~10mH
					l_state = 2;
					l_tick = 0;
				}
				break;

			case 2:		// 冷却，等电流衰减
				if (++l_tick > 600)		// 30ms
				{
					l_state = 3;
					l_tick = 0;
				}
				break;

			case 3:		// q 轴阶跃
				if (l_tick == 0)
				{
					l_i0 = output.i_dq.q;
					l_tick = 1;
				}
				else
				{
					const float di = fabsf(output.i_dq.q - l_i0);
					const float l = (di > 0.10f) ? (config::kLStepVoltQ * dt / di) : 0.0f;
					l_lq = (l > 0.0f && l < 0.01f) ? l : 0.0f;
					l_state = 4;
					l_tick = 0;
				}
				break;

			case 4:		// 结束，恢复 PI
				controller.Set_Voltage_Override(false, 0.0f, 0.0f);
				l_state = 0;
				break;
			}
		}
	}

	/*------------------------------------------------------------------------------
	 * 命令落地（只在控制任务上下文执行）
	 *------------------------------------------------------------------------------*/
	void ControlLoop::Apply_Command(const Command &cmd_)
	{
		last_cmd_ms = HAL_GetTick();

		// 测试信号参数：中心值直接取命令里的转速/位置目标
		if (cmd_.test_type != test_type)
		{
			test_phase = 0.0f;			// 开始/切换时相位归零

			// 停掉上一种测试
			if (l_state != 0)
			{
				l_state = 0;
				controller.Set_Voltage_Override(false, 0.0f, 0.0f);
			}

			sweep_point = 0;
			sweep_cycles = 0;
			sweep_freq = 0.0f;

			if (cmd_.test_type == 3)
			{
				sweep_freq = config::kSweepCurrentFStart;
			}
			else if (cmd_.test_type == 4)
			{
				sweep_freq = config::kSweepSpeedFStart;
			}
			else if (cmd_.test_type == 5)
			{
				l_state = 1;			// 启动 Ld/Lq 阶跃辨识
				l_tick = 0;
				l_i0 = 0.0f;
			}
		}

		test_type = cmd_.test_type;
		test_amp = cmd_.test_amp;
		test_freq = cmd_.test_freq;
		test_center_speed = cmd_.speed_rpm;
		test_center_pos = cmd_.position_deg * (foc_math::PI / 180.0f);

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
				pos_d_filt = encoder->Get_Mech_Rpm() * (foc_math::TWO_PI / 60.0f);
				pos_db_hold = false;
			}

			position_loop = true;
			speed_loop = true;
		}
		else if (use_speed_loop)
		{
			position_loop = false;

			if (mode_changed || !speed_loop || last_speed_target != cmd_.speed_rpm)
			{
				controller.Set_Speed_Target(cmd_.speed_rpm);

				// 只在进入转速环时清一次积分；目标值微调不重置积分
				if (mode_changed || !speed_loop)
				{
					controller.Reset_Speed_Pi();
				}

				last_speed_target = cmd_.speed_rpm;
			}

			speed_loop = true;
		}
		else
		{
			position_loop = false;

			// 转速/位置模式但编码器无效：不要拿旧的 iq 继续跑，直接给 0
			const float fallback_iq =
				(cmd_.mode == MODE_ENCODER || cmd_.mode == MODE_POSITION) ? 0.0f : cmd_.iq_a;
			controller.Set_Target(cmd_.id_a, fallback_iq);
			speed_loop = false;
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
		speed_target_rpm = controller.Get_Speed_Target();
		pos_target_deg = target_position_rad * (180.0f / foc_math::PI);
		test_freq_now = this->test_freq_now;
		vd_now = controller.Get_Voltage_Dq().d;
		vq_now = controller.Get_Voltage_Dq().q;
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
		snapshot.kp_d = pi_kp_d;
		snapshot.ki_d = pi_ki_d;
		snapshot.kp_q = pi_kp_q;
		snapshot.ki_q = pi_ki_q;
		snapshot.offs_a = offs_a;
		snapshot.offs_b = offs_b;
		snapshot.offs_c = offs_c;

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
