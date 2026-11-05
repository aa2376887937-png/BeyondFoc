#include "MT6701.h"

namespace mt6701
{
	namespace
	{
		// 帧可信度：相邻两帧隐含速度（原始计数/秒）允许的相对/绝对偏差。
		// 真实加速度在 1ms 内只会让速度变化很小；坏帧会瞬间差出几万计数/秒。
		constexpr float kImpliedRelTol = 0.5f;			// 允许 50%（全扭矩加速 1ms 约 30%）
		constexpr float kImpliedNoiseFloor = 20000.0f;	// 20 计数/ms
		constexpr float kImpliedMinSpeed = 5000.0f;		// 低于 ~66rpm 不做判断，避免起动误判
	}

	Sensor::Sensor()
	{
		Reset_Internal();
		angle = 0.0f;
		speed = 0.0f;
		Set_Pll_Bandwidth(80.0f, 0.707f);
	}

	void Sensor::Set_Clock(uint32_t cycles_per_second_)
	{
		if (cycles_per_second_ == 0u) return;

		clock_hz = cycles_per_second_;
	}

	void Sensor::Set_Direction(float direction_)
	{
		direction = (direction_ < 0.0f) ? -1.0f : 1.0f;
	}

	/*------------------------------------------------------------------------------
	 * 速度二阶巴特沃斯低通
	 *
	 * 监控任务现在按 ~4kHz（250µs）消费帧，所以按 fs = 4kHz 设计。
	 * 二阶比一阶滚降更陡：同样 50Hz 截止时，20Hz 处相位滞后更小、
	 * 几百 Hz 的量化噪声压得更狠。
	 *------------------------------------------------------------------------------*/
	void Sensor::Set_Speed_Filter_Hz(float fc_hz_)
	{
		if (fc_hz_ <= 0.0f) return;

		constexpr float kFs = 4000.0f;
		const float k = tanf(foc_math::PI * fc_hz_ / kFs);
		const float k2 = k * k;
		const float norm = 1.0f / (1.0f + 1.41421356f * k + k2);

		sp_b0 = k2 * norm;
		sp_b1 = 2.0f * sp_b0;
		sp_b2 = sp_b0;
		sp_a1 = 2.0f * (k2 - 1.0f) * norm;
		sp_a2 = (1.0f - 1.41421356f * k + k2) * norm;
	}

	/*------------------------------------------------------------------------------
	 * 速度 PLL：二阶环路（PI + 积分）
	 *
	 *   e      = 角度误差
	 *   omega  = Kp·e + ∫Ki·e
	 *   theta += omega·dt
	 *
	 * 连续域整定：ωn = 2π·f，Kp = 2ζωn，Ki = ωn²。
	 * 输出 omega 是机械 rad/s，代替原来的"差分 + 低通"。
	 *------------------------------------------------------------------------------*/
	void Sensor::Set_Pll_Bandwidth(float hz_, float damping_)
	{
		if (hz_ <= 0.0f) return;

		if (damping_ <= 0.0f) damping_ = 0.707f;

		const float wn = foc_math::TWO_PI * hz_;
		pll_kp = 2.0f * damping_ * wn;
		pll_ki = wn * wn;
	}

	void Sensor::Set_Electrical_Offset(float offset_rad_)
	{
		elec_offset = foc_math::Norm_Angle(offset_rad_);

		if (started)
		{
			angle = foc_math::Norm_Angle(elec_angle_raw + elec_offset);
		}
	}

	float Sensor::Get_Electrical_Offset_Deg() const
	{
		return elec_offset * (180.0f / foc_math::PI);
	}

	void Sensor::Reset_Internal()
	{
		Reset_Angle_State();

		raw_angle = 0;
		last_word = 0;
		error_cnt = 0;
		crc_err_cnt = 0;
		valid = false;
		elec_offset = 0.0f;
	}

	void Sensor::Reset_Angle_State()
	{
		// 只清角度基准，不动 CRC/通信错误统计
		started = false;
		mech_deg = 0.0f;
		last_angle = 0;
		last_cycles = 0;
		elec_angle_raw = 0.0f;
		angle = 0.0f;
		speed = 0.0f;
		reject_cnt = 0;
		last_implied = 0.0f;
		has_implied = false;
		sp_x1 = sp_x2 = 0.0f;
		sp_y1 = sp_y2 = 0.0f;
		pll_theta = 0.0f;
		pll_omega = 0.0f;
		pll_integral = 0.0f;
	}

	/*------------------------------------------------------------------------------
	 * MT6701 的 CRC6
	 *   多项式 0x43 (x^6 + x + 1)，初值 0，覆盖 18bit（D[23:6]），MSB 先算
	 *------------------------------------------------------------------------------*/
	uint8_t Sensor::Crc6_Calc(uint32_t data_)
	{
		uint8_t crc = 0;

		for (int i = 17; i >= 0; i--)
		{
			const uint8_t bit = (uint8_t)((data_ >> i) & 1u);

			crc <<= 1;
			if ((crc >> 6) ^ bit) crc ^= ENCODER_CRC_POLY;
			crc &= 0x3F;
		}

		return crc;
	}

	bool Sensor::Consume(const Frame &frame_)
	{
		if (reset_requested)
		{
			reset_requested = false;
			Reset_Angle_State();
		}

		const uint32_t word = frame_.word;

		// 全 0 / 全 1：MISO 没接上或没输出（0 帧的 CRC 恰好也是 0，会误判通过）
		if (word == 0x000000u || word == 0xFFFFFFu)
		{
			if (crc_err_cnt < 0xFFFF) crc_err_cnt++;
			if (++error_cnt > 20) valid = false;
			return false;
		}

		// CRC6 校验：帧尾 6bit 是 CRC，数据是 D[23:6]
		if (Crc6_Calc(word >> 6) != (uint8_t)(word & 0x3F))
		{
			if (crc_err_cnt < 0xFFFF) crc_err_cnt++;
			if (++error_cnt > 20) valid = false;
			return false;
		}

		Process_Frame(frame_);
		return true;
	}

	/*------------------------------------------------------------------------------
	 * 消费一帧
	 *------------------------------------------------------------------------------*/
	void Sensor::Process_Frame(const Frame &frame_)
	{
		const uint32_t word = frame_.word;

		error_cnt = 0;
		last_word = word;
		raw_angle = (uint16_t)((word >> 10) & 0x3FFF);

		if (!started)
		{
			// 第一帧只取基准：否则角度和速度都会有一次大跳变
			started = true;
			last_angle = raw_angle;
			mech_deg = 0.0f;
			last_cycles = frame_.cycles;
			elec_angle_raw = foc_math::Norm_Angle(direction * ((float)raw_angle * (foc_math::TWO_PI / ENCODER_CPR)) * (float)pole_pairs);
			angle_seq = angle_seq + 1u;			// 奇数：角度正在更新
			angle = foc_math::Norm_Angle(elec_angle_raw + elec_offset);
			speed = 0.0f;

			// PLL 也从当前角度起步
			pll_theta = foc_math::Norm_Angle(direction * ((float)raw_angle * (foc_math::TWO_PI / ENCODER_CPR)));
			pll_omega = 0.0f;
			pll_integral = 0.0f;

			angle_cycles = frame_.cycles;
			angle_seq = angle_seq + 1u;			// 偶数：更新完成
			valid = true;
			return;
		}

		/*----------------------------------------------------------------------
		 * 增量 + 跨零展开
		 *
		 * 丢帧太多时真实增量可能超过半圈，此时 ±半圈 展开会选错方向，所以再加
		 * 一道"单步合理性"：一次超过 90° 就只重取基准、不累加。
		 *----------------------------------------------------------------------*/
		int32_t delta = (int32_t)raw_angle - last_angle;
		if (delta > (int32_t)(ENCODER_CPR / 2.0f)) delta -= (int32_t)ENCODER_CPR;
		else if (delta < -(int32_t)(ENCODER_CPR / 2.0f)) delta += (int32_t)ENCODER_CPR;

		const int32_t max_step = (int32_t)(ENCODER_CPR / 4.0f);		// 90°

		// 用相邻帧的真实时间戳算 dt（无符号减法天然安全）
		float dt_s = (float)(frame_.cycles - last_cycles) / (float)clock_hz;

		if (dt_s <= 0.0f || dt_s > 0.1f) dt_s = 0.001f;

		/*----------------------------------------------------------------------
		 * 帧可信度检查
		 *
		 * MT6701 只有 6bit CRC，单比特错误有 1/64 概率"合法通过"。这种帧的
		 * 角度是随机的，直接拿去 Park 就是一次"突然换相"——电机顿一下、卡一下，
		 * 下一帧又恢复正常。表现就是转着转着抽一下。
		 *
		 * 用"本帧隐含速度"和"上一帧隐含速度"比较，而不是和滤波后的速度比：
		 * 滤波有 3ms 滞后，加速时会被误判；相邻两帧的速度则几乎不变。
		 * 连续 3 帧都偏离才认为是速度估计丢了，直接重新同步，避免误锁死。
		 *----------------------------------------------------------------------*/
		const float implied = (float)delta / dt_s;		// 原始计数/秒

		if (has_implied && fabsf(last_implied) > kImpliedMinSpeed)
		{
			const float allowed = kImpliedRelTol * fabsf(last_implied) + kImpliedNoiseFloor;

			if (fabsf(implied - last_implied) > allowed)
			{
				if (++reject_cnt >= 3)
				{
					// 连续偏离：更像速度估计丢了，直接接受并重新同步
					reject_cnt = 0;
					last_implied = implied;
				}
				else
				{
					// 帧校验通过但数值不可信：单独计数，不混进 CRC 错误，
					// 这样上位机看到的“坏帧”只反映真正的 SPI/CRC 问题。
					if (plaus_err_cnt < 0xFFFF) plaus_err_cnt++;

					// 用上一拍的速度把角度外推到这一帧，保持连续；
					// 不更新 last_angle / last_cycles，下一帧的 delta 会覆盖这段。
					angle_seq = angle_seq + 1u;
					angle = foc_math::Norm_Angle(angle + speed * dt_s);
					angle_cycles = frame_.cycles;
					angle_seq = angle_seq + 1u;
					return;
				}
			}
			else
			{
				reject_cnt = 0;
			}
		}

		last_implied = implied;
		has_implied = true;

		last_cycles = frame_.cycles;

		if (delta > max_step || delta < -max_step)
		{
			last_angle = raw_angle;		// 只重取基准
			angle_cycles = frame_.cycles;	// 别让外推继续用旧时间戳
			return;
		}

		last_angle = raw_angle;

		// 连续角度：按增量累加，跨零也连续
		mech_deg += direction * (float)delta * (360.0f / ENCODER_CPR);

		const float mech_angle = (float)raw_angle * (foc_math::TWO_PI / ENCODER_CPR);

		elec_angle_raw = foc_math::Norm_Angle(direction * mech_angle * (float)pole_pairs);

		// 角度 / 速度 / 时间戳 成组更新，中断里的外推用序号锁读，
		// 保证不会读到"新角度 + 旧时间戳"这种组合。
		angle_seq = angle_seq + 1u;
		angle = foc_math::Norm_Angle(elec_angle_raw + elec_offset);

		// ---- 速度锁相环（二阶）：用机械角闭环估计速度 ----
		// 相位误差 -> PI 环路滤波 -> 积分得到估计角。
		// pll_omega 是机械 rad/s，乘极对数后作为电角速度给上层。
		{
			const float theta_enc = foc_math::Norm_Angle(direction * mech_angle);
			const float e = foc_math::Norm_Angle(theta_enc - pll_theta);

			pll_integral += pll_ki * e * dt_s;

			// 积分限幅，防止起动/大误差时速度估计冲飞（±200rad/s ≈ ±1900rpm）
			if (pll_integral > 200.0f) pll_integral = 200.0f;
			else if (pll_integral < -200.0f) pll_integral = -200.0f;

			pll_omega = pll_kp * e + pll_integral;
			pll_theta = foc_math::Norm_Angle(pll_theta + pll_omega * dt_s);

			speed = pll_omega * (float)pole_pairs;
		}

		angle_cycles = frame_.cycles;
		angle_seq = angle_seq + 1u;

		valid = true;
	}

	/*------------------------------------------------------------------------------
	 * 电流环中断里用：把最近一帧的角度按电角速度外推到"现在"
	 *
	 * 监控任务 1kHz 才消费一帧，中断读到的角度最多会滞后 1ms；转速一高，
	 * 这个滞后在转子坐标系里就是几十度的 q 轴分量 —— id>0 时表现为阻尼，
	 * id<0 时就是负阻尼（拨一下自己越转越快）。
	 *
	 * 这里只做一次乘加，不解析帧、不碰 SPI/CRC。dt 用帧自带的 DWT 时间戳算，
	 * 所以连"监控任务消费之前的那段延迟"也一起补上了。
	 *------------------------------------------------------------------------------*/
	float Sensor::Get_Predicted_Angle(uint32_t now_cycles) const
	{
		for (int i = 0; i < 2; i++)
		{
			const uint32_t s0 = angle_seq;
			if ((s0 & 1u) != 0u) continue;		// 正在更新，重试一次

			const float a = angle;
			const float w = speed;
			const uint32_t c = angle_cycles;

			if (angle_seq == s0)
			{
				float dt = (float)(uint32_t)(now_cycles - c) / (float)clock_hz;

				// 监控任务被卡住时不要外推太远
				if (dt > 0.002f) dt = 0.002f;

				// 不要在中断里调 Norm_Angle（内部是 fmodf，Debug 下几微秒）。
				// dt 已经限到 2ms，最多偏两圈，两次加减就够；
				// 剩下的越界由 Sin_Cos 内部处理。
				float p = a + w * dt;

				if (p >= foc_math::PI)
				{
					p -= foc_math::TWO_PI;
					if (p >= foc_math::PI) p -= foc_math::TWO_PI;
				}
				else if (p < -foc_math::PI)
				{
					p += foc_math::TWO_PI;
					if (p < -foc_math::PI) p += foc_math::TWO_PI;
				}

				return p;
			}
		}

		return angle;		// 两遍都没读到一致值：退回未外推的角度
	}

	void Sensor::Position_Update(float dt)
	{
		(void)dt;

		// 编码器角度由监控任务在 Consume() 里更新，这里只保留接口。
		// 清零请求也统一在 Consume() 里处理：单消费者，避免和 ISR 竞争 flag。
	}

	float Sensor::Get_Mech_Rpm() const
	{
		return Get_Speed() * 60.0f / (foc_math::TWO_PI * (float)pole_pairs);
	}
}
