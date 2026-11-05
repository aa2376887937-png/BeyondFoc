#include "FOC_current.h"

namespace foc_current
{
	namespace
	{
		// 三数中值
		float Med3(float a, float b, float c)
		{
			if (a > b) { const float t = a; a = b; b = t; }
			if (b > c) { const float t = b; b = c; c = t; }
			if (a > b) { const float t = a; a = b; b = t; }
			return b;
		}
	}

	Processor::Processor()
	{
		Set_Scale(CURRENT_SHUNT_R, CURRENT_AMP_GAIN);
		Set_Offset(CURRENT_OFFSET_ADC, CURRENT_OFFSET_ADC, CURRENT_OFFSET_ADC);
		Set_Sign(1.0f, 1.0f, 1.0f);
	}

	/*------------------------------------------------------------------------------
	 * 采样系数：ADC 码值 -> 安培
	 *
	 *     I = (ADC - 零点) / 满量程 * Vref / (R * 增益)
	 *------------------------------------------------------------------------------*/
	void Processor::Set_Scale(float r_shunt_, float amp_gain_)
	{
		if (r_shunt_ <= 0.0f || amp_gain_ <= 0.0f) return;

		scale = CURRENT_ADC_VREF / (CURRENT_ADC_FULL * r_shunt_ * amp_gain_);
	}

	void Processor::Set_Offset(float offset_a_, float offset_b_, float offset_c_)
	{
		offset.a = offset_a_;
		offset.b = offset_b_;
		offset.c = offset_c_;
	}

	void Processor::Set_Sign(float sign_a_, float sign_b_, float sign_c_)
	{
		sign.a = sign_a_;
		sign.b = sign_b_;
		sign.c = sign_c_;
	}

	void Processor::Start_Calibration(uint16_t samples_, uint16_t settle_)
	{
		if (samples_ == 0) samples_ = 1;

		cal_sum_a = 0.0f;
		cal_sum_b = 0.0f;
		cal_sum_c = 0.0f;
		cal_count = 0;
		cal_target = samples_;
		cal_settle = settle_;
		calibrating = true;

		hist_idx = 0;
		hist_n = 0;
	}

	bool Processor::Update(const Raw &raw_)
	{
		raw.c = (float)raw_.c;
		raw.b = (float)raw_.b;
		raw.a = (float)raw_.a;

		// 零偏校准期间只累加、不输出电流（此时功率级是关的，测到的就是零点）
		if (calibrating)
		{
			// 先等电流衰减/开关尖峰过去，再开始累加，
			// 否则刚关输出那一下的残余电流会把零点带偏
			if (cal_settle > 0)
			{
				cal_settle--;
				current.a = 0.0f;
				current.b = 0.0f;
				current.c = 0.0f;
				return false;
			}

			cal_sum_a += raw.a;
			cal_sum_b += raw.b;
			cal_sum_c += raw.c;
			cal_count++;

			if (cal_count >= cal_target)
			{
				offset.a = cal_sum_a / (float)cal_count;
				offset.b = cal_sum_b / (float)cal_count;
				offset.c = cal_sum_c / (float)cal_count;
				calibrating = false;

				// 校准期间求的遥测平均没有意义，一起清掉，避免上位机看到一次跳变
				sum_a = 0.0f;
				sum_b = 0.0f;
				sum_c = 0.0f;
				sum_n = 0;
			}

			return false;
		}

		// 换算成安培
		current.a = (raw.a - offset.a) * scale * sign.a;
		current.b = (raw.b - offset.b) * scale * sign.b;
		current.c = (raw.c - offset.c) * scale * sign.c;

		// 3 点中值：杀掉单点 ADC 毛刺（PI 吃的是中值，不是瞬时值）
		hist[0][hist_idx] = current.a;
		hist[1][hist_idx] = current.b;
		hist[2][hist_idx] = current.c;

		hist_idx = (uint8_t)((hist_idx + 1u) % 3u);
		if (hist_n < 3u) hist_n++;

		if (hist_n >= 3u)
		{
			current.a = Med3(hist[0][0], hist[0][1], hist[0][2]);
			current.b = Med3(hist[1][0], hist[1][1], hist[1][2]);
			current.c = Med3(hist[2][0], hist[2][1], hist[2][2]);
		}

		// 遥测平均：累加本周期瞬时值，1kHz 里取平均（抗混叠）
		sum_a += current.a;
		sum_b += current.b;
		sum_c += current.c;
		sum_n++;

		return true;
	}

	/*------------------------------------------------------------------------------
	 * 兼容旧接口：ADC1 采 IC、ADC2 采 IB，IA 用 KVL 补出来
	 *------------------------------------------------------------------------------*/
	bool Processor::Update_Two_Phase(uint16_t raw_b_, uint16_t raw_c_)
	{
		raw.b = (float)raw_b_;
		raw.c = (float)raw_c_;

		if (calibrating)
		{
			cal_sum_b += raw.b;
			cal_sum_c += raw.c;
			cal_count++;

			if (cal_count >= cal_target)
			{
				offset.b = cal_sum_b / (float)cal_count;
				offset.c = cal_sum_c / (float)cal_count;
				calibrating = false;
			}

			return false;
		}

		current.b = (raw.b - offset.b) * scale * sign.b;
		current.c = (raw.c - offset.c) * scale * sign.c;
		current.a = -(current.b + current.c);

		// 仅用于遥测显示：把 A 相电流折算回等效 ADC 码值
		raw.a = offset.a - ((raw.b - offset.b) * sign.b + (raw.c - offset.c) * sign.c);

		sum_a += current.a;
		sum_b += current.b;
		sum_c += current.c;
		sum_n++;

		return true;
	}

	/*------------------------------------------------------------------------------
	 * 取上个 1ms 窗口的平均电流（只给上位机看，控制环仍用瞬时值）
	 *------------------------------------------------------------------------------*/
	foc_math::Abc Processor::Take_Average()
	{
		foc_math::Abc out = current;

		if (sum_n > 0)
		{
			const float inv = 1.0f / (float)sum_n;
			out.a = sum_a * inv;
			out.b = sum_b * inv;
			out.c = sum_c * inv;
		}

		sum_a = 0.0f;
		sum_b = 0.0f;
		sum_c = 0.0f;
		sum_n = 0;

		return out;
	}
}
