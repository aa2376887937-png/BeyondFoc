#pragma once

#include "FOC_math.h"

/*==============================================================================
 * 开环强拖（电压开环 V/f）—— 纯算法层
 *
 *   每个控制周期：
 *       angle += ω·dt
 *       mod    = A·(cosθ, sinθ)
 *       duty   = Svpwm(mod, 1)
 *
 *   不需要电流反馈、不需要编码器、不需要母线电压，专门验证
 *   SVPWM / 相序 / 驱动链路。本类不碰定时器、PWM、LED，也不反控控制器。
 *============================================================================*/

#define OL_ELEC_ACCEL		(6.2831853f * 100.0f)	// 电角速度斜率 rad/s²
#define OL_AMP_PER_AMP		0.2f					// 每安培对应的调制度

namespace openloop
{
	class Drive
	{
	public:
		Drive(uint8_t pole_pairs_, float accel_rad_s2_ = OL_ELEC_ACCEL);
		virtual ~Drive() {}

		void Set_Active(bool on_);
		void Set_Target(float rpm_, float iq_);

		// 1kHz 调用：推进角度并返回三相占空比 0~1
		foc_math::Abc Step(float dt);

		bool Is_Active() const { return active; }
		foc_math::Abc Get_Duty() const { return duty; }
		float Get_Angle() const { return angle; }			// 电角度 (rad)
		float Get_Speed_Rpm() const;						// 机械转速
		float Get_Amplitude() const { return amp; }

	private:
		uint8_t pole_pairs = 7;
		float accel = OL_ELEC_ACCEL;

		volatile bool active = false;
		volatile float rpm_cmd = 0.0f;
		volatile float iq_cmd = 0.0f;

		float speed_cmd = 0.0f;		// 目标电角速度
		float speed = 0.0f;			// 当前电角速度（斜坡后）
		float angle = 0.0f;			// 电角度
		float amp = 0.0f;			// 调制度
		foc_math::Abc duty = {0.5f, 0.5f, 0.5f};
	};
}
