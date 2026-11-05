#include "FOC_openloop.h"

namespace openloop
{
	Drive::Drive(uint8_t pole_pairs_, float accel_rad_s2_)
		: pole_pairs(pole_pairs_)
		, accel(accel_rad_s2_)
	{
	}

	void Drive::Set_Active(bool on_)
	{
		if (on_ == active) return;

		active = on_;
		angle = 0.0f;
		speed = 0.0f;
	}

	void Drive::Set_Target(float rpm_, float iq_)
	{
		rpm_cmd = rpm_;

		float a = (iq_ < 0.0f) ? -iq_ : iq_;
		a *= OL_AMP_PER_AMP;
		if (a > 0.95f) a = 0.95f;
		amp = a;
	}

	float Drive::Get_Speed_Rpm() const
	{
		return speed * 60.0f / (foc_math::TWO_PI * (float)pole_pairs);
	}

	foc_math::Abc Drive::Step(float dt)
	{
		if (!active || dt <= 0.0f) return duty;

		// 目标电角速度：rpm -> rpm/60 * 极对数 * 2π
		speed_cmd = foc_math::TWO_PI * (rpm_cmd / 60.0f) * (float)pole_pairs;

		// 电角速度斜坡，避免一步跳到目标导致过流/失步
		const float step = accel * dt;
		if (speed < speed_cmd)
			speed = (speed + step < speed_cmd) ? (speed + step) : speed_cmd;
		else
			speed = (speed - step > speed_cmd) ? (speed - step) : speed_cmd;

		angle += speed * dt;
		angle = foc_math::Norm_Angle(angle);

		const foc_math::SinCos sc = foc_math::Sin_Cos(angle);

		foc_math::AlphaBeta m;
		m.alpha = amp * sc.cos;
		m.beta = amp * sc.sin;

		duty = foc_math::Svpwm(m, 1.0f);

		return duty;
	}
}
