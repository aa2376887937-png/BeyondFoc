#pragma once

#include "FOC_math.h"
#include "FOC_pi.h"

/*==============================================================================
 * FOC 控制器 —— 纯算法层
 *
 *   一个控制周期的流程：
 *     电流采样 -> Clarke -> Park -> PI -> 解耦前馈 -> 圆限幅
 *              -> 逆 Park -> 归一化 -> 死区补偿 -> SVPWM
 *
 *   本类不知道"电流从哪里来、PWM 往哪里写、位置来自编码器还是观测器"。
 *   输入是一个普通结构体，输出是三相占空比。
 *   这样 App 可以自由组合电流源 / 位置源 / 输出设备，算法本身不反向依赖硬件。
 *============================================================================*/

class Foc_Controller
{
public:
	// 解耦补偿模式
	typedef enum Decoupling
	{
		DECOUPLING_DISABLED = 0,
		DECOUPLING_CROSS,
		DECOUPLING_BEMF,
		DECOUPLING_CROSS_BEMF
	} Decoupling;

	struct Input
	{
		foc_math::Abc i_abc;		// 三相电流 (A)
		float angle;				// 电角度 (rad)
		float speed_elec;			// 电角速度 (rad/s)
		float v_bus;				// 母线电压 (V)
		float dt;					// 本周期时间 (s)
	};

	struct Output
	{
		foc_math::Abc duty;			// 三相占空比 0~1（已归一化）
		foc_math::Dq i_dq;			// dq 电流 (A)
		foc_math::Dq v_dq;			// 最终 dq 电压 (V)，含解耦前馈
		foc_math::Dq v_pi_dq;		// PI 净输出 (V)
		foc_math::AlphaBeta mod;	// αβ 调制度
		foc_math::SinCos sc;		// 本周期 sin/cos
		bool active;				// false = 未使能/参数无效，输出零矢量
	};

	Foc_Controller();

	// ---------------- 使能与指令 ----------------
	void Set_Enable(bool enable_);
	bool Get_Enable() const { return enable; }

	void Set_Target(float id_target_, float iq_target_);	// 电流指令 (A)，自动关掉转速环
	void Set_Id_Target(float id_target_);					// 只改 d 轴指令（弱磁/辨识用，不动转速环）
	void Set_Speed_Target(float rpm_target_);				// 转速指令 (rpm)，打开转速环
	void Set_Speed_Target_Accel(float rpm_per_s);			// 目标斜坡 (rpm/s)，0 = 关闭斜坡
	void Reset_Speed_Pi();									// 进入转速环时清一次积分
	float Get_Id_Target() const { return id_target; }
	float Get_Iq_Target() const { return iq_target; }
	float Get_Speed_Target() const { return speed_target; }
	bool Get_Speed_Loop_Enable() const { return speed_loop_enable; }

	// ---------------- 参数配置 ----------------
	void Set_Pi(float kp_, float ki_);
	void Set_Pi_Dq(float kp_d_, float kp_q_, float ki_);
	void Set_Pi_Dq4(float kp_d_, float ki_d_, float kp_q_, float ki_q_);	// 在线整定：d/q 各自独立 Kp/Ki
	void Set_Pi_By_Bandwidth(float w_bandwidth, float r, float l);
	void Set_Pi_Dq_By_Bandwidth(float w_bandwidth, float r, float ld, float lq);
	void Set_Speed_Pi(float kp_, float ki_, float limit_);
	void Set_Speed_Integral_Limit(float limit_);			// 转速环积分项限幅 (A)
	void Set_Speed_Integral_Separation(float error_band_);	// 转速误差大于该值时暂停积分 (rpm)
	void Set_Motor_Params(float r_, float ld_, float lq_, float flux_);
	void Set_Pole_Pairs(uint8_t pole_pairs_);
	void Set_Voltage_Limit(float max_duty_, float overmod_factor_);
	void Set_Decoupling(Decoupling type_);
	void Set_Voltage_Override(bool enable_, float vd_, float vq_);	// 参数辨识：直接给 dq 电压
	void Set_Dead_Time_Comp(float dead_time_us_, bool enable_);
	void Set_Current_Filter(float factor_);		// 解耦前馈用的滤波系数（慢一点没关系）
	void Set_Pwm_Freq(float freq_);

	// ---------------- 运行 ----------------
	Output Step(const Input &input_);						// 电流环，电流采样中断里调用
	void Speed_Loop(float rpm_feedback, float dt);			// 转速环，1kHz 任务里调用

	// ---------------- 调试观察 ----------------
	foc_math::Dq Get_Current_Dq() const { return i_dq; }
	foc_math::Dq Get_Voltage_Dq() const { return {vd, vq}; }
	foc_math::Dq Get_Pi_Voltage_Dq() const { return {vd_pi, vq_pi}; }
	foc_math::AlphaBeta Get_Mod() const { return mod; }
	foc_math::Abc Get_Duty() const { return duty; }
	foc_math::SinCos Get_SinCos() const { return sc; }
	float Get_Speed_Elec() const { return speed_elec; }		// 上周期电角速度 (rad/s)
	float Speed_To_Rpm(float speed_elec_) const;			// 电角速度 -> 机械转速 (rpm)

	// 当前实际生效的电流环参数（给上位机回显/在线整定用）
	float Get_Kp_D() const { return pi_d.Get_Kp(); }
	float Get_Ki_D() const { return pi_d.Get_Ki(); }
	float Get_Kp_Q() const { return pi_q.Get_Kp(); }
	float Get_Ki_Q() const { return pi_q.Get_Ki(); }
	float Get_Speed_Kp() const { return pi_speed.Get_Kp(); }
	float Get_Speed_Ki() const { return pi_speed.Get_Ki(); }

private:
	void Limit_Voltage(float v_bus_);
	void Dead_Time_Compensate(float &mod_alpha_, float &mod_beta_);
	void Reset_State();

	Pi_Controller pi_d;
	Pi_Controller pi_q;
	Pi_Controller pi_speed;

	// 指令
	volatile bool enable = false;
	volatile float id_target = 0.0f;
	volatile float iq_target = 0.0f;
	volatile float speed_target = 0.0f;
	volatile float speed_target_ramped = 0.0f;		// 斜坡后的目标
	float speed_target_accel = 0.0f;				// rpm/s，0 = 不限制
	bool speed_ramp_ready = false;
	volatile bool speed_loop_enable = false;

	// 状态
	foc_math::Dq i_dq = {0.0f, 0.0f};
	foc_math::SinCos sc = {0.0f, 1.0f};
	foc_math::AlphaBeta mod = {0.0f, 0.0f};
	foc_math::Abc duty = {0.5f, 0.5f, 0.5f};
	float vd = 0.0f;
	float vq = 0.0f;
	float vd_pi = 0.0f;
	float vq_pi = 0.0f;
	float id_filter = 0.0f;
	float iq_filter = 0.0f;
	float speed_elec = 0.0f;

	// 参数
	float kp = 0.0f;
	float ki = 0.0f;
	float r = 0.0f;
	float ld = 0.0f;
	float lq = 0.0f;
	float flux = 0.0f;
	uint8_t pole_pairs = 7;
	float max_duty = 0.95f;
	float overmod_factor = 1.0f;
	float current_filter = 0.1f;
	float dt_speed = 0.001f;
	float pwm_freq = 20000.0f;
	float dead_time_us = 0.5f;
	bool dead_time_enable = true;
	Decoupling decoupling = DECOUPLING_DISABLED;
	bool voltage_override = false;
	float ov_vd = 0.0f;
	float ov_vq = 0.0f;
};
