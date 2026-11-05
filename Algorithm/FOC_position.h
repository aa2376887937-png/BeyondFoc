#pragma once

#include "FOC_math.h"

/*==============================================================================
 * 位置（电角度）来源 —— 纯算法层
 *
 *   本文件不依赖任何 HAL / 外设 / FreeRTOS。
 *   编码器、霍尔、无感观测器都从 position_sensor 派生，
 *   控制环只认这个接口，换位置来源不用改控制代码。
 *============================================================================*/

#define SMO_SPEED_MAX	30000.0f		// 滑模观测器转速估计限幅 (rad/s 电角速度)

class position_sensor
{
public:
	position_sensor() : angle(0.0f), speed(0.0f) {}
	virtual ~position_sensor() {}

	// 每个控制周期调用，更新电角度与电角速度。
	// 编码器这类"数据已经在别处算好"的来源可以给空实现。
	virtual void Position_Update(float dt) = 0;

	// 观测器类的位置来源需要电压和电流（单位：V / A，αβ 坐标系）；
	// 编码器、霍尔、开环用不到，基类给空实现，控制环统一调用不用区分类型
	virtual void Update_Input(const foc_math::AlphaBeta &v_ab, const foc_math::AlphaBeta &i_ab)
	{
		(void)v_ab;
		(void)i_ab;
	}

	float Get_Angle() const { return angle; }		// 电角度 (rad)，范围 [-π, π)
	float Get_Speed() const { return speed; }		// 电角速度 (rad/s)

protected:
	volatile float angle;
	volatile float speed;
};


/*------------------------------------------------------------------------------
 * 开环角度发生器
 *
 * 给定电角速度，每个控制周期积分出角度。没有任何位置反馈，
 * 用于第一次上电验证（PWM、采样、相序对不对）以及 I/f 启动拖动。
 *------------------------------------------------------------------------------*/
class open_loop_sensor : public position_sensor
{
public:
	open_loop_sensor();
	virtual ~open_loop_sensor() {}

	void Set_Speed(float speed_);					// 电角速度 (rad/s)
	void Set_Angle(float angle_);					// 直接给定电角度 (rad)
	void Position_Update(float dt) override;
};


/*------------------------------------------------------------------------------
 * 滑模观测器（SMO）
 *
 * 核心思想：用电机电流方程搭一个电流观测器，把"观测电流 - 实测电流"送进
 * 滑模控制律 z = k*sat(误差)，误差被压到 0 并保持住（进入滑模面）之后，
 * z 的等效值恰好就是反电势；再对 z 低通滤掉开关纹波得到 ê，
 * 最后用锁相环(PLL)从反电势里解出电角度和电角速度。
 *
 *   αβ 电流方程：  L*diα/dt = vα - R*iα - eα
 *   观测器：        L*diα/dt = vα - R*iα - zα      z = k*sat(î - i)
 *   误差方程：      L*d(î-i)/dt = -R(î-i) - z + e
 *
 * 取 V = L/2*(î-i)²，只要 k > |e|_max 就有 V̇ < 0（李亚普诺夫到达条件），
 * 误差有限时间内收敛到 0；此时 z 的等效值 z_eq = e，即反电势。
 *------------------------------------------------------------------------------*/
class smo_sensor : public position_sensor
{
public:
	smo_sensor();
	virtual ~smo_sensor() {}

	// ---- 参数配置 ----
	void Set_Motor_Params(float r_, float l_, float flux_);		// 相电阻 / 相电感 / 磁链
	void Set_Gain_Auto(float w_elec_max);						// 按最高电角速度自动整定
	void Set_Sliding_Gain(float k_);							// 手动：开关项幅值 k (V)
	void Set_Boundary_Layer(float phi_);						// 手动：边界层 φ (A)，0 = 纯 sign
	void Set_Emf_Filter(float factor_);							// 反电势低通系数 (0~1)
	void Set_Pll_Gain(float kp_, float ki_);					// 锁相环 PI 参数
	void Set_Emf_Min(float emf_min_);							// 判定估计是否可信的反电势阈值
	void Set_Angle_Compensate(bool enable_);					// 低通相位滞后补偿
	void Reset();												// 清状态（启动/切换时用）

	// ---- 运行 ----
	void Update_Input(const foc_math::AlphaBeta &v_ab, const foc_math::AlphaBeta &i_ab) override;
	void Position_Update(float dt) override;

	// ---- 调试观察 ----
	bool Is_Valid() const { return valid; }
	foc_math::AlphaBeta Get_Emf() const { return emf; }
	foc_math::AlphaBeta Get_Current_Hat() const { return i_hat; }
	float Get_Sliding_Gain() const { return k; }
	float Get_Equivalent_Gain() const { return (phi > 0.0f) ? (k / phi) : 0.0f; }

private:
	float Sat(float error);
	void Auto_Tune(float dt);

	// 电机参数
	float r = 0.0f;
	float l = 0.0f;
	float flux = 0.0f;

	// 观测器参数
	float k = 1.0f;
	float phi = 0.0f;
	float emf_filter = 0.2f;
	float emf_min = 0.5f;
	float pll_kp = 0.0f;
	float pll_ki = 0.0f;
	float pll_integral = 0.0f;
	float pll_angle = 0.0f;
	bool angle_comp = true;
	volatile bool valid = false;

	// 自动整定
	bool auto_tune = false;
	float auto_w_max = 0.0f;
	bool tuned = false;

	// 状态
	foc_math::AlphaBeta v_in = {0.0f, 0.0f};
	foc_math::AlphaBeta i_in = {0.0f, 0.0f};
	foc_math::AlphaBeta i_hat = {0.0f, 0.0f};
	foc_math::AlphaBeta emf = {0.0f, 0.0f};
};
