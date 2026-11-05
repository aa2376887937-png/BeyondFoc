#pragma once 
#ifdef __cplusplus
#include "math.h"
#include <stdint.h>

/*==============================================================================
 * FOC 基础数学
 * 只包含纯数学运算，不依赖任何外设，可以脱离单片机在主控电脑上单独验证
 *============================================================================*/

class foc_math
{
public:
	// ---------------- 预计算常数（把除法变成乘法） ----------------
	static constexpr float SQRT3         = 1.7320508075688772f;	// √3
	static constexpr float ONE_BY_SQRT3 = 0.5773502691896258f;	// 1/√3
	static constexpr float TWO_BY_SQRT3 = 1.1547005383792515f;	// 2/√3
	static constexpr float SQRT3_BY_2   = 0.8660254037844386f;	// √3/2
	static constexpr float PI           = 3.1415926535897932f;
	static constexpr float TWO_PI       = 6.2831853071795865f;

	// ---------------- 数据类型 ----------------
	struct Abc       { float a, b, c; };		// 三相静止坐标系（幅值不变的量）
	struct AlphaBeta { float alpha, beta; };	// 两相静止坐标系 αβ
	struct Dq        { float d, q; };			// 两相旋转坐标系 dq
	struct SinCos    { float sin, cos; };		// 角度对应的 sin 与 cos

	// ---------------- 坐标变换 ----------------
	// 完整 Clarke：三相都采样时使用，精度最高
	static AlphaBeta Clarke(const Abc &i);

	// 简化 Clarke：利用 ia + ib + ic = 0，只采样两相时使用（计算量更小）
	static AlphaBeta Clarke(float ia, float ib);

	// Park：静止 αβ -> 旋转 dq（需要转子电角度）
	static Dq Park(const AlphaBeta &i, const SinCos &sc);

	// 逆 Park：旋转 dq -> 静止 αβ（输出 SVPWM 前的最后一步）
	static AlphaBeta Inv_Park(const Dq &v, const SinCos &sc);

	// 逆 Clarke：αβ -> 三相（幅值不变，结果三相之和为 0）
	static Abc Inv_Clarke(const AlphaBeta &v);

	// ---------------- 角度处理 ----------------
	static SinCos Sin_Cos(float angle);			// 求 sin/cos（angle 单位：弧度）
	static void Init_Lut();						// 上电时调一次：建 sin/cos 查找表
	static float Norm_Angle(float angle);		// 电角度归一化到 [-π, π)

	// ---------------- SVPWM ----------------
	// mod 为归一化调制度：|mod| ≤ 1 就是线性调制区（1 对应母线电压 /√3）
	// 返回三相占空比对应的定时器比较值（0 ~ period），中心对齐模式
	static Abc Svpwm(const AlphaBeta &mod, float period);

	// 扇区判断（1~6），只用于调试观察，实际占空比计算不需要它
	static uint8_t Svpwm_Sector(const AlphaBeta &mod);

	// ---------------- 限幅与滤波 ----------------
	static float Limit(float value, float min, float max);			// 上下限幅
	static float Limit_Abs(float value, float limit);				// 绝对值限幅
	static float Lp_Filter(float last, float target, float factor);	// 一阶低通滤波

private:

protected:

};

/*==============================================================================
 * 高频调用的小函数：放在头文件里强制内联
 *
 * 电流环在 20kHz/10kHz 中断里跑，Debug(-O0) 构建下"跨文件调用 + 结构体按值
 * 返回的拷贝"占了不少时间。这些函数都只有几条算术指令，内联到调用点可以
 * 省掉 call/return 和参数搬运；Release(-Os) 下编译器本来也会自己内联。
 *============================================================================*/
#define FOC_ALWAYS_INLINE inline __attribute__((always_inline))

FOC_ALWAYS_INLINE foc_math::AlphaBeta foc_math::Clarke(const Abc &i)
{
	AlphaBeta out;
	out.alpha = (2.0f / 3.0f) * i.a - (1.0f / 3.0f) * i.b - (1.0f / 3.0f) * i.c;
	out.beta  = ONE_BY_SQRT3 * (i.b - i.c);
	return out;
}

FOC_ALWAYS_INLINE foc_math::AlphaBeta foc_math::Clarke(float ia, float ib)
{
	AlphaBeta out;
	out.alpha = ia;
	out.beta  = ONE_BY_SQRT3 * ia + TWO_BY_SQRT3 * ib;
	return out;
}

FOC_ALWAYS_INLINE foc_math::Dq foc_math::Park(const AlphaBeta &i, const SinCos &sc)
{
	Dq out;
	out.d = sc.cos * i.alpha + sc.sin * i.beta;
	out.q = sc.cos * i.beta - sc.sin * i.alpha;
	return out;
}

FOC_ALWAYS_INLINE foc_math::AlphaBeta foc_math::Inv_Park(const Dq &v, const SinCos &sc)
{
	AlphaBeta out;
	out.alpha = sc.cos * v.d - sc.sin * v.q;
	out.beta  = sc.cos * v.q + sc.sin * v.d;
	return out;
}

FOC_ALWAYS_INLINE foc_math::Abc foc_math::Inv_Clarke(const AlphaBeta &v)
{
	Abc out;
	out.a = v.alpha;
	out.b = -0.5f * v.alpha + SQRT3_BY_2 * v.beta;
	out.c = -0.5f * v.alpha - SQRT3_BY_2 * v.beta;
	return out;
}

FOC_ALWAYS_INLINE float foc_math::Norm_Angle(float angle)
{
	// 电角度一般已经在 [-π, π)，只有位置环的大角度才会走 fmodf。
	// fmodf 是 O(1)，不会像 while 循环那样随运行时间变慢。
	if (angle >= PI || angle < -PI)
	{
		angle = fmodf(angle + PI, TWO_PI);
		if (angle < 0.0f) angle += TWO_PI;
		angle -= PI;
	}

	return angle;
}

FOC_ALWAYS_INLINE float foc_math::Limit(float value, float min, float max)
{
	if (value > max) return max;
	if (value < min) return min;
	return value;
}

FOC_ALWAYS_INLINE float foc_math::Limit_Abs(float value, float limit)
{
	if (value > limit)  return limit;
	if (value < -limit) return -limit;
	return value;
}

FOC_ALWAYS_INLINE float foc_math::Lp_Filter(float last, float target, float factor)
{
	return last + factor * (target - last);
}

#undef FOC_ALWAYS_INLINE

#endif 
