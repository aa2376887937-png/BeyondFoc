#include "FOC_math.h"

/*==============================================================================
 * Clarke / Park / Inv_Park / Inv_Clarke / Norm_Angle / Limit / Lp_Filter
 * 都是高频小函数，定义已移到 FOC_math.h 里强制内联。
 * 本文件只保留需要查表/较大体量的 Sin_Cos、Svpwm 和 LUT 初始化。
 *============================================================================*/

/*==============================================================================
 * 角度处理
 *============================================================================*/
namespace
{
	// sin 查找表：1024 点 + 线性插值，误差 < 1e-5，比 newlib 的 sinf/cosf 快几十倍。
	// 20kHz 中断里每秒要算 2 万次三角，Debug(-O0) 下 sinf+cosf 能吃掉 8~10us，
	// 这张表把它压到 1us 以内。
	constexpr int kSinLutSize = 1024;
	constexpr int kSinLutMask = kSinLutSize - 1;
	float sin_lut[kSinLutSize];
	bool lut_ready = false;
}

void foc_math::Init_Lut()
{
	for (int i = 0; i < kSinLutSize; i++)
	{
		sin_lut[i] = sinf((float)i * (TWO_PI / (float)kSinLutSize));
	}

	lut_ready = true;
}

foc_math::SinCos foc_math::Sin_Cos(float angle)
{
	SinCos out;

	if (!lut_ready)			// 没初始化就退回库函数（正常不会走到）
	{
		out.sin = sinf(angle);
		out.cos = cosf(angle);
		return out;
	}

	// 归一化到 [0, 2π)。电角度一般已经在 [-π, π)，走加法就够，
	// 不要每 50us 都调一次 fmodf（那也是几百个周期）
	float a = angle;
	if (a >= TWO_PI || a < -TWO_PI)
	{
		a = fmodf(a, TWO_PI);
	}
	if (a < 0.0f)
	{
		a += TWO_PI;
	}
	else if (a >= TWO_PI)
	{
		a -= TWO_PI;
	}

	const float x = a * ((float)kSinLutSize / TWO_PI);
	int i = (int)x;
	float f = x - (float)i;
	if (i >= kSinLutSize) { i = 0; f = 0.0f; }

	const float s0 = sin_lut[i];
	const float s1 = sin_lut[(i + 1) & kSinLutMask];
	out.sin = s0 + (s1 - s0) * f;

	// cos(θ) = sin(θ + π/2)，索引错开 1/4 个表就行
	const int j = (i + kSinLutSize / 4) & kSinLutMask;
	const float c0 = sin_lut[j];
	const float c1 = sin_lut[(j + 1) & kSinLutMask];
	out.cos = c0 + (c1 - c0) * f;

	return out;
}

/*==============================================================================
 * SVPWM：调制度 -> 三相占空比（七段式）
 *
 * 【输入约定】
 * mod 是归一化调制度，|mod| = 1 恰好对应线性调制区的边界，
 * 即相电压基波峰值 = 母线电压/√3（比 SPWM 高 15.5%，就是 SVPWM 的价值所在）
 *
 * 【原理】
 * SVPWM 的扇区法（先判断扇区，再算相邻两个基本矢量的作用时间 T1/T2，
 * 零矢量 T0 平均分到周期两端）在数学上等价于在三相正弦调制波上
 * 叠加一个共模（零序）分量：
 *
 *     占空比 = 0.5 + (相调制度 - 三相中点) / √3
 *
 * 以扇区 1 为例，用笔记里的写法：
 *     t1 = (α - 1/√3 * β) * T
 *     t2 = (2/√3 * β) * T
 *     ta = (T + t1 + t2) / 2
 *     tb = ta - t1
 *     tc = tb - t2
 * 展开后与上式逐项相同。用中点法实现的好处是：不需要判断扇区，
 * 扇区边界上不会出现切换抖动，过调制时也只是三相比较值一起被限幅，行为可预期
 *
 * 【为什么叠加共模分量是合法的】
 * 电机中性点不引出，三相同时加减同一个电压，线电压（也就是真正加在
 * 绕组上的电压）不变，但可以多"抢"出 15.5% 的电压裕量
 *============================================================================*/
foc_math::Abc foc_math::Svpwm(const AlphaBeta &mod, float period)
{
	Abc m;

	// 1. 逆 Clarke 得到三相调制度
	m.a = mod.alpha;
	m.b = -0.5f * mod.alpha + SQRT3_BY_2 * mod.beta;
	m.c = -0.5f * mod.alpha - SQRT3_BY_2 * mod.beta;

	// 2. 求三相调制度的中点（这就是要注入的共模分量）
	float max = m.a > m.b ? m.a : m.b;
	float min = m.a < m.b ? m.a : m.b;
	if (m.c > max) max = m.c;
	if (m.c < min) min = m.c;

	const float mid = 0.5f * (max + min);

	// 3. 转为占空比再换算成定时器比较值
	Abc duty;

	duty.a = (0.5f + ONE_BY_SQRT3 * (m.a - mid)) * period;
	duty.b = (0.5f + ONE_BY_SQRT3 * (m.b - mid)) * period;
	duty.c = (0.5f + ONE_BY_SQRT3 * (m.c - mid)) * period;

	// 4. 限幅保护：调制度超界（过调制或参数异常）时也不能写坏寄存器，
	//    注意这里只限幅不做归一化，三相比较值的相对关系保持不变
	duty.a = Limit(duty.a, 0.0f, period);
	duty.b = Limit(duty.b, 0.0f, period);
	duty.c = Limit(duty.c, 0.0f, period);

	return duty;
}

/*------------------------------------------------------------------------------
 * 扇区判断：1~6，扇区 k 覆盖 (k-1)*60° ~ k*60°，分界线正好落在基本矢量上
 * （与 VESC、TI 的编号一致，前两个扇区的判断和笔记里 VESC 的写法相同）
 *
 * 只靠 αβ 的大小关系就能判断，不用算 atan2：
 *     相邻扇区的分界线在 0°、60°、120°...，也就是 |β| = √3 * |α|
 *------------------------------------------------------------------------------*/
uint8_t foc_math::Svpwm_Sector(const AlphaBeta &mod)
{
	const float a = mod.alpha;
	const float b = mod.beta;

	if (b >= 0.0f)
	{
		if (a >= 0.0f)		// 0°~90°：扇区 1（0°~60°）或 2
		{
			return (ONE_BY_SQRT3 * b > a) ? 2 : 1;
		}
		else				// 90°~180°：扇区 2（60°~120°）或 3
		{
			return (ONE_BY_SQRT3 * b > -a) ? 2 : 3;
		}
	}
	else
	{
		if (a >= 0.0f)		// 270°~360°：扇区 5（240°~300°）或 6
		{
			return (-ONE_BY_SQRT3 * b > a) ? 5 : 6;
		}
		else				// 180°~270°：扇区 4（180°~240°）或 5
		{
			return (-ONE_BY_SQRT3 * b > -a) ? 5 : 4;
		}
	}
}

/*==============================================================================
 * 限幅与滤波（实现已移到 FOC_math.h 内联）
 *============================================================================*/
