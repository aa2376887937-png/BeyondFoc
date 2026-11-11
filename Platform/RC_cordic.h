#pragma once
#include "main.h"

/*==============================================================================
 * CORDIC 硬件加速（G474 内置协处理器）
 *
 * 寄存器直写版：绕过 HAL 的锁/状态机，一次 sin/cos 只要 ~10 个周期
 * 的寄存器访问 + 硬件 4~10 周期计算。比 1024 点查表（~40 周期）还快，
 * 且不占 4KB Flash（LUT 表）。
 *
 * 数据格式（Q1.31）：
 *   角度输入：[-1, 1) 对应 [-π, π)
 *   sin/cos 输出：[-1, 1)
 *
 * 用法（FOC_math 的 Sin_Cos 内部调用，其他代码不用直接用）：
 *   foc_math::SinCos sc = foc_math::Sin_Cos(angle);
 *============================================================================*/

namespace cordic
{
	// 初始化：COSINE 模式、Q1.31 进出、一次读 sin（NRD=0 只读 cos 时配 SINE+PHASE）
	// 这里用 "COSINE, 写1读1" 的配置：输入角度，输出 [cos, sin]（NRES=2 时）。
	// 实测 G4 CORDIC 的 COSINE 函数写 1 个参数、读 2 个结果（cos + sin）。
	inline void Init()
	{
		// CSR: FUNC=COSINE(0000), PRECISION=4 cycles(1/256 精度，够 FOC)，
		//       NRES=2（读 cos+sin 两个数），NARG=1，NDMA=0
		CORDIC->CSR = 0u                            // FUNC = 0 (COSINE)
					  | (4u << CORDIC_CSR_PRECISION_Pos)	// 4 周期：1/2^5 精度
					  | CORDIC_CSR_NRES;				// NRES=2：读 2 个结果
	}

	// sin/cos 一次算出。angle 单位弧度，任意范围。
	// 返回 {sin, cos}，精度约 1e-5（4 周期模式）。
	struct SinCosResult
	{
		float sin;
		float cos;
	};

	inline SinCosResult Sin_Cos(float angle_rad)
	{
		// CORDIC 输入约定（ST AN5325 / RM0440）：ARG1 = θ/π，Q1.31。
		// 满量程 1.0 = π 弧度，所以折叠周期是 2.0（= 2π），不是 1.0！
		// 用 roundf 提取 2π 倍数，保证 a ∈ [-1, 1)。
		const float k1ByPi = 0.31830988618379067f;

		float a = angle_rad * k1ByPi;
		a -= 2.0f * floorf(a * 0.5f + 0.5f);		// mod 2π，折叠到 [-1, 1)

		// Q1.31 定点（a ∈ [-1,1) → q ∈ [-2^31, 2^31)，-1.0 = INT_MIN 合法）
		const int32_t q = (int32_t)(a * 2147483648.0f);

		CORDIC->WDATA = (uint32_t)q;

		// RRDY 在写 WDATA 后几个周期置位（4 周期精度下等待极短）
		while ((CORDIC->CSR & CORDIC_CSR_RRDY) == 0u) { }

		SinCosResult r;
		// COSINE 模式 NRES=2：先 cos 后 sin（ST AN5325 CORDIC_CosSin 例程顺序）
		r.cos = (float)((int32_t)CORDIC->RDATA) * (1.0f / 2147483648.0f);
		r.sin = (float)((int32_t)CORDIC->RDATA) * (1.0f / 2147483648.0f);

		return r;
	}
}
