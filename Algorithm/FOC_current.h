#pragma once

#include "FOC_math.h"
#include <stdint.h>

/*==============================================================================
 * 三相电流处理 —— 纯算法层
 *
 *   输入：ADC 规则组的原始码值（由 Platform/RC_adc 提供）
 *   输出：三相电流 (A)
 *
 *   本文件不依赖 HAL、ADC 句柄或 FreeRTOS。
 *   "谁在什么中断里读 ADC"是 Platform + App 的事，
 *   这里只负责零偏校准、码值换算和遥测平均。
 *============================================================================*/

// ---------------- 电流采样硬件参数（按实际板子修改） ----------------
// 依据 SCH_iRDT_Driver_V1.4：三相低边 5mΩ 采样电阻（U3/U4/U5）+ RS724 差分放大（Rf/Rin = 50k/1k）
#define CURRENT_SHUNT_R			0.005f		// 采样电阻阻值 (Ω)，板载 U3/U4/U5 = 5mΩ
#define CURRENT_AMP_GAIN		50.0f		// 运放增益 Rf/Rin = 50k/1k = 50
#define CURRENT_ADC_VREF		3.3f		// ADC 参考电压 (V)
#define CURRENT_ADC_FULL		4095.0f		// 12 位 ADC 满量程
#define CURRENT_OFFSET_ADC		2048.0f		// 零电流对应的 ADC 码值（1.65V 偏置）
#define CURRENT_CAL_SAMPLES		10000	// 零偏校准采样次数（20kHz 下 100ms）
#define CURRENT_CAL_SETTLE		400			// 校准前先丢掉 400 个点（20ms），等电流/开关尖峰衰减

namespace foc_current
{
	struct Raw
	{
		uint16_t a;
		uint16_t b;
		uint16_t c;
	};

	class Processor
	{
	public:
		Processor();

		void Set_Scale(float r_shunt_, float amp_gain_);					// 采样电阻与放大倍数
		void Set_Offset(float offset_a_, float offset_b_, float offset_c_);	// 零点（ADC 码值）
		void Set_Sign(float sign_a_, float sign_b_, float sign_c_);			// 方向 +1 / -1

		// 零偏校准：上电后、打开输出之前调用。此时功率级不工作，电流为零，
		// 把若干次采样求平均作为真正的零点（比固定用 2048 准得多）。
		// settle_：先丢掉多少个采样再开始累加，避免上一拍的电流/开关尖峰混进平均值。
		void Start_Calibration(uint16_t samples_ = CURRENT_CAL_SAMPLES,
							   uint16_t settle_ = CURRENT_CAL_SETTLE);
		bool Is_Calibrating() const { return calibrating; }

		// 每个电流采样周期调用一次。
		// 返回 false 表示正在校准，调用者应输出零电压。
		bool Update(const Raw &raw_);

		// 兼容旧接口：只采两相（B/C），A 相由 KVL 计算；当前三电阻规则采样用 Update()
		bool Update_Two_Phase(uint16_t raw_b_, uint16_t raw_c_);

		foc_math::Abc Get_Current() const { return current; }			// 瞬时值，给电流环用
		foc_math::Abc Take_Average();									// 1kHz 遥测用：上个窗口平均并清零
		foc_math::Abc Get_Current_Raw() const { return raw; }			// 原始 ADC 码值
		foc_math::Abc Get_Offset() const { return offset; }

	private:
		foc_math::Abc raw = {0.0f, 0.0f, 0.0f};
		foc_math::Abc offset = {CURRENT_OFFSET_ADC, CURRENT_OFFSET_ADC, CURRENT_OFFSET_ADC};
		foc_math::Abc sign = {1.0f, 1.0f, 1.0f};
		foc_math::Abc current = {0.0f, 0.0f, 0.0f};

		float scale = 1.0f;

		// 遥测平均累加（40kHz 中断里累加，1kHz 里取走）
		float sum_a = 0.0f;
		float sum_b = 0.0f;
		float sum_c = 0.0f;
		uint16_t sum_n = 0;

		volatile bool calibrating = false;
		uint16_t cal_count = 0;
		uint16_t cal_target = 1000;
		uint16_t cal_settle = 0;
		float cal_sum_a = 0.0f;
		float cal_sum_b = 0.0f;
		float cal_sum_c = 0.0f;

		// 3 点中值滤波：单点采样毛刺直接吃掉，额外延迟只有 1 个采样周期
		float hist[3][3] = {};
		uint8_t hist_idx = 0;
		uint8_t hist_n = 0;
	};
}
