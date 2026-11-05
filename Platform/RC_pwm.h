#pragma once
#include "main.h"

#ifdef __cplusplus

/*==============================================================================
 * 三相 PWM 输出（TIM1 互补输出 + 死区）
 *
 * 硬件配置（CubeMX）：
 *   中央对齐模式 1，PSC = 0，ARR = 4249 -> 20kHz
 *   CH1/CH2/CH3 互补输出，死区 100 个时钟周期（约 600ns）
 *   CH4 输出比较（无引脚）作为 ADC 规则组触发源
 *
 * 中央对齐 PWM2 下：
 *   高边在 CNT >= CCR 时导通，所以实际高边占空比 = 1 - CCR/ARR。
 *   代码里 Pwm::Set_Duty() 接收的是“高边占空比”，内部换算成 CCR = (1-duty)*ARR。
 *   占空比 50% 时 CCR=ARR/2，三相线电压为 0（输出零矢量，电机不受力）。
 *============================================================================*/

namespace pwm
{
	class Pwm
	{
	public:
		Pwm(TIM_HandleTypeDef &htim_);
		virtual ~Pwm() {}

		void Pwm_Start();			// 启动时基与三相互补输出（输出保持关断）
		void Pwm_Stop();			// 关闭 PWM 输出
		void Enable_Update_It() { __HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE); }

		void Output_Enable();		// 使能输出（MOE = 1）
		void Output_Disable();		// 关断输出（MOE = 0，上下桥臂全关）
		bool Output_Is_Enable() const;

		void Set_Duty(float duty_a_, float duty_b_, float duty_c_);		// 占空比 0~1

		float Get_Period() const { return period; }		// 自动重装值 ARR
		float Get_Freq() const;							// PWM 频率 (Hz)

		TIM_HandleTypeDef *htim;

	private:
		float period = 0.0f;		// ARR
		float tim_clk = 0.0f;		// 定时器时钟 (Hz)
	};

	/*------------------------------------------------------------------------------
	 * 占空比 -> CCR：电流环每 100µs 调一次，内联掉函数调用。
	 *
	 * TIM1 CH1~CH3 是 PWM2：实际高边占空比 = 1 - CCR/ARR，所以 CCR = (1-duty)*ARR。
	 *------------------------------------------------------------------------------*/
	inline __attribute__((always_inline)) void Pwm::Set_Duty(float duty_a_, float duty_b_, float duty_c_)
	{
		if (duty_a_ < 0.0f) duty_a_ = 0.0f; else if (duty_a_ > 1.0f) duty_a_ = 1.0f;
		if (duty_b_ < 0.0f) duty_b_ = 0.0f; else if (duty_b_ > 1.0f) duty_b_ = 1.0f;
		if (duty_c_ < 0.0f) duty_c_ = 0.0f; else if (duty_c_ > 1.0f) duty_c_ = 1.0f;

		const uint32_t ccr_a = (uint32_t)((1.0f - duty_a_) * period + 0.5f);
		const uint32_t ccr_b = (uint32_t)((1.0f - duty_b_) * period + 0.5f);
		const uint32_t ccr_c = (uint32_t)((1.0f - duty_c_) * period + 0.5f);

		htim->Instance->CCR1 = ccr_a;
		htim->Instance->CCR2 = ccr_b;
		htim->Instance->CCR3 = ccr_c;
	}
}
#endif
