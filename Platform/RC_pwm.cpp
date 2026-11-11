#include "RC_pwm.h"

namespace pwm
{
	// 只保存句柄。真正的初始化放在 Pwm_Start() 里：
	// 全局对象的构造函数在 main() 之前运行，那时候外设还没初始化（ARR 还是 0）
	Pwm::Pwm(TIM_HandleTypeDef &htim_) : htim(&htim_)
	{
	}

	void Pwm::Pwm_Start()
	{
		period = (float)htim->Instance->ARR;

		// 定时器时钟：TIM1 挂在 APB2 上，APB2 分频不为 1 时定时器时钟 = PCLK2 * 2
		// （如果换成 APB1 上的定时器 TIM2~TIM7，这里改成 HAL_RCC_GetPCLK1Freq()）
		uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
		tim_clk = (pclk2 == HAL_RCC_GetHCLKFreq()) ? (float)pclk2 : (float)(pclk2 * 2);

		// 关闭比较值预装载（影子寄存器）：CCR 写入立即生效，不等到下一个 UEV。
		// 中央对齐模式下这样会有"半拍"风险（三相 CCR 在 CNT 上下行中途跳变），
		// 但换来的是采样→计算→PWM 的延迟少整整一个周期（50µs→0）。
		// 三相都在同一个 20kHz 中断里背靠背写入，实际写入间隔 <1µs，
		// 半拍窗口很小；电流环闭环会吸收这一点差异。
		htim->Instance->CCMR1 &= ~(TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
		htim->Instance->CCMR2 &= ~TIM_CCMR2_OC3PE;

		// CH4 不接引脚。硬件触发源：TRGO=OC4REF，ADC1 注入组由它直接触发。
		//
		// 时序（中央对齐 PWM2）：
		//   CNT=0   → UEV，三相低边全开（零矢量窗口中心）
		//   CNT=CCR4 上行 → OC4REF 跳变 → TRGO → ADC 注入组启动
		// CCR4 = ARR×5% → 采样点在 CNT=0 后约 1.2µs，落在零矢量窗口内、
		// 且避开了 CNT=0 附近的死区/换相尖峰。
		// 必要时微调 Pulse：往后挪 = 躲尖峰，往前挪 = 贴近窗口中心。
		// 注意大占空比时窗口变窄（三相低边导通时间 < 采样时间会采错相），
		// 需要配最小占空比限制（过调制已在 0.95 封顶）。
		TIM_OC_InitTypeDef sConfigOC4 = {0};
		sConfigOC4.OCMode = TIM_OCMODE_PWM2;
		sConfigOC4.Pulse = (uint32_t)(period * 0.05f);
		sConfigOC4.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC4.OCFastMode = TIM_OCFAST_DISABLE;
		if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC4, TIM_CHANNEL_4) != HAL_OK) Error_Handler();
		if (HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

		// 初始占空比 50%：三相线电压为 0，电机不受力，是最安全的上电状态
		Set_Duty(0.5f, 0.5f, 0.5f);

		if (HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
		if (HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
		if (HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3) != HAL_OK) Error_Handler();

		if (HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
		if (HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
		if (HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_3) != HAL_OK) Error_Handler();

		// HAL_TIM_PWM_Start 内部会把 MOE 置 1，这里先关掉：
		// 等零偏校准做完、确认采样正常后再调用 Output_Enable() 打开输出
		Output_Disable();
	}

	void Pwm::Pwm_Stop()
	{
		Output_Disable();

		if (HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
		if (HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
		if (HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_3) != HAL_OK) Error_Handler();

		if (HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
		if (HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
		if (HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
	}

	/*------------------------------------------------------------------------------
	 * 输出使能
	 *
	 * MOE = 0 时上下桥臂全关（OSSI = 0，输出被强制拉低），用于急停
	 * 注意：如果 TIM1 的 Break 功能被使能而 BKIN 引脚悬空或为低电平，
	 *       硬件会一直触发刹车并清零 MOE，输出打不开，可以用 Output_Is_Enable() 判断
	 *------------------------------------------------------------------------------*/
	void Pwm::Output_Enable()
	{
		__HAL_TIM_MOE_ENABLE(htim);
	}

	void Pwm::Output_Disable()
	{
		__HAL_TIM_MOE_DISABLE(htim);
	}

	bool Pwm::Output_Is_Enable() const
	{
		return (htim->Instance->BDTR & TIM_BDTR_MOE) != 0;
	}

	/*------------------------------------------------------------------------------
	 * 设置三相高边占空比（0~1）
	 *
	 * 实现已移到 RC_pwm.h 内联（电流环每 100µs 调一次）。
	 *------------------------------------------------------------------------------*/

	float Pwm::Get_Freq() const
	{
		if (period <= 0.0f || tim_clk <= 0.0f) return 0.0f;

		// 中央对齐：一个 PWM 周期 = 2 * ARR 个计数
		return tim_clk / (2.0f * period);
	}
}
