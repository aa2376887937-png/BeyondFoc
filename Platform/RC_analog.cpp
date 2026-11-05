#include "RC_analog.h"
#include "math.h"

namespace analog
{
	// ADC2 专用 DMA：DMA1_Channel5 + DMAMUX 请求 ADC2
	DMA_HandleTypeDef PowerMonitor::dma_handle;

	PowerMonitor::PowerMonitor(adc::Adc &adc_)
		: adc::AdcHandler(&adc_)		// tim 风格：挂到 ADC2 的规则组完成回调
		, adc(&adc_)
	{
	}

	/*------------------------------------------------------------------------------
	 * 配置 ADC2 的 DMA 通道
	 *
	 * ADC2 规则组 6 个 rank，软件触发一次扫描，DMA 自动把 6 个半字搬进 dma_buf。
	 * 转换完成后 DMA 中断 -> HAL_ADC_ConvCpltCallback -> Adc 框架 -> 本类。
	 *------------------------------------------------------------------------------*/
	void PowerMonitor::Init()
	{
		if (dma_ready) return;

		__HAL_RCC_DMA1_CLK_ENABLE();

		dma_handle.Instance = DMA1_Channel5;
		dma_handle.Init.Request = DMA_REQUEST_ADC2;
		dma_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;
		dma_handle.Init.PeriphInc = DMA_PINC_DISABLE;
		dma_handle.Init.MemInc = DMA_MINC_ENABLE;
		dma_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		dma_handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		dma_handle.Init.Mode = DMA_NORMAL;
		dma_handle.Init.Priority = DMA_PRIORITY_LOW;

		if (HAL_DMA_Init(&dma_handle) != HAL_OK) return;

		__HAL_LINKDMA(adc->hadc, DMA_Handle, dma_handle);

		HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

		dma_ready = true;
	}

	float PowerMonitor::Adc_To_Volt(uint32_t raw_)
	{
		return (float)raw_ * ANALOG_ADC_VREF / ANALOG_ADC_FULL;
	}

	/*------------------------------------------------------------------------------
	 * NTC 转温度
	 *
	 * 电路：3.3V -- Rpu(10k) --+-- NTC -- GND，采样点在中间
	 *   V = VCC * R / (Rpu + R)   ->   R = Rpu * V / (VCC - V)
	 * 电机 NTC 那一路上还有 R43(10k) 与 NTC 并联，要先从并联值反算出 NTC 阻值
	 *   R_ntc = 1 / (1/R - 1/R_parallel)
	 * B 参数方程：1/T = 1/T0 + ln(R_ntc/R25) / B
	 *------------------------------------------------------------------------------*/
	float PowerMonitor::Ntc_To_Temp(float volt_, float r_parallel_)
	{
		if (volt_ <= 0.001f || volt_ >= (ANALOG_ADC_VREF - 0.001f)) return -273.15f;

		const float r_total = NTC_PULLUP_R * volt_ / (ANALOG_ADC_VREF - volt_);

		float r_ntc = r_total;
		if (r_parallel_ > 0.0f)
		{
			if (r_total >= r_parallel_) return -273.15f;	// 相当于开路，NTC 没接
			r_ntc = r_total * r_parallel_ / (r_parallel_ - r_total);
		}

		const float inv_t = 1.0f / 298.15f + logf(r_ntc / NTC_R25) / NTC_B_VALUE;

		return 1.0f / inv_t - 273.15f;
	}

	/*------------------------------------------------------------------------------
	 * DMA 完成中断（生产者）：只置标志，不做浮点运算
	 *------------------------------------------------------------------------------*/
	void PowerMonitor::Adc_ConvCplt_Process()
	{
		__DMB();
		dma_done = true;
		dma_busy = false;
	}

	void PowerMonitor::Dma_Irq_Handler()
	{
		HAL_DMA_IRQHandler(&dma_handle);
	}

	/*------------------------------------------------------------------------------
	 * 监控任务 100Hz 调用
	 *
	 *   1. 消费上一轮 DMA 结果并换算
	 *   2. 启动下一轮 DMA，立即返回
	 *------------------------------------------------------------------------------*/
	void PowerMonitor::Update()
	{
		if (!dma_ready) return;

		if (dma_done)
		{
			__DMB();
			dma_done = false;
			dma_busy = false;

			for (uint8_t i = 0; i < 6; i++)
			{
				raw[i] = dma_buf[i];
			}

			Compute();
		}

		// DMA 卡死保护：正常一轮扫描几十 us，5ms 还没完成就强制复位
		if (dma_busy && ((HAL_GetTick() - busy_since) > 5u))
		{
			HAL_ADC_Stop_DMA(adc->hadc);
			dma_done = false;
			dma_busy = false;
		}

		if (!dma_busy)
		{
			dma_busy = true;
			busy_since = HAL_GetTick();

			// HAL_ADC_Start_DMA 的形参是 uint32_t*，DMA 实际按半字搬运
			if (HAL_ADC_Start_DMA(adc->hadc, (uint32_t *)dma_buf, 6) != HAL_OK)
			{
				dma_busy = false;
			}
		}
	}

	void PowerMonitor::Compute()
	{
		// 母线 / 相电压：分压比换算
		v_bus = Adc_To_Volt(raw[0]) * VBUS_DIVIDER_RATIO;
		v_a = Adc_To_Volt(raw[3]) * PHASE_DIVIDER_RATIO;
		v_b = Adc_To_Volt(raw[4]) * PHASE_DIVIDER_RATIO;
		v_c = Adc_To_Volt(raw[5]) * PHASE_DIVIDER_RATIO;

		// 两路 NTC
		driver_temp = Ntc_To_Temp(Adc_To_Volt(raw[1]), NTC_PARALLEL_R_DRIVER);
		motor_temp = Ntc_To_Temp(Adc_To_Volt(raw[2]), NTC_PARALLEL_R_MOTOR);
	}
}

/* DMA1_Channel5 中断入口。
 * 放在这里而不是 stm32g4xx_it.c，避免每次 CubeMX 重新生成都要改生成文件。 */
extern "C" void PowerMonitor_Dma_Irq_Handler(void)
{
	analog::PowerMonitor::Dma_Irq_Handler();
}

extern "C" void DMA1_Channel5_IRQHandler(void)
{
	PowerMonitor_Dma_Irq_Handler();
}
