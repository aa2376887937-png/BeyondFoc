#pragma once
#include "main.h"
#include "cmsis_os.h"
#include "RC_adc.h"

/*==============================================================================
 * 慢速模拟量：母线电压 / 电机 NTC / 驱动 NTC / 三相电压
 *
 * 全部走 ADC2 规则组（6 个 rank，软件触发 + 扫描）。
 * 采用 DMA 一次性搬运 6 个结果：
 *
 *   监控任务（100Hz） Start_DMA
 *        -> DMA 完成中断（生产者）置 dma_done
 *        -> 监控任务下次 Update() 消费 dma_buf 并换算
 *
 * 这样任务里不再调用 HAL_ADC_PollForConversion() 忙等，ADC2 独立规则组，
 * ADC1 只给 FOC 三相电流注入组用，保证采样节拍稳定。
 *
 * 硬件参数来自 SCH_iRDT_Driver_V1.4：
 *   VBUS      PC5 / ADC2_IN11   分压 R38 15k / R39 2k   -> 系数 8.5
 *   DRV_NTC   PB2 / ADC2_IN12   上拉 10k + NCP18XH103(10k,B=3380)
 *   MOTOR_NTC PC3 / ADC2_IN9    上拉 10k + 电机内置 NTC
 *   VA/VB/VC  PC0/PC1/PC2       分压 15k / 2k           -> 系数 8.5
 *============================================================================*/

#define ANALOG_ADC_VREF			3.3f		// ADC 参考电压 (V)
#define ANALOG_ADC_FULL			4095.0f		// 12 位 ADC 满量程

#define VBUS_DIVIDER_RATIO		16.0f		// 母线分压比：参考固件 0.012890625 V/LSB = 16 × 3.3 / 4096，
										// 即分压 16:1，满量程约 52.8V。实测 12V 输入时旧系数 8.5 只显示 6.1V，
										// 换成 16 后显示 ≈11.6V（V_MOTOR 是 DCBUS 过 D2 肖特基后的电压，掉 0.4V 正常）
#define PHASE_DIVIDER_RATIO		16.0f		// 相电压采样和母线是同一组分压（原理图四个分压块一致），同样按 16:1

#define NTC_R25					10000.0f	// NTC 25°C 时的阻值 (NCP18XH103)
#define NTC_B_VALUE				3380.0f		// B 值 (25/85°C)
#define NTC_PULLUP_R			10000.0f	// 上拉电阻 (R70 / R45)
#define NTC_PARALLEL_R_MOTOR	10000.0f	// 板上 R43 与电机内置 NTC 并联
#define NTC_PARALLEL_R_DRIVER	0.0f		// 驱动 NTC 没有并联电阻

namespace analog
{
	class PowerMonitor : public adc::AdcHandler
	{
	public:
		PowerMonitor(adc::Adc &adc_);
		virtual ~PowerMonitor() {}

		// 配置 ADC2 的 DMA 通道（DMA1_Channel5）并挂中断，必须在 Update() 前调用
		void Init();

		// ADC 自校准（启动转换之前调用）
		bool Calibrate() { return adc->Adc_Calibrate(); }

		// 监控任务 100Hz 调用：
		//   1. 消费上一轮 DMA 结果并换算
		//   2. 启动下一轮 DMA（非阻塞）
		void Update();

		// ADC2 DMA 完成中断 -> ADC 框架分发到这里
		void Adc_ConvCplt_Process() override;

		// DMA1_Channel5 中断入口调用
		static void Dma_Irq_Handler();

		float Get_Bus_Voltage() const { return v_bus; }			// 母线电压 (V)
		float Get_Phase_Voltage_A() const { return v_a; }		// 相电压 (V)
		float Get_Phase_Voltage_B() const { return v_b; }
		float Get_Phase_Voltage_C() const { return v_c; }
		float Get_Motor_Temp() const { return motor_temp; }		// 电机 NTC (°C)
		float Get_Driver_Temp() const { return driver_temp; }	// 驱动 NTC (°C)

		// 原始码值，调试用：0=VBUS 1=DRV_NTC 2=MOTOR_NTC 3=VA 4=VB 5=VC
		uint32_t Get_Raw(uint8_t idx_) const { return (idx_ < 6) ? raw[idx_] : 0; }

	private:
		static float Adc_To_Volt(uint32_t raw_);
		static float Ntc_To_Temp(float volt_, float r_parallel_);
		void Compute();

		adc::Adc *adc;

		// DMA 生产者=完成中断，消费者=监控任务
		static DMA_HandleTypeDef dma_handle;
		uint16_t dma_buf[6] = {0, 0, 0, 0, 0, 0};
		volatile bool dma_ready = false;
		volatile bool dma_busy = false;
		volatile bool dma_done = false;
		uint32_t busy_since = 0;

		uint32_t raw[6] = {0};
		float v_bus = 0.0f;
		float v_a = 0.0f, v_b = 0.0f, v_c = 0.0f;
		float motor_temp = 0.0f, driver_temp = 0.0f;
	};
}

/* DMA1_Channel5 中断的 C 入口（放在 RC_analog.cpp，避免改 CubeMX 生成文件） */
#ifdef __cplusplus
extern "C" {
#endif
void PowerMonitor_Dma_Irq_Handler(void);
#ifdef __cplusplus
}
#endif
