#include "app.h"
#include "runtime.h"
#include "app_config.h"
#include "RC_prof.h"

/*==============================================================================
 * 应用入口
 *
 *   main.c 只负责 HAL/CubeMX 生成的底层初始化，然后调用：
 *     All_Init()   -> 装配并启动 Platform/Algorithm 模块
 *     App_Start()  -> 创建 App 任务（在 osKernelInitialize() 之后）
 *============================================================================*/

namespace
{
	app::Runtime g_runtime;
}

namespace app
{
	Runtime &Get_Runtime()
	{
		return g_runtime;
	}

	Runtime::Runtime()
		: tim1(htim1)
		, tim2(htim2)
		, timer(&tim2)
		, adc1(hadc1)
		, adc2(hadc2)
		, pwm1(htim1)
		, power(adc2)
		, spi_encoder(hspi1)
		, encoder_sensor()
		, uart_port(huart3, uart_rx_dma, (uint16_t)sizeof(uart_rx_dma))
		, control_loop(adc1, pwm1, tim2, config::kPolePairs, command_queue, control_channel, power_channel)
		, monitor(spi_encoder, encoder_sensor, power, encoder_channel, power_channel)
		, command_router(uart_port, command_queue)
		, telemetry(uart_port, control_channel, encoder_channel, power_channel, system_state)
	{
		control_loop.Set_Encoder(&encoder_sensor);
	}

	void Runtime::Init()
	{
		system_state.boot_stage = 1;						// All_Init 开始
		prof_init();										// 打开 DWT 周期计数器

		// ADC 自校准：必须在启动任何转换之前做，能明显减小零偏。
		// 失败也不锁死（没校准一样能跑，只是零点偏一点），状态位报给上位机。
		bool cal_ok = adc1.Adc_Calibrate();
		if (!power.Calibrate()) cal_ok = false;
		system_state.adc_cal_failed = !cal_ok;
		system_state.boot_stage = 2;						// ADC 校准完成

		tim2.Tim_It_Start();								// 启动 1kHz 中断（时间戳）
		system_state.boot_stage = 3;						// TIM2 启动完成

		uart_port.Start();									// 启动上位机命令接收
		system_state.boot_stage = 4;

		pwm1.Pwm_Start();									// 启动 PWM 时基与三相互补输出（输出保持关断）
		system_state.boot_stage = 5;

		monitor.Init();										// 编码器 + ADC2 规则组慢速量
		control_loop.Init();								// 控制器参数（ADC1 注入组由 TIM1 更新中断启动）

		// 上电自动校零（此时功率级没开，电流就是 0）。
		// 注入中断已经打开，用临界区避免 ISR 看到改到一半的校准状态。
		taskENTER_CRITICAL();
		control_loop.Current().Start_Calibration();
		taskEXIT_CRITICAL();

		// 现在才允许 TIM1 更新中断启动 ADC 注入组，避免在控制器/校准准备好之前进电流环
		pwm1.Enable_Update_It();
		system_state.boot_stage = 6;

		if (config::kSvpwmViewDefaultOn)
		{
			// 上电先在示波器上看 SVPWM 调制波（不打开 MOE，功率级不动）
			Command cmd;
			cmd.enable = false;
			cmd.mode = MODE_SCOPE;
			cmd.iq_a = 0.5f / OL_AMP_PER_AMP;					// 0.5 调制度
			cmd.speed_rpm = 5.0f * 60.0f / (float)config::kPolePairs;	// 5Hz 电角速度
			command_queue.Try_Push(cmd);
		}

		system_state.boot_stage = 7;						// 初始化全部完成
		system_state.ready = true;							// 放开所有任务
	}
}

extern "C" void All_Init(void)
{
	app::Get_Runtime().Init();
}
