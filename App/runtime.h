#pragma once

#include "RC_adc.h"
#include "RC_analog.h"
#include "RC_encoder.h"
#include "RC_pwm.h"
#include "RC_tim.h"
#include "RC_timer.h"
#include "RC_uart.h"

#include "MT6701.h"
#include "Framework/latest.h"
#include "Framework/spsc_ring.h"

#include "app_messages.h"
#include "command_router.h"
#include "control_loop.h"
#include "monitor.h"
#include "telemetry.h"

/*==============================================================================
 * 运行时装配（Composition Root）
 *
 *   只有这里同时看见 Platform / Algorithm / App 三层，
 *   负责把"谁生产、谁消费"接起来。其它模块只依赖消息和抽象接口。
 *============================================================================*/

namespace app
{
	class Runtime
	{
	public:
		Runtime();

		void Init();

		ControlLoop &Control() { return control_loop; }
		Monitor &Monitor_() { return monitor; }
		CommandRouter &Router() { return command_router; }
		Telemetry &Telemetry_() { return telemetry; }
		uart::Port &Uart() { return uart_port; }
		SystemState &State() { return system_state; }

	private:
		// 底层外设（构造时只保存句柄/挂中断，真正启动在 Init() 里）
		tim::Tim tim1;
		tim::Tim tim2;
		timer::Timer timer;
		adc::Adc adc1;
		adc::Adc adc2;
		pwm::Pwm pwm1;
		analog::PowerMonitor power;
		encoder::SpiEncoder spi_encoder;

		// 算法
		mt6701::Sensor encoder_sensor;

		// 串口
		// 必须 >= 最大命令帧（当前 4+87+1 = 92 字节）。
		// 64 字节时 DMA 会在帧中间就满，重启 DMA 的间隙会丢字节，
		// 表现就是遥测正常、但所有命令都解析不出来。
		uint8_t uart_rx_dma[256] = {};
		uart::Port uart_port;

		// 生产者/消费者通道
		framework::Latest<ControlSnapshot> control_channel;
		framework::Latest<EncoderSnapshot> encoder_channel;
		framework::Latest<PowerSnapshot> power_channel;
		framework::SpscRing<Command, 8> command_queue;

		// 应用模块
		SystemState system_state;
		ControlLoop control_loop;
		Monitor monitor;
		CommandRouter command_router;
		Telemetry telemetry;
	};

	Runtime &Get_Runtime();
}
