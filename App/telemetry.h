#pragma once

#include "RC_uart.h"
#include "Framework/latest.h"
#include "app_messages.h"

/*==============================================================================
 * 遥测（应用层）
 *
 *   从 Latest 邮箱读取控制 / 编码器 / 母线快照，拼成 ScopeData，
 *   交给 protocol 组帧，再写进 UART 的发送环形缓冲。
 *
 *   本类不认识 ADC、PWM、编码器对象，只认识消息结构体。
 *============================================================================*/

namespace app
{
	class Telemetry
	{
	public:
		Telemetry(uart::Port &port_,
				  const framework::Latest<ControlSnapshot> &control_,
				  const framework::Latest<EncoderSnapshot> &encoder_,
				  const framework::Latest<PowerSnapshot> &power_,
				  const SystemState &state_);

		virtual ~Telemetry() {}

		void Send();

	private:
		uart::Port &port;
		const framework::Latest<ControlSnapshot> &control;
		const framework::Latest<EncoderSnapshot> &encoder;
		const framework::Latest<PowerSnapshot> &power;
		const SystemState &state;

		// 上一次成功读到的编码器快照。Latest 读失败时沿用它，
		// 避免把默认构造的 0 角度发到上位机。
		EncoderSnapshot last_encoder;
	};
}
