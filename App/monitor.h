#pragma once

#include "RC_analog.h"
#include "RC_encoder.h"
#include "MT6701.h"
#include "Framework/latest.h"
#include "app_messages.h"

/*==============================================================================
 * 慢速监控（应用层）
 *
 *   1kHz 任务：
 *     - 发布编码器快照（原始帧由 Monitor 任务消费，控制中断只读最新角度/速度）
 *     - 100Hz 读一次 ADC2（母线/NTC/相电压）
 *     - 把结果发布到 Latest 邮箱，供控制任务和遥测任务读取
 *
 *   本类不直接调用控制环，也不组装遥测帧。
 *============================================================================*/

namespace app
{
	class Monitor
	{
	public:
		Monitor(encoder::SpiEncoder &spi_,
				mt6701::Sensor &encoder_,
				analog::PowerMonitor &power_,
				framework::Latest<EncoderSnapshot> &encoder_out_,
				framework::Latest<PowerSnapshot> &power_out_);

		virtual ~Monitor() {}

		void Init();
		void Update(float dt);

		mt6701::Sensor &Encoder() { return encoder; }

	private:
		void Publish_Encoder();
		void Publish_Power();
		void Read_Power();

		encoder::SpiEncoder &spi;
		mt6701::Sensor &encoder;
		analog::PowerMonitor &power;

		framework::Latest<EncoderSnapshot> &encoder_out;
		framework::Latest<PowerSnapshot> &power_out;

		uint8_t power_div = 0;
		uint16_t heartbeat = 0;
	};
}
