#include "monitor.h"
#include "RC_io.h"
#include "app_config.h"

namespace app
{
	Monitor::Monitor(encoder::SpiEncoder &spi_,
					 mt6701::Sensor &encoder_,
					 analog::PowerMonitor &power_,
					 framework::Latest<EncoderSnapshot> &encoder_out_,
					 framework::Latest<PowerSnapshot> &power_out_)
		: spi(spi_)
		, encoder(encoder_)
		, power(power_)
		, encoder_out(encoder_out_)
		, power_out(power_out_)
	{
	}

	void Monitor::Init()
	{
		encoder.Set_Pole_Pairs(config::kPolePairs);
		encoder.Set_Direction(config::kEncoderDirection);
		encoder.Set_Pll_Bandwidth(config::kPllBandwidthHz, config::kPllDamping);
		encoder.Set_Clock(SystemCoreClock);

		if (config::kEncoderEnable)
		{
			spi.Start();
		}

		// 配置 ADC2 的 DMA，之后 Update() 只启动/消费，不再忙等
		power.Init();

		// 先给遥测一个有效快照，避免上电初期通信任务读到空数据
		Publish_Encoder();
		Read_Power();
	}

	void Monitor::Update(float dt)
	{
		(void)dt;

		// 编码器帧在任务里消费：DMA 只保存最新帧，控制中断只读最新角度/速度。
		if (config::kEncoderEnable)
		{
			encoder::Frame frame;
			if (spi.Try_Take_Frame(frame))
			{
				encoder.Consume(frame);
			}

			spi.Poll();			// 看门狗 + 异常时重启
		}

		// 1. 100Hz 慢速模拟量（监控任务 4kHz，40 次 = 10ms）
		if (++power_div >= 40)
		{
			power_div = 0;
			Read_Power();
		}

		// 2. 发布编码器快照
		Publish_Encoder();

		// 5. 任务活体指示：LED3 500ms 翻转（监控任务 4kHz → 2000 次）
		if (++heartbeat >= 2000)
		{
			heartbeat = 0;
			io::Led_Toggle(3);
		}
	}

	void Monitor::Read_Power()
	{
		power.Update();
		Publish_Power();
	}

	void Monitor::Publish_Encoder()
	{
		EncoderSnapshot snapshot;
		snapshot.valid = encoder.Is_Valid();
		snapshot.mech_deg = encoder.Get_Mech_Angle_Deg();
		snapshot.mech_rpm = encoder.Get_Mech_Rpm();
		snapshot.crc_errors = encoder.Get_Crc_Errors();
		snapshot.transport_errors = spi.Get_Transport_Errors();
		snapshot.raw_word = encoder.Get_Last_Word();
		snapshot.elec_offset_deg = encoder.Get_Electrical_Offset_Deg();

		encoder_out.Publish(snapshot);
	}

	void Monitor::Publish_Power()
	{
		PowerSnapshot snapshot;

		snapshot.v_bus = power.Get_Bus_Voltage();
		snapshot.v_a = power.Get_Phase_Voltage_A();
		snapshot.v_b = power.Get_Phase_Voltage_B();
		snapshot.v_c = power.Get_Phase_Voltage_C();
		snapshot.motor_temp = power.Get_Motor_Temp();
		snapshot.driver_temp = power.Get_Driver_Temp();

		for (uint8_t i = 0; i < 6; i++)
		{
			snapshot.raw[i] = power.Get_Raw(i);
		}

		snapshot.valid = (snapshot.v_bus > 5.0f && snapshot.v_bus < 60.0f);

		power_out.Publish(snapshot);
	}
}
