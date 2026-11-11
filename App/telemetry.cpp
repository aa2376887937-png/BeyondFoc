#include "telemetry.h"
#include "protocol.h"

namespace app
{
	Telemetry::Telemetry(uart::Port &port_,
						 const framework::Latest<ControlSnapshot> &control_,
						 const framework::Latest<EncoderSnapshot> &encoder_,
						 const framework::Latest<PowerSnapshot> &power_,
						 const SystemState &state_,
						 CaptureBuffer &capture_)
		: port(port_)
		, control(control_)
		, encoder(encoder_)
		, power(power_)
		, state(state_)
		, capture(capture_)
	{
	}

	void Telemetry::Send()
	{
		// 板内捕获优先：分块发送期间暂停普通遥测。
		// 1024 点 / 每帧 8 点 = 128 帧 = 128ms，期间波形不动是正常的。
		if (capture.ready)
		{
			uint8_t frame[protocol::CAPTURE_FRAME_SIZE];
			const uint16_t len = protocol::Encode_Capture(capture, cap_seq, frame, sizeof(frame));

			if (len != 0)
			{
				port.Write(frame, len);
				cap_seq++;

				if ((uint32_t)cap_seq * protocol::CAPTURE_CHUNK >= capture.len)
				{
					capture.ready = false;
					cap_seq = 0;
				}
				return;
			}

			capture.ready = false;
			cap_seq = 0;
		}

		ControlSnapshot c;
		if (!control.Try_Read(c) || !c.valid) return;

		EncoderSnapshot e;
		if (encoder.Try_Read(e))
		{
			last_encoder = e;
		}
		else
		{
			// Latest 是 seqlock：监控任务正在发布时会读失败。
			// 不能把默认构造的 0 值发出去，否则上位机会看到角度突然变 0。
			e = last_encoder;
		}
		PowerSnapshot p;
		const bool power_ok = power.Try_Read(p);

		ScopeData data;

		data.ia = c.current.a;
		data.ib = c.current.b;
		data.ic = c.current.c;

		data.da = c.duty.a;
		data.db = c.duty.b;
		data.dc = c.duty.c;

		data.id = c.i_dq.d;
		data.iq = c.i_dq.q;

		data.vbus = power_ok ? p.v_bus : c.v_bus;
		data.speed_rpm = (int16_t)c.speed_rpm;
		data.speed_target_rpm = c.speed_target_rpm;
		data.pos_target_deg = c.pos_target_deg;
		data.test_freq_now = c.test_freq_now;
		data.vd = c.vd;
		data.vq = c.vq;
		data.ld_x10 = (uint16_t)((c.ld_h * 1.0e7f) + 0.5f);		// H → 0.1µH
		data.lq_x10 = (uint16_t)((c.lq_h * 1.0e7f) + 0.5f);
		data.motor_temp = c.motor_temp;
		data.driver_temp = c.driver_temp;
		data.thermal_state = c.thermal_state;
		data.psi_est_mwb = c.psi_est_mwb;
		data.psi_state = c.psi_state;
		data.cog_state = c.cog_state;

		uint16_t status = 0;
		if (c.output_enable) status |= 0x01;
		if (c.speed_loop) status |= 0x02;			// bit1：转速环运行中
		if (e.valid) status |= 0x04;
		if (c.calibrating) status |= 0x08;
		if (state.adc_cal_failed) status |= 0x10;
		if (c.cal_status == 1) status |= 0x20;		// 电角度标定中
		if (c.cal_status == 2) status |= 0x40;		// 电角度标定完成
		if (c.cal_status == 3) status |= 0x80;		// 电角度标定失败
		status |= (uint16_t)((state.boot_stage & 0x0F) << 8);
		status |= (uint16_t)((c.mode & 0x07) << 12);	// bit12-14：固件实际模式
		if (c.cmd_ok) status |= 0x8000;					// bit15：最近 200ms 收到命令
		data.status = status;

		data.raw_a = (uint16_t)c.raw.a;
		data.raw_b = (uint16_t)c.raw.b;
		data.raw_c = (uint16_t)c.raw.c;

		data.isr_avg_x10 = c.isr_avg_x10;
		data.isr_max_x10 = c.isr_max_x10;

		data.enc_deg = e.mech_deg;
		data.enc_rpm = (int16_t)e.mech_rpm;
		data.enc_crc_err = e.crc_errors;
		data.enc_raw_word = e.raw_word;
		data.elec_offset_deg = e.elec_offset_deg;

		data.iq_target = c.iq_target;
		data.id_target = c.id_target;
		data.speed_kp = c.speed_kp;
		data.speed_ki = c.speed_ki;
		data.pos_kp = c.pos_kp;
		data.pos_kd = c.pos_kd;

		data.kp_d = c.kp_d;
		data.ki_d = c.ki_d;
		data.kp_q = c.kp_q;
		data.ki_q = c.ki_q;
		data.offs_a = c.offs_a;
		data.offs_b = c.offs_b;
		data.offs_c = c.offs_c;

		uint8_t frame[protocol::SCOPE_FRAME_SIZE];
		const uint16_t length = protocol::Encode_Scope(data, frame, sizeof(frame));

		if (length != 0)
		{
			port.Write(frame, length);
		}
	}
}
