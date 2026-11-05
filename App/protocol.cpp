#include "protocol.h"
#include <string.h>

namespace protocol
{
	CommandParser::CommandParser()
	{
	}

	void CommandParser::Feed(const uint8_t *data_, uint16_t len_)
	{
		for (uint16_t i = 0; i < len_; i++)
		{
			// 缓冲满时丢掉最旧的字节，避免坏帧把解析器永久卡死
			if (length >= sizeof(buffer))
			{
				memmove(buffer, buffer + 1, sizeof(buffer) - 1);
				length = sizeof(buffer) - 1;
			}

			buffer[length++] = data_[i];
		}

		while (length >= 5)
		{
			// 找帧头
			if (!(buffer[0] == HEADER0 && buffer[1] == HEADER1))
			{
				memmove(buffer, buffer + 1, length - 1);
				length--;
				continue;
			}

			const uint8_t type = buffer[2];
			const uint8_t payload_len = buffer[3];

			if (type != CMD_TYPE || payload_len != CMD_PAYLOAD)
			{
				memmove(buffer, buffer + 2, length - 2);
				length -= 2;
				continue;
			}

			const uint16_t frame_len = (uint16_t)(4 + payload_len + 1);
			if (length < frame_len) break;			// 等后续字节

			uint8_t checksum = 0;
			for (uint16_t k = 2; k < 4 + payload_len; k++) checksum ^= buffer[k];

			if (checksum == buffer[4 + payload_len])
			{
				app::Command cmd;

				cmd.enable = (buffer[4] != 0);
				cmd.mode = buffer[5];

				int16_t iq_ma = 0;
				int16_t speed_rpm = 0;
				uint16_t flags = 0;
				int32_t position_cdeg = 0;
				int16_t id_ma = 0;

				memcpy(&iq_ma, &buffer[6], 2);
				memcpy(&speed_rpm, &buffer[8], 2);
				memcpy(&flags, &buffer[10], 2);
				memcpy(&position_cdeg, &buffer[12], 4);
				memcpy(&cmd.kp_d, &buffer[16], 4);
				memcpy(&cmd.ki_d, &buffer[20], 4);
				memcpy(&cmd.kp_q, &buffer[24], 4);
				memcpy(&cmd.ki_q, &buffer[28], 4);
				memcpy(&id_ma, &buffer[32], 2);
				memcpy(&cmd.speed_kp, &buffer[34], 4);
				memcpy(&cmd.speed_ki, &buffer[38], 4);
				memcpy(&cmd.pos_kp, &buffer[42], 4);
				memcpy(&cmd.pos_kd, &buffer[46], 4);
				cmd.test_type = buffer[50];
				memcpy(&cmd.test_amp, &buffer[51], 4);
				memcpy(&cmd.test_freq, &buffer[55], 4);

				cmd.iq_a = (float)iq_ma * 0.001f;
				cmd.id_a = (float)id_ma * 0.001f;
				cmd.speed_rpm = (float)speed_rpm;
				cmd.position_deg = (float)position_cdeg * 0.01f;
				cmd.calibrate = ((flags & FLAG_CAL_CURRENT) != 0u);
				cmd.reset_angle = ((flags & FLAG_RESET_ANGLE) != 0u);
				cmd.calibrate_angle = ((flags & FLAG_CAL_ANGLE) != 0u);
				cmd.tune_pi = ((flags & FLAG_TUNE_PI) != 0u);
				cmd.tune_speed = ((flags & FLAG_TUNE_SPEED_PI) != 0u);
				cmd.tune_pos = ((flags & FLAG_TUNE_POS_PI) != 0u);

				latest = cmd;
				ready = true;
			}

			memmove(buffer, buffer + frame_len, length - frame_len);
			length -= frame_len;
		}
	}

	bool CommandParser::Take(app::Command &out_)
	{
		if (!ready) return false;

		out_ = latest;
		ready = false;

		return true;
	}

	/*------------------------------------------------------------------------------
	 * 示波器数据帧
	 *
	 *   4..35   8 × float32：ia, ib, ic, da, db, dc, id, iq
	 *   36..37  uint16 vbus_mV
	 *   38..39  int16  speed_rpm
	 *   40..41  uint16 status
	 *   42..43  uint16 seq
	 *   44..49  uint16 raw_a / raw_b / raw_c
	 *   50..53  uint16 isr_avg_x10 / isr_max_x10
	 *   54..57  float  enc_deg
	 *   58..59  int16  enc_rpm
	 *   60..61  uint16 enc_crc_err
	 *   62..65  uint32 enc_raw_word
	 *   66..69  float  elec_offset_deg
	 *   70..73  float  kp_d
	 *   74..77  float  ki_d
	 *   78..81  float  kp_q
	 *   82..85  float  ki_q
	 *   86..87  uint16 offs_a
	 *   88..89  uint16 offs_b
	 *   90..91  uint16 offs_c
	 *   92..95  float  iq_target
	 *   96..99  float  id_target
	 *   100..103 float speed_kp
	 *   104..107 float speed_ki
	 *   108..111 float pos_kp
	 *   112..115 float pos_kd
	 *   116..119 float speed_target_rpm
	 *   120..123 float pos_target_deg
	 *   124..127 float test_freq_now
	 *   128..131 float vd
	 *   132..135 float vq
	 *   136..137 uint16 ld_x10  (0.1µH)
	 *   138..139 uint16 lq_x10  (0.1µH)
	 *   140      XOR(2..139)
	 *------------------------------------------------------------------------------*/
	uint16_t Encode_Scope(const app::ScopeData &data_, uint8_t *out_, uint16_t capacity_)
	{
		if (out_ == nullptr || capacity_ < SCOPE_FRAME_SIZE) return 0;

		static uint16_t seq = 0;

		uint8_t *frame = out_;
		frame[0] = HEADER0;
		frame[1] = HEADER1;
		frame[2] = SCOPE_TYPE;
		frame[3] = SCOPE_PAYLOAD;

		// 任何 NaN/Inf 都替换成 0：上位机一旦收到非有限值，曲线会整个废掉
		float values[8] = {
			data_.ia, data_.ib, data_.ic,
			data_.da, data_.db, data_.dc,
			data_.id, data_.iq
		};
		for (int i = 0; i < 8; i++)
		{
			const float v = values[i];
			if (!(v > -1.0e30f && v < 1.0e30f)) values[i] = 0.0f;
		}
		memcpy(&frame[4], values, sizeof(values));					// 4..35

		const float vb = (data_.vbus > 0.0f && data_.vbus < 60.0f) ? data_.vbus : 0.0f;
		const uint16_t vbus_mv = (uint16_t)(vb * 1000.0f);

		memcpy(&frame[36], &vbus_mv, 2);
		memcpy(&frame[38], &data_.speed_rpm, 2);
		memcpy(&frame[40], &data_.status, 2);
		memcpy(&frame[42], &seq, 2);
		memcpy(&frame[44], &data_.raw_a, 2);
		memcpy(&frame[46], &data_.raw_b, 2);
		memcpy(&frame[48], &data_.raw_c, 2);
		memcpy(&frame[50], &data_.isr_avg_x10, 2);
		memcpy(&frame[52], &data_.isr_max_x10, 2);
		memcpy(&frame[54], &data_.enc_deg, 4);
		memcpy(&frame[58], &data_.enc_rpm, 2);
		memcpy(&frame[60], &data_.enc_crc_err, 2);
		memcpy(&frame[62], &data_.enc_raw_word, 4);
		memcpy(&frame[66], &data_.elec_offset_deg, 4);
		memcpy(&frame[70], &data_.kp_d, 4);
		memcpy(&frame[74], &data_.ki_d, 4);
		memcpy(&frame[78], &data_.kp_q, 4);
		memcpy(&frame[82], &data_.ki_q, 4);
		memcpy(&frame[86], &data_.offs_a, 2);
		memcpy(&frame[88], &data_.offs_b, 2);
		memcpy(&frame[90], &data_.offs_c, 2);
		memcpy(&frame[92], &data_.iq_target, 4);
		memcpy(&frame[96], &data_.id_target, 4);
		memcpy(&frame[100], &data_.speed_kp, 4);
		memcpy(&frame[104], &data_.speed_ki, 4);
		memcpy(&frame[108], &data_.pos_kp, 4);
		memcpy(&frame[112], &data_.pos_kd, 4);
		memcpy(&frame[116], &data_.speed_target_rpm, 4);
		memcpy(&frame[120], &data_.pos_target_deg, 4);
		memcpy(&frame[124], &data_.test_freq_now, 4);
		memcpy(&frame[128], &data_.vd, 4);
		memcpy(&frame[132], &data_.vq, 4);
		memcpy(&frame[136], &data_.ld_x10, 2);
		memcpy(&frame[138], &data_.lq_x10, 2);
		seq++;

		uint8_t checksum = 0;
		for (uint16_t i = 2; i < 4u + SCOPE_PAYLOAD; i++) checksum ^= frame[i];
		frame[4u + SCOPE_PAYLOAD] = checksum;

		return SCOPE_FRAME_SIZE;
	}
}
