#pragma once

#include "FOC_position.h"
#include "Framework/encoder_frame.h"
#include <stdint.h>

/*==============================================================================
 * MT6701CT 磁编码器帧解码 —— 纯算法层
 *
 *   Platform/RC_encoder 负责 SPI/DMA/CS 和环形缓冲，只把
 *   "{24bit 原始帧, DWT 时间戳}" 交给这里；本文件只做协议解析：
 *
 *     3 字节 = 24bit
 *       D[23:10]  14bit 绝对角度  angle = (rx[0] << 6) | (rx[1] >> 2)
 *       D[9:6]     4bit 磁场强度状态
 *       D[5:0]     6bit CRC6（多项式 0x43，覆盖 D[23:6] 共 18bit）
 *
 *   速度用相邻帧的真实时间戳算，不受任务抖动影响。
 *============================================================================*/

#define ENCODER_CPR				16384.0f	// 14bit = 16384 线
#define ENCODER_CRC_POLY		0x43		// x^6 + x + 1

namespace mt6701
{
	using Frame = framework::EncoderFrame;

	class Sensor : public position_sensor
	{
	public:
		Sensor();
		virtual ~Sensor() {}

		void Set_Pole_Pairs(uint8_t pole_pairs_) { pole_pairs = pole_pairs_; }

		// 速度二阶巴特沃斯低通（按监控任务 1kHz 的采样率设计）
		void Set_Speed_Filter_Hz(float fc_hz_);

		// 速度锁相环：用角度闭环估计速度（机械 rad/s）
		void Set_Pll_Bandwidth(float hz_, float damping_);
		float Get_Pll_Speed() const { return pll_omega; }	// 机械 rad/s
		void Set_Direction(float direction_);		// +1 / -1，编码器与电机相序方向相反时用
		void Set_Clock(uint32_t cycles_per_second_);

		// 电角度零位偏移：电机 d 轴对准定子电角度 0 后，
		// offset = -编码器原始电角度。控制环使用 angle = raw + offset。
		void Set_Electrical_Offset(float offset_rad_);
		float Get_Electrical_Offset() const { return elec_offset; }
		float Get_Electrical_Offset_Deg() const;
		float Get_Electrical_Angle_Raw() const { return elec_angle_raw; }

		// 消费一帧（由 App 的监控任务调用，任务上下文）
		bool Consume(const Frame &frame_);

		// 电流环中断里调用：用最近一帧的角度 + 电角速度外推到现在。
		// 只是乘加，不解析帧、不碰 SPI；序号锁保证读到一致的 angle/speed/时间戳。
		float Get_Predicted_Angle(uint32_t now_cycles) const;

		// 每个控制周期调用；编码器的角度已经由 Consume() 算好，
		// 这里只处理"请求清零"这类跨任务的小动作。
		void Position_Update(float dt) override;

		void Request_Reset_Angle() { reset_requested = true; }

		bool Is_Valid() const { return valid; }
		uint16_t Get_Raw_Angle() const { return raw_angle; }
		uint32_t Get_Last_Word() const { return last_word; }
		uint16_t Get_Crc_Errors() const { return crc_err_cnt; }
		uint16_t Get_Error_Count() const { return error_cnt; }

		// 累计机械角（连续，可超过 ±360°，方向已按 direction 修正）
		float Get_Mech_Angle_Deg() const { return mech_deg; }
		float Get_Mech_Angle_Rad() const { return mech_deg * (foc_math::PI / 180.0f); }

		// 单圈机械角 [-π, π)，和上位机显示的 enc_deg 同一套坐标（累计角折算）。
		// 位置环用它算误差：目标/反馈口径一致，归一化也是 O(1)。
		float Get_Mech_Angle_Wrapped_Rad() const
		{
			return foc_math::Norm_Angle(mech_deg * (foc_math::PI / 180.0f));
		}

		float Get_Mech_Rpm() const;

	private:
		static uint8_t Crc6_Calc(uint32_t data_);
		void Process_Frame(const Frame &frame_);
		void Reset_Internal();
		void Reset_Angle_State();

		uint32_t clock_hz = 170000000u;

		uint16_t raw_angle = 0;
		float mech_deg = 0.0f;
		float elec_angle_raw = 0.0f;
		float elec_offset = 0.0f;
		uint32_t last_word = 0;
		int32_t last_angle = 0;
		uint32_t last_cycles = 0;
		volatile uint32_t angle_cycles = 0;		// 最近一次角度更新对应的 DWT 时间戳
		volatile uint32_t angle_seq = 0;		// 奇数 = 正在更新角度/速度/时间戳
		uint8_t reject_cnt = 0;					// 连续被判为坏帧的次数
		float last_implied = 0.0f;				// 上一帧隐含速度（原始计数/秒）
		bool has_implied = false;
		bool started = false;
		uint8_t pole_pairs = 7;

		// 速度二阶巴特沃斯低通（biquad）
		float sp_b0 = 1.0f, sp_b1 = 0.0f, sp_b2 = 0.0f;
		float sp_a1 = 0.0f, sp_a2 = 0.0f;
		float sp_x1 = 0.0f, sp_x2 = 0.0f;
		float sp_y1 = 0.0f, sp_y2 = 0.0f;

		// 速度 PLL 状态（机械 rad / rad/s）
		float pll_kp = 0.0f;
		float pll_ki = 0.0f;
		float pll_theta = 0.0f;
		float pll_omega = 0.0f;
		float pll_integral = 0.0f;

		float direction = 1.0f;
		uint16_t error_cnt = 0;
		uint16_t crc_err_cnt = 0;
		uint16_t plaus_err_cnt = 0;			// 帧可信度检查拒绝次数（不上报）
		bool valid = false;
		volatile bool reset_requested = false;
	};
}
