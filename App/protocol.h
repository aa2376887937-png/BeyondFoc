#pragma once

#include "app_messages.h"
#include "capture.h"
#include <stdint.h>

/*==============================================================================
 * 上位机串口协议（和 tools/serial_scope.py 保持一致）
 *
 *   A5 5A | type(1) | len(1) | payload | XOR(1)
 *
 *   type=0x01 (MCU->PC)：示波器数据（136 字节 payload）
 *   type=0x10 (PC->MCU)：控制命令（55 字节 payload）
 *
 *   本文件只做"字节 <-> 结构体"，不碰串口外设，也不碰控制逻辑。
 *============================================================================*/

namespace protocol
{
	constexpr uint8_t HEADER0 = 0xA5;
	constexpr uint8_t HEADER1 = 0x5A;

	constexpr uint8_t SCOPE_TYPE = 0x01;
	constexpr uint8_t SCOPE_PAYLOAD = 145;
	constexpr uint16_t SCOPE_FRAME_SIZE = 4u + SCOPE_PAYLOAD + 1u;		// 150

	// 板内捕获分块：type=0x02，每帧 8 点 × 4 路 float
	constexpr uint8_t CAPTURE_TYPE = 0x02;
	constexpr uint8_t CAPTURE_PAYLOAD = 140;
	constexpr uint16_t CAPTURE_FRAME_SIZE = 4u + CAPTURE_PAYLOAD + 1u;
	constexpr uint8_t CAPTURE_CHUNK = 8;

	constexpr uint8_t CMD_TYPE = 0x10;
	// 63 原有字段 + MIT 6 个 float：pos_des / vel_des / kp / kd / tau_ff / tau_limit
	constexpr uint8_t CMD_PAYLOAD = 87;

	// 命令 flags 位定义（和 tools/serial_scope.py 保持一致）
	constexpr uint16_t FLAG_CAL_CURRENT = 0x0001;
	constexpr uint16_t FLAG_RESET_ANGLE = 0x0002;
	constexpr uint16_t FLAG_CAL_ANGLE = 0x0004;
	constexpr uint16_t FLAG_TUNE_PI = 0x0008;		// 电流环 Kp/Ki 在线整定有效
	constexpr uint16_t FLAG_PI_INCREMENTAL = 0x0010;	// PI 用增量式（0 = 位置式）
	constexpr uint16_t FLAG_TUNE_SPEED_PI = 0x0020;	// 转速环 Kp/Ki 在线整定有效
	constexpr uint16_t FLAG_TUNE_POS_PI = 0x0040;	// 位置环 Kp/Ki 在线整定有效
	constexpr uint16_t FLAG_CAPTURE = 0x0080;		// 触发一次板内捕获
	constexpr uint16_t FLAG_L_IDENT = 0x0100;		// 触发一次 Ld/Lq 辨识
	constexpr uint16_t FLAG_COG_LEARN = 0x0200;		// 触发一次齿槽建表
	constexpr uint16_t FLAG_COG_ENABLE = 0x0400;	// 齿槽补偿使能（电平）
	constexpr uint16_t FLAG_COG_INVERT = 0x0800;	// 齿槽补偿反号（电平，调试用）

	class CommandParser
	{
	public:
		CommandParser();

		void Feed(const uint8_t *data_, uint16_t len_);
		bool Take(app::Command &out_);

	private:
		uint8_t buffer[256] = {};
		uint16_t length = 0;
		app::Command latest;
		bool ready = false;
	};

	// 返回写入的字节数，0 表示输出缓冲不够
	uint16_t Encode_Scope(const app::ScopeData &data_, uint8_t *out_, uint16_t capacity_);

	// 板内捕获分块编码：seq 从 0 开始，每帧 CAPTURE_CHUNK 个点
	uint16_t Encode_Capture(const app::CaptureBuffer &cap_, uint16_t seq_,
							uint8_t *out_, uint16_t capacity_);
}
