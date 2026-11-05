#pragma once

#include "app_messages.h"
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
	constexpr uint8_t SCOPE_PAYLOAD = 136;
	constexpr uint16_t SCOPE_FRAME_SIZE = 4u + SCOPE_PAYLOAD + 1u;		// 141

	constexpr uint8_t CMD_TYPE = 0x10;
	constexpr uint8_t CMD_PAYLOAD = 55;

	// 命令 flags 位定义（和 tools/serial_scope.py 保持一致）
	constexpr uint16_t FLAG_CAL_CURRENT = 0x0001;
	constexpr uint16_t FLAG_RESET_ANGLE = 0x0002;
	constexpr uint16_t FLAG_CAL_ANGLE = 0x0004;
	constexpr uint16_t FLAG_TUNE_PI = 0x0008;		// 电流环 Kp/Ki 在线整定有效
	constexpr uint16_t FLAG_TUNE_SPEED_PI = 0x0020;	// 转速环 Kp/Ki 在线整定有效
	constexpr uint16_t FLAG_TUNE_POS_PI = 0x0040;	// 位置环 Kp/Ki 在线整定有效

	class CommandParser
	{
	public:
		CommandParser();

		void Feed(const uint8_t *data_, uint16_t len_);
		bool Take(app::Command &out_);

	private:
		uint8_t buffer[96] = {};
		uint16_t length = 0;
		app::Command latest;
		bool ready = false;
	};

	// 返回写入的字节数，0 表示输出缓冲不够
	uint16_t Encode_Scope(const app::ScopeData &data_, uint8_t *out_, uint16_t capacity_);
}
