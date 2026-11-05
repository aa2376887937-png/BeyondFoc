#pragma once

#include <stdint.h>

/*==============================================================================
 * 编码器原始帧
 *
 *   Platform 的 SPI 驱动负责生产它，Algorithm 的 MT6701 解码器负责消费它。
 *   放在 Framework 里是为了让两层共享同一个数据类型，而不用互相包含。
 *============================================================================*/

namespace framework
{
	struct EncoderFrame
	{
		uint32_t word;		// 24bit 原始帧
		uint32_t cycles;	// 时间戳（DWT 周期计数）
	};
}
