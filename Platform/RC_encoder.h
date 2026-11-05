#pragma once

#include "main.h"
#include "Framework/encoder_frame.h"
#include "Framework/latest.h"
#include <stdint.h>

/*==============================================================================
 * MT6701 的 SPI 传输层
 *
 *   只负责硬件时序：
 *     CS 拉低 -> 等 T_H -> SPI 全双工 DMA 收发 3 字节 -> 完成中断拉高 CS
 *
 *   数据流（生产者/消费者）：
 *     DMA 完成中断（生产者） -> Latest<Frame> -> 监控任务（消费者）
 *
 *   DMA 完成中断只保存最新一帧并启动下一帧，不做 CRC / 角度 / 速度计算；
 *   控制中断只读 MT6701 已经算好的最新角度/速度。
 *   协议解析在 Algorithm/MT6701。
 *============================================================================*/

namespace encoder
{
	using Frame = framework::EncoderFrame;

	class SpiEncoder
	{
	public:
		SpiEncoder(SPI_HandleTypeDef &hspi_);
		virtual ~SpiEncoder() {}

		void Start();								// 启动第一次读取
		void Poll();								// 监控任务：看门狗 + 异常/空闲时重启
		bool Try_Take_Frame(Frame &frame_);			// 取最新一帧；读写冲突返回 false

		uint16_t Get_Transport_Errors() const { return transport_err; }

		static void On_TxRx_Cplt(SPI_HandleTypeDef *hspi_);
		static void On_Error(SPI_HandleTypeDef *hspi_);

	private:
		static SpiEncoder *Find(SPI_HandleTypeDef *hspi_);

		void Start_Read();
		void Push_Raw(const uint8_t *buf_, uint32_t cycles_);

		SPI_HandleTypeDef *hspi;

		uint8_t tx_buf[3] = {0xFF, 0xFF, 0xFF};
		uint8_t rx_buf[3] = {0, 0, 0};

		// 生产者 = SPI DMA 完成中断，消费者 = Monitor 任务
		framework::Latest<Frame> latest;

		volatile bool reading = false;
		uint32_t read_start_cycles = 0;
		uint16_t transport_err = 0;

		static SpiEncoder *encoder_list[4];
		static uint8_t encoder_num;
	};
}
