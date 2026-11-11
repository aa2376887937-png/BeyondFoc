#include "RC_encoder.h"
#include "cmsis_os.h"

namespace encoder
{
	SpiEncoder *SpiEncoder::encoder_list[4] = {nullptr};
	uint8_t SpiEncoder::encoder_num = 0;

	SpiEncoder::SpiEncoder(SPI_HandleTypeDef &hspi_) : hspi(&hspi_)
	{
		if (encoder_num < 4)
		{
			encoder_list[encoder_num] = this;
			encoder_num++;
		}
	}

	SpiEncoder *SpiEncoder::Find(SPI_HandleTypeDef *hspi_)
	{
		for (uint8_t i = 0; i < encoder_num; i++)
		{
			if (encoder_list[i] != nullptr && encoder_list[i]->hspi == hspi_)
			{
				return encoder_list[i];
			}
		}

		return nullptr;
	}

	void SpiEncoder::Start()
	{
		Start_Read();
	}

	/*------------------------------------------------------------------------------
	 * 监控任务调用（不是控制中断）
	 *
	 *   1. 看门狗：超过 10ms 没完成就强制收尾，避免 reading 永远卡住
	 *   2. 异常/空闲时重新启动；正常由 DMA 完成中断自启动下一帧
	 *------------------------------------------------------------------------------*/
	void SpiEncoder::Poll()
	{
		// 任务和 SPI/DMA 中断都可能操作 reading，这里用临界区避免两边同时启动
		taskENTER_CRITICAL();

		if (reading && ((DWT->CYCCNT - read_start_cycles) > (SystemCoreClock / 100u)))
		{
			HAL_GPIO_WritePin(ENC_CSN_GPIO_Port, ENC_CSN_Pin, GPIO_PIN_SET);
			reading = false;
			if (transport_err < 0xFFFF) transport_err++;
		}

		if (!reading)
		{
			Start_Read();
		}

		taskEXIT_CRITICAL();
	}

	void SpiEncoder::Start_Read()
	{
		if (reading) return;

		// 先占位再启动：DMA 完成中断可能在 HAL 调用返回前就触发，
		// 先置 reading 可避免中断里再次启动一帧。
		reading = true;

		HAL_GPIO_WritePin(ENC_CSN_GPIO_Port, ENC_CSN_Pin, GPIO_PIN_RESET);

		for (volatile uint32_t d = 0; d < 40; d++) { __NOP(); }		// ~1us @170MHz

		read_start_cycles = DWT->CYCCNT;

		if (HAL_SPI_TransmitReceive_DMA(hspi, tx_buf, rx_buf, 3) == HAL_OK)
		{
			return;
		}

		// DMA 启动失败：直接放弃这一帧，不再回退到阻塞式 SPI。
		// 下一轮 Poll() 会重新尝试，避免把监控任务卡住。
		reading = false;
		HAL_GPIO_WritePin(ENC_CSN_GPIO_Port, ENC_CSN_Pin, GPIO_PIN_SET);
		if (transport_err < 0xFFFF) transport_err++;
	}

	/*------------------------------------------------------------------------------
	 * DMA 完成（SPI 中断上下文）：只保存最新一帧，不做 CRC / 角度计算
	 *------------------------------------------------------------------------------*/
	void SpiEncoder::Push_Raw(const uint8_t *buf_, uint32_t cycles_)
	{
		Frame frame;
		frame.word = ((uint32_t)buf_[0] << 16) | ((uint32_t)buf_[1] << 8) | (uint32_t)buf_[2];
		frame.cycles = cycles_;

		latest.Publish(frame);
	}

	bool SpiEncoder::Try_Take_Frame(Frame &frame_)
	{
		// 只取最新一帧；读到一半被 DMA 中断打断就返回 false，下一轮再取
		if (!latest.Try_Read(frame_)) return false;

		// Latest 是"邮箱"不是队列：没有新帧时它会反复返回同一帧。
		// 监控任务 4kHz 消费，如果某一拍没有新帧（SPI 抖动/限速），
		// 重复消费同一帧会让 dt_s=0 -> 走 1ms 兜底、implied=0 -> 可信度
		// 检查误判为坏帧 -> 速度尖峰。这里用时间戳挡住重复帧。
		if (frame_.cycles == last_taken_cycles) return false;

		last_taken_cycles = frame_.cycles;
		return true;
	}

	void SpiEncoder::On_TxRx_Cplt(SPI_HandleTypeDef *hspi_)
	{
		SpiEncoder *self = Find(hspi_);
		if (self == nullptr) return;

		HAL_GPIO_WritePin(ENC_CSN_GPIO_Port, ENC_CSN_Pin, GPIO_PIN_SET);

		self->reading = false;
		self->Push_Raw(self->rx_buf, DWT->CYCCNT);

		// DMA 完成中断只负责启动下一帧，不做帧解析。
		// 编码器帧现在由 20kHz 电流环中断消费，帧产出限到 20kfps
		// （10.6MHz 下一帧约 4µs，占空比 ~8%），保证每拍都有新帧。
		static uint32_t last_start = 0;
		if ((DWT->CYCCNT - last_start) > (SystemCoreClock / 20000u))
		{
			last_start = DWT->CYCCNT;
			self->Start_Read();
		}
	}

	void SpiEncoder::On_Error(SPI_HandleTypeDef *hspi_)
	{
		SpiEncoder *self = Find(hspi_);
		if (self == nullptr) return;

		HAL_GPIO_WritePin(ENC_CSN_GPIO_Port, ENC_CSN_Pin, GPIO_PIN_SET);

		self->reading = false;
		if (self->transport_err < 0xFFFF) self->transport_err++;
	}
}

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	encoder::SpiEncoder::On_TxRx_Cplt(hspi);
}

extern "C" void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	encoder::SpiEncoder::On_Error(hspi);
}
