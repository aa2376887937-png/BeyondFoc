#pragma once

#include <stdint.h>

/*==============================================================================
 * 板内高速捕获
 *
 *   中断里按控制节拍把 4 路信号写进缓冲（电流环 20kHz / 转速环 4kHz），
 *   写满后置 ready，遥测任务再分块读出（每帧 8 点 × 4 路）。
 *
 *   写入阶段和读出阶段不重叠：armed 期间 ready=false，写满后 armed=false，
 *   所以读出过程中没有生产者，不需要加锁。
 *============================================================================*/
namespace app
{
	struct CaptureBuffer
	{
		static constexpr uint16_t POINTS = 1024;
		static constexpr uint8_t CHANNELS = 4;

		float data[POINTS][CHANNELS] = {};

		volatile uint16_t len = 0;
		volatile bool armed = false;
		volatile bool ready = false;

		// 自适应抽取：按当前扫频频率选 1/N 抽取，保证窗口覆盖足够周期
		uint16_t decim = 1;			// 抽取比（1 = 不抽）
		uint16_t decim_cnt = 0;
		uint16_t fs = 20000;		// 原始采样率 (Hz)

		void Arm(uint16_t decim_, uint16_t fs_)
		{
			len = 0;
			ready = false;
			decim = (decim_ > 0u) ? decim_ : 1u;
			decim_cnt = 0;
			fs = (fs_ > 0u) ? fs_ : 20000u;
			armed = true;
		}

		// 中断里调用：4 路信号各写一个点
		void Push(float ch0, float ch1, float ch2, float ch3)
		{
			if (!armed) return;

			if (++decim_cnt < decim) return;
			decim_cnt = 0;

			const uint16_t i = len;

			data[i][0] = ch0;
			data[i][1] = ch1;
			data[i][2] = ch2;
			data[i][3] = ch3;

			const uint16_t n = (uint16_t)(i + 1u);
			len = n;

			if (n >= POINTS)
			{
				armed = false;
				ready = true;
			}
		}
	};
}
