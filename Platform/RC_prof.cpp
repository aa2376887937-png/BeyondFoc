#include "RC_prof.h"
#include "main.h"

namespace
{
	volatile unsigned long sum_cycles = 0;
	volatile unsigned long sample_cnt = 0;
	volatile unsigned long max_cycles = 0;

	unsigned long begin_cycles = 0;
	volatile bool prof_ready = false;
}

extern "C" void prof_init(void)
{
	/* 打开 DWT 周期计数器（Cortex-M4 自带，不用额外外设） */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	sum_cycles = 0;
	sample_cnt = 0;
	max_cycles = 0;
	prof_ready = true;
}

extern "C" void prof_isr_begin(void)
{
	if (!prof_ready) return;
	begin_cycles = DWT->CYCCNT;
}

extern "C" void prof_isr_end(void)
{
	if (!prof_ready) return;

	const unsigned long d = DWT->CYCCNT - begin_cycles;

	sum_cycles += d;
	sample_cnt++;
	if (d > max_cycles) max_cycles = d;
}

extern "C" void prof_take(unsigned short *avg_x10_, unsigned short *max_x10_)
{
	/* cycles -> us：SystemCoreClock / 1e6 就是每 us 的周期数 */
	float clk_per_us = (float)SystemCoreClock / 1000000.0f;
	if (clk_per_us < 1.0f) clk_per_us = 1.0f;

	float avg_us = 0.0f;
	float max_us = 0.0f;

	if (sample_cnt > 0)
	{
		avg_us = (float)sum_cycles / (float)sample_cnt / clk_per_us;
		max_us = (float)max_cycles / clk_per_us;
	}

	sum_cycles = 0;
	sample_cnt = 0;
	max_cycles = 0;

	if (avg_x10_ != nullptr) *avg_x10_ = (unsigned short)(avg_us * 10.0f + 0.5f);
	if (max_x10_ != nullptr) *max_x10_ = (unsigned short)(max_us * 10.0f + 0.5f);
}
