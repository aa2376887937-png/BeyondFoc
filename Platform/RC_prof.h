#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 * FOC 中断耗时统计（用 DWT 周期计数器，分辨率 ~6ns @170MHz）
 *
 *   prof_init()      上电调一次
 *   prof_isr_begin() ADC 中断入口
 *   prof_isr_end()   ADC 中断出口
 *   prof_take()      1kHz 里取一次：返回 0.1us 为单位的平均值/最大值，并清窗口
 *============================================================================*/
void prof_init(void);
void prof_isr_begin(void);
void prof_isr_end(void);
void prof_take(unsigned short *avg_x10_, unsigned short *max_x10_);

#ifdef __cplusplus
}
#endif
