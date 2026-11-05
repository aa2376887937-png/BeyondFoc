#pragma once
#include "main.h"

#ifdef __cplusplus

/*==============================================================================
 * 板载按键 / LED
 *
 * SCH_iRDT_Driver_V1.4:
 *   KEY1~KEY4 = PC6/PC7/PC8/PC9，10k 上拉到 VCC + 100nF，按下为低电平
 *   LED1~LED3 = PC13/PC14/PC15，高电平点亮（1k 限流）
 *============================================================================*/

namespace io
{
	// 编号从 1 开始，和原理图上的丝印一致
	bool Key_Read(uint8_t key_);			// true = 按下
	bool Key_Is_Pressed(uint8_t key_);

	void Led_Set(uint8_t led_, bool on_);
	void Led_Toggle(uint8_t led_);
	void Led_All_Off();
}

#endif
