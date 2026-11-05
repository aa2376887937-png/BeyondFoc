#include "RC_io.h"

namespace io
{
	namespace
	{
		// 按键：1~4 -> PC6~PC9
		GPIO_TypeDef *const key_port[4] = {KEY1_GPIO_Port, KEY2_GPIO_Port, KEY3_GPIO_Port, KEY4_GPIO_Port};
		const uint16_t key_pin[4] = {KEY1_Pin, KEY2_Pin, KEY3_Pin, KEY4_Pin};

		// LED：1~3 -> PC13~PC15
		GPIO_TypeDef *const led_port[3] = {LED1_GPIO_Port, LED2_GPIO_Port, LED3_GPIO_Port};
		const uint16_t led_pin[3] = {LED1_Pin, LED2_Pin, LED3_Pin};
	}

	bool Key_Read(uint8_t key_)
	{
		if (key_ < 1 || key_ > 4) return false;

		// 10k 上拉，按下接地 -> 低电平有效
		return HAL_GPIO_ReadPin(key_port[key_ - 1], key_pin[key_ - 1]) == GPIO_PIN_RESET;
	}

	bool Key_Is_Pressed(uint8_t key_)
	{
		return Key_Read(key_);
	}

	void Led_Set(uint8_t led_, bool on_)
	{
		if (led_ < 1 || led_ > 3) return;

		HAL_GPIO_WritePin(led_port[led_ - 1], led_pin[led_ - 1], on_ ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}

	void Led_Toggle(uint8_t led_)
	{
		if (led_ < 1 || led_ > 3) return;

		HAL_GPIO_TogglePin(led_port[led_ - 1], led_pin[led_ - 1]);
	}

	void Led_All_Off()
	{
		for (uint8_t i = 1; i <= 3; i++) Led_Set(i, false);
	}
}


/*==============================================================================
 * HardFault 处理：让板子自己"说出来"，不用调试器也能判断
 *
 *   正常 : LED1/LED3 1Hz 心跳
 *   硬fault : 三颗灯一起快闪 ~7Hz
 *
 * 故障状态机寄存器留在全局里，接调试器可以直接看：
 *   g_hf_cfsr / g_hf_hfsr / g_hf_mmfar / g_hf_bfar
 *============================================================================*/
extern "C" {

volatile uint32_t g_hf_cfsr = 0;
volatile uint32_t g_hf_hfsr = 0;
volatile uint32_t g_hf_mmfar = 0;
volatile uint32_t g_hf_bfar = 0;

}
