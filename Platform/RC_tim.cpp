#include "RC_tim.h"

// C语言接口（.c文件不支持c++语法）（放入hal库定时器中断函数中）
extern "C" void All_Tim_It_Process(TIM_HandleTypeDef *htim)
{
	tim::Tim::All_Tim_It_Process(htim);
}

namespace tim
{
	uint8_t Tim::tim_num = 0;// 初始化定时器数量
	Tim *Tim::tim_list[MAX_TIM_NUM] = {nullptr};// 初始化定时器指针

	// 新增定时器对象
	Tim::Tim(TIM_HandleTypeDef &htim_) : htim(&htim_)
	{
		taskENTER_CRITICAL();
		
		tim_num++;
		if (tim_num > MAX_TIM_NUM) Error_Handler();

		tim_list[tim_num - 1] = this;
		
		taskEXIT_CRITICAL();
	}

	// 所有定时器中断函数
	void Tim::All_Tim_It_Process(TIM_HandleTypeDef *htim)
	{
		for (uint8_t i = 0; i < tim_num; i++)
		{
			if (tim_list[i]->htim == htim)
			{
				tim_list[i]->handlers.Notify([](TimHandler *handler)
				{
					handler->Tim_It_Process();
				});
				return;// 退出中断
			}
		}
	}

	// 新增处理对象
	void Tim::Add_TimHandle(TimHandler *hd) 
	{
		if (!handlers.Attach(hd)) Error_Handler();
	}


	/*------------------------------------------------------------*/
	
	TimHandler::TimHandler(Tim* tim_)
	{
		if (tim_ != nullptr)
		{
			tim_->Add_TimHandle(this);
		}
	}
}