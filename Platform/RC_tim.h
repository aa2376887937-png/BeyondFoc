#pragma once
#include "main.h"
#include "cmsis_os.h"

#ifdef __cplusplus
#include "Framework/subject.h"
#endif

#ifdef __cplusplus

#define MAX_TIM_HANDLE_NUM 23
#define MAX_TIM_NUM 13

namespace tim
{
	class TimHandler;// 向前声明
	
	class Tim
	{
	public:
		Tim(TIM_HandleTypeDef &htim_);
		virtual ~Tim() {}
			
		// 开启定时器中断
		inline void Tim_It_Start()
		{
			if (HAL_TIM_Base_Start_IT(htim) != HAL_OK) Error_Handler();
		}

		inline void Tim_It_Stop()
		{
			if (HAL_TIM_Base_Stop_IT(htim) != HAL_OK) Error_Handler();
		}
	
		void Add_TimHandle(TimHandler *hd);
		
		static void All_Tim_It_Process(TIM_HandleTypeDef *htim);
		
		TIM_HandleTypeDef *htim;
		
	protected:
		
	private:
		static uint8_t tim_num;// 定时器数量
		static Tim *tim_list[MAX_TIM_NUM];// 所有定时器

		framework::Subject<TimHandler, MAX_TIM_HANDLE_NUM> handlers;// 处理对象（tim 风格挂载）
	};

	class TimHandler
	{
	public:
		TimHandler(Tim* tim_);
		virtual ~TimHandler() {}
		
		virtual void Tim_It_Process() = 0;
	};
}
#endif

#ifdef __cplusplus
extern "C" {
#endif

void All_Tim_It_Process(TIM_HandleTypeDef *htim);// 放在main.c中的定时中断函数

#ifdef __cplusplus
}
#endif
