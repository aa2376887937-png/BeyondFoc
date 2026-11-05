#pragma once

#include <stdint.h>

/*==============================================================================
 * 静态观察者列表（tim 模块的对象挂载方式）
 *
 *   tim::Tim 的做法是：
 *     1. 外设对象维护一个固定容量的处理对象数组
 *     2. 处理对象在构造时把自己 Attach 上去
 *     3. 中断到来时由外设对象统一分发
 *
 *   这里把同样的模式抽成通用模板，新的外设/数据源不用再各写一套
 *   static list + count + for 循环。
 *============================================================================*/

namespace framework
{
	template <typename Handler, uint8_t Capacity>
	class Subject
	{
	public:
		bool Attach(Handler *handler)
		{
			if (handler == nullptr || count_ >= Capacity) return false;

			handlers_[count_] = handler;
			count_ = count_ + 1u;

			return true;
		}

		template <typename Function>
		void Notify(Function &&function)
		{
			const uint8_t count = count_;

			for (uint8_t i = 0; i < count; i++)
			{
				function(handlers_[i]);
			}
		}

		uint8_t Count() const { return count_; }

	private:
		Handler *handlers_[Capacity] = {};
		volatile uint8_t count_ = 0;
	};
}
