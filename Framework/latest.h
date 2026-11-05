#pragma once

#include <stdint.h>
#include "spsc_ring.h"

/*==============================================================================
 * 单生产者 / 多消费者"最新值"邮箱
 *
 *   适合遥测这类"只关心最新值、旧值可以覆盖"的数据：
 *     生产者（控制任务 / 监控任务）Publish()
 *     消费者（通信任务）Try_Read()
 *
 *   用序号锁（seqlock）保证读者不会读到写了一半的数据：
 *     写之前 seq 变奇数，写完再变偶数；读者发现前后 seq 不一致就重读。
 *
 *   约束：只能有一个生产者，可以有多个读者。
 *============================================================================*/

namespace framework
{
	template <typename T>
	class Latest
	{
	public:
		/* 生产者调用 */
		void Publish(const T &value)
		{
			seq_ = seq_ + 1u;			// 奇数：正在写
			Memory_Barrier();
			value_ = value;
			Memory_Barrier();
			seq_ = seq_ + 1u;			// 偶数：写完成
		}

		/* 消费者调用；读取期间被生产者打断时返回 false */
		bool Try_Read(T &out) const
		{
			const uint32_t seq_first = seq_;

			if ((seq_first & 1u) != 0u) return false;

			Memory_Barrier();
			out = value_;
			Memory_Barrier();

			return seq_first == seq_;
		}

		// 注意：不要在这里做"重试到成功"的阻塞读。
		// 如果读者优先级高于生产者，读者可能把正在写的生产者饿死。
		// 读失败时让调用者跳过这一帧即可。

		bool Is_Valid() const { return (seq_ >= 2u) && ((seq_ & 1u) == 0u); }

	private:
		T value_ = {};
		volatile uint32_t seq_ = 0;
	};
}
