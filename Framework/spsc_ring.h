#pragma once

#include <stdint.h>

/*==============================================================================
 * 单生产者 / 单消费者环形队列（无锁）
 *
 *   生产者只写 head_，消费者只写 tail_，两边各自独占一个索引。
 *   队列满时 Try_Push 返回 false（整帧丢弃），队列空时 Try_Pop 返回 false。
 *
 *   使用约束：
 *     1. 只能有一个生产者上下文（例如 DMA 完成中断）
 *     2. 只能有一个消费者上下文（例如任务）
 *     3. N 必须是 2 的幂
 *============================================================================*/

namespace framework
{
	inline void Memory_Barrier()
	{
#if defined(__GNUC__) && (defined(__arm__) || defined(__ARM_ARCH))
		__asm volatile("dmb sy" ::: "memory");
#else
		__asm volatile("" ::: "memory");
#endif
	}

	template <typename T, uint32_t N>
	class SpscRing
	{
		static_assert(N > 1u && (N & (N - 1u)) == 0u, "SpscRing size must be a power of two");

	public:
		/* 生产者调用 */
		bool Try_Push(const T &value)
		{
			const uint32_t head = head_;

			if ((head - tail_) >= N) return false;

			buffer_[head & (N - 1u)] = value;
			Memory_Barrier();
			head_ = head + 1u;

			return true;
		}

		/* 消费者调用 */
		bool Try_Pop(T &value)
		{
			const uint32_t tail = tail_;

			if (tail == head_) return false;

			value = buffer_[tail & (N - 1u)];
			Memory_Barrier();
			tail_ = tail + 1u;

			return true;
		}

		uint32_t Size() const { return head_ - tail_; }
		bool Empty() const { return head_ == tail_; }
		bool Full() const { return (head_ - tail_) >= N; }

		/* 消费者调用：丢掉所有尚未消费的数据 */
		void Clear() { tail_ = head_; }

	private:
		T buffer_[N] = {};
		volatile uint32_t head_ = 0;
		volatile uint32_t tail_ = 0;
	};
}
