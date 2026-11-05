#include "command_router.h"

namespace app
{
	CommandRouter::CommandRouter(uart::Port &port_, framework::SpscRing<Command, 8> &queue_)
		: port(port_)
		, queue(queue_)
	{
	}

	void CommandRouter::Poll()
	{
		uint8_t buffer[32];
		uint16_t length;

		while ((length = port.Read(buffer, sizeof(buffer))) > 0)
		{
			parser.Feed(buffer, length);
		}

		Command cmd;
		while (parser.Take(cmd))
		{
			cmd.sequence = ++sequence;

			if (!queue.Try_Push(cmd))
			{
				// 队列满：丢弃这一条，下一帧（50ms 后）会再发。
				// 控制任务 1kHz 消费，正常情况下不会走到这里。
			}
		}
	}
}
