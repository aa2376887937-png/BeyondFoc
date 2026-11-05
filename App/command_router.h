#pragma once

#include "RC_uart.h"
#include "Framework/spsc_ring.h"
#include "app_messages.h"
#include "protocol.h"

/*==============================================================================
 * 命令路由（应用层）
 *
 *   通信任务：
 *     UART 字节流 -> protocol::CommandParser -> SpscRing<Command>
 *
 *   控制任务：
 *     从队列里取命令并落地。
 *
 *   这样通信任务不会直接调用控制器/编码器，控制任务也不需要知道串口协议。
 *============================================================================*/

namespace app
{
	class CommandRouter
	{
	public:
		CommandRouter(uart::Port &port_, framework::SpscRing<Command, 8> &queue_);

		virtual ~CommandRouter() {}

		void Poll();

	private:
		uart::Port &port;
		framework::SpscRing<Command, 8> &queue;
		protocol::CommandParser parser;
		uint32_t sequence = 0;
	};
}
