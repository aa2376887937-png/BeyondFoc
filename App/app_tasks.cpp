#include "app.h"
#include "runtime.h"
#include "app_config.h"
#include "RC_io.h"
#include "RC_task.h"
#include "cmsis_os.h"

/*==============================================================================
 * 应用任务
 *
 *   每个任务只做一件事，任务之间通过 Runtime 里的队列/邮箱通信：
 *     foc     : 消费命令、电角度标定、发布控制快照（转速环在 ADC 中断里 1kHz 执行）
 *     comms   : 解析命令、发送遥测、启动 UART DMA
 *     monitor : 消费编码器帧、读 ADC2、发布监控快照
 *============================================================================*/

namespace
{
	constexpr uint16_t kHeartbeatTicks = 500;

	void Foc_Task(void *argument)
	{
		app::Runtime &runtime = *static_cast<app::Runtime *>(argument);

		while (!runtime.State().ready) osDelay(1);

		uint16_t heartbeat = 0;

		for (;;)
		{
			runtime.Control().Speed_Loop_Process(0.001f);

			// 任务活体指示：LED1 500ms 翻转（LED3 在 monitor 任务里）
			if (++heartbeat >= kHeartbeatTicks)
			{
				heartbeat = 0;
				io::Led_Toggle(1);
			}

			osDelay(app::config::kTaskPeriodTicks);
		}
	}

	void Comms_Task(void *argument)
	{
		app::Runtime &runtime = *static_cast<app::Runtime *>(argument);

		while (!runtime.State().ready) osDelay(1);

		for (;;)
		{
			// 1) 收：UART 字节 -> 命令队列
			runtime.Router().Poll();

			// 2) 发：Latest 快照 -> 协议帧 -> UART 发送环形缓冲
			runtime.Telemetry_().Send();

			// 3) 启动 DMA 发送
			runtime.Uart().Poll();

			osDelay(app::config::kTaskPeriodTicks);
		}
	}

	void Monitor_Task(void *argument)
	{
		app::Runtime &runtime = *static_cast<app::Runtime *>(argument);

		while (!runtime.State().ready) osDelay(1);

		for (;;)
		{
			runtime.Monitor_().Update(0.00025f);
			osDelay(app::config::kMonitorPeriodTicks);
		}
	}
}

extern "C" void App_Start(void)
{
	app::Runtime &runtime = app::Get_Runtime();

	// TaskCreator 内部调用 osThreadNew，必须放在 osKernelInitialize() 之后
	static task::TaskCreator foc_task(
		"foc",
		app::config::kTaskPriorityFoc,
		app::config::kTaskStack,
		Foc_Task,
		&runtime);

	static task::TaskCreator comms_task(
		"comms",
		app::config::kTaskPriorityComms,
		app::config::kTaskStack,
		Comms_Task,
		&runtime);

	static task::TaskCreator monitor_task(
		"monitor",
		app::config::kTaskPriorityMonitor,
		app::config::kTaskStack,
		Monitor_Task,
		&runtime);

	(void)foc_task;
	(void)comms_task;
	(void)monitor_task;
}
