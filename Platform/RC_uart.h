#pragma once

#include "main.h"
#include <stdint.h>

/*==============================================================================
 * 非阻塞 UART 字节管道
 *
 *   发送：任务 Write() -> 4KB 环形缓冲 -> Poll() 启动 DMA -> 完成中断推进读指针
 *   接收：DMA + 空闲中断 -> 256B 环形缓冲 -> 任务 Read()
 *
 *   本文件只搬运字节，不认识命令帧和遥测帧。
 *   帧格式在 App/protocol，业务解析在 App/command_router。
 *============================================================================*/

// 发送环形缓冲：遥测一帧 71B @1kHz = 71KB/s，2Mbaud 链路 200KB/s，链路本身够用；
// 这里留的是"抖动余量"——一次 DMA 发送超时(50ms)、任务被饿死或临界区拖长时，
// 攒下的帧先存在环形缓冲里而不是被 Write() 整帧丢掉。
// 16KB ≈ 230 帧 ≈ 230ms；改大不改小，且必须是 2 的幂（指针靠掩码回绕）。
#define UART_TX_RING_SIZE		16384	// 必须是 2 的幂
#define UART_RX_RING_SIZE		256		// 必须是 2 的幂

#ifdef __cplusplus
namespace uart
{
	class Port
	{
	public:
		Port(UART_HandleTypeDef &huart_, uint8_t *rx_dma_buf_, uint16_t rx_dma_size_);
		virtual ~Port() {}

		void Start();									// 挂上 DMA + 空闲中断
		void Poll();									// 任务里调用：空闲时启动一次 DMA 发送
		int Write(const uint8_t *data_, uint16_t len_);	// 生产者：0 = 缓冲满，整帧丢弃
		uint16_t Tx_Free() const;

		uint16_t Read(uint8_t *out_, uint16_t max_len_);	// 消费者：取已收到的字节
		bool Read_Byte(uint8_t &byte_);

		UART_HandleTypeDef *handle() const { return huart; }

		static void On_Rx_Event(UART_HandleTypeDef *huart_, uint16_t size_);
		static void On_Tx_Cplt(UART_HandleTypeDef *huart_);
		static void On_Error(UART_HandleTypeDef *huart_);

	private:
		static Port *Find(UART_HandleTypeDef *huart_);

		void Rx_Push(const uint8_t *data_, uint16_t len_);
		bool Rx_Pop(uint8_t &byte_);
		void Tx_Advance(uint16_t len_);

		UART_HandleTypeDef *huart;
		uint8_t *rx_dma;
		uint16_t rx_dma_size;

		// 发送环形缓冲：生产者 = 任务，消费者 = DMA 完成中断
		uint8_t tx_ring[UART_TX_RING_SIZE];
		volatile uint32_t tx_head = 0;
		volatile uint32_t tx_tail = 0;
		volatile uint16_t tx_chunk = 0;
		volatile bool tx_busy = false;
		uint16_t tx_wait_ms = 0;

		// 接收环形缓冲：生产者 = 空闲中断，消费者 = 任务
		uint8_t rx_ring[UART_RX_RING_SIZE];
		volatile uint32_t rx_head = 0;
		volatile uint32_t rx_tail = 0;
		volatile bool rx_active = false;
		uint16_t rx_retry = 0;

		static Port *port_list[4];
		static uint8_t port_num;
	};
}
#endif
