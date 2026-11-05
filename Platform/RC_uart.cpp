#include "RC_uart.h"
#include "Framework/spsc_ring.h"

namespace uart
{
	Port *Port::port_list[4] = {nullptr};
	uint8_t Port::port_num = 0;

	Port::Port(UART_HandleTypeDef &huart_, uint8_t *rx_dma_buf_, uint16_t rx_dma_size_)
		: huart(&huart_)
		, rx_dma(rx_dma_buf_)
		, rx_dma_size(rx_dma_size_)
	{
		if (port_num < 4 && rx_dma != nullptr && rx_dma_size != 0)
		{
			port_list[port_num] = this;
			port_num++;
		}
	}

	Port *Port::Find(UART_HandleTypeDef *huart_)
	{
		for (uint8_t i = 0; i < port_num; i++)
		{
			if (port_list[i] != nullptr && port_list[i]->huart == huart_)
			{
				return port_list[i];
			}
		}

		return nullptr;
	}

	void Port::Start()
	{
		if (HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_dma, rx_dma_size) == HAL_OK)
		{
			rx_active = true;

			// 只要 IDLE 事件，不要半传输事件（否则同一段数据会被解析两次）
			if (huart->hdmarx != nullptr)
			{
				__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
			}
		}
		else
		{
			rx_active = false;
		}
	}

	/*------------------------------------------------------------------------------
	 * 发送：环形缓冲 + DMA（单生产者/单消费者，无锁）
	 *------------------------------------------------------------------------------*/
	int Port::Write(const uint8_t *data_, uint16_t len_)
	{
		if (data_ == nullptr || len_ == 0 || len_ > UART_TX_RING_SIZE) return 0;

		const uint32_t head = tx_head;
		const uint32_t used = head - tx_tail;

		if (used + len_ > UART_TX_RING_SIZE) return 0;	// 空间不够，整帧丢弃

		uint32_t w = head & (UART_TX_RING_SIZE - 1u);
		for (uint16_t i = 0; i < len_; i++)
		{
			tx_ring[w] = data_[i];
			w = (w + 1u) & (UART_TX_RING_SIZE - 1u);
		}

		framework::Memory_Barrier();		// 数据写完之后再更新 head
		tx_head = head + len_;

		return 1;
	}

	void Port::Tx_Advance(uint16_t len_)
	{
		tx_tail += len_;
		tx_chunk = 0;
		tx_busy = false;
	}

	void Port::Poll()
	{
		// RX 自恢复：先做非阻塞重挂；连续失败 100ms 才 AbortReceive 一次。
		// 不要每 1ms 都 AbortReceive，否则会干扰 UART 状态，拖慢发送/遥测。
		if (!rx_active)
		{
			if (++rx_retry > 100)
			{
				HAL_UART_AbortReceive(huart);
				rx_retry = 0;
			}

			Start();
		}
		else
		{
			rx_retry = 0;
		}

		if (tx_busy)
		{
			// HAL 的 TC 中断可能被 RX 错误分支提前 return 掉，导致 gState 卡在
			// BUSY_TX。超时就强制复位，保证链路能恢复。
			if (++tx_wait_ms > 50)
			{
				HAL_UART_AbortTransmit(huart);

				tx_tail += tx_chunk;
				tx_chunk = 0;
				tx_busy = false;
				tx_wait_ms = 0;
			}
			return;
		}

		tx_wait_ms = 0;

		const uint32_t head = tx_head;
		const uint32_t tail = tx_tail;
		if (head == tail) return;

		uint32_t n = head - tail;
		const uint32_t idx = tail & (UART_TX_RING_SIZE - 1u);
		const uint32_t contig = UART_TX_RING_SIZE - idx;	// 到环形末尾的连续长度
		if (n > contig) n = contig;

		tx_chunk = (uint16_t)n;
		if (HAL_UART_Transmit_DMA(huart, &tx_ring[idx], (uint16_t)n) == HAL_OK)
		{
			tx_busy = true;
		}
		else
		{
			tx_chunk = 0;			// 启动失败，下次再试
		}
	}

	uint16_t Port::Tx_Free() const
	{
		return (uint16_t)(UART_TX_RING_SIZE - (tx_head - tx_tail));
	}

	/*------------------------------------------------------------------------------
	 * 接收：空闲中断只把字节推进环形缓冲，任务里再取出来解析
	 *------------------------------------------------------------------------------*/
	void Port::Rx_Push(const uint8_t *data_, uint16_t len_)
	{
		for (uint16_t i = 0; i < len_; i++)
		{
			const uint32_t head = rx_head;
			if ((head - rx_tail) >= UART_RX_RING_SIZE) break;		// 满，丢新字节

			rx_ring[head & (UART_RX_RING_SIZE - 1u)] = data_[i];
			framework::Memory_Barrier();
			rx_head = head + 1u;
		}
	}

	bool Port::Rx_Pop(uint8_t &byte_)
	{
		if (rx_tail == rx_head) return false;

		byte_ = rx_ring[rx_tail & (UART_RX_RING_SIZE - 1u)];
		rx_tail++;

		return true;
	}

	uint16_t Port::Read(uint8_t *out_, uint16_t max_len_)
	{
		if (out_ == nullptr || max_len_ == 0) return 0;

		uint16_t n = 0;
		uint8_t byte;

		while (n < max_len_ && Rx_Pop(byte))
		{
			out_[n++] = byte;
		}

		return n;
	}

	bool Port::Read_Byte(uint8_t &byte_)
	{
		return Rx_Pop(byte_);
	}

	void Port::On_Rx_Event(UART_HandleTypeDef *huart_, uint16_t size_)
	{
		Port *self = Find(huart_);
		if (self == nullptr) return;

		self->rx_active = false;

		if (size_ != 0)
		{
			self->Rx_Push(self->rx_dma, size_);
		}

		self->Start();
	}

	void Port::On_Tx_Cplt(UART_HandleTypeDef *huart_)
	{
		Port *self = Find(huart_);
		if (self == nullptr) return;

		if (self->tx_busy)
		{
			self->Tx_Advance(self->tx_chunk);
		}
	}

	void Port::On_Error(UART_HandleTypeDef *huart_)
	{
		Port *self = Find(huart_);
		if (self == nullptr) return;

		self->rx_active = false;

		// 错误标志不清掉，HAL_UART_IRQHandler 会一直走错误分支提前 return，
		// 连 TC（发送完成）都处理不到，TX 就会永久停在 BUSY_TX。
		if (__HAL_UART_GET_FLAG(huart_, UART_FLAG_ORE)) __HAL_UART_CLEAR_OREFLAG(huart_);
		if (__HAL_UART_GET_FLAG(huart_, UART_FLAG_NE)) __HAL_UART_CLEAR_NEFLAG(huart_);
		if (__HAL_UART_GET_FLAG(huart_, UART_FLAG_FE)) __HAL_UART_CLEAR_FEFLAG(huart_);
		if (__HAL_UART_GET_FLAG(huart_, UART_FLAG_PE)) __HAL_UART_CLEAR_PEFLAG(huart_);

		self->Start();
	}
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	uart::Port::On_Rx_Event(huart, size);
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	uart::Port::On_Tx_Cplt(huart);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	uart::Port::On_Error(huart);
}
