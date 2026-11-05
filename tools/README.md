# tools / 上位机

## serial_scope.py

```bash
python tools/serial_scope.py              # 接板子
python tools/serial_scope.py --demo       # 没板子看界面（也响应命令）
python tools/serial_scope.py --self-test  # 只验协议
```

依赖：`pyserial pyqtgraph PyQt5 numpy`

### 用法

1. 选串口 → **连接**
2. 选模式：
   * **电流闭环**：给 iq 目标（A），在当前角度上跑电流环（测电流环/采样用）
   * **开环强拖**：不跑电流/转速环，TIM2 1kHz 推进电角度；`iq` 作为电压幅值（amp = |iq| × 0.2，最大 0.95），转速作为强拖目标，内部有加速度斜坡，用于验证 PWM/相序/驱动链路
   * **转速闭环**：用 MT6701 做位置/速度反馈，转速 PI 4kHz 输出 iq，电流环 20kHz 跑（PWM/ADC 20kHz，三电阻注入组采样）
   * **位置闭环**：位置 P 800Hz -> 速度目标 -> 转速 PI 4kHz -> 电流环 20kHz
3. **电角度标定**：电机可以自由转动、电源限流，点「电角度标定」：
   * 固件先给一个很小的 d 轴电流（默认 0.5A），把转子吸到电角度 0
   * 等待约 500ms 后读取编码器原始电角度，计算 `offset = -raw`
   * 上位机状态栏会显示 `电角度标定中 / 完成 / 失败`，以及当前 `offset`
   * 标定期间会强制打开输出，标定完成后自动关闭输出
   * 标定电流/等待时间在 `App/app_config.h` 的 `kAngleCalCurrent` / `kAngleCalSettleMs`
   * offset 目前保存在 RAM，重新上电后需要重新标定（后续可加 Flash 保存）
   * 编码器方向和电机相序不一致时，先改 `App/app_config.h` 的 `kEncoderDirection` 或交换电机线
   * 电流反馈方向相反时改 `kCurrentSignA/B/C`；不要在算法里改符号
4. 点 **使能输出**（会同时打开 TIM1 的 MOE）
5. 曲线：三相电流、SVPWM 占空比、转速/iq、编码器角度、母线电压；状态栏有帧率、丢帧、状态位
6. **第三行「显示」**：9 个通道各一个勾选框，不看的取消勾选即可；另有「全选 / 仅电流」快捷键
7. **「刷新」**：画面刷新率 5~60Hz（默认 25Hz），电脑卡就调低，数据本身仍是 1kHz

### 性能优化（实时曲线不占 CPU）

- 只把**勾选了**的通道从环形缓存转成 numpy 数组（没勾的完全跳过）
- 每次刷新最多画 **1500 个点**，超过就等间隔抽点（时间轴同步抽）
- 关闭抗锯齿 `antialias=False`
- 每条曲线 `setClipToView(True)` + `setDownsampling(auto=True)`，只画视窗内的点
- 一个通道都没勾的图直接 `setVisible(False)`，pyqtgraph 不渲染
- 状态栏改用 `last_values()`，不再为统计去转换整个数组

命令每 50ms 重发一次，丢一帧也不会失去控制。

## 通信协议（对应固件 `App/protocol.h` + `Platform/RC_uart.h`）

UART3，**2,000,000 bps，8N1**，固件每 1ms 发一帧，全部小端。

### MCU → PC（type `0x01`，len 112）

| 字节 | 内容 |
|---|---|
| 0 | `0xA5` |
| 1 | `0x5A` |
| 2 | `0x01` |
| 3 | `112` |
| 4..35 | 8 × float32：`ia, ib, ic`(A)、`da, db, dc`(0~1)、`id, iq`(A) |
| 36..37 | uint16 `vbus_mV` |
| 38..39 | int16 `speed_rpm` |
| 40..41 | uint16 `status`：bit0 使能 / bit1 转速环 / bit2 编码器有效 / bit3 电流零偏校准中 / bit4 ADC校准失败 / bit5 电角度标定中 / bit6 电角度标定完成 / bit7 电角度标定失败 / **bit8-11 启动阶段** |
| 42..43 | uint16 `seq`（丢帧统计） |
| 44..49 | uint16 `raw_a / raw_b / raw_c`：三相电流 ADC 原始码值（≈2048 = 采样正常） |
| 50..51 | uint16 `isr_avg_x10`：上个 1ms 窗口内 FOC 中断平均耗时（0.1µs） |
| 52..53 | uint16 `isr_max_x10`：同一窗口内的最大耗时（0.1µs） |
| 54..57 | float32 `enc_deg`：编码器机械角度 0~360° |
| 58..59 | int16 `enc_rpm`：编码器测的机械转速 |
| 60..61 | uint16 `enc_crc_err`：CRC 错误累计数（一直涨 = 读数不对） |
| 62..65 | uint32 `enc_raw_word`：编码器 24bit 原始帧 |
| 66..69 | float32 `elec_offset_deg`：当前电角度零位 offset（°） |
| 70..73 | float32 `kp_d`：d 轴电流环 Kp（当前实际生效值） |
| 74..77 | float32 `ki_d`：d 轴电流环 Ki |
| 78..81 | float32 `kp_q`：q 轴电流环 Kp |
| 82..85 | float32 `ki_q`：q 轴电流环 Ki |
| 86..87 | uint16 `offs_a`：实际生效的 A 相电流零偏（ADC 码值） |
| 88..89 | uint16 `offs_b`：B 相零偏 |
| 90..91 | uint16 `offs_c`：C 相零偏 |
| 92..95 | float32 `iq_target`：电流环 q 轴指令（A），用于和反馈对比 |
| 96..99 | float32 `id_target`：电流环 d 轴指令（A） |
| 100..103 | float32 `speed_kp`：转速环 Kp（当前实际值） |
| 104..107 | float32 `speed_ki`：转速环 Ki |
| 108..111 | float32 `pos_kp`：位置环 Kp |
| 112..115 | float32 `pos_ki`：位置环 Ki |
| 116 | XOR 校验（字节 2..115） |

### PC → MCU（type `0x10`，len 46）

| 字节 | 内容 |
|---|---|
| 4 | uint8 `enable` |
| 5 | uint8 `mode`（0 电流闭环 / 1 开环强拖 / 2 波形观察 / 3 转速闭环 / 4 位置闭环） |
| 6..7 | int16 `iq_mA` |
| 8..9 | int16 `speed_rpm` |
| 10..11 | uint16 `flags`：bit0 电流零偏校准 / bit1 编码器角度清零 / bit2 电角度零位标定 / **bit3 电流环 PI 整定** / **bit5 转速环 PI 整定** / **bit6 位置环 PI 整定** |
| 12..15 | int32 `position_cdeg`：位置目标（0.01°） |
| 16..19 | float32 `kp_d` |
| 20..23 | float32 `ki_d` |
| 24..27 | float32 `kp_q` |
| 28..31 | float32 `ki_q` |
| 32..33 | int16 `id_mA`：d 轴电流指令（0.001A） |
| 34..37 | float32 `speed_kp` |
| 38..41 | float32 `speed_ki` |
| 42..45 | float32 `pos_kp` |
| 46..49 | float32 `pos_ki` |
| 50 | XOR 校验 |

## 固件侧实现

固件分为四层，详细说明见根目录 `ARCHITECTURE.md`：

```
App/        应用层：装配、任务、命令、控制环、遥测
Algorithm/  纯算法：FOC 数学、PI、电流处理、位置/观测器、控制器、开环、MT6701 解码
Platform/   底层封装：TIM/ADC/PWM/UART/SPI 编码器/IO/任务/计时
Framework/  生产者/消费者基础设施：SpscRing、Latest、Subject
```

**发送（comms 任务，非阻塞）**
- `Telemetry::Send()` 从 `Latest` 邮箱取快照 → `protocol::Encode_Scope()` → `Port::Write()`
- `Port::Write()` 只写 4KB 环形缓冲；`Port::Poll()` 启动 DMA
- DMA 完成中断只推进读指针，**中断里完全没有组帧/解析逻辑**

**接收（comms 任务解析）**
- USART3 DMA + 空闲中断：中断里只把字节推进 256B 环形缓冲，然后重新挂 DMA
- `CommandRouter::Poll()` 取字节 → `protocol::CommandParser` 组帧 → `SpscRing<Command>`
- 控制任务 `ControlLoop::Speed_Loop_Process()` 消费命令并落地
- 单生产者/单消费者无锁，缓冲满整帧丢弃；命令队列满则丢当前命令，下一帧重发

**FOC 中断耗时统计**
- DWT 周期计数器（~6ns 分辨率）在 `ADC1_2_IRQHandler` 进出各取一次
- 1kHz 窗口统计平均/最大值，随遥测上传，上位机显示 `ISR 4.2/9.8us (8%)`
- ADC 中断周期 50µs（20kHz）；电流环 50µs（20kHz），转速环 1ms（1kHz）

**转速环**
- PWM 20kHz；TIM1 CH4 单边沿触发 ADC，ADC 20kHz
- ADC1 注入组 3 rank：IC(PA0)、IB(PA1)、IA(PA2)；TIM1 更新中断软件启动，注入完成进电流环
- `Foc_Controller::Step()` 每个 ADC 执行一次，电流环 20kHz
- `Foc_Controller::Speed_Loop()` 每 20 次 ADC 执行一次，转速环 1kHz
- 编码器速度由 Monitor 任务 1kHz 更新，1kHz 转速环读取最新值
- 转速 PI 输出 iq 指令，电流环 20kHz 跑
- `App/ControlLoop` 的 1kHz 任务只负责消费命令、标定和发布遥测
- 只在进入转速环时清一次积分；目标值微调不重置积分
- 编码器无效时自动退回电流模式，编码器恢复有效后自动切回转速环

**电角度零位标定**
- 上位机命令 `flags bit2`
- 固件强制用开环角度 0、d 轴小电流、iq=0，等待转子稳定
- 读取编码器原始电角度，写入 `offset = -raw`
- 控制环之后使用 `angle = raw + offset`
- 标定期间状态位 bit5=进行中、bit6=完成、bit7=失败

**编码器（MT6701，SPI + DMA + 环形缓冲）**
- `Platform/RC_encoder` 只负责 SPI/DMA/CS：完成中断把 `{24bit 原始帧, 时间戳}` 压进队列
- `App/Monitor` 在 1kHz 任务里消费最新帧，交给 `Algorithm/MT6701` 做 CRC、角度展开、速度计算、电角度 offset
- ADC2 的母线/NTC/相电压走 DMA：完成中断置标志，监控任务 100Hz 消费结果并启动下一轮，不再忙等 `HAL_ADC_PollForConversion`
- 速度用相邻帧的真实时间戳算，不受任务抖动影响
- 传输超 10ms 未完成 → 判死锁，强制拉高 CS 重新开始
- 帧格式：14bit 角度 + 4bit 磁场状态 + 6bit CRC（多项式 0x43）
