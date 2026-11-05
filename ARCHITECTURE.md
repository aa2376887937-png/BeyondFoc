# BeyondFOC 架构说明

## 1. 分层

```
┌─────────────────────────────────────────────────────────────┐
│ App/                  应用层（业务编排）                     │
│   app / app_tasks     运行时装配、任务创建                   │
│   control_loop        电流环桥接、模式切换、遥测快照          │
│   command_router      字节流 -> 命令队列                     │
│   monitor             编码器快照、ADC2 慢速量                │
│   telemetry           快照 -> 协议帧 -> UART                 │
│   protocol            帧格式编解码                           │
└───────────────┬───────────────────────────┬─────────────────┘
                │ 依赖                       │ 依赖
┌───────────────▼───────────────┐  ┌────────▼─────────────────┐
│ Algorithm/   纯算法层          │  │ Platform/  底层封装层     │
│   FOC_math                   │  │   RC_tim / RC_timer      │
│   FOC_pi                     │  │   RC_adc / RC_pwm        │
│   FOC_current                │  │   RC_uart                │
│   FOC_position / MT6701      │  │   RC_encoder (SPI/DMA)   │
│   FOC_control                │  │   RC_analog / RC_io      │
│   FOC_openloop               │  │   RC_task / RC_prof      │
└──────────────────────────────┘  └────────┬─────────────────┘
                                           │ 依赖
                                  ┌────────▼─────────────────┐
                                  │ Core / Drivers / Middlewares │
                                  │ CubeMX、HAL、FreeRTOS       │
                                  └──────────────────────────┘

Framework/ 被 Platform 和 App 共用：SpscRing、Latest、Subject
```

依赖方向是单向的：

- `App -> Algorithm`
- `App -> Platform`
- `Platform -> HAL/CMSIS/FreeRTOS`
- `Algorithm` 不包含任何 HAL、外设句柄、FreeRTOS 头文件
- `Platform` 不包含 `App` 或 `Algorithm` 的头文件

`Framework` 只依赖 `<stdint.h>`，不依赖 HAL。

## 2. 生产者 / 消费者

| 数据/事件 | 生产者 | 消费者 | 通道 |
|---|---|---|---|
| 相电流采样 | ADC1 注入完成中断 | `ControlLoop`（中断内） | 函数调用（ADC 20kHz，电流环 20kHz，最低延迟） |
| 控制命令 | `CommandRouter`（comms 任务） | `ControlLoop`（foc 任务） | `SpscRing<Command, 8>` |
| 控制快照 | `ControlLoop`（foc 任务） | `Telemetry`（comms 任务） | `Latest<ControlSnapshot>` |
| 母线/温度/相电压 | ADC2 DMA 完成中断 | `Monitor`（monitor 任务） | DMA 缓冲 + 完成标志 |
| 母线/温度/相电压快照 | `Monitor` | `ControlLoop`、`Telemetry` | `Latest<PowerSnapshot>` |
| 编码器原始帧 | SPI DMA 完成中断 | `Monitor`（monitor 任务，1kHz） | `Latest<EncoderFrame>`（只保留最新帧） |
| 编码器快照 | `Monitor` | `Telemetry` | `Latest<EncoderSnapshot>` |
| UART 接收字节 | USART3 空闲中断 | `CommandRouter` | `Port` 内部 SPSC 环形缓冲 |
| UART 发送字节 | `Telemetry` / 命令回复 | USART3 DMA 完成中断 | `Port` 内部 SPSC 环形缓冲 |

`SpscRing` 用于"不能丢、需要排队"的数据；`Latest` 用于"只关心最新值、允许覆盖"的遥测。

## 3. 任务

| 任务 | 优先级 | 周期 | 职责 |
|---|---:|---:|---|
| `foc` | 20 | 1ms | 消费命令、电角度标定、发布控制快照、LED1 心跳 |
| `comms` | 22 | 1ms | 解析命令、发送遥测、启动 UART DMA |
| `monitor` | 10 | 1ms | 消费编码器帧、发布编码器快照、100Hz 启动/消费 ADC2 DMA、发布监控快照、LED3 心跳 |

PWM 20kHz；TIM1 更新中断软件启动 ADC1 注入组（3 通道），ADC 注入完成中断直接跑电流环 20kHz；
转速环 4kHz（5 分频）、位置环 800Hz（25 分频）。
电角度标定由 `foc` 任务推进：小 d 轴电流吸住转子 -> 等 500ms -> 读编码器原始电角度 -> `offset = -raw`。

三电阻低边采样：ADC1 注入组 rank1=PA0/IC，rank2=PA1/IB，rank3=PA2/IA；
ADC2 规则组继续采母线/NTC/相电压。

PWM 配置为 PWM2，`Pwm::Set_Duty()` 内部按 `CCR=(1-duty)*ARR` 换算；
电流方向 `kCurrentSignA/B/C=-1`（iRDT 5mΩ + RS724 硬件方向）。
电流环 d/q 分别按 Ld/Lq 整定，并开启交叉项 + 反电势解耦。

## 4. 启动顺序

`Core/Src/main.c` 只保留 CubeMX 生成的底层初始化，然后：

1. `All_Init()`（在 `osKernelInitialize()` 之前）
   - ADC 自校准
   - 启动 TIM2 1kHz
   - 启动 UART DMA 接收
   - 启动 PWM 时基（MOE 保持关闭）
   - 启动编码器 SPI、ADC2、ADC1 注入组、电流零偏校准
2. `osKernelInitialize()`
3. `App_Start()` 创建 `foc` / `comms` / `monitor` 任务
4. `osKernelStart()`

## 5. 关键解耦点

| 旧代码 | 问题 | 新代码 |
|---|---|---|
| `RC_bench` 继承 `tim::TimHandler`，直接反控 `controller` 和 `PWM` | 调试功能侵入控制环 | `Algorithm/FOC_openloop` 只算占空比，`ControlLoop` 决定写不写 PWM |
| `RC_info` 同时依赖电流/控制/PWM/母线/编码器/强拖 | 遥测层和所有模块强耦合 | `Telemetry` 只读 `Latest` 消息结构体 |
| `RC_encoder` 继承 `position_sensor`，直接绑 HAL SPI 回调 | 底层依赖算法层 | `Platform/RC_encoder` 只产原始帧；`Algorithm/MT6701` 只解码 |
| `FOC_contrl` 直接持有 `current_sensor*`、`pwm::Pwm*` | 算法反向依赖硬件 | `Foc_Controller::Step(Input) -> Output`，输入输出都是普通结构体 |
| `FOC_sensor` 把电流采样和位置观测混在一起 | 职责不清 | 拆成 `FOC_current` 与 `FOC_position` |
| `RC_init.cpp` 一个文件做装配/任务/命令/遥测/初始化 | 入口文件变成"上帝对象" | `App/Runtime` 只做装配，任务/命令/遥测各自独立文件 |
| 任务对象是全局构造，在 `osKernelInitialize()` 之前调用 `osThreadNew` | 启动顺序不规范 | `App_Start()` 在 `osKernelInitialize()` 之后创建任务 |

## 6. 扩展方式

- **换电机参数**：只改 `App/app_config.h`
- **换位置来源**：实现 `position_sensor`，在 `ControlLoop::Apply_Command` 里选择
- **加一路传感器**：Platform 层生产 `Latest<XxxSnapshot>`，App 层消费，不需要改控制算法
- **加一个遥测字段**：在 `App/app_messages.h` 的 `ScopeData` 加字段，在 `App/protocol.cpp` 组帧，PC 端同步解析
- **换 UART**：只改 `Runtime` 里 `uart::Port` 绑定的句柄和 DMA 缓冲
