#pragma once

#include <stdint.h>

/*==============================================================================
 * 应用配置
 *
 *   电机参数、控制参数、任务周期集中放在这里。
 *   这些是"业务选择"，不属于底层驱动，也不应该散落在算法类里。
 *============================================================================*/

namespace app
{
	namespace config
	{
		// ---------------- 电机参数：4006 KV380（14 极 = 7 对极） ----------------
		//   Rs  = 0.19067 Ω（规格书 170 mΩ，取辨识值）
		//   Ld  = 74.55 uH
		//   Lq  = 42.52 uH        ← 注意 Ld > Lq，和 4008 分支正好相反
		//   Flux= 2.4 mWb（Kt = 9.549/Kv = 9.549/380 = 0.0251，ψf = Kt/(1.5×7)）
		constexpr uint8_t kPolePairs = 7;
		constexpr float kMotorR = 0.19067f;				// 相电阻 (Ω)
		constexpr float kMotorLd = 0.0000745519338f;	// d 轴电感 (H)
		constexpr float kMotorLq = 0.000042519338f;		// q 轴电感 (H)
		constexpr float kMotorFlux = 0.0024f;			// 永磁体磁链 (Wb)

		// ---------------- 控制参数 ----------------
		constexpr float kCurrentLoopBw = 6.2831853f * 1000.0f;	// 电流环带宽 1kHz (rad/s)

		// 转速环 PI（1kHz，和编码器速度更新率一致）
		//
		// 被控对象：iq(A) -> 转速(rpm)
		//     d(rpm)/dt = 9.549 · Kt / J · iq
		//     Kt = 1.5 · 极对数 · ψf = 1.5 × 7 × 0.0024 = 0.0252 N·m/A
		//     9.549 · Kt = 0.2407
		//
		// 按穿越频率 f_c 整定（PI 零点放在 f_c/5）：
		//     Kp = J · ω_c / 0.2407      [A/rpm]
		//     Ki = Kp · ω_c / 5          [A/(rpm·s)]
		//
		// 下面这组按 J ≈ 4e-6 kg·m²（4006 空载转子）算，f_c ≈ 20Hz。
		// 如果带桨/联轴器，J 会大 10~50 倍，Kp/Ki 要同比放大。
		// 带宽上限受"编码器速度 1kHz + 3.3ms 滤波"限制，f_c 不要超过 25Hz。
		constexpr float kSpeedPiKp = 0.0040f;					// ≈ J·ωc/0.2407
		constexpr float kSpeedPiKi = 0.052f;					// ≈ Kp·ωc/5
		constexpr float kSpeedPiLimit = 5.6f;					// 转速环输出 iq 限幅 (A)

		// 转速环抗积分饱和：
		// 积分项单独限幅（不再跟着 5.6A 输出限幅跑），
		// 误差大于分离阈值时暂停积分（阶跃时先靠 P 拉过去）。
		constexpr float kSpeedPiIntegralLimit = 4.0f;			// 积分项限幅 (A)
		constexpr float kSpeedPiIntegralSeparation = 300.0f;	// 误差 > 100rpm 时暂停积分

		// 转速目标斜坡 (rpm/s)：速度突变时避免 iq 瞬间跳变（会啸叫/顿一下）。
		// 5000 = 200rpm 走 40ms；0 = 关闭斜坡。
		constexpr float kSpeedTargetAccel = 0.0f;
		constexpr float kPositionKp = 100.0f;						// 位置环 PD：Kp (1/s)
		// Kd 是"速度反馈阻尼"：输出 = Kp·误差 - Kd·实际机械角速度。
		// 位置环输出是速度指令（内环还有转速环），Kd 不宜过大，
		// 0.5 起步，靠上位机在线加到"接近目标不超调"即可。
		constexpr float kPositionKd = 0.5f;
		constexpr float kPositionSpeedLimit = 100.0f;				// 位置环输出速度限幅 (rad/s, 机械)

		// ---------------- 位置环增强 ----------------
		// D 项低通：抑制编码器速度量化噪声进入微分项
		constexpr float kPositionKdFilterHz = 30.0f;
		// 带回差的小死区：进 deadband 后误差清零，直到超出 exit 才重新接管
		constexpr float kPositionDeadbandEnterDeg = 0.40f;
		constexpr float kPositionDeadbandExitDeg = 0.80f;
		// 速度前馈：用轨迹规划的速度直接加到速度指令上
		constexpr float kPositionSpeedFf = 1.0f;
		// ADRC 离散 TD 轨迹规划（最速控制 / 最速下落曲线）
		constexpr bool kPositionTdEnable = false;
		constexpr float kPositionTdAccel = 150.0f;					// 规划最大加速度 (rad/s²)

		// ---------------- 扫频 / 参数辨识 ----------------
		constexpr float kSweepCurrentFStart = 5.0f;		// 电流环扫频起止 (Hz)
		constexpr float kSweepCurrentFEnd = 400.0f;
		constexpr float kSweepSpeedFStart = 0.3f;		// 转速环扫频起止 (Hz)
		constexpr float kSweepSpeedFEnd = 50.0f;
		constexpr uint16_t kSweepPoints = 20;			// 扫频点数（对数分布）
		constexpr uint16_t kSweepCyclesPerPoint = 6;	// 每个频点停留周期数

		constexpr float kLStepVoltD = 1.0f;				// Ld 阶跃电压 (V)
		constexpr float kLStepVoltQ = 0.25f;			// Lq 阶跃电压 (V)
		constexpr uint16_t kLStepTicks = 2;				// 阶跃持续电流环拍数（2×50µs）				// 位置环输出速度限幅 (rad/s, 机械)
		constexpr float kDefaultBusVoltage = 24.0f;

		// ---------------- 死区补偿 ----------------
		// TIM1 的 DeadTime = 15 个 t_DTS，t_DTS = 1/170MHz = 5.88ns -> 88ns。
		// 补偿量 = t_dead × f_pwm × √3 的调制度，按电流方向叠加。
		// 注意：实际"有效死区"还要算上驱动/开关延时，最好的办法是用示波器
		// 看低电流时的相电流过零畸变，再把 kDeadTimeUs 往大/往小调。
		constexpr float kDeadTimeUs = 0.088f;		// 88ns
		constexpr bool kDeadTimeCompEnable = true;	// 先关掉：死区补偿没实测过，而且占中断时间。
													// 等低电流波形核对好之后再打开

		// 电流采样方向：参考框架里 iRDT + 5mΩ + RS724 使用 -1。
		// 如果上电后 iq 反馈方向和指令相反，先把对应相改成 +1 再测。
		constexpr float kCurrentSignA = -1.0f;
		constexpr float kCurrentSignB = -1.0f;
		constexpr float kCurrentSignC = -1.0f;

		// 编码器方向：手转电机正向，机械角应该增大；如果减小就改成 -1
		constexpr float kEncoderDirection = -1.0f;

		// 转速反馈二阶巴特沃斯低通截止频率 (Hz)。
		// 监控任务按 1kHz 消费编码器帧，滤波器按 fs=1kHz 设计。
		// 截止越低越平滑、但相位滞后越大；转速环带宽 ~20Hz，这里 50Hz 比较平衡。
		constexpr float kEncoderSpeedFilterHz = 50.0f;

		// 速度锁相环（PLL）：用角度闭环估计速度，替代"差分 + 低通"
		constexpr float kPllBandwidthHz = 80.0f;		// 环路自然频率 (Hz)
		constexpr float kPllDamping = 0.707f;			// 阻尼比

		// 电角度零位（6 次标定均值 138.74°）。
		// 已实测：138.74° 时给 iq 转子能正常旋转。
		// 48.57° / 48.74° 都比正确值小 90°，是"差 90°"的错误值，
		// 那个角度下 id 会转、iq 不转（d/q 轴互换），不要用。
		constexpr float kElecAngleOffsetDeg = 138.74f;
		
		// 电角度标定：多点对齐。
		// 只在一个角度对齐时，齿槽/静摩擦会把转子拉偏 1~3°，offset 就跟着偏。
		// 改成在 8 个电角度（45° 一步）分别对齐、各读一次编码器，再对 8 个
		// offset 做矢量平均，齿槽/摩擦的偏差会相互抵消；同时用它们的离散度
		// 检查编码器安装偏心/方向问题（离散度 > 15° 直接判失败）。
		constexpr float kAngleCalCurrent = 1.0f;		// d 轴电流 (A)
		constexpr uint16_t kAngleCalRampMs = 50;		// 电流上升斜坡 (ms)
		constexpr uint16_t kAngleCalSettleMs = 500;		// 每个点等待转子稳定 (ms)
		constexpr uint8_t kAngleCalPoints = 8;			// 标定点数（8 × 45° = 一圈）

		// ---------------- 功能开关 ----------------
		constexpr bool kEncoderEnable = true;
		constexpr bool kUseSmo = false;				// 现在只做开环拖动，先关掉
		constexpr bool kSvpwmViewDefaultOn = false;	// 波形观察默认关闭（由上位机下发 mode=2 切换）

		// ---------------- 任务参数 ----------------
		constexpr uint16_t kTaskStack = 512;
		constexpr uint8_t kTaskPriorityFoc = 20;
		constexpr uint8_t kTaskPriorityComms = 22;
		constexpr uint8_t kTaskPriorityMonitor = 10;

		// RTOS tick = 4kHz（FreeRTOSConfig.h 的 configTICK_RATE_HZ）
		// FOC / 通信任务保持 1kHz（4 tick）；监控任务 1 tick = 250µs，
		// 编码器帧（角度 + 速度）因此以 4kHz 更新，和转速环 4kHz 对齐。
		constexpr uint32_t kTaskPeriodTicks = 4;		// 4 tick = 1ms
		constexpr uint32_t kMonitorPeriodTicks = 1;		// 1 tick = 250µs

		// PWM = ADC 注入 = 20kHz；电流环 20kHz，转速环 4kHz（5 分频），位置环 800Hz（25 分频）
		constexpr uint32_t kCurrentLoopHz = 20000;	// 基础节拍（PWM/ADC 注入），
													// 实际电流环 = kCurrentLoopHz / kCurrentLoopDecim
		constexpr uint32_t kSpeedLoopHz = 4000;
		constexpr uint32_t kPositionLoopHz = 800;

		// 电流环带宽上限 = 实际节拍 × kCurrentBwLoopRatio（再乘 2π）。
		// 0.05 = 1/20（20kHz 时 1kHz）
		constexpr float kCurrentBwLoopRatio = 0.05f;

		// 电流环降频：1 = 20kHz（默认），2 = 10kHz。
		// 如果 Debug 构建下 20kHz 中断跑不完（心跳灯灭、串口停），改成 2 直接减半。
		constexpr uint8_t kCurrentLoopDecim = 1;
	}
}
