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
		// ---------------- 电机参数：4006（14 极 = 7 对极） ----------------
		// 2026-11-10 实测（Rs/L 双频辨识 + J/B 惰行法）：
		//   Rs  = 0.1911 Ω
		//   Ld  = 47.5 µH
		//   Lq  = 75.8 µH
		//   J   = 467.8 g·cm² = 4.678e-5 kg·m²
		//   B   = 0.022 mN·m·s = 2.2e-5 N·m·s/rad
		//   ψf  = 2.5 mWb（在线辨识约 2.48，取整 2.5）
		constexpr uint8_t kPolePairs = 7;
		constexpr float kMotorR = 0.1911f;				// 相电阻 (Ω)
		constexpr float kMotorLd = 0.0000475f;			// d 轴电感 (H)
		constexpr float kMotorLq = 0.0000758f;			// q 轴电感 (H)
		constexpr float kMotorFlux = 0.0025f;			// 永磁体磁链 (Wb)
		constexpr float kMotorJ = 4.678e-5f;			// 转动惯量 (kg·m²)，467.8 g·cm²
		// Kt = 1.5·极对数·ψf = 0.02625 N·m/A（加速度前馈换算用）
		constexpr float kMotorKt = 1.5f * (float)kPolePairs * kMotorFlux;

		// ---------------- 磁链在线辨识（方法一：q 轴稳态反电势） ----------------
		//   ψf = (vq − R·iq − ωe·Ld·id) / ωe
		// 只在高速、有一定电流、电压未饱和、非扫频时更新。
		constexpr float kFluxEstMinElecW = 300.0f;	// 电角速度门限 (rad/s) ≈ 410rpm
		constexpr float kFluxEstMinIq = 0.2f;		// 最小 |iq| (A)
		constexpr float kFluxEstLpfHz = 1.0f;		// 估计值一阶低通 (Hz)
		constexpr float kFluxEstMinRatio = 0.5f;	// 合理性下限（相对标称）
		constexpr float kFluxEstMaxRatio = 1.5f;	// 合理性上限

		// ---------------- 控制参数 ----------------
		constexpr float kCurrentLoopBw = 6.2831853f * 1000.0f;	// 电流环带宽 1kHz (rad/s)

		// 转速环 PI（4kHz）
		//   按实测：J = 467.8 g·cm² = 4.678e-5 kg·m²
		//            Kt = 1.5×7×2.5mWb = 0.02625 N·m/A
		//   对象增益 A = (60/2π)·Kt/J = 5358 (rpm/s²)/A
		//   当前 Kp=0.0117 → 实际带宽 ≈ 10Hz
		//   若要 30Hz：Kp = 2π·30/5358 = 0.0352，Ki = Kp·13 = 0.457
		//   （30Hz 是和当前 PLL 95.5Hz + 79.6Hz 低通匹配的上限）
		constexpr float kSpeedPiKp = 0.0117f;
		constexpr float kSpeedPiKi = 0.1525f;
		constexpr float kSpeedPiLimit = 5.6f;					// 转速环输出 iq 限幅 (A)

		// 转速环抗积分饱和：
		// 积分项单独限幅（不再跟着 5.6A 输出限幅跑），
		// 误差大于分离阈值时暂停积分（阶跃时先靠 P 拉过去）。
		constexpr float kSpeedPiIntegralLimit = 2.0f;			// 积分项限幅 (A)
		constexpr float kSpeedPiIntegralSeparation = 100.0f;	// 误差 > 100rpm 时暂停积分

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
		// 加速度前馈：iq_ff = kPositionAccelFf · J·a / Kt（0 = 关闭）
		// 加速段直接给转速环预支转矩，减少大加速度时的位置滞后
		constexpr float kPositionAccelFf = 1.0f;
		// ADRC 离散 TD 轨迹规划（最速控制 / 最速下落曲线）
		constexpr bool kPositionTdEnable = false;
		constexpr float kPositionTdAccel = 150.0f;					// 规划最大加速度 (rad/s²)

		// 位置环之上的"最速控制"轨迹规划：
		//   相位平面开关线 σ = d − v|v|/(2a)，加速度只有 ±a_max 两态；
		//   v_max 只是长位移的速度上限（短位移是三角形，长位移才出现匀速段）。
		// 规划输出 pos_td_x1(位置) / pos_td_x2(速度) 交给下面的 PD + 前馈。
		// 这两个值只是上电默认值，运行中可由上位机实时下发覆盖。
		constexpr bool kPositionPlanEnable = true;
		constexpr float kPositionPlanVmax = 100.0f;					// 规划最大速度 (rad/s 机械)
		constexpr float kPositionPlanAccel = 300.0f;				// 规划最大加速度 (rad/s²)

		// ---------------- MIT 阻抗控制 ----------------
		// τ = Kp·(θd − θ) + Kd·(ωd − ω) + τff，然后 iq = τ / Kt。
		// 该模式旁路转速环，直接给电流环 q 轴指令。
		constexpr float kIqLimit = 5.6f;				// 电流限幅 (A)，MIT / 电流环通用
		constexpr float kMitMaxKp = 20.0f;				// 刚度上限 (N·m/rad)
		constexpr float kMitMaxKd = 2.0f;				// 阻尼上限 (N·m·s/rad)
		constexpr float kMitVelMax = 300.0f;			// 目标/反馈速度限幅 (rad/s)
		constexpr float kMitTauFfMax = 1.0f;			// 前馈力矩限幅 (N·m)
		constexpr float kMitTauLimitMax = 0.5f;			// 力矩指令上限 (N·m)
		constexpr float kMitDefaultTauLimit = 0.15f;	// 默认力矩限幅 (N·m)，约 3.4A
		constexpr float kMitPosErrMax = 0.5f;			// 位置误差限幅 (rad)，防止堵转储能过大
		constexpr float kMitGainRampMs = 100.0f;		// 进入 MIT 时 Kp/Kd 软启动 (ms)
		constexpr float kMitPosSlewRadS = 0.0f;			// 目标位置斜率限制 (rad/s)，0 = 直接跟随
		constexpr float kMitDefaultKp = 0.5f;			// 上电默认刚度 (N·m/rad)
		constexpr float kMitDefaultKd = 0.01f;			// 上电默认阻尼 (N·m·s/rad)
		// 力矩正方向：给 +iq 时机械角增大的方向为正。实测反了就改 -1。
		constexpr float kTorqueSign = 1.0f;
		// Kt 修正系数：Kt = 1.5·极对数·ψf·kKtScale。
		// 没有扭矩传感器时，可以用杠杆+电子秤测：给已知 iq，测力 F、力臂 r，
		// Kt_实测 = F·r / iq，然后 kKtScale = Kt_实测 / (1.5·pp·ψf)。
		// 只影响力矩换算，不影响反电势解耦。
		constexpr float kKtScale = 1.0f;

		// ---------------- 齿槽补偿（下一步用） ----------------
		// 12 槽 14 极：每转齿槽周期数 N = LCM(12,14) = 84
		constexpr uint16_t kStatorSlots = 12;
		constexpr uint16_t kCoggingOrder = 84;

		// ---------------- 齿槽补偿 ----------------
		// 齿槽力矩是位置周期函数：按机械角建表，学习后按角度注入 iq 前馈。
		constexpr uint16_t kCogBins = 720;			// 表分辨率（0.5°/点）
		constexpr float kCogLearnRpm = 8.0f;		// 学习转速 (rpm)
		constexpr float kCogLearnRevs = 3.0f;		// 学习圈数
		constexpr float kCogMaxA = 3.0f;			// 表值限幅 (A)
		constexpr bool kCogCompEnableDefault = true;

		// ---------------- 扫频 / 参数辨识 ----------------
		constexpr float kSweepCurrentFStart = 5.0f;		// 电流环扫频起止 (Hz)
		constexpr float kSweepCurrentFEnd = 2000.0f;	// 20kHz 中断内解调，不受 1kHz 遥测限制
		constexpr float kSweepSpeedFStart = 0.3f;		// 转速环扫频起止 (Hz)
		constexpr float kSweepSpeedFEnd = 200.0f;
		constexpr uint16_t kSweepPoints = 20;			// 扫频点数（对数分布）
		constexpr uint16_t kSweepCyclesPerPoint = 6;	// 每个频点停留周期数
		// 扫频（电流环 20kHz / 转速环 4kHz）都在中断里生成并做 I/Q 解调。
		// 每个频点至少积分这么久（秒），保证 1kHz 遥测能收到每个点的结果。
		constexpr float kSweepMinMeasureS = 0.08f;
		constexpr uint16_t kSweepSettleCycles = 2;		// 每点先空转几个周期再开始积分

		// Ld/Lq 辨识：双频正弦阻抗法
		//   在 f1、f2 各测一次阻抗 Z，R 在相减时被消掉：
		//     L = sqrt((Z2² − Z1²) / (ω2² − ω1²))
		//   同时可以从低频点反推 R = sqrt(Z1² − (ω1·L)²)
		constexpr float kLSineF1 = 500.0f;				// 低频点 (Hz)
		constexpr float kLSineF2 = 2000.0f;				// 高频点 (Hz)
		constexpr float kLSineVolt = 0.8f;				// 电压幅值 (V)
		constexpr uint16_t kLSineSettleCycles = 5;		// 先空转几个周期
		constexpr uint16_t kLSineMeasCycles = 20;		// 积分多少个周期
		constexpr uint8_t kLSineRepeat = 4;				// 每轴重复几次，取中位数
		constexpr float kDefaultBusVoltage = 12.0f;

		// ---------------- 温度保护（NTC 实测） ----------------
		// 电机 NTC 在绕组里，响应慢，分两级：预警降流 + 过温停机
		// 驱动板 NTC 贴在功率级附近，响应快，直接停机
		constexpr float kMotorTempDerateC = 80.0f;	// 降流起始 (°C)
		constexpr float kMotorTempStopC = 100.0f;		// 停机 (°C)
		constexpr float kDriverTempStopC = 90.0f;	// 驱动过温停机 (°C)
		constexpr float kTempRecoverHystC = 8.0f;	// 回差：低于阈值这么低才恢复
		// 降流曲线：80~100°C 线性把电流限幅从 100% 压到 30%
		constexpr float kTempDerateMinRatio = 0.3f;

		// ---------------- Rs 温度补偿 ----------------
		// 铜绕组温度系数约 +0.4%/°C，Rs(T)=Rs25·(1+0.004·(T-25))。
		// Ki = ω·Rs(T) 跟着修，否则温差 50°C 时 Ki 漂 20%。
		constexpr float kRsTempCoef = 0.004f;		// 1/°C
		constexpr float kRsTempRefC = 25.0f;		// 标称阻值的基准温度
		// 补偿上下限（NTC 开路/短路会读出 -273，限一下别让增益跑飞）
		constexpr float kRsTempMinC = -10.0f;
		constexpr float kRsTempMaxC = 130.0f;
		constexpr float kRsCompMaxRatio = 1.5f;	// 补偿后 Rs 最多放大 1.5 倍

		// ---------------- 死区补偿 ----------------
		// 补偿量只来自死区本身：ΔV = Vbus · Td · fs · sign(i)
		// 120ns = "MCU 88ns + 栅极驱动传播延迟" 的总有效死区。
		constexpr float kDeadTimeUs = 0.120f;		// 有效死区 120ns
		constexpr bool kDeadTimeCompEnable = true;
		// 附加压降项：死区期间续流二极管把相电压钳在 ±Vf，理论上只有
		// (Td/Ts)·Vf ≈ 1.7mV，可以忽略。MOS 导通压降 Vds_on 不是死区效应，
		// 它是绕组回路电阻的一部分，已经由电流环积分消掉，不能加在这里。
		constexpr float kDtCompDrop = 0.0f;			// 0 = 纯死区补偿
		constexpr float kDtCompKnee = 0.60f;		// S 曲线拐点 (A)，越小转折越陡
		constexpr float kDtCompSign = 1.0f;		// 补偿方向：实测 -1 才对

		// 电流采样方向：参考框架里 iRDT + 5mΩ + RS724 使用 -1。
		// 如果上电后 iq 反馈方向和指令相反，先把对应相改成 +1 再测。
		constexpr float kCurrentSignA = -1.0f;
		constexpr float kCurrentSignB = -1.0f;
		constexpr float kCurrentSignC = -1.0f;

		// 编码器方向：手转电机正向，机械角应该增大；如果减小就改成 -1
		constexpr float kEncoderDirection = -1.0f;

		// ---------------- 速度来源 ----------------
		// true = 只用 PLL；false = 差分 + 二阶巴特沃斯低通
		constexpr bool kUseSpeedPll = true;

		// 仅 kUseSpeedPll = false 时生效：二阶巴特沃斯截止频率 (Hz)。
		// 监控任务按 4kHz 消费编码器帧，滤波器按 fs=4kHz 设计。
		constexpr float kEncoderSpeedFilterHz = 50.0f;

		// 速度锁相环（PLL）：用角度闭环估计速度（仅 kUseSpeedPll = true 时生效）
		// 按参考设计给定的增益反推：
		//   Kp = 848.528 = 2ζωn  ┐
		//   Ki = 360000  = ωn²   ┘ → ωn = 600rad/s (95.5Hz)，ζ = 0.707
		// 这组 PLL 只支持到 ~30Hz 速度环（30Hz 处 PM≈39°，100Hz 处为负）。
		constexpr float kPllBandwidthHz = 95.493f;		// ωn = 600 rad/s
		constexpr float kPllDamping = 0.7071f;			// 2ζωn = 848.528

		// PLL 速度输出的一阶低通 (Hz)，0 = 关闭。
		// 参考设计给的是 Wc = 500 rad/s = 79.577Hz。
		// 注意：它和 PLL 一起限制了速度环上限，别在 100Hz 速度环下用这组。
		constexpr float kPllSpeedFilterHz = 79.577f;

		// 电角度零位。
		// 最新 8 点矢量平均实测：140.2°。
		// 已实测：138.74° 时给 iq 转子能正常旋转。
		// 48.57° / 48.74° 都比正确值小 90°，是"差 90°"的错误值，
		// 那个角度下 id 会转、iq 不转（d/q 轴互换），不要用。
		constexpr float kElecAngleOffsetDeg = 141.2f;
		
		// 电角度标定：多点对齐。
		// 只在一个角度对齐时，齿槽/静摩擦会把转子拉偏 1~3°，offset 就跟着偏。
		// 改成在 8 个电角度（45° 一步）分别对齐、各读一次编码器，再对 8 个
		// offset 做矢量平均，齿槽/摩擦的偏差会相互抵消；同时用它们的离散度
		// 检查编码器安装偏心/方向问题（离散度 > 15° 直接判失败）。
		constexpr float kAngleCalCurrent = 1.0f;		// d 轴电流 (A)
		constexpr uint16_t kAngleCalRampMs = 50;		// 电流上升斜坡 (ms)
		constexpr uint16_t kAngleCalSettleMs = 1000;		// 每个点等待转子稳定 (ms)
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
