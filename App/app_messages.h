#pragma once

#include "FOC_math.h"
#include <stdint.h>

/*==============================================================================
 * 应用层消息
 *
 *   所有跨模块传递的数据都定义在这里，模块之间不直接引用对方的类。
 *   生产者只写结构体，消费者只读结构体，队列/邮箱负责同步。
 *============================================================================*/

namespace app
{
	// 上位机模式，和 tools/serial_scope.py 保持一致
	enum Mode : uint8_t
	{
		MODE_CURRENT = 0,		// 电流闭环
		MODE_OPEN_LOOP = 1,		// 开环强拖
		MODE_SCOPE = 2,			// 波形观察（不开 MOE）
		MODE_ENCODER = 3,		// 转速闭环
		MODE_POSITION = 4		// 位置闭环
	};

	// 通信任务 -> 控制任务
	struct Command
	{
		bool enable = false;
		uint8_t mode = MODE_CURRENT;
		float iq_a = 0.0f;
		float id_a = 0.0f;				// 电流环 d 轴指令（A）
		float speed_rpm = 0.0f;
		float position_deg = 0.0f;		// 位置模式目标（机械角度，deg）
		// 电流环 PI 在线整定（flags bit3 置位时生效，否则保持固件默认）
		float kp_d = 0.0f;
		float ki_d = 0.0f;
		float kp_q = 0.0f;
		float ki_q = 0.0f;
		// 转速环 / 位置环 PI 在线整定（flags bit5 / bit6）
		float speed_kp = 0.0f;
		float speed_ki = 0.0f;
		float pos_kp = 0.0f;
		float pos_kd = 0.0f;
		bool tune_pi = false;			// true = 本次下发的 Kp/Ki 生效
		bool tune_speed = false;		// true = 转速环 Kp/Ki 生效
		bool tune_pos = false;			// true = 位置环 Kp/Ki 生效
		bool calibrate = false;
		bool reset_angle = false;
		bool calibrate_angle = false;		// 电角度零位标定
		// 正弦测试信号：0=关闭 1=速度正弦 2=位置正弦
		// 中心值用 speed_rpm / position_deg，幅值和频率用下面两个
		uint8_t test_type = 0;
		float test_amp = 0.0f;				// rpm 或 deg
		float test_freq = 0.0f;				// Hz
		uint32_t sequence = 0;
	};

	// 控制任务 -> 遥测（Latest）
	struct ControlSnapshot
	{
		bool valid = false;
		foc_math::Abc current = {0.0f, 0.0f, 0.0f};
		foc_math::Abc duty = {0.5f, 0.5f, 0.5f};
		foc_math::Dq i_dq = {0.0f, 0.0f};
		foc_math::Dq v_dq = {0.0f, 0.0f};
		foc_math::Abc raw = {0.0f, 0.0f, 0.0f};
		float v_bus = 0.0f;
		float speed_rpm = 0.0f;
		float speed_target_rpm = 0.0f;		// 转速环目标（rpm），给上位机画参考线
		float pos_target_deg = 0.0f;		// 位置环目标（机械 deg，可多圈）
		float test_freq_now = 0.0f;			// 扫频当前频率 (Hz)，0 = 未扫频
		float vd = 0.0f;					// 电流环 d 轴电压 (V)
		float vq = 0.0f;					// 电流环 q 轴电压 (V)
		float ld_h = 0.0f;					// 阶跃辨识的 Ld (H)
		float lq_h = 0.0f;					// 阶跃辨识的 Lq (H)
		float angle = 0.0f;
		float iq_target = 0.0f;			// 电流环 q 轴指令（A），给上位机对比反馈
		float id_target = 0.0f;			// 电流环 d 轴指令（A）
		uint16_t isr_avg_x10 = 0;
		uint16_t isr_max_x10 = 0;
		bool output_enable = false;
		bool open_loop = false;
		bool speed_loop = false;
		bool calibrating = false;
		bool cmd_ok = false;			// 最近 200ms 内收到并执行过命令
		uint8_t mode = MODE_CURRENT;
		uint8_t cal_status = 0;			// 0=空闲 1=电角度标定中 2=完成 3=失败
		// 当前实际生效的电流环参数（回显给上位机，便于在线整定）
		float kp_d = 0.0f;
		float ki_d = 0.0f;
		float kp_q = 0.0f;
		float ki_q = 0.0f;
		// 当前实际生效的转速环 / 位置环参数
		float speed_kp = 0.0f;
		float speed_ki = 0.0f;
		float pos_kp = 0.0f;
		float pos_kd = 0.0f;
		// 当前实际生效的电流零偏（ADC 码值，回显给上位机核对校准结果）
		uint16_t offs_a = 0;
		uint16_t offs_b = 0;
		uint16_t offs_c = 0;
	};

	// 监控任务 -> 遥测（Latest）
	struct EncoderSnapshot
	{
		bool valid = false;
		float mech_deg = 0.0f;
		float mech_rpm = 0.0f;
		uint16_t crc_errors = 0;
		uint16_t transport_errors = 0;
		uint32_t raw_word = 0;
		uint32_t frames = 0;
		float elec_offset_deg = 0.0f;
	};

	// 监控任务 -> 遥测/控制（Latest）
	struct PowerSnapshot
	{
		bool valid = false;
		float v_bus = 0.0f;
		float v_a = 0.0f;
		float v_b = 0.0f;
		float v_c = 0.0f;
		float motor_temp = 0.0f;
		float driver_temp = 0.0f;
		uint32_t raw[6] = {0, 0, 0, 0, 0, 0};
	};

	// 启动状态，初始化完成后不再变化
	struct SystemState
	{
		uint8_t boot_stage = 0;
		bool ready = false;
		bool adc_cal_failed = false;
	};

	// 遥测组帧的输入
	struct ScopeData
	{
		float ia = 0.0f;
		float ib = 0.0f;
		float ic = 0.0f;
		float da = 0.5f;
		float db = 0.5f;
		float dc = 0.5f;
		float id = 0.0f;
		float iq = 0.0f;
		float iq_target = 0.0f;
		float id_target = 0.0f;
		float vbus = 0.0f;
		int16_t speed_rpm = 0;
		float speed_target_rpm = 0.0f;
		float pos_target_deg = 0.0f;
		float test_freq_now = 0.0f;
		float vd = 0.0f;
		float vq = 0.0f;
		uint16_t ld_x10 = 0;		// Ld，单位 0.1µH
		uint16_t lq_x10 = 0;		// Lq，单位 0.1µH
		uint16_t status = 0;
		uint16_t raw_a = 0;
		uint16_t raw_b = 0;
		uint16_t raw_c = 0;
		uint16_t isr_avg_x10 = 0;
		uint16_t isr_max_x10 = 0;
		float enc_deg = 0.0f;
		int16_t enc_rpm = 0;
		uint16_t enc_crc_err = 0;
		uint32_t enc_raw_word = 0;
		float elec_offset_deg = 0.0f;
		float kp_d = 0.0f;
		float ki_d = 0.0f;
		float kp_q = 0.0f;
		float ki_q = 0.0f;
		float speed_kp = 0.0f;
		float speed_ki = 0.0f;
		float pos_kp = 0.0f;
		float pos_kd = 0.0f;
		uint16_t offs_a = 0;
		uint16_t offs_b = 0;
		uint16_t offs_c = 0;
	};
}
