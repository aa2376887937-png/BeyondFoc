#pragma once

#include "RC_adc.h"
#include "RC_pwm.h"
#include "RC_tim.h"
#include "FOC_control.h"
#include "FOC_current.h"
#include "FOC_openloop.h"
#include "FOC_position.h"
#include "MT6701.h"
#include "RC_encoder.h"
#include "Framework/latest.h"
#include "Framework/spsc_ring.h"
#include "app_messages.h"
#include "app_config.h"
#include "capture.h"

/*==============================================================================
 * 控制环（应用层）
 *
 *   把"电流采样中断 -> 电流处理 -> 控制器 -> PWM 输出"串起来。
 *   它知道要调用哪些底层模块，但底层模块不知道它存在。
 *
 *   生产者/消费者：
 *     - 命令：通信任务 Push -> 控制任务 Pop
 *     - 电流：ADC 注入中断生产采样 -> 本类在中断里消费
 *     - 遥测：控制任务 Publish 最新快照 -> 通信任务读取
 *     - 母线：监控任务 Publish 最新值 -> 控制任务读取
 *============================================================================*/

namespace app
{
	class ControlLoop : public adc::AdcInjectedHandler, public tim::TimHandler
	{
	public:
		ControlLoop(adc::Adc &adc_, pwm::Pwm &pwm_, tim::Tim &tick_, uint8_t pole_pairs_,
					framework::SpscRing<Command, 8> &commands_,
					framework::Latest<ControlSnapshot> &snapshot_out_,
					framework::Latest<PowerSnapshot> &power_in_);

		virtual ~ControlLoop() {}

		void Init();
		void Set_Encoder(mt6701::Sensor *encoder_) { encoder = encoder_; }

		// 板内捕获缓冲：中断写、遥测任务读
		CaptureBuffer &Capture() { return capture; }

		// 编码器 SPI 帧源：由 20kHz 电流环中断直接消费（速度估计 20kHz 更新）
		void Set_Frame_Source(encoder::SpiEncoder *src_) { frame_src = src_; }
		void Set_Bus_Voltage(float v_bus_);

		// 控制任务 1kHz：消费命令、标定、发布遥测快照（转速/位置环在 ADC 中断里执行）
		void Speed_Loop_Process(float dt);

		foc_current::Processor &Current() { return current; }
		openloop::Drive &OpenLoop() { return open_loop; }

	protected:
		void Adc_InjectedConvCplt_Process() override;
		void Tim_It_Process() override;			// TIM2 1kHz：开环强拖/波形观察

	private:
		void Apply_Command(const Command &cmd_);
		void Start_Angle_Calibration();
		void Update_Angle_Calibration();
		void Finish_Angle_Calibration(bool ok_);
		void Set_Output_Enable(bool on_);
		void Publish_Snapshot();
		void Thermal_Protect();		// 过温状态机：降流/停机/恢复
		void Update_Rs_Temp_Comp(bool force_reset_);	// Rs 温补 -> Ki
		void Update_Flux_Estimate();	// 在线磁链辨识（q 轴稳态反电势）
		void Mit_Step(float dt);	// MIT 阻抗：4kHz 算 τ -> iq

		adc::Adc &adc;
		pwm::Pwm &pwm;
		mt6701::Sensor *encoder = nullptr;
		encoder::SpiEncoder *frame_src = nullptr;	// 20kHz 中断里直接消费 SPI 帧

		framework::SpscRing<Command, 8> &commands;
		framework::Latest<ControlSnapshot> &snapshot_out;
		framework::Latest<PowerSnapshot> &power_in;

		foc_current::Processor current;
		Foc_Controller controller;
		float position_kp = config::kPositionKp;		// 位置环 PD
		float position_kd = config::kPositionKd;

		// MIT 阻抗模式（力矩级，旁路转速环）
		float mit_pos_des = 0.0f;		// 目标机械角 (rad，连续多圈)
		float mit_pos_slew = 0.0f;		// 斜率限制后的目标
		float mit_vel_des = 0.0f;		// 目标机械角速度 (rad/s)
		float mit_kp = config::kMitDefaultKp;	// N·m/rad
		float mit_kd = config::kMitDefaultKd;	// N·m·s/rad
		float mit_tau_ff = 0.0f;		// N·m
		float mit_tau_limit = config::kMitDefaultTauLimit;	// N·m
		float mit_gain_scale = 0.0f;	// Kp/Kd 软启动 0..1
		float mit_tau_cmd = 0.0f;		// 本拍实际力矩指令，调试用
		bool mit_active = false;

		// 位置环增强：D 低通、死区回差、轨迹规划状态
		float pos_d_alpha = 0.0f;
		float pos_d_filt = 0.0f;
		float pos_td_x1 = 0.0f;			// TD 规划位置 (rad)
		float pos_td_x2 = 0.0f;			// TD 规划速度 (rad/s)
		float pos_td_a = 0.0f;			// 规划加速度 (rad/s²)，用于加速度前馈
		bool pos_db_hold = false;		// 死区回差状态
		CaptureBuffer capture;			// 板内高速捕获
		bool last_capture_cmd = false;	// 捕获命令上升沿检测
		bool last_l_ident_cmd = false;	// L 辨识命令上升沿检测
		bool last_cog_learn_cmd = false;	// 齿槽建表命令上升沿检测
		float cap_exc_q = 0.0f;			// 电流扫频当前激励 (A)
		float cap_exc_speed = 0.0f;		// 转速扫频当前激励 (rpm)
		// 最速规划的运行参数（可由上位机在线改，0 = 用配置默认值）
		float plan_vmax = config::kPositionPlanVmax;
		float plan_accel = config::kPositionPlanAccel;

		// 正弦测试信号（由 1kHz 任务生成）
		uint8_t test_type = 0;			// 0=off 1=速度 2=位置
		float test_amp = 0.0f;			// rpm 或 deg
		float test_freq = 0.0f;			// Hz
		float test_phase = 0.0f;		// rad
		float test_center_speed = 0.0f;	// rpm
		float test_center_pos = 0.0f;	// rad

		// 扫频
		float sweep_freq = 0.0f;		// 当前频率 (Hz)
		uint16_t sweep_point = 0;
		uint16_t sweep_cycles = 0;
		float test_freq_now = 0.0f;		// 上报给上位机

		// 电流环扫频：20kHz 中断里生成正弦 + 板内 I/Q 解调
		volatile bool sweep_isr_active = false;
		float sweep_phase = 0.0f;
		uint16_t sweep_meas_cycles = 3;
		float sweep_i_acc = 0.0f;
		float sweep_q_acc = 0.0f;
		uint32_t sweep_acc_n = 0;

		// 转速环扫频：4kHz（转速环节拍）生成 + 解调
		volatile bool speed_sweep_active = false;
		uint16_t speed_sweep_meas_cycles = 3;

		void Sweep_Advance(float feedback, float sc_sin, float sc_cos, float dt, bool is_speed);
		volatile bool sweep_res_valid = false;
		volatile float sweep_res_freq = 0.0f;
		volatile float sweep_res_mag_db = 0.0f;
		volatile float sweep_res_phase_deg = 0.0f;

		// Ld/Lq 双频正弦阻抗辨识
		uint8_t l_state = 0;			// 0=idle 1=运行中
		uint8_t l_sine_axis = 0;		// 0=d 轴 1=q 轴
		float l_sine_f = config::kLSineF1;	// 当前注入频率
		uint8_t l_freq_idx = 0;			// 0=低频点 1=高频点
		float l_z1 = 0.0f;				// 低频点测到的阻抗
		uint16_t l_sine_cycles = 0;
		float l_sine_phase = 0.0f;
		float l_sine_i = 0.0f;
		float l_sine_q = 0.0f;
		uint32_t l_sine_n = 0;
		float l_ld = 0.0f;
		float l_lq = 0.0f;

		uint8_t l_rep = 0;
		uint8_t l_cnt_d = 0;
		uint8_t l_cnt_q = 0;
		float l_samp_d[config::kLSineRepeat] = {0.0f};
		float l_samp_q[config::kLSineRepeat] = {0.0f};

		// 双频法顺便得到的电阻（和测量时的温度），给磁链估计用
		float r_samp[config::kLSineRepeat] = {0.0f};
		uint8_t r_cnt = 0;
		float r_ident = config::kMotorR;
		float r_ident_temp = config::kRsTempRefC;

		void L_Sine_Advance(float id, float iq, float sc_sin, float sc_cos);
		static float Median_Of(float *a, uint8_t n);

		// ---------------- 齿槽补偿 ----------------
		bool cog_learning = false;		// 正在学习
		bool cog_valid = false;			// 表有效
		bool cog_enabled = config::kCogCompEnableDefault;
		bool cog_invert = false;		// 补偿反号（调试）
		float cog_sum[config::kCogBins] = {};
		uint16_t cog_cnt[config::kCogBins] = {};
		int16_t cog_table[config::kCogBins] = {};	// 单位 mA，负号表示补偿方向
		float cog_angle_acc = 0.0f;		// 累计转过的角度（判圈数）
		float cog_last_angle = 0.0f;

		void Cogging_Start();
		void Cogging_Apply();
		void Cogging_Learn(float iq_fb);
		void Cogging_Finish();
		open_loop_sensor open_loop_position;
		smo_sensor smo;
		openloop::Drive open_loop;

		position_sensor *position = &open_loop_position;

		volatile uint8_t mode = MODE_CURRENT;
		volatile bool requested_enable = false;
		bool output_enable = false;
		volatile bool speed_loop = false;
		volatile bool position_loop = false;
		bool last_encoder_valid = false;
		float last_speed_target = 0.0f;
		float target_position_rad = 0.0f;

		volatile bool cal_active = false;
		uint16_t cal_ticks = 0;
		uint8_t cal_status = 0;			// 0=空闲 1=标定中 2=完成 3=失败
		uint8_t cal_point = 0;			// 多点标定：当前第几个电角度
		float cal_off[config::kAngleCalPoints] = {0.0f};	// 每个点算出的 offset

		uint32_t last_cmd_ms = 0;		// 最近一次执行命令的时间

		// 过温保护状态
		uint8_t thermal_state = 0;		// 0=正常 1=降流 2=停机
		float derate_ratio = 1.0f;		// 当前电流限幅比例
		float motor_temp_latched = 25.0f;	// 最近一次有效绕组温度
		float driver_temp_latched = 25.0f;
		float psi_est = config::kMotorFlux;	// 在线磁链估计 (Wb)
		uint8_t psi_state = 0;				// 磁链估计门控状态

		uint8_t speed_decim = 5;		// 20kHz ADC / 4kHz 转速环
		uint8_t speed_tick = 0;
		uint8_t position_decim = 25;	// 20kHz ADC / 800Hz 位置环
		uint8_t position_tick = 0;
		uint8_t current_decim = 1;		// 电流环降频：1=20kHz，2=10kHz
		uint8_t current_tick = 0;
		float speed_dt = 0.00025f;		// 转速环实际周期（随分频变化）
		float position_dt = 0.00125f;	// 位置环实际周期

		// 电流环 PI 在线整定：保存上电默认值，上位机取消整定时恢复
		float pi_kp_d0 = 0.0f;
		float pi_ki_d0 = 0.0f;
		float pi_kp_q0 = 0.0f;
		float pi_ki_q0 = 0.0f;
		bool pi_tuned = false;
		bool speed_tuned = false;
		bool pos_tuned = false;
		bool pi_incremental = false;	// 电流/转速 PI 模式（上位机可切）

		float v_bus = 24.0f;
		float dt = 50.0e-6f;
		foc_math::AlphaBeta last_mod = {0.0f, 0.0f};

		uint8_t pole_pairs = 7;
	};
}
