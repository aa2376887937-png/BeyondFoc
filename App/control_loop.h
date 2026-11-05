#pragma once

#include "RC_adc.h"
#include "RC_pwm.h"
#include "RC_tim.h"
#include "FOC_control.h"
#include "FOC_current.h"
#include "FOC_openloop.h"
#include "FOC_position.h"
#include "MT6701.h"
#include "Framework/latest.h"
#include "Framework/spsc_ring.h"
#include "app_messages.h"
#include "app_config.h"

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

		adc::Adc &adc;
		pwm::Pwm &pwm;
		mt6701::Sensor *encoder = nullptr;

		framework::SpscRing<Command, 8> &commands;
		framework::Latest<ControlSnapshot> &snapshot_out;
		framework::Latest<PowerSnapshot> &power_in;

		foc_current::Processor current;
		Foc_Controller controller;
		float position_kp = config::kPositionKp;		// 位置环 PD
		float position_kd = config::kPositionKd;

		// 位置环增强：D 低通、死区回差、轨迹规划状态
		float pos_d_alpha = 0.0f;
		float pos_d_filt = 0.0f;
		float pos_td_x1 = 0.0f;			// TD 规划位置 (rad)
		float pos_td_x2 = 0.0f;			// TD 规划速度 (rad/s)
		bool pos_db_hold = false;		// 死区回差状态

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

		// Ld/Lq 阶跃辨识
		uint8_t l_state = 0;			// 0=idle 1=D 阶跃 2=冷却 3=Q 阶跃 4=结束
		uint16_t l_tick = 0;
		float l_i0 = 0.0f;
		float l_ld = 0.0f;
		float l_lq = 0.0f;
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

		float v_bus = 24.0f;
		float dt = 50.0e-6f;
		foc_math::AlphaBeta last_mod = {0.0f, 0.0f};

		uint8_t pole_pairs = 7;
	};
}
